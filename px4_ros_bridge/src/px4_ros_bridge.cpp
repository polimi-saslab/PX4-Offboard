#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/rc_channels.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("px4_ros_bridge"),
        debounce_timer_setpoint_(nullptr),
		debounce_timer_offboard_(nullptr),
        debounce_duration_(std::chrono::milliseconds(100)),
		setpoint_button_pressed_(false),
		offboard_switch_activated_(false)
	{
		// Initialize the publishers
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		// Initialize the subscribers
		rclcpp::QoS qos_status(rclcpp::KeepLast(10)), qos_ack(rclcpp::KeepLast(10));
		qos_status.best_effort(); // reduce Quality of Service setting to align with vehicle status settings
		qos_ack.best_effort(); // reduce Quality of Service setting to align with vehicle command ack settings

		// Subscribe to land detected to check if the drone is flying
		vehicle_land_detected_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
            "/fmu/out/vehicle_land_detected",
            qos_status,
            [this](px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
				flying_ = !msg->landed;
			}
		);

		// Subscribe to vehicle status to check if the drone is ready to arm
        vehicle_status_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status",
            qos_status,
            [this](px4_msgs::msg::VehicleStatus::SharedPtr status_ptr) {
				ready_to_arm_ = status_ptr->pre_flight_checks_pass;
				armed_ = status_ptr->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
            }
        );

		// Subscribe to flight mode to check if the drone is in offboard mode
		control_mode_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleControlMode>(
			"/fmu/out/vehicle_control_mode",
			qos_status,
			[this](px4_msgs::msg::VehicleControlMode::SharedPtr msg) {
				flag_control_offboard_enabled_ = msg->flag_control_offboard_enabled;
			}
		);

		// Subscribe to RC channels to get switches values
		rc_channels_subscriber_ = this->create_subscription<px4_msgs::msg::RcChannels>(
			"/fmu/out/rc_channels",
			qos_status,
			std::bind(&OffboardControl::rc_channels_callback, this, std::placeholders::_1)
		);

		// Subscribe to vehicle command ack
		vehicle_command_ack_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleCommandAck>(
			"/fmu/out/vehicle_command_ack",
			qos_ack,
			[this](px4_msgs::msg::VehicleCommandAck::SharedPtr msg) {
				switch (status_) {
					case STATUS_HOMING:
						if (msg->result == VEHICLE_CMD_RESULT_ACCEPTED && msg->command == VehicleCommand::VEHICLE_CMD_DO_SET_HOME) {
							RCLCPP_INFO(this->get_logger(), "Home position set - ack received");
							position_homed_ = true;
						}
						break;
					default:
						if (msg->result == VEHICLE_CMD_RESULT_ACCEPTED) {
							RCLCPP_INFO(this->get_logger(), "Vehicle command accepted");
						} else if (msg->result == VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED) {
							RCLCPP_INFO(this->get_logger(), "Vehicle command temporarily rejected");
						} else if (msg->result == VEHICLE_CMD_RESULT_DENIED) {
							RCLCPP_INFO(this->get_logger(), "Vehicle command denied");
						} else if (msg->result == VEHICLE_CMD_RESULT_UNSUPPORTED) {
							RCLCPP_INFO(this->get_logger(), "Vehicle command unsupported");
						} else if (msg->result == VEHICLE_CMD_RESULT_FAILED) {
							RCLCPP_INFO(this->get_logger(), "Vehicle command failed");
						} else if (msg->result == VEHICLE_CMD_RESULT_IN_PROGRESS) {
							RCLCPP_INFO(this->get_logger(), "Vehicle command in progress");
						} else if (msg->result == VEHICLE_CMD_RESULT_CANCELLED) {
							RCLCPP_INFO(this->get_logger(), "Vehicle command cancelled");
						}
						break;
				}
				
			}
		);

		position_homed_ = false;
		status_ = STATUS_IDLE;

		auto timer_callback = [this]() -> void {
			
			// if offboard mode is not activated, set status to offboard deactivated
			if (!offboard_switch_activated_) {
				status_ = STATUS_OFFBOARD_DEACTIVATED;
			}
			
			// offboard_control_mode needs to be paired with trajectory_setpoint


			switch (status_) {
				case STATUS_OFFBOARD_DEACTIVATED:
					if (flag_control_offboard_enabled_) {
						RCLCPP_INFO(this->get_logger(), "Offboard mode switch activated");
						status_ = STATUS_IDLE;
					}
					break;
				case STATUS_IDLE:
					if (ready_to_arm_ && !armed_) {
						if (!position_homed_) {
							status_ = STATUS_READY_TO_HOME;
						} else {
							status_ = STATUS_READY_TO_ARM;
						}
					} else if (ready_to_arm_ && armed_ && !flying_) {
						status_ = STATUS_READY_TO_TAKEOFF;
					} else if (ready_to_arm_ && armed_ && flying_) {
						status_ = STATUS_HOLDING;
					} else
						RCLCPP_INFO(this->get_logger(), "Waiting for vehicle to be ready to arm");
					break;
				case STATUS_READY_TO_HOME:
					publish_offboard_control_mode();
					// Send homing command
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_HOME, 1, 0);
					RCLCPP_INFO(this->get_logger(), "Homing command sent");
					// change status and wait for ack
					position_homed_ = false;
					status_ = STATUS_HOMING;
					break;
				case STATUS_HOMING:
					publish_offboard_control_mode();
					if (position_homed_) {
						status_ = STATUS_READY_TO_ARM;
					}
					break;
				case STATUS_READY_TO_ARM:
					publish_offboard_control_mode();
					// Set the vehicle to offboard mode
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
					// Arm the vehicle
					publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
					RCLCPP_INFO(this->get_logger(), "Arm command sent");
					status_ = STATUS_ARMING;
					break;
				case STATUS_ARMING:
					publish_offboard_control_mode();
					if (armed_) {
						RCLCPP_INFO(this->get_logger(), "Vehicle armed");
						status_ = STATUS_READY_TO_TAKEOFF;
					}
					break;
				case STATUS_READY_TO_TAKEOFF:
					publish_offboard_control_mode();
					this->publish_trajectory_setpoint(std::vector<float>{NAN, NAN, 2.0, 0.0});
						RCLCPP_INFO(this->get_logger(), "Takeoff command sent");
						status_ = STATUS_TAKINGOFF;
					break;
				case STATUS_TAKINGOFF:
					publish_offboard_control_mode();
					this->publish_trajectory_setpoint(std::vector<float>{NAN, NAN, 2.0, 0.0});
					if (flying_) {
						status_ = STATUS_HOLDING;
					} 
					break;
				case STATUS_HOLDING:
					publish_offboard_control_mode();
					this->publish_trajectory_setpoint(std::vector<float>{NAN, NAN, 2.0, 0.0});
					step_counter_ = 0;
					status_ = STATUS_PARSING_WAYPOINTS;
					break;
				case STATUS_PARSING_WAYPOINTS:
					publish_offboard_control_mode();
					if (step_counter_ < 25)
						publish_trajectory_setpoint(std::vector<float>{0.0, 0.0, 2.0, 0.0});
					else if (step_counter_ < 50)
						publish_trajectory_setpoint(std::vector<float>{5.0, 0.0, 2.0, 0.0});
					else if (step_counter_ < 75)
						publish_trajectory_setpoint(std::vector<float>{5.0, 5.0, 2.0, 0.0});
					else if (step_counter_ < 100)
						publish_trajectory_setpoint(std::vector<float>{0.0, 5.0, 2.0, 0.0});
					else if (step_counter_ < 125)
						publish_trajectory_setpoint(std::vector<float>{0.0, 0.0, 2.0, 0.0});
					else
						publish_trajectory_setpoint(std::vector<float>{0.0, 0.0, 2.0, 0.0});
					step_counter_++;
					break;
				default:
					status_ = STATUS_IDLE;
					break;
			}			
		};
		timer_ = this->create_wall_timer(200ms, timer_callback);
	}
	
	void rc_channels_callback(const px4_msgs::msg::RcChannels::SharedPtr msg) {
		if (debounce_timer_setpoint_ == nullptr || debounce_timer_setpoint_->is_canceled()) {
			if (msg->channels[9] > 0.5 && !setpoint_button_pressed_) {
				// go to next step
				setpoint_button_pressed_ = true;
				step_counter_++;
			}
			else if (msg->channels[9] < 0.5 && setpoint_button_pressed_)
				setpoint_button_pressed_ = false;

			debounce_timer_setpoint_ = this->create_wall_timer(debounce_duration_, [this]() {debounce_timer_setpoint_->cancel();});
		}
		if (debounce_timer_offboard_ == nullptr || debounce_timer_offboard_->is_canceled()) {
			if (msg->channels[10] > 0.5 && !offboard_switch_activated_) {
				// go to next step
				offboard_switch_activated_ = true;
				step_counter_++;
			}
			else if (msg->channels[10] < 0.5 && offboard_switch_activated_)
				offboard_switch_activated_ = false;

			debounce_timer_offboard_ = this->create_wall_timer(debounce_duration_, [this]() {debounce_timer_offboard_->cancel();});
		}
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr vehicle_land_detected_subscriber_;
	rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr control_mode_subscriber_;
	rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_command_ack_subscriber_;
	rclcpp::Subscription<px4_msgs::msg::RcChannels>::SharedPtr rc_channels_subscriber_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

    rclcpp::TimerBase::SharedPtr debounce_timer_setpoint_, debounce_timer_offboard_;
    std::chrono::milliseconds debounce_duration_;

    geometry_msgs::msg::PoseStamped target_pos_;

	uint8_t status_;
	bool flying_, ready_to_arm_, armed_, position_homed_;
	bool flag_control_offboard_enabled_;
	float navigation_altitude;
	float angular_vel;

	bool setpoint_button_pressed_, offboard_switch_activated_;
	float step_counter_;

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(const std::vector<float> &target_pos);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

	static constexpr uint8_t STATUS_IDLE = 00;
	static constexpr uint8_t STATUS_OFFBOARD_DEACTIVATED = 01;
	static constexpr uint8_t STATUS_READY_TO_HOME = 10;
	static constexpr uint8_t STATUS_HOMING = 11;
	static constexpr uint8_t STATUS_READY_TO_ARM = 20;
	static constexpr uint8_t STATUS_ARMING = 21;
	static constexpr uint8_t STATUS_READY_TO_TAKEOFF= 30;
	static constexpr uint8_t STATUS_TAKINGOFF = 31;
	static constexpr uint8_t STATUS_HOLDING = 40;
	static constexpr uint8_t STATUS_PARSING_WAYPOINTS = 41;

	static constexpr uint8_t VEHICLE_CMD_RESULT_ACCEPTED = 0;
	static constexpr uint8_t VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED = 1;
	static constexpr uint8_t VEHICLE_CMD_RESULT_DENIED = 2;
	static constexpr uint8_t VEHICLE_CMD_RESULT_UNSUPPORTED = 3;
	static constexpr uint8_t VEHICLE_CMD_RESULT_FAILED = 4;
	static constexpr uint8_t VEHICLE_CMD_RESULT_IN_PROGRESS = 5;
	static constexpr uint8_t VEHICLE_CMD_RESULT_CANCELLED = 6;

	static constexpr uint16_t ARM_AUTH_DENIED_REASON_GENERIC = 0;
	static constexpr uint16_t ARM_AUTH_DENIED_REASON_NONE = 1;
	static constexpr uint16_t ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT = 2;
	static constexpr uint16_t ARM_AUTH_DENIED_REASON_TIMEOUT = 3;
	static constexpr uint16_t ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE = 4;
	static constexpr uint16_t ARM_AUTH_DENIED_REASON_BAD_WEATHER = 5;
};

void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

void OffboardControl::publish_trajectory_setpoint(const std::vector<float> &target_pos) // East North Up Yaw
{
	if (target_pos.size() != 4) {
		RCLCPP_ERROR(this->get_logger(), "Target position vector must have 4 elements: x (East), y (North), z (Up), yaw");
		return;
	}

	TrajectorySetpoint msg{};
	msg.position = {
		static_cast<float>(-target_pos[1]),
		static_cast<float>(target_pos[0]),
		static_cast<float>(-target_pos[2])
	};

	// Set yaw value
	msg.yaw = target_pos[3];
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);

	RCLCPP_INFO(this->get_logger(), "Trajectory setpoint sent: %2.2f, %2.2f, %2.2f, %2.2f", target_pos[0], target_pos[1], target_pos[2], target_pos[3]);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());
	rclcpp::shutdown();
	return 0;
}