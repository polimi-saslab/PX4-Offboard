#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
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
		rclcpp::QoS qos_status(rclcpp::KeepLast(10));
		qos_status.best_effort(); // reduce Quality of Service setting to align with vehicle status settings

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

		position_homed_ = false;

		auto timer_callback = [this]() -> void {

			if(flag_control_offboard_enabled_ && offboard_switch_activated_) {
				// offboard_control_mode needs to be paired with trajectory_setpoint
				publish_offboard_control_mode();

				// when the vehicle is ready to arm, switch to offboard mode and arm
				if (ready_to_arm_ && !flying_ && !armed_){
					
					if (!position_homed_) {
						// Set home position
						this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_HOME, 1, 0);
						position_homed_ = true;
					}
					else {
						// Set the vehicle to offboard mode
						this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
						// Arm the vehicle
						publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
						RCLCPP_INFO(this->get_logger(), "Arm command sent");
					}
					
					step_counter_ = 0;
				}

				// if armed and not flying: takeoff to 1 meter
				if (armed_ && !flying_) {
					this->publish_trajectory_setpoint(std::vector<float>{0.0, 0.0, 1.0, 0.0});
				}


				// if armed and flying, publish waypoints
				if (armed_ && flying_) {
					if (step_counter_ < 1)
						publish_trajectory_setpoint(std::vector<float>{0.0, 0.0, 2.0, 0.0});
					else if (step_counter_ < 2)
						publish_trajectory_setpoint(std::vector<float>{2.0, 0.0, 2.0, 0.0});
					else if (step_counter_ < 3)
						publish_trajectory_setpoint(std::vector<float>{2.0, 2.0, 2.0, 0.0});
					else if (step_counter_ < 4)
						publish_trajectory_setpoint(std::vector<float>{0.0, 2.0, 2.0, 0.0});
					else if (step_counter_ < 5)
						publish_trajectory_setpoint(std::vector<float>{0.0, 0.0, 2.0, 0.0});
					else {
						// Land the vehicle
						this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND, 0, 0);
						RCLCPP_INFO(this->get_logger(), "Landing command sent");
					}
				}
			}
			else
				RCLCPP_INFO(this->get_logger(), "Waiting for offboard control mode");
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
	rclcpp::Subscription<px4_msgs::msg::RcChannels>::SharedPtr rc_channels_subscriber_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

    rclcpp::TimerBase::SharedPtr debounce_timer_setpoint_, debounce_timer_offboard_;
    std::chrono::milliseconds debounce_duration_;

    geometry_msgs::msg::PoseStamped target_pos_;
	bool flying_, ready_to_arm_, armed_, position_homed_;
	bool flag_control_offboard_enabled_;
	float navigation_altitude;
	float angular_vel;

	bool setpoint_button_pressed_, offboard_switch_activated_;
	float step_counter_;

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(const std::vector<float> &target_pos);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
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