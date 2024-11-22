#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
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
	OffboardControl() : Node("px4_ros_bridge"), yaw_(0.0)
	{
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		rclcpp::QoS qos_status(rclcpp::KeepLast(10));
		qos_status.best_effort(); // reduce Quality of Service setting to align with vehicle status settings

		vehicle_land_detected_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
            "/fmu/out/vehicle_land_detected",
            qos_status,
            [this](px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
				flying_ = !msg->landed;
			}
		);

        pos_ref_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/gbeam/gbeam_pos_ref", 
			1,
			[this](const geometry_msgs::msg::PoseStamped::SharedPtr tar_pos_ptr){
				//RCLCPP_INFO(this->get_logger(), "refCallback executed. Point received x: %f y: %f",tar_pos_ptr->pose.position.x, tar_pos_ptr->pose.position.y);
				target_pos_ = *tar_pos_ptr;
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

		mapping_enabled_ = false;
		position_homed_ = false;

		auto timer_callback = [this]() -> void {
			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();

			// RCLCPP_INFO(this->get_logger(),"Armed: %d - Ready to arm: %d - Flying: %d - Mapping %d", armed_, ready_to_arm_, flying_, mapping_enabled_);
			RCLCPP_INFO(this->get_logger(),"Setting home");
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_HOME, 1, 0);

			// when the vehicle is ready to arm, switch to offboard mode and arm
			if (ready_to_arm_ && !flying_ && !armed_) {
				
				if (!position_homed_) {
					// Set home position
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_HOME, 1, 0);
					position_homed_ = true;
				}
				else {
					// Set the vehicle to offboard mode
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
					// Arm the vehicle
					this->arm();
				}
			}

			// if armed and not flying: takeoff
			if (armed_ && !flying_) {
				this->takeoff();
			}

			// if armed and flying, but not mapping, publish (0,0) setpoint
			if (armed_ && flying_ && !mapping_enabled_) {
				target_pos_.pose.position.x = 0.0,
				target_pos_.pose.position.y = 0.0;
				publish_trajectory_setpoint();
			}

			// if armed and flying, publish setpoint
			if (armed_ && flying_) {
				if (!mapping_enabled_) {
            	}
            	publish_trajectory_setpoint();
        	}
			
		};
		timer_ = this->create_wall_timer(200ms, timer_callback);

		// DECLARATION OF PARAMETERS FROM YAML FILE
        this-> declare_parameter<float>("navigation_altitude",3.0);
        this-> declare_parameter<float>("angular_vel",0.0);

        // Get parameter from yaml file
        navigation_altitude = this->get_parameter("navigation_altitude").get_parameter_value().get<float>();
        angular_vel = this->get_parameter("angular_vel").get_parameter_value().get<float>();

		// RCLCPP_INFO(this->get_logger(),"1) navigation_altitude: %f", navigation_altitude);
	}

	void arm();
	void disarm();
	void takeoff();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr vehicle_land_detected_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pos_ref_subscriber_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
    geometry_msgs::msg::PoseStamped target_pos_;
	bool flying_, ready_to_arm_, armed_, mapping_enabled_, position_homed_;
	float navigation_altitude;
	float angular_vel;

	float yaw_;

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
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

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint()
{

	// RCLCPP_INFO(this->get_logger(), "GBEAM!");

	TrajectorySetpoint msg{};
	msg.position = {
        static_cast<float>(target_pos_.pose.position.x),
        static_cast<float>(target_pos_.pose.position.y),
        static_cast<float>(-navigation_altitude)
    };

	//increment yaw value for continuous spinning
	yaw_ += angular_vel*0.2;

	if (yaw_ > M_PI) {
		yaw_ -= 2 * M_PI; // keep yaw within [-PI, PI]
	}

	msg.yaw = yaw_; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);

	// RCLCPP_INFO(this->get_logger(), "Yaw given: %f", yaw_);
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

void OffboardControl::takeoff()
{
	// RCLCPP_INFO(this->get_logger(), "Taking off!");

	TrajectorySetpoint msg{};
	msg.position = {
        static_cast<float>(0.0),
        static_cast<float>(0.0),
        static_cast<float>(-navigation_altitude)
    };
	msg.yaw = 0.0; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
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