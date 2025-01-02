#include "rclcpp/rclcpp.hpp"
#include "chrono"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/msg/twist_stamped.hpp"

using std::placeholders::_1;

using namespace std::chrono;
using namespace std::chrono_literals;

class OdomBroadcaster : public rclcpp::Node
{
public:
  OdomBroadcaster() : Node("odom_broadcaster")
  {
    // Set use_sim_time parameter -- Commented out: set this in the launcher
    // this->set_parameter(rclcpp::Parameter("use_sim_time", true));
    
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    // Create the subscriber
    subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "fmu/out/vehicle_odometry", qos, std::bind(&OdomBroadcaster::OdomCallback, this, std::placeholders::_1));

    // Create the dynamic transform broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Create the vehicle velocity publisher
    twist_NED_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("vehicle_speed_NED", 10);
    twist_vehicle_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("vehicle_speed_vehicle", 10);
  }

private:
  void OdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    // Create a TransformStamped message
    geometry_msgs::msg::TransformStamped transform_stamped;    

    // Fill in the header
    transform_stamped.header.stamp = this->get_clock()->now();
    transform_stamped.header.frame_id = "odom_NED";

    if (msg->pose_frame == POSE_FRAME_NED)
    {  
      // Set the child frame
      transform_stamped.child_frame_id = "vehicle_FRD";
      // Fill in the transform
      transform_stamped.transform.translation.x = msg->position[0];
      transform_stamped.transform.translation.y = msg->position[1];
      transform_stamped.transform.translation.z = msg->position[2];
      
      transform_stamped.transform.rotation.x = msg->q[1];
      transform_stamped.transform.rotation.y = msg->q[2];
      transform_stamped.transform.rotation.z = msg->q[3];
      transform_stamped.transform.rotation.w = msg->q[0];
      // Send the transform
      tf_broadcaster_->sendTransform(transform_stamped);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Unknown pose frame");
    }

    // Create a TwistStamped message
    geometry_msgs::msg::TwistStamped twist_stamped_NED, twist_stamped_vehicle;
    // fill in the header
    twist_stamped_NED.header.stamp = this->get_clock()->now();
    // fill in the twist
    if (msg->velocity_frame == VELOCITY_FRAME_NED)
    {
      twist_stamped_NED.header.frame_id = "odom_NED";
      twist_stamped_NED.twist.linear.x = msg->velocity[0];
      twist_stamped_NED.twist.linear.y = msg->velocity[1];
      twist_stamped_NED.twist.linear.z = msg->velocity[2];
      twist_stamped_NED.twist.angular.x = msg->angular_velocity[0];
      twist_stamped_NED.twist.angular.y = msg->angular_velocity[1];
      twist_stamped_NED.twist.angular.z = msg->angular_velocity[2];
      twist_NED_pub_->publish(twist_stamped_NED);

      // Apply transform from NED to vehicle
      tf2::Quaternion q(msg->q[1], msg->q[2], msg->q[3], msg->q[0]);
      tf2::Quaternion q_inv = q.inverse();
      tf2::Matrix3x3 m(q_inv);
      tf2::Vector3 velocity_NED(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
      tf2::Vector3 angular_velocity_NED(msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2]);
      tf2::Vector3 velocity_vehicle = m * velocity_NED;
      tf2::Vector3 angular_velocity_vehicle = m * angular_velocity_NED;

      twist_stamped_vehicle.header.frame_id = "vehicle_FRD";
      twist_stamped_vehicle.header.stamp = this->get_clock()->now();
      twist_stamped_vehicle.twist.linear.x = velocity_vehicle.x();
      twist_stamped_vehicle.twist.linear.y = velocity_vehicle.y();
      twist_stamped_vehicle.twist.linear.z = velocity_vehicle.z();
      twist_stamped_vehicle.twist.angular.x = angular_velocity_vehicle[0];
      twist_stamped_vehicle.twist.angular.y = angular_velocity_vehicle[1];
      twist_stamped_vehicle.twist.angular.z = angular_velocity_vehicle[2];
      twist_vehicle_pub_->publish(twist_stamped_vehicle);
    }
    else if (msg->velocity_frame == VELOCITY_FRAME_BODY_FRD)
    {
      RCLCPP_WARN(this->get_logger(), "VELOCITY_FRAME_BODY_FRD not tested");
      twist_stamped_vehicle.header.frame_id = "vehicle_FRD";
      twist_stamped_vehicle.twist.linear.x = msg->velocity[0];
      twist_stamped_vehicle.twist.linear.y = msg->velocity[1];
      twist_stamped_vehicle.twist.linear.z = msg->velocity[2];
      twist_stamped_vehicle.twist.angular.x = msg->angular_velocity[0];
      twist_stamped_vehicle.twist.angular.y = msg->angular_velocity[1];
      twist_stamped_vehicle.twist.angular.z = msg->angular_velocity[2];
      twist_NED_pub_->publish(twist_stamped_vehicle);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Unknown velocity frame");
    }
  }

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_NED_pub_, twist_vehicle_pub_;

  uint8_t VELOCITY_FRAME_UNKNOWN  = 0u;
  uint8_t VELOCITY_FRAME_NED      = 1u; // NED earth-fixed frame
  uint8_t VELOCITY_FRAME_FRD      = 2u; // FRD world-fixed frame, arbitrary heading reference
  uint8_t VELOCITY_FRAME_BODY_FRD = 3u; // FRD body-fixed frame
  uint8_t POSE_FRAME_UNKNOWN      = 0u;
  uint8_t POSE_FRAME_NED          = 1u;
  uint8_t POSE_FRAME_FRD          = 2u;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomBroadcaster>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
