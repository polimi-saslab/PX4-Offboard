#include "rclcpp/rclcpp.hpp"
#include "chrono"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"

using std::placeholders::_1;

using namespace std::chrono;
using namespace std::chrono_literals;

class OdomBroadcaster : public rclcpp::Node
{
public:
  OdomBroadcaster() : Node("odom_broadcaster")
  {
    // Set use_sim_time parameter
    this->set_parameter(rclcpp::Parameter("use_sim_time", false));
    
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    // Create the subscriber
    subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "fmu/out/vehicle_odometry", qos, std::bind(&OdomBroadcaster::OdomCallback, this, std::placeholders::_1));

    // Create the dynamic transform broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Create the static transform broadcaster
    // static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Broadcast the static transform
    // broadcastStaticTransform();
  }

private:
  void OdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    // Create a TransformStamped message
    geometry_msgs::msg::TransformStamped transform_stamped;    

    // Fill in the header
    transform_stamped.header.stamp = this->get_clock()->now();
    transform_stamped.header.frame_id = "odom";

    // Set the child frame
    transform_stamped.child_frame_id = "vehicle";
    
    // Convert position from NED to ENU
    transform_stamped.transform.translation.x = msg->position[0];
    transform_stamped.transform.translation.y = -msg->position[1];
    transform_stamped.transform.translation.z = -msg->position[2];

    transform_stamped.transform.rotation.x = msg->q[1];
    transform_stamped.transform.rotation.y = -msg->q[2];
    transform_stamped.transform.rotation.z = -msg->q[3];
    transform_stamped.transform.rotation.w = msg->q[0];
   
    // Send the transform
    tf_broadcaster_->sendTransform(transform_stamped);
  }

  // void broadcastStaticTransform()
  // {
  //   geometry_msgs::msg::TransformStamped static_transform_stamped;

  //   // Fill in the header
  //   static_transform_stamped.header.stamp = this->get_clock()->now();
  //   static_transform_stamped.header.frame_id = "vehicle";

  //   // Set the child frame
  //   static_transform_stamped.child_frame_id = "x500_lidar_0/link/lidar_2d_v2";

  //   // Set the translation
  //   static_transform_stamped.transform.translation.x = 0.0;
  //   static_transform_stamped.transform.translation.y = 0.0;
  //   static_transform_stamped.transform.translation.z = 0.1;  // 10 cm offset in Z direction

  //   // Set the rotation (no rotation, so identity quaternion)
  //   static_transform_stamped.transform.rotation.x = 0.0;
  //   static_transform_stamped.transform.rotation.y = 0.0;
  //   static_transform_stamped.transform.rotation.z = 0.0;
  //   static_transform_stamped.transform.rotation.w = 1.0;

  //   // Send the static transform
  //   static_tf_broadcaster_->sendTransform(static_transform_stamped);
  // }

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomBroadcaster>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
