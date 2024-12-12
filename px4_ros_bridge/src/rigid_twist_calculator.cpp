#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>

using std::placeholders::_1;

using namespace std::chrono_literals;

class TwistCalculator : public rclcpp::Node
{
public:
    TwistCalculator() : Node("twist_calculator")
    {
        // Declare parameters
        this->declare_parameter<std::string>("origin_frame", "base_link");
        this->declare_parameter<std::string>("target_frame", "target_link");
        this->declare_parameter<std::string>("origin_twist_topic", "base_link_twist");
        this->declare_parameter<std::string>("target_twist_topic", "target_link_twist");

        // Get parameters
        origin_frame_ = this->get_parameter("origin_frame").as_string();
        target_frame_ = this->get_parameter("target_frame").as_string();
        origin_twist_topic_ = this->get_parameter("origin_twist_topic").as_string();
        target_twist_topic_ = this->get_parameter("target_twist_topic").as_string();

        // Create the subscriber
        twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            origin_twist_topic_, 10, std::bind(&TwistCalculator::compute_speed, this, std::placeholders::_1));
        // Create the publisher
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(target_twist_topic_, 10);

        // Initialize the transform listener
        tf_buffer_= std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    geometry_msgs::msg::Twist transformTwist(
        const geometry_msgs::msg::Twist& twist, 
        const std::string& source_frame, 
        const std::string& target_frame,
        tf2_ros::Buffer& tf_buffer) 
    {
        geometry_msgs::msg::Twist transformed_twist;
        try {
            // Lookup the transform
            geometry_msgs::msg::TransformStamped transform =
                tf_buffer.lookupTransform(target_frame, source_frame, tf2::TimePointZero);

            // Convert transform rotation to a tf2 quaternion
            tf2::Quaternion rotation;
            tf2::convert(transform.transform.rotation, rotation);

            // Convert linear velocity to a tf2 vector
            tf2::Vector3 linear_velocity(twist.linear.x, twist.linear.y, twist.linear.z);
            tf2::Vector3 transformed_linear_velocity = tf2::quatRotate(rotation, linear_velocity);

            // Convert angular velocity to a tf2 vector
            tf2::Vector3 angular_velocity(twist.angular.x, twist.angular.y, twist.angular.z);
            tf2::Vector3 transformed_angular_velocity = tf2::quatRotate(rotation, angular_velocity);

            // Fill the transformed Twist message
            transformed_twist.linear.x = transformed_linear_velocity.x();
            transformed_twist.linear.y = transformed_linear_velocity.y();
            transformed_twist.linear.z = transformed_linear_velocity.z();

            transformed_twist.angular.x = transformed_angular_velocity.x();
            transformed_twist.angular.y = transformed_angular_velocity.y();
            transformed_twist.angular.z = transformed_angular_velocity.z();
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(rclcpp::get_logger("transform_twist"), "Transform failed: %s", ex.what());
        }
        return transformed_twist;
    }

    void compute_speed(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped transform;
        geometry_msgs::msg::Twist target_twist;
        geometry_msgs::msg::TwistStamped target_twist_stamped;

        // Verify that the frame_id of the twist coincides with the origin frame
        if (msg->header.frame_id != origin_frame_)
        {
            RCLCPP_WARN(
                this->get_logger(), "Received twist message with frame_id %s, expected %s",
                msg->header.frame_id.c_str(), origin_frame_.c_str());
            return;
        }

        // Get the transform
        try
        {
            transform = tf_buffer_->lookupTransform(target_frame_, origin_frame_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException & ex)
        {
            RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                    target_frame_.c_str(), origin_frame_.c_str(), ex.what());
            return;
        }

        // Compute the target twist, using rigid body kinematics
        target_twist.linear.x = msg->twist.linear.x + msg->twist.angular.y * transform.transform.translation.z - msg->twist.angular.z * transform.transform.translation.y;
        target_twist.linear.y = msg->twist.linear.y + msg->twist.angular.z * transform.transform.translation.x - msg->twist.angular.x * transform.transform.translation.z;
        target_twist.linear.z = msg->twist.linear.z + msg->twist.angular.x * transform.transform.translation.y - msg->twist.angular.y * transform.transform.translation.x;
        target_twist.angular.x = msg->twist.angular.x;
        target_twist.angular.y = msg->twist.angular.y;
        target_twist.angular.z = msg->twist.angular.z;

        // Transform the twist to the target frame
        target_twist = transformTwist(target_twist, origin_frame_, target_frame_, *tf_buffer_);

        target_twist_stamped.header.stamp = this->get_clock()->now();
        target_twist_stamped.header.frame_id = target_frame_;
        target_twist_stamped.twist = target_twist;
        twist_pub_->publish(target_twist_stamped);
    }

    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string origin_frame_;
    std::string target_frame_;
    std::string origin_twist_topic_;
    std::string target_twist_topic_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistCalculator>());
    rclcpp::shutdown();
    return 0;
}