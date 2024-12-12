#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/rc_channels.hpp>
#include <boost/process.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <memory>
#include <string>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace bp = boost::process;

class TelemetryBagRecorder : public rclcpp::Node
{
public:
    TelemetryBagRecorder()
    : Node("telemetry_bag_recorder"),
        debounce_timer_(nullptr),
        debounce_duration_(std::chrono::milliseconds(100)),
        bag_recording_(false)
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("platform_name", "unknown_platform");
        this->declare_parameter<std::string>("test_name", "test");

        // Define the QoS for RC channels subscription
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        qos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
        // Subscribe to the RC channels topic
        rc_channels_sub_ = this->create_subscription<px4_msgs::msg::RcChannels>(
                "/fmu/out/rc_channels", qos, std::bind(&TelemetryBagRecorder::rcChannelsCallback, this, std::placeholders::_1));
    }

    ~TelemetryBagRecorder()
    {
        if(bag_recording_)
        {
            stop_bag_recording();
        }
    }

private:
    void rcChannelsCallback(const px4_msgs::msg::RcChannels::SharedPtr msg)
    {
        if (debounce_timer_ == nullptr || debounce_timer_->is_canceled())
        {
            if (msg->channels[8] > 0.5 && !bag_recording_)
            {
                // get updated parameters
                platform_name_ = this->get_parameter("platform_name").as_string();
                test_name_ = this->get_parameter("test_name").as_string();
                RCLCPP_INFO(this->get_logger(), "Starting bag recorder... ");
                start_bag_recording();
            }
            else if (msg->channels[8] < 0.5 && bag_recording_)
            {
                RCLCPP_INFO(this->get_logger(), "Stopping bag recorder... ");
                stop_bag_recording();
            }

            debounce_timer_ = this->create_wall_timer(debounce_duration_, [this]() {
                debounce_timer_->cancel();
            });
        }
    }

    void start_bag_recording()
    {
        if (bag_recording_) {
            RCLCPP_WARN(this->get_logger(), "Bag recording is already active.");
            return;
        }

        // Generate a timestamp for the bag file name
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm now_tm = *std::localtime(&now_time);
        std::ostringstream oss;
        oss << std::put_time(&now_tm, "%Y.%m.%d_%H.%M.%S");
        std::string timestamp = oss.str();

        // Get parameters
        std::string platform_name;
        std::string test_name;
        this->get_parameter("platform_name", platform_name);
        this->get_parameter("test_name", test_name);

        // Construct the bag directory and file name
        std::string package_prefix = ament_index_cpp::get_package_prefix("px4_ros_bridge").c_str();
        std::string bag_dir = package_prefix+"/../../"+"rosbags/";
        std::string bag_file_name = timestamp + "_" + platform_name + "_" + test_name;

        try {
            // Start the ros2 bag record process
            bag_process_ = std::make_unique<bp::child>(
                bp::search_path("ros2"),
                "bag", "record", "-a", "-o", bag_dir+bag_file_name
                // bp::start_dir("")
            );
            bag_recording_ = true;
            RCLCPP_INFO(this->get_logger(), "ros2 bag record started with file name: %s", bag_file_name.c_str());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start ros2 bag record: %s", e.what());
        }
    }

    void stop_bag_recording()
    {
        if (bag_process_ && bag_process_->running()) {
            bag_process_->terminate();
            bag_process_->wait();
            bag_recording_ = false;
            RCLCPP_INFO(this->get_logger(), "ros2 bag record stopped.");
        }
        else {
            RCLCPP_WARN(this->get_logger(), "ros2 bag record is not running.");
            bag_recording_ = false;
        }
    }

    rclcpp::Subscription<px4_msgs::msg::RcChannels>::SharedPtr rc_channels_sub_;
    rclcpp::TimerBase::SharedPtr debounce_timer_;
    std::chrono::milliseconds debounce_duration_;
    std::unique_ptr<bp::child> bag_process_;
    bool bag_recording_;
    std::string platform_name_;
    std::string test_name_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TelemetryBagRecorder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}