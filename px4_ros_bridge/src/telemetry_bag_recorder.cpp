#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/rc_channels.hpp>
#include <boost/process.hpp>
#include <memory>
#include <string>
#include <chrono>

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
        rc_channels_sub_ = this->create_subscription<px4_msgs::msg::RcChannels>(
            "/fmu/out/rc_channels", 10, std::bind(&TelemetryBagRecorder::rcChannelsCallback, this, std::placeholders::_1));
        bag_recording_ = false;
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
        // if (debounce_timer_ == nullptr || debounce_timer_->is_canceled())
        // {
            if (msg->channels[8] > 0.5 && !bag_recording_)
            {
                RCLCPP_INFO(this->get_logger(), "Starting bag recorder... ");
                start_bag_recording();
            }
            else if (msg->channels[8] < 0.5 && bag_recording_)
            {
                RCLCPP_INFO(this->get_logger(), "Stopping bag recorder... ");
                stop_bag_recording();
            }

        //     debounce_timer_ = this->create_wall_timer(debounce_duration_, [this]() {
        //         debounce_timer_->cancel();
        //     });
        // }
    }
    void start_bag_recording()
    {
        try
        {
            // Start the ros2 bag record process
            bag_process_ = std::make_unique<bp::child>(
                bp::search_path("ros2"),
                "bag", "record", "-a",
                bp::std_out > bp::null, // Redirect stdout to null or handle as needed
                bp::std_err > bp::null  // Redirect stderr to null or handle as needed
            );
            bag_recording_ = true;
            RCLCPP_INFO(this->get_logger(), "ros2 bag record started.");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to start ros2 bag record: %s", e.what());
        }
    }

    void stop_bag_recording()
    {
        if (bag_process_ && bag_process_->running())
        {
            bag_process_->terminate();
            bag_process_->wait();
            bag_recording_ = false;
            RCLCPP_INFO(this->get_logger(), "ros2 bag record stopped.");
        }
    }

    rclcpp::Subscription<px4_msgs::msg::RcChannels>::SharedPtr rc_channels_sub_;
    rclcpp::TimerBase::SharedPtr debounce_timer_;
    std::chrono::milliseconds debounce_duration_;
    
    std::unique_ptr<bp::child> bag_process_;
    bool bag_recording_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TelemetryBagRecorder>());
    rclcpp::shutdown();
    return 0;
}