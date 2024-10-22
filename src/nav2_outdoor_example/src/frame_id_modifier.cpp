#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class FrameIdModifier : public rclcpp::Node
{
public:
    FrameIdModifier()
        : Node("frame_id_modifier")
    {
        // Create a subscription to the /gazebo/scan topic
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/gazebo/scan",
            10,
            std::bind(&FrameIdModifier::scan_callback, this, std::placeholders::_1)
        );

        // Create a publisher for the modified laser scan
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan",
            10
        );
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Create a new LaserScan message
        auto modified_msg = sensor_msgs::msg::LaserScan(*msg);
        
        // Modify the frame_id to your desired value
        modified_msg.header.frame_id = "laser_frame";

        // Publish the modified message
        publisher_->publish(modified_msg);
        
        // RCLCPP_INFO(this->get_logger(), "Modified frame_id to: '%s'", modified_msg.header.frame_id.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrameIdModifier>());
    rclcpp::shutdown();
    return 0;
}