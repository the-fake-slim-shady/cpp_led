#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include <memory>


class LedInfoSub : public rclcpp::Node
{
    public:
        LedInfoSub()
        : Node("led_info_sub")
        {
            auto sub_callback = 
                [this](std_msgs::msg::Int16::UniquePtr msg) -> void {
                    RCLCPP_INFO(this->get_logger(), "ON: %d", msg->data);
                };
            subscription_ = 
                this->create_subscription<std_msgs::msg::Int16>("on_off", 10, sub_callback);
        }

    private:
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LedInfoSub>());
    rclcpp::shutdown();
    return 0;
}