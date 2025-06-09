#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include <gpiod.h>
#include <chrono>
#include <random>

using namespace std::chrono_literals;

#define SWITCH 4
#define LED0   23
#define LED1   24
#define LED2   22

class LedInfoPub : public rclcpp::Node
{
public:
    LedInfoPub()
    : Node("led_info_publisher"),
      prev_sw_state_(0),
      current_led_count_(0),
      command_(-1),
      gen_(std::random_device{}()), dist_(1, 3)   // ★ 1〜3 の一様乱数
    {
        chip_   = gpiod_chip_open_by_name("gpiochip0");
        if (!chip_)  throw std::runtime_error("Failed to open gpiochip0");

        lines_[0] = gpiod_chip_get_line(chip_, LED0);
        lines_[1] = gpiod_chip_get_line(chip_, LED1);
        lines_[2] = gpiod_chip_get_line(chip_, LED2);
        swline_   = gpiod_chip_get_line(chip_, SWITCH);
        if (!lines_[0] || !lines_[1] || !lines_[2] || !swline_)
            throw std::runtime_error("Failed to get GPIO lines");

        if (gpiod_line_request_output(lines_[0], "LED0", 0) < 0 ||
            gpiod_line_request_output(lines_[1], "LED1", 0) < 0 ||
            gpiod_line_request_output(lines_[2], "LED2", 0) < 0 ||
            gpiod_line_request_input (swline_ , "switch") < 0)
            throw std::runtime_error("Failed to request GPIO line access");

        /** サブスクライバ : キーボード入力 0-3 を受信 */
        subscription_ = this->create_subscription<std_msgs::msg::Int16>(
            "your_token", 10,
            [this](const std_msgs::msg::Int16::SharedPtr msg)
            {
                command_ = msg->data;  // 0〜3
            });

        publisher_ = this->create_publisher<std_msgs::msg::Int16>("on_off", 10);
        timer_ = this->create_wall_timer(
            50ms, std::bind(&LedInfoPub::timer_callback, this));
    }

private:
    void light_first_n(int n)   
    {
        for (int i = 0; i < 3; ++i)
            gpiod_line_set_value(lines_[i], (i < n) ? 1 : 0);
    }

    void timer_callback()
    {
        int sw_state = gpiod_line_get_value(swline_);
        bool rising_edge = (prev_sw_state_ == 0 && sw_state == 1);
        prev_sw_state_ = sw_state;   

        std_msgs::msg::Int16 msg;

        /* ------ スイッチ操作を優先的に処理する ------ */
        if (rising_edge)
        {
            command_ = -1; 
            if (current_led_count_ == 0) {          
                current_led_count_ = dist_(gen_);   
                light_first_n(current_led_count_);
                RCLCPP_INFO(get_logger(), "SWITCH: random %d LEDs ON",
                             current_led_count_);
            } else {                                
                current_led_count_ = 0;
                light_first_n(0);
                RCLCPP_INFO(get_logger(), "SWITCH: all OFF");
            }
            msg.data = static_cast<int16_t>(current_led_count_);
            publisher_->publish(msg);
            return;   
        }

        /* ------ キーボードからの指示があれば処理する ------ */
        if (command_ >= 0)   
        {
            current_led_count_ = command_;
            light_first_n(current_led_count_);
            RCLCPP_INFO(get_logger(), "KEYBOARD: set %d LEDs", current_led_count_);

            msg.data = static_cast<int16_t>(current_led_count_);
            publisher_->publish(msg);
            command_ = -1;      
        }
    }

    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr    publisher_;
    rclcpp::TimerBase::SharedPtr                           timer_;

    struct gpiod_chip *chip_;
    struct gpiod_line *lines_[3];
    struct gpiod_line *swline_;


    int  prev_sw_state_;         
    int  current_led_count_;     
    int  command_;               

    std::mt19937                       gen_;
    std::uniform_int_distribution<int> dist_;
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LedInfoPub>());
    rclcpp::shutdown();
    return 0;
}