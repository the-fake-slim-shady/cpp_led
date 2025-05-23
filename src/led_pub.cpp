#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include <stdio.h>
#include <gpiod.h>
#include <unistd.h>
#include <chrono>
#include <random>

using namespace std::chrono_literals;


#define SWITCH 4
#define LED0 23
#define LED1 24
#define LED2 22

class LedInfoPub : public rclcpp::Node
{
    public:
        LedInfoPub()
        : Node("led_info_publisher"), i(0), status(0), rP(0), command(-1), count(0)
        {
            chip = gpiod_chip_open_by_name("gpiochip0");
            if (!chip) {
                throw std::runtime_error("Failed to open gpiochip4");
            }

            line = gpiod_chip_get_line(chip, LED0);
            line1 = gpiod_chip_get_line(chip, LED1);
            line2 = gpiod_chip_get_line(chip, LED2);
            swline = gpiod_chip_get_line(chip, SWITCH);
            if (!line || !line1 || !line2 || !swline) {
                throw std::runtime_error("Failed to get GPIO lines");
            }

            if (gpiod_line_request_output(line, "LED0", 0) < 0 ||
                gpiod_line_request_output(line1, "LED1", 0) < 0 ||
                gpiod_line_request_output(line2, "LED2", 0) < 0 ||
                gpiod_line_request_input(swline, "switch") < 0) {
                throw std::runtime_error("Failed to request GPIO line access");
            }

            auto sub_callback = 
                [this](std_msgs::msg::Int16::UniquePtr msg) -> void {
                    RCLCPP_INFO(this->get_logger(), "Receive command: %d", msg->data);
                    command = msg -> data;
                };
            subscription_ = 
                this->create_subscription<std_msgs::msg::Int16>("your_token", 10, sub_callback);
            
                
            publisher_ = this->create_publisher<std_msgs::msg::Int16>("on_off", 10);
            timer_ = this->create_wall_timer(50ms, 
            [this](){this -> timer_callback();}
            );

            lines[0] = line;
            lines[1] = line1;
            lines[2] = line2;
        }
    
    private:
        
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscription_;
        

        void timer_callback(){
            int judge = gpiod_line_get_value(swline);
            auto message = std_msgs::msg::Int16();


            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> dist(0, 2);

            if(status == 0 && judge == 1){
                i++;
                command = -1;
            }
                        
            if(command == -1){

                if(i == 1){
                    int r = dist(gen); 
                    int x;
                    for(x = 0; x<=r; x++){
                        for(int y = 0; y <= 2; y++){
                            gpiod_line_set_value(lines[y], 0);
                        }
                        gpiod_line_set_value(lines[x], 1);
                            
                    }
                    message.data = count;
                    RCLCPP_INFO(this->get_logger(), "mse1");
                    rP = r;
                    count =
                }else if(i == 2){
                    for(int x = 0; x<=rP; x++){
                        gpiod_line_set_value(lines[x], 0);
                    }
                    message.data = 0;
                    RCLCPP_INFO(this->get_logger(), "mse 0");
                    i = 0;
                }else if(i == 0){
                    RCLCPP_INFO(this->get_logger(), "Please push button to start!");
                }else{
                    RCLCPP_INFO(this->get_logger(), "something is an error.");
                }

                status = judge;
                publisher_ -> publish(message);
            }else{
                if(command == 0){
                    for(int y = 0; y <= 2; y++){
                        gpiod_line_set_value(lines[y], 0);
                    }
                    i = 0;
                    status = 0;                   
                }else{
                    for(int y = 0; y <= 2; y++){
                        gpiod_line_set_value(lines[y], 0);
                    }
                    for(int y = 0; y<=(command-1); y++){
                        gpiod_line_set_value(lines[y], 1);
                    }
                    i = 1;
                    status = 1;
                }
                message.data = command;
                publisher_ -> publish(message);
                RCLCPP_INFO(this->get_logger(), "Litght * %d", command);
            }
        }

        int i;
        int status;
        int rP;
        int command;
        int count;
        rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        struct gpiod_chip *chip;
        struct gpiod_line *line;
        struct gpiod_line *line1;
        struct gpiod_line *line2;
        struct gpiod_line *swline;
        struct gpiod_line* lines[3];
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<LedInfoPub>());
    } catch (const std::exception &e) {
        std::cerr << "Exception caught: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}
