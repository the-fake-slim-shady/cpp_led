#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include <chrono>
#include <iostream>
#include <termios.h>

using namespace std::chrono_literals;

class KeyBoadPub : public rclcpp::Node
{
    public:
        KeyBoadPub()
        : Node("keyboad_publisher")
        {
            publisher_ = this-> create_publisher<std_msgs::msg::Int16>("your_token", 10);
            timer_ = this->create_wall_timer(50ms, [this]() {this -> call_back(); });
        }

    private:
        void call_back(){
            int number;
            auto message = std_msgs::msg::Int16();

            RCLCPP_INFO(this -> get_logger(), "Put a number");
            number = not_need_enter();
            if(number >= 0 && number <= 3){
                message.data = number;
                publisher_ -> publish(message);
            }/*else if(number == -1){
                message.data = -1;
                publisher_ -> publish(message);
            }*/else{
                RCLCPP_INFO(this -> get_logger(), "Not allowed number");
            }
        }

        rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        int not_need_enter(){
            struct termios oldt, newt;
            tcgetattr(STDIN_FILENO, &oldt);         // 現在の設定を保存
            newt = oldt;
            newt.c_lflag &= ~(ICANON | ECHO);       // カノニカルモードとエコーをオフ
            tcsetattr(STDIN_FILENO, TCSANOW, &newt);// 設定を適用

            char ch = getchar();                          // 1文字取得

            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);// 設定を戻す
            if (ch >= '0' && ch <= '9') {
                return ch - '0';  // char → int 変換
            } else {
                return -1; // 数字以外が押された
            }
        }
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<KeyBoadPub>());
    } catch (const std::exception &e) {
        std::cerr << "Exception caught: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}