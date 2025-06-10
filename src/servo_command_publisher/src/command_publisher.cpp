#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <string>

class CommandPublisher : public rclcpp::Node
{
public:
    CommandPublisher()
    : Node("command_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("servo_command", 10);
        RCLCPP_INFO(this->get_logger(), "Publisher Started");

        input_thread_ = std::thread(std::bind(&CommandPublisher::keyboard_loop, this));
    }

    ~CommandPublisher() {
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    void keyboard_loop()
    {
        std::string input;
        while (rclcpp::ok()) {
            std::cout << "Enter command (S1, S2, O, C, STOP, END): ";
            std::getline(std::cin, input);

            if (input.empty()) continue;

            auto msg = std_msgs::msg::String();
            msg.data = input;
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published: '%s'", msg.data.c_str());

            if (input == "END") {
                rclcpp::shutdown();
            }
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::thread input_thread_;
};
  
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CommandPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

