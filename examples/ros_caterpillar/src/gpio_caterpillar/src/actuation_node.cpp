#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
// #include <wiringPi.h>

using namespace std::chrono_literals;

class ActuationNode : public rclcpp::Node {
public:
    ActuationNode(const std::string &node_name, int gpio_pin)
        : Node(node_name), gpio_pin_(gpio_pin) {
        subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            node_name + "_actuation", 10, std::bind(&ActuationNode::topic_callback, this, std::placeholders::_1));
        //wiringPiSetupGpio();
        // pinMode(gpio_pin_, OUTPUT);
    }

private:
    void topic_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        // digitalWrite(gpio_pin_, msg->data);
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        RCLCPP_INFO(this->get_logger(), "Would write to pin %d the value %s. TS: %ld",
                    gpio_pin_, msg->data ? "true" : "false", duration);
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
    int gpio_pin_;
};

int main(int argc, char *argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: actuation_node <node_name> <gpio_pin>" << std::endl;
        return 1;
    }

    std::string node_name = argv[1];
    int gpio_pin = std::stoi(argv[2]);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActuationNode>(node_name, gpio_pin));
    rclcpp::shutdown();
    return 0;
}

