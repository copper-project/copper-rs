#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class PropagateNode : public rclcpp::Node {
public:
    PropagateNode(const std::string &node_name, const std::string &input_topic, const std::string &output_topic)
        : Node(node_name) {
        subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            input_topic, 10, std::bind(&PropagateNode::topic_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<std_msgs::msg::Bool>(output_topic, 10);
        actuation_publisher_ = this->create_publisher<std_msgs::msg::Bool>(node_name + "_actuation", 10);
	RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", input_topic.c_str());
    }

private:
    void topic_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Propagating boolean %s", msg->data ? "true" : "false");
        actuation_publisher_->publish(*msg);  // Actuate GPIO
        publisher_->publish(*msg);  // Propagate the same value to the next node
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr actuation_publisher_;
};

int main(int argc, char *argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: propagate_node <node_name> <input_topic> <output_topic>" << std::endl;
        return 1;
    }

    std::string node_name = argv[1];
    std::string input_topic = argv[2];
    std::string output_topic = argv[3];

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PropagateNode>(node_name, input_topic, output_topic));
    rclcpp::shutdown();
    return 0;
}

