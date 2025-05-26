#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class FlipNode : public rclcpp::Node {
public:
    FlipNode() : Node("flip_node"), flip_count_(0) {
        publisher_ = this->create_publisher<std_msgs::msg::Bool>("flip_topic_1", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&FlipNode::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Publisher ready: %s", publisher_->get_topic_name());
    }

private:
    void timer_callback() {
        auto message = std_msgs::msg::Bool();
        message.data = flip_count_ % 2 == 0;
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        RCLCPP_INFO(this->get_logger(), "Start chain. TS: %ld", duration);
        publisher_->publish(message);
        flip_count_++;
    }

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int flip_count_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlipNode>());
    rclcpp::shutdown();
    return 0;
}

