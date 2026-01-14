#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include "gpio_caterpillar/msg/caterpillar_msg.hpp"
#include "gpio_caterpillar/msg/stat_sample.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class PropagateNode : public rclcpp::Node {
public:
    PropagateNode(const std::string &node_name, const std::string &input_topic, const std::string &output_topic)
        : Node(node_name),
          task_name_(task_from_input(input_topic)) {
        subscription_ = this->create_subscription<gpio_caterpillar::msg::CaterpillarMsg>(
            input_topic, 10, std::bind(&PropagateNode::topic_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<gpio_caterpillar::msg::CaterpillarMsg>(output_topic, 10);
        actuation_publisher_ =
            this->create_publisher<gpio_caterpillar::msg::CaterpillarMsg>(node_name + "_actuation", 10);
        stats_pub_ = this->create_publisher<gpio_caterpillar::msg::StatSample>("caterpillar_stats", 1000);
        RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", input_topic.c_str());
    }

private:
    static std::string task_from_input(const std::string &input_topic) {
        const std::string prefix = "flip_topic_";
        if (input_topic.rfind(prefix, 0) == 0) {
            const std::string suffix = input_topic.substr(prefix.size());
            char *end = nullptr;
            long index = std::strtol(suffix.c_str(), &end, 10);
            if (end != suffix.c_str() && *end == '\0' && index > 0) {
                return "ct-" + std::to_string(index - 1);
            }
        }
        return input_topic;
    }

    void publish_stat(uint64_t sample_ns) {
        gpio_caterpillar::msg::StatSample sample;
        sample.task = task_name_;
        sample.sample_ns = sample_ns;
        stats_pub_->publish(sample);
    }

    void topic_callback(const gpio_caterpillar::msg::CaterpillarMsg::SharedPtr msg) {
        auto tick_start = std::chrono::steady_clock::now();
        actuation_publisher_->publish(*msg);  // Actuate GPIO
        publisher_->publish(*msg);           // Propagate the same value to the next node
        auto tick_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                           std::chrono::steady_clock::now() - tick_start)
                           .count();
        publish_stat(static_cast<uint64_t>(tick_ns));
    }

    std::string task_name_;
    rclcpp::Subscription<gpio_caterpillar::msg::CaterpillarMsg>::SharedPtr subscription_;
    rclcpp::Publisher<gpio_caterpillar::msg::CaterpillarMsg>::SharedPtr publisher_;
    rclcpp::Publisher<gpio_caterpillar::msg::CaterpillarMsg>::SharedPtr actuation_publisher_;
    rclcpp::Publisher<gpio_caterpillar::msg::StatSample>::SharedPtr stats_pub_;
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
