#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include "gpio_caterpillar/msg/caterpillar_msg.hpp"
#include "gpio_caterpillar/msg/stat_sample.hpp"
#include "rclcpp/rclcpp.hpp"
// #include <wiringPi.h>

using namespace std::chrono_literals;

class ActuationNode : public rclcpp::Node {
public:
    ActuationNode(const std::string &node_name, int gpio_pin)
        : Node(node_name),
          gpio_pin_(gpio_pin),
          task_name_(task_from_node(node_name)),
          record_end_to_end_(is_last_gpio(node_name)) {
        subscription_ = this->create_subscription<gpio_caterpillar::msg::CaterpillarMsg>(
            node_name + "_actuation",
            10,
            std::bind(&ActuationNode::topic_callback, this, std::placeholders::_1));
        stats_pub_ = this->create_publisher<gpio_caterpillar::msg::StatSample>("caterpillar_stats", 1000);
        RCLCPP_INFO(this->get_logger(), "Actuation ready on pin %d (%s)",
                    gpio_pin_, task_name_.c_str());
        //wiringPiSetupGpio();
        // pinMode(gpio_pin_, OUTPUT);
    }

private:
    static uint64_t now_ns() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    static std::string task_from_node(const std::string &node_name) {
        const std::string suffix = "gpio_node_";
        auto pos = node_name.rfind(suffix);
        if (pos != std::string::npos) {
            const std::string num = node_name.substr(pos + suffix.size());
            char *end = nullptr;
            long index = std::strtol(num.c_str(), &end, 10);
            if (end != num.c_str() && *end == '\0' && index > 0) {
                return "gpio-" + std::to_string(index - 1);
            }
        }
        return node_name;
    }

    static bool is_last_gpio(const std::string &node_name) {
        const std::string suffix = "gpio_node_";
        auto pos = node_name.rfind(suffix);
        if (pos == std::string::npos) {
            return false;
        }
        const std::string num = node_name.substr(pos + suffix.size());
        char *end = nullptr;
        long index = std::strtol(num.c_str(), &end, 10);
        return end != num.c_str() && *end == '\0' && index == 8;
    }

    void publish_stat(const std::string &task, uint64_t sample_ns) {
        gpio_caterpillar::msg::StatSample sample;
        sample.task = task;
        sample.sample_ns = sample_ns;
        stats_pub_->publish(sample);
    }

    void topic_callback(const gpio_caterpillar::msg::CaterpillarMsg::SharedPtr msg) {
        auto tick_start = std::chrono::steady_clock::now();
        // digitalWrite(gpio_pin_, msg->data);
        auto tick_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                           std::chrono::steady_clock::now() - tick_start)
                           .count();
        publish_stat(task_name_, static_cast<uint64_t>(tick_ns));

        if (record_end_to_end_) {
            auto end_to_end_ns = now_ns() - msg->origin_ns;
            publish_stat("End2End", end_to_end_ns);
        }
    }

    rclcpp::Subscription<gpio_caterpillar::msg::CaterpillarMsg>::SharedPtr subscription_;
    rclcpp::Publisher<gpio_caterpillar::msg::StatSample>::SharedPtr stats_pub_;
    int gpio_pin_;
    std::string task_name_;
    bool record_end_to_end_;
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
