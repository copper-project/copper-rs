#include <chrono>
#include <memory>
#include "gpio_caterpillar/msg/caterpillar_msg.hpp"
#include "gpio_caterpillar/msg/stat_sample.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class FlipNode : public rclcpp::Node {
public:
    FlipNode() : Node("flip_node"), flip_count_(0), seq_(0) {
        publisher_ = this->create_publisher<gpio_caterpillar::msg::CaterpillarMsg>("flip_topic_1", 10);
        stats_pub_ = this->create_publisher<gpio_caterpillar::msg::StatSample>("caterpillar_stats", 1000);
        timer_ = this->create_wall_timer(100ms, std::bind(&FlipNode::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Publisher ready: %s", publisher_->get_topic_name());
    }

private:
    static uint64_t now_ns() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    void publish_stat(uint64_t sample_ns) {
        gpio_caterpillar::msg::StatSample sample;
        sample.task = "src";
        sample.sample_ns = sample_ns;
        stats_pub_->publish(sample);
    }

    void timer_callback() {
        auto tick_start = std::chrono::steady_clock::now();
        auto message = gpio_caterpillar::msg::CaterpillarMsg();
        message.state = flip_count_ % 2 == 0;
        message.origin_ns = now_ns();
        message.seq = seq_++;
        publisher_->publish(message);
        auto tick_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                           std::chrono::steady_clock::now() - tick_start)
                           .count();
        publish_stat(static_cast<uint64_t>(tick_ns));
        flip_count_++;
    }

    rclcpp::Publisher<gpio_caterpillar::msg::CaterpillarMsg>::SharedPtr publisher_;
    rclcpp::Publisher<gpio_caterpillar::msg::StatSample>::SharedPtr stats_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    uint64_t flip_count_;
    uint64_t seq_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlipNode>());
    rclcpp::shutdown();
    return 0;
}
