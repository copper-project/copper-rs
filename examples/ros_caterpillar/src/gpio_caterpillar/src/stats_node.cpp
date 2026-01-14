#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>
#include "gpio_caterpillar/msg/stat_sample.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace {
struct Stat {
    uint64_t count = 0;
    uint64_t min_ns = 0;
    uint64_t max_ns = 0;
    double mean_ns = 0.0;
    double m2 = 0.0;
    bool has_last = false;
    uint64_t last_ns = 0;
    long double jitter_sum = 0.0L;
    uint64_t jitter_max = 0;
    uint64_t jitter_count = 0;

    void record(uint64_t sample_ns) {
        count++;
        if (count == 1) {
            min_ns = sample_ns;
            max_ns = sample_ns;
            mean_ns = static_cast<double>(sample_ns);
            m2 = 0.0;
        } else {
            min_ns = std::min(min_ns, sample_ns);
            max_ns = std::max(max_ns, sample_ns);
            double delta = static_cast<double>(sample_ns) - mean_ns;
            mean_ns += delta / static_cast<double>(count);
            double delta2 = static_cast<double>(sample_ns) - mean_ns;
            m2 += delta * delta2;
        }

        if (has_last) {
            uint64_t jitter = sample_ns > last_ns ? sample_ns - last_ns : last_ns - sample_ns;
            jitter_sum += static_cast<long double>(jitter);
            jitter_max = std::max(jitter_max, jitter);
            jitter_count++;
        }
        last_ns = sample_ns;
        has_last = true;
    }

    uint64_t mean() const {
        return count == 0 ? 0 : static_cast<uint64_t>(std::llround(mean_ns));
    }

    uint64_t stddev() const {
        if (count == 0) {
            return 0;
        }
        double variance = m2 / static_cast<double>(count);
        return static_cast<uint64_t>(std::llround(std::sqrt(variance)));
    }

    uint64_t jitter_avg() const {
        if (jitter_count == 0) {
            return 0;
        }
        return static_cast<uint64_t>(std::llround(jitter_sum / jitter_count));
    }
};

std::string format_ns(uint64_t ns) {
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    if (ns >= 1'000'000) {
        oss << std::setprecision(3) << (static_cast<double>(ns) / 1'000'000.0) << " ms";
    } else if (ns >= 1'000) {
        oss << std::setprecision(3) << (static_cast<double>(ns) / 1'000.0) << " us";
    } else {
        oss << ns << " ns";
    }
    return oss.str();
}

std::string render_table(const std::vector<std::array<std::string, 7>> &rows) {
    std::array<size_t, 7> widths{};
    for (const auto &row : rows) {
        for (size_t i = 0; i < row.size(); ++i) {
            widths[i] = std::max(widths[i], row[i].size());
        }
    }

    std::ostringstream oss;
    for (size_t i = 0; i < rows.size(); ++i) {
        const auto &row = rows[i];
        oss << std::left << std::setw(static_cast<int>(widths[0])) << row[0] << "  "
            << std::right << std::setw(static_cast<int>(widths[1])) << row[1] << "  "
            << std::right << std::setw(static_cast<int>(widths[2])) << row[2] << "  "
            << std::right << std::setw(static_cast<int>(widths[3])) << row[3] << "  "
            << std::right << std::setw(static_cast<int>(widths[4])) << row[4] << "  "
            << std::right << std::setw(static_cast<int>(widths[5])) << row[5] << "  "
            << std::right << std::setw(static_cast<int>(widths[6])) << row[6];
        if (i + 1 < rows.size()) {
            oss << '\n';
        }
    }
    return oss.str();
}

const std::vector<std::string> kTaskOrder = {
    "src",    "ct-0",  "gpio-0", "ct-1",  "gpio-1", "ct-2",
    "gpio-2", "ct-3",  "gpio-3", "ct-4",  "gpio-4", "ct-5",
    "gpio-5", "ct-6",  "gpio-6", "ct-7",  "gpio-7", "End2End",
};
}  // namespace

class StatsNode : public rclcpp::Node {
public:
    StatsNode() : Node("caterpillar_stats") {
        subscription_ = this->create_subscription<gpio_caterpillar::msg::StatSample>(
            "caterpillar_stats", 1000, std::bind(&StatsNode::on_sample, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(1s, std::bind(&StatsNode::print_table, this));
        RCLCPP_INFO(this->get_logger(), "Stats table enabled (1s interval)");
    }

private:
    void on_sample(const gpio_caterpillar::msg::StatSample::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        stats_[msg->task].record(msg->sample_ns);
    }

    void print_table() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stats_.empty()) {
            return;
        }

        std::vector<std::array<std::string, 7>> rows;
        rows.push_back({"Task", "Min", "Max", "Mean", "Stddev", "Jitter", "Max Jitter"});

        for (const auto &task : kTaskOrder) {
            auto it = stats_.find(task);
            if (it == stats_.end() || it->second.count == 0) {
                rows.push_back({task, "-", "-", "-", "-", "-", "-"});
                continue;
            }
            const auto &stat = it->second;
            rows.push_back({
                task,
                format_ns(stat.min_ns),
                format_ns(stat.max_ns),
                format_ns(stat.mean()),
                format_ns(stat.stddev()),
                format_ns(stat.jitter_avg()),
                format_ns(stat.jitter_max),
            });
        }

        std::cout << '\n' << render_table(rows) << std::endl;
    }

    std::mutex mutex_;
    std::map<std::string, Stat> stats_;
    rclcpp::Subscription<gpio_caterpillar::msg::StatSample>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatsNode>());
    rclcpp::shutdown();
    return 0;
}
