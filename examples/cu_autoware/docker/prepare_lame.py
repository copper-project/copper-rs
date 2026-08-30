#!/usr/bin/env python3
"""Make the pinned LaME source safe in a container and align its Fig. 4 workload."""

from pathlib import Path
import re

ROOT = Path("/opt/lame_ws/src/reference-system-latency-management")
MAIN = ROOT / "latency_mgmt/src/latency_mgmt.cpp"


def replace_once(text: str, old: str, new: str, subject: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{subject}: expected one match, found {count}")
    return text.replace(old, new, 1)


text = MAIN.read_text()
text = replace_once(
    text,
    """    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(5, &cpuset);
    CPU_SET(6, &cpuset);
    CPU_SET(7, &cpuset);
    if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger(\"rclcpp\"), \"Failed to set CPU affinity: %s\", strerror(errno));
        return -1;
    }
""",
    "",
    "process affinity",
)
text = replace_once(
    text,
    """    // Set RT runtime to unlimited
    // echo -1 > /proc/sys/kernel/sched_rt_runtime_us
    set_rt_runtime_unlimited();
    // Set SCHED_DEADLINE system period to 10ms
    // echo 10000 > /proc/sys/kernel/sched_rt_period_us
    set_rt_period();
    set_cpu_frequency(\"2265600\");
""",
    "    // Container runner deliberately leaves host scheduler sysctls and CPU frequency alone.\n",
    "host tuning",
)

calibration = r'''
uint64_t number_cruncher(const uint64_t maximum_number);

static double measure_workload(uint64_t limit)
{
    std::vector<double> samples;
    for (int i = 0; i < 7; ++i) {
        auto start = std::chrono::steady_clock::now();
        volatile uint64_t result = number_cruncher(limit);
        (void)result;
        auto elapsed = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - start).count();
        samples.push_back(elapsed);
    }
    std::sort(samples.begin(), samples.end());
    return samples[samples.size() / 2];
}

static int workload_limit(uint64_t target_us)
{
    static std::map<uint64_t, int> calibrated;
    auto found = calibrated.find(target_us);
    if (found != calibrated.end()) return found->second;
    if (target_us <= 1) return calibrated[target_us] = 1;

    uint64_t lo = 1;
    uint64_t hi = 20000;
    while (measure_workload(hi) < target_us) hi *= 2;
    while (lo + 1 < hi) {
        uint64_t mid = lo + (hi - lo) / 2;
        if (measure_workload(mid) < target_us) lo = mid;
        else hi = mid;
    }
    double measured = measure_workload(hi);
    std::cout << "CALIBRATION target_us=" << target_us << " limit=" << hi
              << " measured_us=" << measured << std::endl;
    return calibrated[target_us] = static_cast<int>(hi);
}
'''
text = replace_once(text, "int main(int argc, char *argv[])\n", calibration + "\nint main(int argc, char *argv[])\n", "calibration insertion")
text = replace_once(text, "#include <chrono>\n", "#include <chrono>\n#include <algorithm>\n#include <map>\n#include <vector>\n", "calibration includes")

start = text.index("void test_case_1()\n{")
end = text.index("void test_case_2()\n{", start)
case = text[start:end]
costs = {
    "front_lidar_driver": 100,
    "points_transformer_front": 10100,
    "point_cloud_fusion": 10005,
    "ray_ground_filter": 10100,
    "euclidean_cluster_detector": 10100,
    "object_collision_estimator": 10100,
    "behavior_planner_input_0": 1,
    "voxel_grid_downsampler": 10100,
    "ndt_localizer_input": 2100,
    "behavior_planner_timer": 100,
    "mpc_controller": 10100,
    "vehicle_interface": 10100,
    "vehicle_dbw_system": 1000,
    "vehicle_interface_input": 2100,
    "visualizer": 100,
    "lanelet_2_global_planner": 10200,
    "behavior_planner_input_1": 1,
    "lanelet_2_map_loader_input": 2100,
    "rear_lidar_driver": 100,
    "points_transformer_rear": 10100,
    "pointcloud_fusion_input": 2100,
    "lanelet_2_map": 100,
    "lanelet_2_map_loader": 10200,
    "parking_planner": 10100,
    "behavior_planner_input_2": 1,
    "lane_planner": 10100,
    "behavior_planner_input_3": 1,
    "behavior_planner_input_4": 1,
    "pointcloud_map": 100,
    "pointcloud_map_loader": 10100,
    "ndt_localizer": 10100,
    "lanelet_2_global_planner_input": 2100,
    "behavior_planner_input_5": 1,
    "euclidean_cluster_settings": 100,
    "euclidean_cluster_detector_1": 10200,
    "intersection_output": 1000,
}
for name, target in costs.items():
    pattern = rf'("{re.escape(name)}",\s*"[^"]*",\s*"[^"]*",\s*)\d+'
    case, count = re.subn(pattern, rf'\g<1>workload_limit({target})', case, count=1)
    if count != 1:
        raise RuntimeError(f"callback {name}: expected one workload match, found {count}")

# Correct two periods that differ from Fig. 4 and the Copper RON.
case = replace_once(case, 'timeval{0, 120000}, 2, 1', 'timeval{0, 60000}, 2, 1', "visualizer period")
case = replace_once(case, 'chain5_6->setPeriod({0, 120000});', 'chain5_6->setPeriod({0, 60000});', "visualizer chain period")
case = replace_once(case, 'chain5_6->setDeadline({0, 120000});', 'chain5_6->setDeadline({0, 60000});', "visualizer chain deadline")
case = replace_once(case, 'timeval{0, 100000}, 6, 1', 'timeval{0, 25000}, 6, 1', "settings period")
case = replace_once(case, 'chain13->setPeriod({0, 100000});', 'chain13->setPeriod({0, 25000});', "settings chain period")
case = replace_once(case, 'chain13->setDeadline({0, 100000});', 'chain13->setDeadline({0, 25000});', "settings chain deadline")
text = text[:start] + case + text[end:]
MAIN.write_text(text)

thread = ROOT / "latency_mgmt/src/thread.cpp"
text = thread.read_text()
old = """    // if sched deadline policy, set affinity using cgroups
    if (policy == SCHED_DEADLINE)
    {
        std::string thread_name = thread_name_stream.str();
        std::string cpu_core = cpu_core_stream.str();
        // Create cgroup for the thread
        create_cgroup(thread_name.c_str(), cpu_core_stream.str().c_str());
        // Add thread to cgroup
        add_thread_to_cgroup(thread_name.c_str(), threadID);
    }
"""
new = """    // Linux requires a SCHED_DEADLINE task's affinity to span its scheduler root
    // domain. Keep the requested mask as metadata, but do not mutate cgroups or narrow
    // kernel affinity from an ordinary Docker container.
    (void)thread_name_stream;
    (void)cpu_core_stream;
"""
thread.write_text(replace_once(text, old, new, "deadline affinity"))

# Deadline bandwidth reclaim needs broader privilege than SYS_NICE; fixed reservations
# preserve the scheduler under test without granting the container that privilege.
for source_path in [ROOT / "latency_mgmt/src/executor.cpp", ROOT / "latency_mgmt/src/mpccontroller.cpp"]:
    text = source_path.read_text()
    text = text.replace("0 | SCHED_FLAG_RECLAIM", "0")
    source_path.write_text(text)

for cmake_path in [ROOT / "latency_mgmt/CMakeLists.txt", ROOT / "ros2-picas/rclcpp/CMakeLists.txt"]:
    text = cmake_path.read_text()
    text = re.sub(r'\n# --- Add NVTX3 include path ---.*?endif\(\)\n', '\n', text, count=1, flags=re.S)
    text = re.sub(r'\n# Link the NVTX library to your executable\ntarget_link_libraries\(latency_mgmt \$\{NVTX_LIBRARY\}\)\n', '\n', text, count=1)
    text = re.sub(r'\nfind_library\(NVTX_LIBRARY.*?endif\(\)\n', '\n', text, count=1, flags=re.S)
    text = text.replace('target_link_libraries(${PROJECT_NAME} ${NVTX_LIBRARY})\n', '')
    cmake_path.write_text(text)
