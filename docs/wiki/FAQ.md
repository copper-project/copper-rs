# FAQ

## How is Copper better or different from the ROS (Robot Operating System)?

Copper is a user-friendly runtime engine written in Rust. It manages task execution, data flow, logging, and deterministic replay.

ROS is an open-source set of software libraries and tools primarily written for C++ and Python.

## Is it that more performant?

In the example directory, there are two equivalent applications: one written in C++ for ROS and a port in Rust with Copper.

```bash
examples/cu_caterpillar
examples/ros_caterpillar
```

You can run them on a desktop or with GPIOs on a Raspberry Pi. You should see an order-of-magnitude difference in performance.

Copper is designed for performance first, using a data-oriented approach to minimize latency and maximize throughput.

## What about safety considerations?

Copper is fully deterministic, which means you can replay captured data and get the same results you observed on the robot.

Copper is written in Rust, so it is memory safe and thread safe by design.
