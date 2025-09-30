## High Performance Monotonic Clock for robotics

This crate provides a monotonic clock and a mockable clock for testing and replay.
It has a wide range of support for robots running on OSes and bare metal.

It is no-std compatible with some limitations, see below.

Low level CPU counter implementations are provided for:

- x86-64
- arm64
- armv7
- risc-v

We also provide a TOV (Time Of Validity) enum for tagging sensor data.

It can fall back to the posix monotonic clock for other platforms (with std).

It has been created originally for the Copper runtime but can be perfectly used independently.
See the main crate cu29 for more information about the overall Copper project.
