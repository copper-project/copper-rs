# Copper Project

A user friendly robotics framework to create fast and reliable robots.

Easy: Copper combines a high level configuration and a natural Rust first API.

Fast: Copper leverages the 0-cost abstraction features of Rust with a data oriented approach (ie. hardware friendly, no allocation on head during the execution etc...) to achieve sub microsecond latency on commodity hardware.

Reliable: Copper leverages Rust's ownership, type system, and concurrency model to reduce bugs and ensure thread safety.

[![copper](https://github.com/gbin/copper-project/actions/workflows/general.yml/badge.svg)](https://github.com/gbin/copper-project/actions/workflows/general.yml)
![GitHub last commit](https://img.shields.io/github/last-commit/gbin/copper-project)
![](https://img.shields.io/badge/Rust-1.79+-orange.svg)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Overview

Copper is a data oriented runtime that is made of those key components:

* A task graph described in RON configuring the overall topology of the system (ie. which task talk to which) and sets types for the nodes and messages betweens the tasks.
* A runtime generator that will decide on an execution plan based on the metadata from the graph. Based on this execution plan, the internal datastructure for the communication will be preallocated called a "Copper List" that will maximize sequential memory accesses during execution
* A 0 copy data logging facility that will record all the messages between all the tasks
* A super fast structured logging for regular textual logs. All the logging strings are interned and indexes at compile time to avoid any string construction at runtime.

## Hello World for Copper

Enough! Show me how it is easy to build something!



