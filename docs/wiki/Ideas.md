# Idea Box

## Systems ideas

### Running the full Copper robotics stack in a browser for resimulation

**Description**

This project explores running the Copper runtime and core system components inside a browser environment for deterministic replay and resimulation. The goal is faithful execution of Copper systems against recorded sensor and execution logs, not a reduced or illustrative mockup.

The work likely involves:
- Compiling parts of the Copper runtime to WebAssembly.
- Providing browser-compatible abstractions for time & IO.
- Deterministic replay of recorded execution traces, including message ordering and timing.
- If we have time: Basic tooling for stepping through time, inspecting state, and visualizing system behavior.

**Relevance to Copper**

Copper’s execution model relies on explicit control over time and scheduling. Being able to replay a full system in a browser validates these properties in a constrained environment. It also provides a low-friction way to inspect behavior, share reproductions, and reason about failures without requiring hardware or a full native setup.

This work directly exercises the boundaries of Copper’s determinism model and clarifies which assumptions are essential versus incidental.

**Technical depth**

This is systems-level work involving compilers, runtime design, and execution semantics. It requires precise reasoning about time, isolation, and reproducibility.

---

### Running Copper as an OS guest under a hypervisor

**Description**

This project investigates running Copper as a guest operating system under a hypervisor instead of as a userspace runtime on Linux or baremetal. The focus is on bootstrapping, isolation, and mapping Copper’s execution model onto virtualized hardware.

Key areas include:
- Defining the minimal virtual hardware required by Copper.
- Booting Copper in a virtual machine.
- Integrating Copper’s scheduler with virtual CPUs and timers.
- Measuring timing behavior, jitter, and determinism under virtualization.

**Relevance to Copper**

This RT supervisor deployment is really interesting for applications where the BOM cost is critical, with one safe/RT component running copper on one Guest and a traditional OS on the other for the UI.

**Technical depth**

This project involves OS concepts, virtualization, boot processes, and low-level scheduling. It is suitable for work that sits close to hardware and system boundaries.

---

## User experience

### First class Python API for algorithm experimentation

**Description**

This project focuses on designing and implementing a Python API that allows users to write and test algorithms against Copper systems while respecting real-time and determinism constraints.

The scope includes:
- Defining a clear boundary between deterministic runtime code and exploratory Python code.
- Exposing Copper concepts in a way that is natural to Python users.
- Making timing, data ownership, and side effects explicit in the API.
- Supporting a path from Python experimentation to production implementations.

**Relevance to Copper**

Many users prototype algorithms in Python. A well-defined Python API lowers the barrier to entry while preserving Copper’s execution guarantees. It also forces clarity around which interactions are safe, which are constrained, and how those constraints are communicated.

This work directly impacts usability without changing Copper’s core execution model.

**Technical depth**

The primary challenge is API and boundary design under strict constraints. This requires understanding both user workflows and the internal guarantees of the system.

---

### Prototype deterministic debugger for Copper

**Description**

This project explores debugging tools built around deterministic execution. Instead of traditional breakpoints and logging, the debugger operates on time, state history, and replay.

Possible components include:
- Time-indexed state inspection.
- Replaying execution forward and backward.
- Tracing causal relationships between inputs and outputs.
- Selective instrumentation that preserves determinism.

**Relevance to Copper**

Determinism enables a different class of debugging tools. Building even a prototype debugger makes these properties tangible and exposes gaps in observability and introspection. It also feeds back into runtime design by identifying what state must be exposed or recorded.

**Technical depth**

This is a mix of systems work and tooling design. It requires careful handling of state, time, and execution order.

---

## Robotics ideas

### Aerobatics demo on the drone platform

**Description**

This project implements aggressive flight maneuvers on Copper’s drone platform that require precise timing and control. The focus is on repeatability and execution accuracy rather than pilot skill.

Work includes:
- High-rate control loops.
- Tight integration between planning and control.
- Demonstrating consistent behavior across runs.

**Relevance to Copper**

Control performance is sensitive to timing and scheduling. This demo provides a concrete way to observe the impact of deterministic execution on physical behavior. It also serves as a validation of Copper on real hardware.

**Technical depth**

This involves control theory, real-time systems, and hardware integration with immediate physical feedback.

---

### Visual inertial odometry for forest traversal

**Description**

This project focuses on implementing a visual inertial odometry pipeline capable of operating in cluttered outdoor environments such as forests.

Key challenges include:
- Robust feature tracking under motion and lighting variation.
- State estimation with intermittent or ambiguous visual information.
- Integration with Copper’s execution and timing model.

**Relevance to Copper**

Perception pipelines stress system integration and timing assumptions. Implementing VIO on Copper exercises the runtime under realistic, noisy workloads and helps validate its suitability for complex estimation tasks.

**Technical depth**

This work combines estimation theory, computer vision, and systems integration.

---

### Indoor micro drone demo with minimal sensing

**Description**

This project targets an indoor, small-scale drone using constrained sensors such as monocular visual odometry and a single-point lidar.

The focus is on:
- Efficient state estimation under limited sensing.
- Careful resource and timing management.
- Stable flight and navigation in confined spaces.

**Relevance to Copper**

Small platforms amplify scheduling, latency, and resource issues. Success here demonstrates that Copper’s execution model holds under tight constraints and embedded conditions.

**Technical depth**

This is end-to-end robotics work involving estimation, control, and runtime behavior in a constrained system.
