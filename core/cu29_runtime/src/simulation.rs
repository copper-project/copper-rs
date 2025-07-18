//! # `cu29::simulation` Module
//!
//! The `cu29::simulation` module provides an interface to simulate tasks in Copper-based systems.
//! It offers structures, traits, and enums that enable hooking into the lifecycle of tasks, adapting
//! their behavior, and integrating them with simulated hardware environments.
//!
//! ## Overview
//!
//! This module is specifically designed to manage the lifecycle of tasks during simulation, allowing
//! users to override specific simulation steps and simulate sensor data or hardware interaction using
//! placeholders for real drivers. It includes the following components:
//!
//! - **`CuTaskCallbackState`**: Represents the lifecycle states of tasks during simulation.
//! - **`SimOverride`**: Defines how the simulator should handle specific task callbacks, either
//!   executing the logic in the simulator or deferring to the real implementation.
//!
//! ## Hooking Simulation Events
//!
//! You can control and simulate task behavior using a callback mechanism. A task in the Copper framework
//! has a lifecycle, and for each stage of the lifecycle, a corresponding callback state is passed to
//! the simulation. This allows you to inject custom logic for each task stage.
//!
//! ### `CuTaskCallbackState` Enum
//!
//! The `CuTaskCallbackState` enum represents different stages in the lifecycle of a Copper task during a simulation:
//!
//! - **`New(Option<ComponentConfig>)`**: Triggered when a task is created. Use this state to adapt the simulation
//!   to a specific component configuration if needed.
//! - **`Start`**: Triggered when a task starts. This state allows you to initialize or set up any necessary data
//!   before the task processes any input.
//! - **`Preprocess`**: Called before the main processing step. Useful for preparing or validating data.
//! - **`Process(I, O)`**: The core processing state, where you can handle the input (`I`) and output (`O`) of
//!   the task. For source tasks, `I` is `CuMsg<()>`, and for sink tasks, `O` is `CuMsg<()>`.
//! - **`Postprocess`**: Called after the main processing step. Allows for cleanup or final adjustments.
//! - **`Stop`**: Triggered when a task is stopped. Use this to finalize any data or state before task termination.
//!
//! ### Example Usage: Callback
//!
//! You can combine the expressiveness of the enum matching to intercept and override the task lifecycle for the simulation.
//!
//! ```rust,ignore
//! let mut sim_callback = move |step: SimStep<'_>| -> SimOverride {
//!     match step {
//!         // Handle the creation of source tasks, potentially adapting the simulation based on configuration
//!         SimStep::SourceTask(CuTaskCallbackState::New(Some(config))) => {
//!             println!("Creating Source Task with configuration: {:?}", config);
//!             // You can adapt the simulation using the configuration here
//!             SimOverride::ExecuteByRuntime
//!         }
//!         SimStep::SourceTask(CuTaskCallbackState::New(None)) => {
//!             println!("Creating Source Task without configuration.");
//!             SimOverride::ExecuteByRuntime
//!         }
//!         // Handle the processing step for sink tasks, simulating the response
//!         SimStep::SinkTask(CuTaskCallbackState::Process(input, output)) => {
//!             println!("Processing Sink Task...");
//!             println!("Received input: {:?}", input);
//!
//!             // Simulate a response by setting the output payload
//!             output.set_payload(your_simulated_response());
//!             println!("Set simulated output for Sink Task.");
//!
//!             SimOverride::ExecutedBySim
//!         }
//!         // Generic handling for other phases like Start, Preprocess, Postprocess, or Stop
//!         SimStep::SourceTask(CuTaskCallbackState::Start)
//!         | SimStep::SinkTask(CuTaskCallbackState::Start) => {
//!             println!("Task started.");
//!             SimOverride::ExecuteByRuntime
//!         }
//!         SimStep::SourceTask(CuTaskCallbackState::Stop)
//!         | SimStep::SinkTask(CuTaskCallbackState::Stop) => {
//!             println!("Task stopped.");
//!             SimOverride::ExecuteByRuntime
//!         }
//!         // Default fallback for any unhandled cases
//!         _ => {
//!             println!("Unhandled simulation step: {:?}", step);
//!             SimOverride::ExecuteByRuntime
//!         }
//!     }
//! };
//! ```
//!
//! In this example, `example_callback` is a function that matches against the current step in the simulation and
//! determines if the simulation should handle it (`SimOverride::ExecutedBySim`) or defer to the runtime's real
//! implementation (`SimOverride::ExecuteByRuntime`).
//!
//! ## Task Simulation with `CuSimSrcTask` and `CuSimSinkTask`
//!
//! The module provides placeholder tasks for source and sink tasks, which do not interact with real hardware but
//! instead simulate the presence of it.
//!
//! - **`CuSimSrcTask<T>`**: A placeholder for a source task that simulates a sensor or data acquisition hardware.
//!   This task provides the ability to simulate incoming data without requiring actual hardware initialization.
//!
//! - **`CuSimSinkTask<T>`**: A placeholder for a sink task that simulates sending data to hardware. It serves as a
//!   mock for hardware actuators or output devices during simulations.
//!
//! ## Controlling Simulation Flow: `SimOverride` Enum
//!
//! The `SimOverride` enum is used to control how the simulator should proceed at each step. This allows
//! for fine-grained control of task behavior in the simulation context:
//!
//! - **`ExecutedBySim`**: Indicates that the simulator has handled the task logic, and the real implementation
//!   should be skipped.
//! - **`ExecuteByRuntime`**: Indicates that the real implementation should proceed as normal.
//!

use crate::config::ComponentConfig;

use crate::cutask::{CuMsg, CuMsgPayload, CuSinkTask, CuSrcTask, Freezable};
use crate::{input_msg, output_msg};
use cu29_clock::RobotClock;
use cu29_traits::CuResult;
use std::marker::PhantomData;

/// This is the state that will be passed to the simulation support to hook
/// into the lifecycle of the tasks.
pub enum CuTaskCallbackState<I, O> {
    /// Callbacked when a task is created.
    /// It gives you the opportunity to adapt the sim to the given config.
    New(Option<ComponentConfig>),
    /// Callbacked when a task is started.
    Start,
    /// Callbacked when a task is getting called on pre-process.
    Preprocess,
    /// Callbacked when a task is getting called on process.
    /// I and O are the input and output messages of the task.
    /// if this is a source task, I will be CuMsg<()>
    /// if this is a sink task, O will be CuMsg<()>
    Process(I, O),
    /// Callbacked when a task is getting called on post-process.
    Postprocess,
    /// Callbacked when a task is stopped.
    Stop,
}

/// This is the answer the simulator can give to control the simulation flow.
#[derive(PartialEq)]
pub enum SimOverride {
    /// The callback took care of the logic on the simulation side and the actual
    /// implementation needs to be skipped.
    ExecutedBySim,
    /// The actual implementation needs to be executed.
    ExecuteByRuntime,
    /// Emulated the behavior of an erroring task (same as return Err(..) in the normal tasks methods).
    Errored(String),
}

/// This is a placeholder task for a source task for the simulations.
/// It basically does nothing in place of a real driver so it won't try to initialize any hardware.
pub struct CuSimSrcTask<T> {
    boo: PhantomData<T>,
}

impl<T> Freezable for CuSimSrcTask<T> {}

impl<'cl, T: CuMsgPayload + 'cl> CuSrcTask<'cl> for CuSimSrcTask<T> {
    type Output = output_msg!('cl, T);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self { boo: PhantomData })
    }

    fn process(&mut self, _clock: &RobotClock, _new_msg: Self::Output) -> CuResult<()> {
        unimplemented!("A placeholder for sim was called for a source, you need answer SimOverride to ExecutedBySim for the Process step.")
    }
}

/// This is a placeholder task for a sink task for the simulations.
/// It basically does nothing in place of a real driver so it won't try to initialize any hardware.
pub struct CuSimSinkTask<T> {
    boo: PhantomData<T>,
}

impl<T: CuMsgPayload> Freezable for CuSimSinkTask<T> {}

impl<'cl, T: CuMsgPayload + 'cl> CuSinkTask<'cl> for CuSimSinkTask<T> {
    type Input = input_msg!('cl, T);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self { boo: PhantomData })
    }

    fn process(&mut self, _clock: &RobotClock, _input: Self::Input) -> CuResult<()> {
        unimplemented!("A placeholder for sim was called for a sink, you need answer SimOverride to ExecutedBySim for the Process step.")
    }
}
