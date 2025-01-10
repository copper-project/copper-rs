## Copper: Task Lifecycle

### Here is a quick illustration of a task lifecycle

<img src="https://raw.githubusercontent.com/copper-project/copper-rs/master/doc/lifecycle.svg" alt="Copper Task Lifecycle">

The framework is designed to give as many chances as possible for the tasks to do its heavy lifting out of the critical path.

* The **time critical part** will try to be scheduled as back to back as possible between tasks on the same thread (low latency / low jitter)
* The **best effort part** will try be scheduled by packing them to minimize the interference with the critical path (high throughput / high jitter).

In the critical loop, no allocation should occur, messages are pre-allocated and reused.

Note on freeze / thaw: A task internal state can be serialized to give the opportunity for the framework to "snapshot" the system, it would be at a low rate (ie. ~ a second). 
You can think of it a little bit as a "key frame" in a video. It is useful at replay to be able to jump into a state without having to replay all the previous messages.
