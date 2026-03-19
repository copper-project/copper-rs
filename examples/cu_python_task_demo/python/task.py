"""Example Python task used by ``cu_python_task_demo``.

The function mutates both persistent state and preallocated output messages. It is
deliberately small so the data contract is easy to read.
"""

def start(ctx, state):
    """Optional hook called once before the first process() invocation."""
    state.calls = 0
    state.total = 0
    state.last_tag = 0


def process(ctx, inp, state, output):
    """Update the demo state and fill the two output messages."""
    left_msg, right_msg = inp

    left_value = 0
    right_tag = 0

    if left_msg.payload is not None:
        left_value = left_msg.payload.value
    if right_msg.payload is not None:
        right_tag = right_msg.payload.tag

    state.calls += 1
    state.total += left_value
    state.last_tag = right_tag

    output[0].payload.doubled = left_value * 2
    output[0].payload.tag = right_tag
    output[0].payload.calls = state.calls

    output[1].payload.total = state.total
    output[1].payload.last_tag = state.last_tag


def stop(ctx, state):
    """Optional hook called once during shutdown."""
    _ = ctx.cl_id
