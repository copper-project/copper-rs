def process(inp, state, output):
    left_msg, right_msg = inp

    left_value = 0
    right_label = ""

    if left_msg.payload is not None:
        left_value = left_msg.payload.value
    if right_msg.payload is not None:
        right_label = right_msg.payload.label

    state.calls += 1
    state.total += left_value
    state.last_label = right_label

    output[0].payload = {
        "doubled": left_value * 2,
        "label": right_label,
        "calls": state.calls,
    }
    output[1].payload = {
        "total": state.total,
        "last_label": state.last_label,
    }
