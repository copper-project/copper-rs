#!/usr/bin/env python3
"""Export the upstream PyTorch model and deterministic parity fixtures.

This is a development tool only. Runtime inference and Rust tests do not need
Python, PyTorch, ROS, or the original repository.
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys
from collections import OrderedDict
from pathlib import Path

import torch
from torch.nn.utils import remove_spectral_norm


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_UPSTREAM = ROOT.parent / "vitfly"


def write_safetensors(
    path: Path, tensors: dict[str, torch.Tensor], source: str
) -> None:
    """Write contiguous F32 tensors using the documented Safetensors layout."""
    header: OrderedDict[str, object] = OrderedDict()
    payloads: list[bytes] = []
    offset = 0
    for name in sorted(tensors):
        tensor = tensors[name].detach().cpu().contiguous().to(torch.float32)
        payload = tensor.numpy().tobytes(order="C")
        header[name] = {
            "dtype": "F32",
            "shape": list(tensor.shape),
            "data_offsets": [offset, offset + len(payload)],
        }
        payloads.append(payload)
        offset += len(payload)
    header["__metadata__"] = {
        "source": source,
        "spectral_norm": "materialized before export",
    }
    encoded = json.dumps(header, separators=(",", ":")).encode("utf-8")
    encoded += b" " * ((8 - len(encoded) % 8) % 8)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as handle:
        handle.write(struct.pack("<Q", len(encoded)))
        handle.write(encoded)
        for payload in payloads:
            handle.write(payload)


def make_inputs() -> list[tuple[torch.Tensor, torch.Tensor, torch.Tensor]]:
    y = torch.linspace(-1.0, 1.0, 60).view(1, 1, 60, 1)
    x = torch.linspace(-1.0, 1.0, 90).view(1, 1, 1, 90)
    radius = torch.sqrt(x.square() + y.square())
    depth0 = (0.15 + 0.60 * radius).clamp(0.0, 0.8)
    depth0 = depth0 + 0.03 * torch.sin(7.0 * x) * torch.cos(5.0 * y)
    depth0 = depth0.clamp(0.0, 0.8).float()

    depth1 = torch.flip(depth0, dims=(-1,))
    depth1 = (depth1 * 0.85 + 0.05 * torch.cos(9.0 * y)).clamp(0.0, 0.8)

    attitude1 = torch.tensor([[0.9746794, 0.1, -0.05, 0.1870829]])
    attitude1 = attitude1 / torch.linalg.vector_norm(attitude1, dim=1, keepdim=True)
    return [
        (depth0, torch.tensor([[4.0]]), torch.tensor([[1.0, 0.0, 0.0, 0.0]])),
        (depth1, torch.tensor([[7.0]]), attitude1),
    ]


def trace_encoder(stage, input_tensor: torch.Tensor, prefix: str, trace: dict[str, torch.Tensor]):
    output, height, width = stage.patchMerge(input_tensor)
    trace[f"{prefix}.patch"] = output
    for index in range(len(stage._attn)):
        output = output + stage._attn[index](output, height, width)
        trace[f"{prefix}.attention{index}"] = output
        output = output + stage._ffn[index](output, height, width)
        trace[f"{prefix}.ffn{index}"] = output
        output = stage._lNorm[index](output)
        trace[f"{prefix}.norm{index}"] = output
    batch = output.shape[0]
    output = output.reshape(batch, height, width, -1).permute(0, 3, 1, 2).contiguous()
    trace[f"{prefix}.output"] = output
    return output


def lstm_cell(model, layer: int, input_tensor: torch.Tensor, hidden, cell):
    gates = torch.nn.functional.linear(
        input_tensor,
        getattr(model.lstm, f"weight_ih_l{layer}"),
        getattr(model.lstm, f"bias_ih_l{layer}"),
    )
    gates += torch.nn.functional.linear(
        hidden,
        getattr(model.lstm, f"weight_hh_l{layer}"),
        getattr(model.lstm, f"bias_hh_l{layer}"),
    )
    input_gate, forget_gate, cell_gate, output_gate = gates.chunk(4, dim=1)
    cell = torch.sigmoid(forget_gate) * cell + torch.sigmoid(input_gate) * torch.tanh(cell_gate)
    hidden = torch.sigmoid(output_gate) * torch.tanh(cell)
    return hidden, cell


def trace_forward(model, depth, desired_velocity, attitude, state=None):
    trace: dict[str, torch.Tensor] = {"input.depth": depth}
    encoder0 = trace_encoder(model.encoder_blocks[0], depth, "encoder0", trace)
    encoder1 = trace_encoder(model.encoder_blocks[1], encoder0, "encoder1", trace)
    shuffled = model.pxShuffle(encoder1)
    trace["decoder.pixel_shuffle"] = shuffled
    upsampled = model.up_sample(encoder0)
    trace["decoder.upsample"] = upsampled
    decoded = torch.cat([shuffled, upsampled], dim=1)
    trace["decoder.concat"] = decoded
    decoded = model.down_sample(decoded)
    trace["decoder.down_sample"] = decoded
    decoded = model.decoder(decoded.flatten(1))
    trace["decoder.linear"] = decoded
    recurrent = torch.cat([decoded, desired_velocity / 10, attitude], dim=1).float()
    trace["metadata.concat"] = recurrent

    if state is None:
        state = [
            (
                torch.zeros((1, 128), dtype=torch.float32),
                torch.zeros((1, 128), dtype=torch.float32),
            )
            for _ in range(3)
        ]
    next_state = []
    for layer in range(3):
        hidden, cell = lstm_cell(model, layer, recurrent, *state[layer])
        trace[f"lstm{layer}.h"] = hidden
        trace[f"lstm{layer}.c"] = cell
        next_state.append((hidden, cell))
        recurrent = hidden
    output = model.nn_fc2(recurrent)
    trace["output"] = output
    return output, next_state, trace


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--upstream", type=Path, default=DEFAULT_UPSTREAM)
    parser.add_argument(
        "--checkpoint",
        type=Path,
        help="checkpoint to export (defaults to the upstream published model)",
    )
    args = parser.parse_args()
    upstream = args.upstream.resolve()
    checkpoint_path = (
        args.checkpoint.resolve()
        if args.checkpoint is not None
        else upstream / "models" / "ViTLSTM_model.pth"
    )
    sys.path.insert(0, str(upstream / "models"))
    from model import LSTMNetVIT  # pylint: disable=import-error,import-outside-toplevel

    torch.manual_seed(0)
    model = LSTMNetVIT().cpu().float()
    checkpoint = torch.load(
        checkpoint_path,
        map_location="cpu",
        weights_only=True,
    )
    model.load_state_dict(checkpoint)
    model.eval()

    inputs = make_inputs()
    with torch.inference_mode():
        reference_before, _ = model([*inputs[0]])
    remove_spectral_norm(model.decoder)
    remove_spectral_norm(model.nn_fc2)
    with torch.inference_mode():
        reference_after, _ = model([*inputs[0]])
    torch.testing.assert_close(reference_before, reference_after, rtol=1e-6, atol=1e-6)

    weights = {name: value for name, value in model.state_dict().items()}
    source = f"vitfly {checkpoint_path.name}"
    write_safetensors(
        ROOT / "weights" / "vitfly-vitlstm-f32.safetensors", weights, source
    )

    fixtures: dict[str, torch.Tensor] = {}
    state = None
    with torch.inference_mode():
        for sample_index, (depth, desired_velocity, attitude) in enumerate(inputs):
            output, state, trace = trace_forward(
                model, depth, desired_velocity, attitude, state
            )
            direct_hidden = torch.stack([item[0].squeeze(0) for item in state], dim=0)
            direct_cell = torch.stack([item[1].squeeze(0) for item in state], dim=0)
            # Validate the explicit, layer-visible cells against PyTorch's fused
            # three-layer LSTM. A few ULPs are expected from accumulation order.
            previous = None if sample_index == 0 else previous_combined
            combined_input = trace["metadata.concat"]
            combined_output, combined_state = model.lstm(combined_input, previous)
            direct_inputs = [depth, desired_velocity, attitude]
            if previous is not None:
                direct_inputs.append(previous)
            direct_output, direct_state = model(direct_inputs)
            torch.testing.assert_close(combined_output, trace["lstm2.h"], rtol=2e-5, atol=3e-6)
            torch.testing.assert_close(combined_state[0], direct_hidden, rtol=2e-5, atol=3e-6)
            torch.testing.assert_close(combined_state[1], direct_cell, rtol=2e-5, atol=3e-6)
            torch.testing.assert_close(model.nn_fc2(combined_output), output, rtol=2e-5, atol=3e-6)
            torch.testing.assert_close(direct_output, output, rtol=2e-5, atol=3e-6)
            torch.testing.assert_close(direct_state[0], combined_state[0], rtol=2e-5, atol=3e-6)
            torch.testing.assert_close(direct_state[1], combined_state[1], rtol=2e-5, atol=3e-6)
            previous_combined = combined_state

            fixtures[f"sample{sample_index}.desired_velocity"] = desired_velocity
            fixtures[f"sample{sample_index}.attitude"] = attitude
            for name, tensor in trace.items():
                fixtures[f"sample{sample_index}.{name}"] = tensor

    write_safetensors(
        ROOT / "fixtures" / "pytorch-parity.safetensors", fixtures, source
    )
    print(f"exported {len(weights)} weights and {len(fixtures)} fixture tensors")


if __name__ == "__main__":
    main()
