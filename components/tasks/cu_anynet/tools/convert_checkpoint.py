#!/usr/bin/env python3
"""Convert an AnyNet PyTorch checkpoint to Candle-readable safetensors."""

import argparse

import torch
from safetensors.torch import save_file


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("checkpoint", help="AnyNet checkpoint.tar")
    parser.add_argument("output", help="output .safetensors path")
    args = parser.parse_args()

    checkpoint = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    state = checkpoint.get("state_dict", checkpoint)
    tensors = {
        key.removeprefix("module."): value.detach().cpu().contiguous()
        for key, value in state.items()
        if isinstance(value, torch.Tensor)
    }
    save_file(tensors, args.output)


if __name__ == "__main__":
    main()
