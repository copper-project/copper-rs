#!/usr/bin/env python3
"""Generate deterministic, torch-free-at-test-time AnyNet fixtures."""

import argparse
import sys
from pathlib import Path
from types import SimpleNamespace

import torch
import torch.nn as nn
import torch.nn.functional as F
from safetensors.torch import save_file


class DisparityRegression(nn.Module):
    def __init__(self, start: int, end: int, stride: int = 1) -> None:
        super().__init__()
        values = torch.arange(start * stride, end * stride, stride).view(1, -1, 1, 1)
        self.register_buffer("disp", values.float())

    def forward(self, value: torch.Tensor) -> torch.Tensor:
        return torch.sum(value * self.disp, 1, keepdim=True)


def warp(self, value: torch.Tensor, disparity: torch.Tensor) -> torch.Tensor:
    batch, _channels, height, width = value.shape
    xx = torch.arange(width, device=value.device).view(1, 1, 1, width)
    xx = xx.expand(batch, 1, height, width)
    yy = torch.arange(height, device=value.device).view(1, 1, height, 1)
    yy = yy.expand(batch, 1, height, width)
    grid = torch.cat((xx - disparity, yy), 1).float()
    grid[:, 0] = 2.0 * grid[:, 0] / max(width - 1, 1) - 1.0
    grid[:, 1] = 2.0 * grid[:, 1] / max(height - 1, 1) - 1.0
    return F.grid_sample(
        value,
        grid.permute(0, 2, 3, 1),
        mode="bilinear",
        padding_mode="zeros",
        align_corners=True,
    )


def build_volume(self, left, right, max_disparity, stride=1):
    del self
    cost = torch.zeros(
        (left.shape[0], max_disparity // stride, left.shape[2], left.shape[3]),
        device=left.device,
    )
    for disparity in range(0, max_disparity, stride):
        index = disparity // stride
        cost[:, index, :, :disparity] = left[:, :, :, :disparity].abs().sum(1)
        if disparity > 0:
            cost[:, index, :, disparity:] = torch.norm(
                left[:, :, :, disparity:] - right[:, :, :, :-disparity], 1, 1
            )
        else:
            cost[:, index] = torch.norm(left - right, 1, 1)
    return cost.contiguous()


def build_residual_volume(self, left, right, max_disparity, disparity, stride=1):
    size = left.shape
    count = max_disparity * 2 - 1
    batch_disparity = disparity[:, None].repeat(1, count, 1, 1, 1)
    batch_disparity = batch_disparity.view(-1, 1, size[-2], size[-1])
    shifts = torch.arange(-max_disparity + 1, max_disparity, device=left.device)
    shifts = shifts.repeat(size[0])[:, None, None, None] * stride
    batch_disparity = batch_disparity - shifts.float()
    batch_left = left[:, None].repeat(1, count, 1, 1, 1)
    batch_left = batch_left.view(-1, size[-3], size[-2], size[-1])
    batch_right = right[:, None].repeat(1, count, 1, 1, 1)
    batch_right = batch_right.view(-1, size[-3], size[-2], size[-1])
    cost = torch.norm(batch_left - warp(self, batch_right, batch_disparity), 1, 1)
    return cost.view(size[0], -1, size[2], size[3]).contiguous()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--anynet-repo", required=True, type=Path)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "tests" / "fixtures",
    )
    args = parser.parse_args()

    sys.path.insert(0, str(args.anynet_repo))
    from models import anynet as reference  # pylint: disable=import-outside-toplevel

    reference.disparityregression2 = DisparityRegression
    reference.AnyNet.warp = warp
    reference.AnyNet._build_volume_2d = build_volume
    reference.AnyNet._build_volume_2d3 = build_residual_volume

    torch.manual_seed(7)
    options = SimpleNamespace(
        init_channels=1,
        maxdisplist=[12, 3, 3],
        spn_init_channels=8,
        nblocks=2,
        layers_3d=4,
        channels_3d=4,
        growth_rate=[4, 1, 1],
        with_spn=False,
    )
    model = reference.AnyNet(options).eval()
    left = torch.rand(1, 3, 64, 128)
    right = torch.rand(1, 3, 64, 128)
    with torch.no_grad():
        stages = model(left, right)
        left_features = model.feature_extraction(left)
        right_features = model.feature_extraction(right)
        stage2_scaled = F.interpolate(
            stages[0],
            (left_features[1].shape[2], left_features[1].shape[3]),
            mode="bilinear",
            align_corners=False,
        ) * left_features[1].shape[2] / left.shape[2]
        stage2_cost = model._build_volume_2d3(
            left_features[1], right_features[1], 3, stage2_scaled
        )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    weights = {
        key: value.detach().cpu().contiguous()
        for key, value in model.state_dict().items()
        if value.is_floating_point()
    }
    save_file(weights, args.output_dir / "weights.safetensors")
    save_file(
        {
            "left": left.contiguous(),
            "right": right.contiguous(),
            "stage1": stages[0].contiguous(),
            "stage2": stages[1].contiguous(),
            "stage3": stages[2].contiguous(),
            "stage2_left_features": left_features[1].contiguous(),
            "stage2_right_features": right_features[1].contiguous(),
            "stage2_scaled_disparity": stage2_scaled.contiguous(),
            "stage2_cost": stage2_cost.contiguous(),
        },
        args.output_dir / "golden.safetensors",
    )


if __name__ == "__main__":
    main()
