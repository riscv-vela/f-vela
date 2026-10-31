#!/usr/bin/env python3
"""Export LeNet-5, VGG16, ResNet18 to ncnn (.param/.bin) with pnnx.

For each model this writes into models/<name>/:
  <name>.ncnn.param / <name>.ncnn.bin   ncnn model (input blob "in0", output blob "out0")
  input.bin                              float32 input from INPUT_SEED 1234 (NCHW, batch 1), see seeded_input()
  ref.bin, ref_seed1234.bin              PyTorch float32 output for that input
  calib_input.bin                        input from seed 4321, for the velaNPU int8 calibration

VGG16 / ResNet18 use the official torchvision ImageNet weights (downloaded by URL,
torchvision itself is not required). LeNet-5 uses fixed-seed random weights.

usage: python export_models.py [lenet vgg16 resnet18] [--fp32]
       python export_models.py --refs-only     # inputs/references only, ncnn model files untouched
"""
import argparse
import os
import sys

import numpy as np
import torch
import torch.nn as nn

HERE = os.path.dirname(os.path.abspath(__file__))

WEIGHT_URLS = {
    "vgg16": "https://download.pytorch.org/models/vgg16-397923af.pth",
    "resnet18": "https://download.pytorch.org/models/resnet18-f37072fd.pth",
}


class LeNet5(nn.Module):
    """Classic LeNet-5 for 1x32x32 input, 10 classes."""

    def __init__(self):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(1, 6, 5), nn.ReLU(inplace=True), nn.MaxPool2d(2),
            nn.Conv2d(6, 16, 5), nn.ReLU(inplace=True), nn.MaxPool2d(2),
        )
        self.classifier = nn.Sequential(
            nn.Linear(16 * 5 * 5, 120), nn.ReLU(inplace=True),
            nn.Linear(120, 84), nn.ReLU(inplace=True),
            nn.Linear(84, 10),
        )

    def forward(self, x):
        x = self.features(x)
        x = torch.flatten(x, 1)
        return self.classifier(x)


class VGG16(nn.Module):
    """Same layer layout / state_dict keys as torchvision.models.vgg16."""

    CFG = [64, 64, "M", 128, 128, "M", 256, 256, 256, "M", 512, 512, 512, "M", 512, 512, 512, "M"]

    def __init__(self, num_classes=1000):
        super().__init__()
        layers, c = [], 3
        for v in self.CFG:
            if v == "M":
                layers.append(nn.MaxPool2d(2, 2))
            else:
                layers += [nn.Conv2d(c, v, 3, padding=1), nn.ReLU(inplace=True)]
                c = v
        self.features = nn.Sequential(*layers)
        self.avgpool = nn.AdaptiveAvgPool2d((7, 7))
        self.classifier = nn.Sequential(
            nn.Linear(512 * 7 * 7, 4096), nn.ReLU(True), nn.Dropout(),
            nn.Linear(4096, 4096), nn.ReLU(True), nn.Dropout(),
            nn.Linear(4096, num_classes),
        )

    def forward(self, x):
        x = self.features(x)
        x = self.avgpool(x)
        x = torch.flatten(x, 1)
        return self.classifier(x)


class BasicBlock(nn.Module):
    def __init__(self, cin, cout, stride=1):
        super().__init__()
        self.conv1 = nn.Conv2d(cin, cout, 3, stride, 1, bias=False)
        self.bn1 = nn.BatchNorm2d(cout)
        self.relu = nn.ReLU(inplace=True)
        self.conv2 = nn.Conv2d(cout, cout, 3, 1, 1, bias=False)
        self.bn2 = nn.BatchNorm2d(cout)
        self.downsample = None
        if stride != 1 or cin != cout:
            self.downsample = nn.Sequential(nn.Conv2d(cin, cout, 1, stride, bias=False), nn.BatchNorm2d(cout))

    def forward(self, x):
        idt = x if self.downsample is None else self.downsample(x)
        out = self.relu(self.bn1(self.conv1(x)))
        out = self.bn2(self.conv2(out))
        return self.relu(out + idt)


class ResNet18(nn.Module):
    """Same layer layout / state_dict keys as torchvision.models.resnet18."""

    def __init__(self, num_classes=1000):
        super().__init__()
        self.conv1 = nn.Conv2d(3, 64, 7, 2, 3, bias=False)
        self.bn1 = nn.BatchNorm2d(64)
        self.relu = nn.ReLU(inplace=True)
        self.maxpool = nn.MaxPool2d(3, 2, 1)
        self.layer1 = nn.Sequential(BasicBlock(64, 64), BasicBlock(64, 64))
        self.layer2 = nn.Sequential(BasicBlock(64, 128, 2), BasicBlock(128, 128))
        self.layer3 = nn.Sequential(BasicBlock(128, 256, 2), BasicBlock(256, 256))
        self.layer4 = nn.Sequential(BasicBlock(256, 512, 2), BasicBlock(512, 512))
        self.avgpool = nn.AdaptiveAvgPool2d((1, 1))
        self.fc = nn.Linear(512, num_classes)

    def forward(self, x):
        x = self.maxpool(self.relu(self.bn1(self.conv1(x))))
        x = self.layer4(self.layer3(self.layer2(self.layer1(x))))
        x = torch.flatten(self.avgpool(x), 1)
        return self.fc(x)


MODELS = {
    "lenet": (LeNet5, (1, 1, 32, 32)),
    "vgg16": (VGG16, (1, 3, 224, 224)),
    "resnet18": (ResNet18, (1, 3, 224, 224)),
}


def build_model(name):
    """Same model + weights that were exported to ncnn (LeNet: torch seed 0, others: torchvision)."""
    cls, _ = MODELS[name]
    torch.manual_seed(0)
    model = cls()
    if name in WEIGHT_URLS:
        sd = torch.hub.load_state_dict_from_url(WEIGHT_URLS[name], progress=True, map_location="cpu")
        model.load_state_dict(sd)
    return model.eval()


# ---- seeded input ----------------------------------------------------------------------------
# Same generator as seeded_input() in src/ncnn_conv_test.cpp, so the target can build the input
# from INPUT_SEED and still be compared with the PyTorch output saved here:
#   splitmix64(seed) -> k = z >> 40 (24 bits) -> x = k * 2^-24 * 4 - 2   in [-2, 2), NCHW order.
# Every step is exact in float32, so C and numpy give bit-identical inputs.
TEST_SEED = 1234
CALIB_SEED = 4321


def seeded_input(shape, seed):
    n = int(np.prod(shape))
    mask = (1 << 64) - 1
    state = seed & mask
    k = np.empty(n, dtype=np.uint32)
    for i in range(n):
        state = (state + 0x9E3779B97F4A7C15) & mask
        z = state
        z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & mask
        z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & mask
        z ^= z >> 31
        k[i] = z >> 40
    x = k.astype(np.float32) * np.float32(1.0 / 16777216.0) * np.float32(4.0) - np.float32(2.0)
    return torch.from_numpy(x.reshape(shape))


def write_inputs_and_refs(name, model=None):
    """input.bin / ref.bin (seed 1234, also saved as ref_seed1234.bin) and calib_input.bin (seed 4321)."""
    _, shape = MODELS[name]
    out_dir = os.path.join(HERE, name)
    os.makedirs(out_dir, exist_ok=True)
    if model is None:
        model = build_model(name)
    x = seeded_input(shape, TEST_SEED)
    with torch.no_grad():
        ref = model(x)
    x.numpy().astype(np.float32).tofile(os.path.join(out_dir, "input.bin"))
    ref.numpy().astype(np.float32).tofile(os.path.join(out_dir, "ref.bin"))
    ref.numpy().astype(np.float32).tofile(os.path.join(out_dir, f"ref_seed{TEST_SEED}.bin"))
    seeded_input(shape, CALIB_SEED).numpy().astype(np.float32).tofile(os.path.join(out_dir, "calib_input.bin"))
    top5 = torch.topk(ref[0], 5)
    print(f"[{name}] seed {TEST_SEED}: ref top5 idx={top5.indices.tolist()} "
          f"val={[round(v, 4) for v in top5.values.tolist()]}; calib_input.bin seed {CALIB_SEED}")
    return x


def export(name, fp16):
    import pnnx

    _, shape = MODELS[name]
    model = build_model(name)
    out_dir = os.path.join(HERE, name)
    os.makedirs(out_dir, exist_ok=True)
    x = write_inputs_and_refs(name, model)

    base = os.path.join(out_dir, name)
    pnnx.export(
        model, base + ".pt", (x,),
        ncnnparam=base + ".ncnn.param", ncnnbin=base + ".ncnn.bin", ncnnpy=base + "_ncnn.py",
        pnnxparam=base + ".pnnx.param", pnnxbin=base + ".pnnx.bin", pnnxpy=base + "_pnnx.py",
        pnnxonnx=base + ".pnnx.onnx", fp16=fp16,
    )
    # Keep only what the target needs (plus the TorchScript file for re-export).
    for ext in (".pnnx.param", ".pnnx.bin", "_pnnx.py", ".pnnx.onnx", "_ncnn.py"):
        p = base + ext
        if os.path.exists(p):
            os.remove(p)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("models", nargs="*", default=list(MODELS))
    ap.add_argument("--fp32", action="store_true", help="store ncnn weights as fp32 (default fp16)")
    ap.add_argument("--refs-only", "--calib-input-only", dest="refs_only", action="store_true",
                    help="only (re)write input.bin, ref*.bin and calib_input.bin; keep the ncnn model files")
    args = ap.parse_args()
    for m in args.models:
        if m not in MODELS:
            sys.exit(f"unknown model {m}; choose from {list(MODELS)}")
        if args.refs_only:
            write_inputs_and_refs(m)
        else:
            export(m, fp16=not args.fp32)


if __name__ == "__main__":
    main()
