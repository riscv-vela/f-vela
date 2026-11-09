#!/usr/bin/env python3
"""Rewrite the ternary (I2_S) tensors of a BitNet GGUF into the layout BitNet's kernels read.

model/i2s_original.gguf stores I2_S tensors as
    row-major W[N][K], 4 weights per byte along K, LSB first (k = 4*b + i at bits [2i+1:2i]),
    codes 00 = 0, 01 = +1, 11 = -1   (code 10 never appears)
BitNet (3rdparty/llama.cpp @ 40ed0f29, ggml-bitnet-mad.cpp quantize_i2_s) expects
    blocks of 128 weights in 32 bytes: byte p of a block holds weights p, 32+p, 64+p, 96+p
    at bit shifts 6, 4, 2, 0, codes 0 / 1 / 2 = -1 / 0 / +1
The tensor size (N*K/4 bytes + fp32 scale) is the same, so the file is copied and only the
packed bytes are rewritten in place; metadata, scales and the other tensors are untouched.
Tensors already in the BitNet layout (code 11 absent, code 10 present) are left as they are.

usage: convert_i2s_layout.py IN.gguf OUT.gguf      (needs numpy)
"""
import shutil
import struct
import sys

import numpy as np

GGML_TYPE_I2_S = 36
# GGUF metadata value types -> fixed size in bytes (8 = string, 9 = array are variable)
_GGUF_SIZES = {0: 1, 1: 1, 2: 2, 3: 2, 4: 4, 5: 4, 6: 4, 7: 1, 10: 8, 11: 8, 12: 8}


def read_gguf_tensors(path):
    """Minimal GGUF v2/v3 reader: [(name, dims, ggml_type, absolute data offset)]."""
    with open(path, "rb") as f:
        def rd(fmt):
            return struct.unpack("<" + fmt, f.read(struct.calcsize("<" + fmt)))[0]

        def rd_str():
            return f.read(rd("Q")).decode("utf-8", "replace")

        def skip_value(vtype):
            if vtype == 8:
                f.seek(rd("Q"), 1)
            elif vtype == 9:
                itype, count = rd("I"), rd("Q")
                if itype in _GGUF_SIZES:
                    f.seek(_GGUF_SIZES[itype] * count, 1)
                else:
                    for _ in range(count):
                        skip_value(itype)
            else:
                f.seek(_GGUF_SIZES[vtype], 1)

        if f.read(4) != b"GGUF":
            sys.exit(f"{path}: not a GGUF file")
        version, n_tensors, n_kv = rd("I"), rd("Q"), rd("Q")
        if version < 2:
            sys.exit(f"{path}: GGUF v{version} not supported")
        alignment = 32
        for _ in range(n_kv):
            key, vtype = rd_str(), rd("I")
            if key == "general.alignment" and vtype == 4:
                alignment = rd("I")
            else:
                skip_value(vtype)
        infos = []
        for _ in range(n_tensors):
            name = rd_str()
            dims = [rd("Q") for _ in range(rd("I"))]
            infos.append((name, dims, rd("I"), rd("Q")))
        base = (f.tell() + alignment - 1) // alignment * alignment
    return [(n, d, t, base + off) for n, d, t, off in infos]


def codes_lsb(raw):
    """2-bit fields of each byte, LSB first -> shape (bytes*4,)"""
    return np.stack([(raw >> s) & 3 for s in (0, 2, 4, 6)], axis=1).reshape(-1)


def to_bitnet(raw):
    """custom layout bytes -> BitNet I2_S bytes (same length)"""
    c = codes_lsb(raw)
    w = np.zeros(c.shape, dtype=np.uint8)          # BitNet code = w + 1
    w[c == 0] = 1
    w[c == 1] = 2
    w[c == 3] = 0
    blk = w.reshape(-1, 4, 32)                      # [block][group][pos], element = group*32 + pos
    return ((blk[:, 0] << 6) | (blk[:, 1] << 4) | (blk[:, 2] << 2) | blk[:, 3]).astype(np.uint8).reshape(-1)


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    src, dst = sys.argv[1], sys.argv[2]
    tensors = [t for t in read_gguf_tensors(src) if t[2] == GGML_TYPE_I2_S]
    print(f"{src}: {len(tensors)} I2_S tensors")

    data = np.memmap(src, dtype=np.uint8, mode="r")
    plan = []
    for name, dims, _, off in tensors:
        k, n = int(dims[0]), int(dims[1])
        nbytes = n * k // 4
        hist = np.bincount(codes_lsb(np.asarray(data[off:off + nbytes])), minlength=4)
        if hist[2] == 0 and hist[3] > 0:
            plan.append((name, off, nbytes))
        elif hist[3] == 0:
            print(f"  {name}: already BitNet layout, kept")
        else:
            sys.exit(f"  {name}: unknown layout (code histogram {hist.tolist()})")
    del data

    shutil.copyfile(src, dst)
    out = np.memmap(dst, dtype=np.uint8, mode="r+")
    for i, (name, off, nbytes) in enumerate(plan):
        out[off:off + nbytes] = to_bitnet(np.array(out[off:off + nbytes]))
        if i % 30 == 0 or i == len(plan) - 1:
            print(f"  [{i + 1}/{len(plan)}] {name}")
    out.flush()
    print(f"wrote {dst}: {len(plan)} tensors converted")


if __name__ == "__main__":
    main()
