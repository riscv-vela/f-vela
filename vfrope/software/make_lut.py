import numpy as np
import struct

# RoPE parameters
D_MODEL = 128		# d = 128
Q_SIZE  = 64		# q = m / 64
R_SIZE  = 64		# r = m % 64
I_SIZE  = 64		# i = (idx % 128) / 2
BASE    = 10000.0

def f16(x):
    """float -> FP16 bit pattern"""
    return struct.unpack('<H', struct.pack('<e', np.float16(x)))[0]

def q1p7(x):
    """float -> signed Q1.7 (2's complement, uint8)"""
    x = np.clip(x, -1.0, 0.9921875)
    q = int(round(x * 128))
    return q & 0xFF

theta = [BASE ** (-2.0 * i / D_MODEL) for i in range(I_SIZE)]

lutA = []
lutB = []

# LUT A: base angle (q-major), FP16 {sin, cos}
for q in range(Q_SIZE):
    for i in range(I_SIZE):
        phase = (q * 64.0) * theta[i]
        cosA = f16(np.cos(phase))
        sinA = f16(np.sin(phase))
        lutA.append(f"{(sinA << 16 | cosA):08x}")

# LUT B: m scaling (r-major), Q1.7 {sin, cos}
for r in range(R_SIZE):
    for i in range(I_SIZE):
        phase = float(r) * theta[i]
        cosB = q1p7(np.cos(phase))
        sinB = q1p7(np.sin(phase))
        lutB.append(f"{(sinB << 8 | cosB):04x}")

with open("lutA.hex", "w") as f:
    f.write("\n".join(lutA) + "\n")

with open("lutB.hex", "w") as f:
    f.write("\n".join(lutB) + "\n")
