import numpy as np

# rope golden
x_data_hex = [
    0x2000, 0x4000, 0x6000, 0x7FFF,
    0xE000, 0xC000, 0xA000, 0x8000
]

D_MODEL = 128
BASE = 10000.0

# RoPE position
m = 65

# 벡터의 시작 dimension pair index
# idx=0이면 theta[0], theta[1], theta[2], theta[3] 사용
idx = 0


def to_int16(value):
    """16-bit 값을 signed int16로 변환"""
    return np.int16(value & 0xFFFF)


def q15_to_float(value):
    """signed Q1.15 -> float"""
    return float(to_int16(value)) / 32768.0


def float_to_q15(value):
    """float -> signed Q1.15"""
    value = np.clip(value, -1.0, 32767.0 / 32768.0)
    quantized = int(np.round(value * 32768.0))

    # int16 범위 제한
    quantized = np.clip(quantized, -32768, 32767)
    return np.int16(quantized)


def int16_to_hex(value):
    """signed int16 -> 4자리 hex"""
    return f"0x{int(value) & 0xFFFF:04x}"


# 입력을 signed int16 및 Q1.15 float로 변환
x_int16 = np.array(
    [to_int16(x) for x in x_data_hex],
    dtype=np.int16
)

x_float = np.array(
    [q15_to_float(x) for x in x_data_hex],
    dtype=np.float64
)

y_float = np.zeros(8, dtype=np.float64)
y_q15 = np.zeros(8, dtype=np.int16)

# 2개씩 묶어서 RoPE 적용
#
# pair 0: x[0], x[1]
# pair 1: x[2], x[3]
# pair 2: x[4], x[5]
# pair 3: x[6], x[7]
for pair in range(4):
    dimension_index = idx + pair

    theta = BASE ** (-2.0 * dimension_index / D_MODEL)
    angle = m * theta

    cos_val = np.cos(angle)
    sin_val = np.sin(angle)

    x_even = x_float[2 * pair]
    x_odd = x_float[2 * pair + 1]

    # 일반적인 RoPE 회전
    y_even = x_even * cos_val - x_odd * sin_val
    y_odd = x_even * sin_val + x_odd * cos_val

    y_float[2 * pair] = y_even
    y_float[2 * pair + 1] = y_odd

    y_q15[2 * pair] = float_to_q15(y_even)
    y_q15[2 * pair + 1] = float_to_q15(y_odd)

    print(f"pair {pair}")
    print(f"  theta = {theta:.10f}")
    print(f"  angle = {angle:.10f}")
    print(f"  cos   = {cos_val:.10f}")
    print(f"  sin   = {sin_val:.10f}")
    print(f"  input = ({x_even:.10f}, {x_odd:.10f})")
    print(f"  output= ({y_even:.10f}, {y_odd:.10f})")


print("\nInput")
for i in range(8):
    print(
        f"x_data[{i}] = {int16_to_hex(x_int16[i])}, "
        f"signed={int(x_int16[i]):6d}, "
        f"float={x_float[i]: .10f}"
    )


print("\nRoPE result")
for i in range(8):
    print(
        f"y_data[{i}] = {int16_to_hex(y_q15[i])}, "
        f"signed={int(y_q15[i]):6d}, "
        f"float={y_float[i]: .10f}"
    )