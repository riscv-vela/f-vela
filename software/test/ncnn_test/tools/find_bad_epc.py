#!/usr/bin/env python3
"""Stream a Rocket commit log (+verbose) and stop at the first trap whose return PC is wrong.

Rocket prints one line per trace event:
  C0: <cycle> [1] pc=[...] ... inst=[...]   retired instruction
  C0: <cycle> [0] pc=[...] ... inst=[...]   trap (exception/interrupt) at pc = epc

For an interrupt, epc must be the next instruction after the last retired one. If the last
retired instruction is not control flow, that is last_pc + its length; for branches/jal the
target or fall-through. Anything else (typically an already-retired instruction) is the bug.

usage: <simulator> +verbose ... 2>&1 | find_bad_epc.py OUT_FILE [--max-anomalies N]
Exits after N anomalies (default 1), which also stops the simulator (broken pipe).
"""
import re
import sys
from collections import deque

LINE = re.compile(r"C0:\s+(\d+)\s+\[(\d)\]\s+pc=\[([0-9a-f]+)\].*inst=\[([0-9a-f]+)\]")


def sext(v, bits):
    return v - (1 << bits) if v & (1 << (bits - 1)) else v


def next_pcs(pc, inst):
    """Possible next PCs after `inst` at `pc`; None if unknown (indirect jump)."""
    if inst & 3 != 3:  # compressed
        op, f3 = inst & 3, (inst >> 13) & 7
        seq = pc + 2
        if op == 1 and f3 == 5:  # c.j
            imm = (((inst >> 12) & 1) << 11 | ((inst >> 11) & 1) << 4 | ((inst >> 9) & 3) << 8 |
                   ((inst >> 8) & 1) << 10 | ((inst >> 7) & 1) << 6 | ((inst >> 6) & 1) << 7 |
                   ((inst >> 3) & 7) << 1 | ((inst >> 2) & 1) << 5)
            return {pc + sext(imm, 12)}
        if op == 1 and f3 in (6, 7):  # c.beqz / c.bnez
            imm = (((inst >> 12) & 1) << 8 | ((inst >> 10) & 3) << 3 | ((inst >> 5) & 3) << 6 |
                   ((inst >> 3) & 3) << 1 | ((inst >> 2) & 1) << 5)
            return {seq, pc + sext(imm, 9)}
        if op == 2 and f3 == 4 and ((inst >> 2) & 0x1f) == 0 and ((inst >> 7) & 0x1f) != 0:
            return None  # c.jr / c.jalr
        return {seq}
    opc = inst & 0x7f
    seq = pc + 4
    if opc == 0x63:  # branch
        imm = (((inst >> 31) & 1) << 12 | ((inst >> 7) & 1) << 11 | ((inst >> 25) & 0x3f) << 5 |
               ((inst >> 8) & 0xf) << 1)
        return {seq, pc + sext(imm, 13)}
    if opc == 0x6f:  # jal
        imm = (((inst >> 31) & 1) << 20 | ((inst >> 12) & 0xff) << 12 | ((inst >> 20) & 1) << 11 |
               ((inst >> 21) & 0x3ff) << 1)
        return {pc + sext(imm, 21)}
    if opc == 0x67:  # jalr
        return None
    if inst == 0x30200073:  # mret
        return None
    return {seq}


def main():
    out_path = sys.argv[1]
    max_anom = int(sys.argv[sys.argv.index("--max-anomalies") + 1]) if "--max-anomalies" in sys.argv else 1
    ring = deque(maxlen=60)
    last = None  # (cycle, pc, inst) of the last retired instruction
    after = 0
    pending = None
    found = 0
    n = 0
    with open(out_path, "w") as out:
        for raw in sys.stdin:
            m = LINE.search(raw)
            if not m:
                if not raw.startswith("C0:"):
                    out.write("[sim] " + raw)  # program output, warnings
                    out.flush()
                continue
            n += 1
            ring.append(raw.rstrip())
            if n % 1000000 == 0:
                out.write(f"[progress] {n} trace lines\n")
                out.flush()
            cyc, valid, pc, inst = int(m.group(1)), m.group(2) == "1", int(m.group(3), 16), int(m.group(4), 16)
            if pending is not None:
                pending.append(raw.rstrip())
                after -= 1
                if after == 0:
                    out.write("\n".join(pending) + "\n" + "=" * 100 + "\n")
                    out.flush()
                    pending = None
                    if found >= max_anom:
                        return 3
            if valid:
                last = (cyc, pc, inst)
                continue
            # trap line: pc = epc
            if last is None:
                continue
            nxt = next_pcs(last[1], last[2])
            if nxt is None or pc in nxt:
                continue
            found += 1
            out.write(f"ANOMALY #{found} at cycle {cyc}: trap epc={pc:#x}, but last retired was pc={last[1]:#x} "
                      f"inst={last[2]:#010x} at cycle {last[0]} (expected next: {sorted(hex(x) for x in nxt)})\n")
            out.write(f"--- last {len(ring)} trace lines before/at the trap ---\n" + "\n".join(ring) + "\n")
            out.write("--- next 40 trace lines ---\n")
            out.flush()
            pending, after = [], 40
    return 0


if __name__ == "__main__":
    sys.exit(main())
