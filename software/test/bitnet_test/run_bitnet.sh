#!/bin/sh
# run_bitnet.sh — BitNet b1.58 2B (llama-cli) on the F-Vela SoC: Rocket core only or with the velaNPU.
# Runs on the target (Buildroot busybox sh). Files in the same directory: llama-cli.linux,
# vela_selftest.linux, i2s_bitnet.gguf, i2s_original.gguf.
#
# usage: ./run_bitnet.sh [options] [-- extra llama-cli args]
#   -m MODE    cpu | mp | mpt | int8 | fp16 | npu | npu-mpt | npu-int8   (default npu)
#                cpu       Rocket core only (BitNet's own I2_S kernel, scalar)
#                mp        ternary matmuls on MpExeUnit (2-bit mpgemm)
#                mpt       ternary matmuls on MpExeUnit with b_transpose (original weight packing)
#                int8      ternary matmuls on the int8 array + AdderTree (weights unpacked to int8)
#                fp16      LM head (F16 token_embd) on FpExeUnit, ternary on the CPU
#                npu       mp + fp16
#                npu-mpt   mpt + fp16
#                npu-int8  int8 + fp16
#   -o         use i2s_original.gguf (the original packing; cpu mode then runs the vela scalar kernel)
#   -a         run cpu, mp, mpt, int8, fp16, npu one after another (same prompt; output must be the same
#              text for cpu/mp/int8, fp16/npu may differ in rare near-ties of the LM head)
#   -p TEXT    question (or text to continue with -r)           (default "What is the capital of France?")
#   -n N       at most N tokens in the answer                    (default -1: until the end-of-turn token
#              <|eot_id|>; it also stops when the 512-token context is full)
#   -r         raw text completion instead of a chat turn: the model continues -p and rarely ends by itself
#              (default -p "The capital of France is", -n 16)
#   -t N       threads (the SoC has one hart)                    (default 1)
#   -R         RoPE (Q, K) on the velaVPU RoPE unit (VELA_ROPE=on), with any -m MODE. The unit's ROM is probed at
#              start; it is used only when its base equals the model's (BitNet 2B-4T: 500000), else RoPE stays on
#              the CPU with a warning (bitstreams before 2026-11-05 have base 10000; make_lut.py now: 500000)
#   --rope M   VELA_ROPE=M: off | on | force (force: run the unit even with another base, wrong angles; for tests)
#   -c N       VELA_CHECK=N: verify the first N NPU matmuls per lane against the CPU (and the first N RoPE ops)
#   -v         verbose: show llama's whole log (model info, system info, sampler, ...) as it runs
#   -V         VELA_VERBOSE=1: one [vela] line per NPU matmul (shown with -v, or in the log file)
#   -w         keep llama's warm-up run (skipped by default: it costs a whole extra forward pass)
#   -s         run vela_selftest.linux -q first (quick check of the three NPU lanes)
#   -L DIR     where the full llama logs go                      (default /tmp)
# Output (default): the generated text, then a short summary (llama timings + [vela] matmul summary).
#   llama's full log is kept in DIR/bitnet_<mode>.log; with -v it is printed instead.
# The question is sent in BitNet b1.58 2B-4T's chat format "User: <question><|eot_id|>Assistant: ", so the model
# answers and ends with <|eot_id|> ("[end of text]").
# Example:  ./run_bitnet.sh -s -m npu -c 4
#           ./run_bitnet.sh -m npu -p "Explain what a neural network is in two sentences."
#           ./run_bitnet.sh -m npu -R -c 2
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
MODE=npu
PROMPT=
NTOK=
CHAT=1
THREADS=1
ALL=0
SELFTEST=0
VERBOSE=0
LOGDIR=/tmp
WARMUP=--no-warmup
MODEL="$HERE/i2s_bitnet.gguf"

while [ $# -gt 0 ]; do
  case "$1" in
    -m) MODE=$2; shift 2 ;;
    -a) ALL=1; shift ;;
    -p) PROMPT=$2; shift 2 ;;
    -n) NTOK=$2; shift 2 ;;
    -t) THREADS=$2; shift 2 ;;
    -c) export VELA_CHECK=$2; shift 2 ;;
    -R) export VELA_ROPE=on; shift ;;
    --rope) export VELA_ROPE=$2; shift 2 ;;
    -v) VERBOSE=1; shift ;;
    -V) export VELA_VERBOSE=1; shift ;;
    -L) LOGDIR=$2; shift 2 ;;
    -w) WARMUP=; shift ;;
    -s) SELFTEST=1; shift ;;
    -o) MODEL="$HERE/i2s_original.gguf"; shift ;;
    -r) CHAT=0; shift ;;
    --model) MODEL=$2; shift 2 ;;
    -h|--help) sed -n '2,/^set -e/p' "$0" | sed '$d'; exit 0 ;;
    --) shift; break ;;
    *) echo "unknown option: $1 (see -h)" >&2; exit 2 ;;
  esac
done

if [ "$SELFTEST" = 1 ]; then
  "$HERE/vela_selftest.linux" -q
fi

if [ "$CHAT" = 1 ]; then
  : "${PROMPT:=What is the capital of France?}"
  : "${NTOK:=-1}"
  FULL_PROMPT="User: $PROMPT<|eot_id|>Assistant: "
  SHOW=--no-display-prompt       # the template is not echoed; the question is printed by run()
else
  : "${PROMPT:=The capital of France is}"
  : "${NTOK:=16}"
  FULL_PROMPT=$PROMPT
  SHOW=
fi

# llama-cli.linux is llama-cli (BITNET_VERSION=pinned) or llama-completion (latest). The latest one would start a
# chat (the model has a chat template) unless -no-cnv is given; the old one does not know that option.
CNV=
if "$HERE/llama-cli.linux" --help 2>&1 | grep -q -- '-no-cnv'; then CNV=-no-cnv; fi

# short summary of a llama log: timings (both llama versions' formats), [vela] lines except per-matmul ones
summary() {
  grep -E '(load|prompt eval|eval|total) time' "$1" | sed 's/^.*_print: *//; s/^ */  /' || true
  grep '^\[vela\]' "$1" | grep -vE '^\[vela\] (repack |[a-z0-9]+ +[^ ]+ +M=[0-9]+ N=[0-9]+ K=[0-9]+  prep |rope  [^ ]+ +rows=)' | \
    grep -vE '^\[vela\] (VELA_MODE=|VELA_ROPE=|ternary weight packing|WARNING|$)' | sed 's/^/  /' || true
}

run() {
  m=$1; shift
  echo "===== VELA_MODE=$m ($(basename "$MODEL"), -n $NTOK) ====="
  [ "$CHAT" = 1 ] && printf 'Q: %s\nA: ' "$PROMPT"
  if [ "$VERBOSE" = 1 ]; then
    VELA_MODE=$m "$HERE/llama-cli.linux" -m "$MODEL" -p "$FULL_PROMPT" -n "$NTOK" -t "$THREADS" \
      --temp 0 -c 512 --no-context-shift $SHOW $WARMUP $CNV "$@" < /dev/null
    echo
    return
  fi
  log="$LOGDIR/bitnet_$m.log"
  rc=0
  VELA_MODE=$m "$HERE/llama-cli.linux" -m "$MODEL" -p "$FULL_PROMPT" -n "$NTOK" -t "$THREADS" \
    --temp 0 -c 512 --no-context-shift $SHOW $WARMUP $CNV "$@" < /dev/null 2> "$log" || rc=$?
  grep -E '^\[vela\] (VELA_MODE=|VELA_ROPE=|ternary weight packing|WARNING)' "$log" | sed 's/^/  /' || true
  summary "$log"
  if [ "$rc" != 0 ]; then
    echo "  llama-cli exited with $rc; last lines of the log:"
    tail -n 20 "$log" | sed 's/^/  | /'
  fi
  if grep -qE '^\[vela\] check .*: FAIL' "$log"; then echo "  ! NPU result differs from the CPU (see the check lines above)"; fi
  echo "  (full log: $log)"
  echo
}

if [ "$ALL" = 1 ]; then
  for m in cpu mp mpt int8 fp16 npu; do run "$m" "$@"; done
else
  run "$MODE" "$@"
fi
