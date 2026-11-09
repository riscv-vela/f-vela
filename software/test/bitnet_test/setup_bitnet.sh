#!/usr/bin/env bash
# Clone BitNet (with its llama.cpp submodule) at a fixed commit and apply patches/<version>/.
#
# usage: ./setup_bitnet.sh [latest|pinned] [--dir DIR] [--save]
#
#   latest  microsoft/BitNet 0b341e5 (2026-10), llama.cpp 390c30775 (isHuangXin fork, upstream-based)
#           patches/latest/llama.cpp.patch:
#             - "bitnet-25" arch name (the model file and BitNet's own converter use it; this llama.cpp dropped it)
#             - bitnet-b1.58 / bitnet-25 graph with ReLU^2 FFN (this llama.cpp runs BitNet 2B-4T with SiLU -> wrong text)
#             - velaNPU hook (GGML_VELA)
#   pinned  microsoft/BitNet 404980e (2025-06), llama.cpp 40ed0f29 (Eddie-Wang1120 fork, merge-dev)
#           patches/pinned/bitnet.patch (scalar I2_S kernel for RISC-V), patches/pinned/llama.cpp.patch (velaNPU hook)
#
#   --dir DIR  where to put the sources (default: ./BitNet-<version>)
#   --save     write the current edits in DIR back into patches/<version>/ (after changing the sources;
#              a new file must be registered once with `git add -N <file>` in its repo)
#
# Safe to run again: it applies the patches only when they changed since the last run (stamp DIR/.vela-patched);
# in that case files touched by the patches are reset first. Other local edits in DIR are not touched.
# env BITNET_URL overrides the clone source (e.g. a local mirror); the submodule URL comes from BitNet's .gitmodules.
set -Eeuo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERSION=latest
DIR=
SAVE=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    latest|pinned) VERSION=$1; shift ;;
    --dir) DIR=$2; shift 2 ;;
    --save) SAVE=1; shift ;;
    -h|--help) sed -n 2,23p "$0"; exit 0 ;;
    *) echo "unknown argument: $1 (see --help)" >&2; exit 2 ;;
  esac
done

case "$VERSION" in
  latest) BITNET_COMMIT=0b341e582afbf9e1011f24744b554c96a3477eb5; LLAMA_COMMIT=390c307752ab78fd8189f359d6954c9ba1be74af ;;
  pinned) BITNET_COMMIT=404980eecae38affa4871c3e419eae3f44536a95; LLAMA_COMMIT=40ed0f290203a9a78540b8f7eb18bd828043fe21 ;;
esac
BITNET_URL=${BITNET_URL:-https://github.com/microsoft/BitNet.git}
DIR=${DIR:-$HERE/BitNet-$VERSION}
LLAMA="$DIR/3rdparty/llama.cpp"
PATCHES="$HERE/patches/$VERSION"
STAMP="$DIR/.vela-patched"
APPLIED="$DIR/.vela-applied"     # copies of the patches applied last (to undo them)
log() { echo "[setup_bitnet] $*"; }
patch_hash() { shopt -s nullglob; local p=("$PATCHES"/*.patch); cat /dev/null "${p[@]}" | sha1sum | cut -d' ' -f1; }

# repo -> patch file pairs (a missing patch file is skipped)
pairs() { echo "$DIR $PATCHES/bitnet.patch"; echo "$LLAMA $PATCHES/llama.cpp.patch"; }

if (( SAVE )); then
  while read -r repo patch; do
    # new files: those the existing patch creates (+ any registered with git add -N)
    if [[ -f "$patch" ]]; then
      git -C "$repo" apply --summary "$patch" | awk '$1 == "create" {print $4}' | while read -r f; do
        [[ -e "$repo/$f" ]] && git -C "$repo" add -N -- "$f"; done
    fi
    git -C "$repo" diff --ignore-submodules > "$patch.tmp"
    git -C "$repo" reset -q                      # drop the intent-to-add entries again (index = HEAD)
    if [[ -s "$patch.tmp" ]]; then mv "$patch.tmp" "$patch"; log "wrote ${patch#$HERE/}"; else rm -f "$patch.tmp" "$patch"; fi
  done < <(pairs)
  mkdir -p "$APPLIED"; rm -f "$APPLIED"/*.patch; cp "$PATCHES"/*.patch "$APPLIED"/ 2>/dev/null || true
  patch_hash > "$STAMP"
  exit 0
fi

# ---- clone / checkout ------------------------------------------------------------------------------------------------
if [[ ! -d "$DIR/.git" ]]; then
  log "clone $BITNET_URL -> $DIR"
  git clone -q --no-checkout "$BITNET_URL" "$DIR"
fi
if [[ "$(git -C "$DIR" rev-parse HEAD 2>/dev/null)" != "$BITNET_COMMIT" || -z "$(git -C "$DIR" ls-files -- .gitmodules)" ]]; then
  git -C "$DIR" cat-file -e "$BITNET_COMMIT^{commit}" 2>/dev/null || git -C "$DIR" fetch -q origin
  log "BitNet checkout ${BITNET_COMMIT:0:9}"
  git -C "$DIR" checkout -q -f "$BITNET_COMMIT"
  rm -f "$STAMP"
fi
if [[ "$(git -C "$LLAMA" rev-parse HEAD 2>/dev/null)" != "$LLAMA_COMMIT" ]]; then
  log "llama.cpp submodule ${LLAMA_COMMIT:0:9} (first time: a few minutes)"
  git -C "$DIR" submodule sync -q 3rdparty/llama.cpp
  # the submodule's own submodules (kompute, ...) are not needed
  git -C "$DIR" submodule update -q --init --force 3rdparty/llama.cpp
  [[ "$(git -C "$LLAMA" rev-parse HEAD)" == "$LLAMA_COMMIT" ]] || { echo "llama.cpp is not at $LLAMA_COMMIT" >&2; exit 1; }
  rm -f "$STAMP"
fi

# ---- patches ---------------------------------------------------------------------------------------------------------
want=$(patch_hash)
if [[ -f "$STAMP" && "$(cat "$STAMP")" == "$want" ]]; then
  log "$VERSION sources ready (patches already applied): $DIR"
  exit 0
fi

mkdir -p "$APPLIED"
while read -r repo patch; do
  old="$APPLIED/$(basename "$patch")"
  # reset the files touched by the previously applied and by the new patch, then apply the new one
  mapfile -t files < <( { [[ -f "$old" ]] && git -C "$repo" apply --numstat "$old"
                          [[ -f "$patch" ]] && git -C "$repo" apply --numstat "$patch"; } | cut -f3 | sort -u)
  for f in "${files[@]}"; do
    if git -C "$repo" cat-file -e "HEAD:$f" 2>/dev/null; then git -C "$repo" checkout -q HEAD -- "$f"
    else git -C "$repo" rm -q --cached --ignore-unmatch -- "$f"; rm -f "$repo/$f"; fi
  done
  rm -f "$old"
  [[ -f "$patch" ]] || continue
  git -C "$repo" apply --whitespace=nowarn "$patch"
  cp "$patch" "$old"
  log "applied ${patch#$HERE/}"
done < <(pairs)
echo "$want" > "$STAMP"
log "$VERSION sources ready: $DIR"
