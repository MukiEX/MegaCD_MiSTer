#!/bin/sh
# Build mmidsc.wasm with plain clang (wasm32 target) and wasm-ld; no
# Emscripten or WASI needed.
set -e
cd "$(dirname "$0")"
${CLANG:-clang} --target=wasm32 -O3 -flto -nostdlib -ffreestanding -fno-builtin-malloc \
	-mbulk-memory -msimd128 -isystem shim -I. \
	-Wall -Wno-unused-function -Wno-unused-variable -Wno-unused-parameter \
	-Wl,--no-entry -Wl,--export=__heap_base -Wl,--initial-memory=33554432 -Wl,--lto-O3 \
	-o mmidsc.wasm mmidsc_wasm.c libc.c
ls -l mmidsc.wasm
