#!/bin/sh
# Build the Verilator testbench. Needs verilator (5.x) and gcc.
set -e
cd "$(dirname "$0")"
# the RTL itself lives in the core: rtl/LD
LD=../../../../rtl/LD
mkdir -p build
for f in dsc dscv mmi util; do
	${CC:-gcc} -O2 -D_FILE_OFFSET_BITS=64 -D_GNU_SOURCE $REFFLAGS -c ../../src/$f.c -o build/$f.o
done
ar rcs build/libref.a build/dsc.o build/dscv.o build/mmi.o build/util.o
${VERILATOR:-verilator} --cc --exe --build -j 8 -O3 -Wno-fatal -Wno-WIDTH -Wno-BLKSEQ $VFLAGS \
	--top-module dsc_decoder -Mdir obj $LD/dsc_decoder.sv tb_dsc_decoder.cpp \
	-LDFLAGS "$PWD/build/libref.a -lpthread -lm" -CFLAGS -O2 >/dev/null
echo "built tb/obj/Vdsc_decoder"
${VERILATOR:-verilator} --cc --exe --build -j 8 -O3 -Wno-fatal -Wno-WIDTH -Wno-BLKSEQ $VFLAGS \
	--top-module ld_tb_top -Mdir obj_mix ld_tb_top.sv $LD/ld_video.sv $LD/ld_mix.sv $LD/ld_fifo.sv $LD/dsc_decoder.sv tb_ld_mix.cpp \
	-LDFLAGS "$PWD/build/libref.a -lpthread -lm" -CFLAGS -O2 >/dev/null
echo "built tb/obj_mix/Vld_tb_top"
