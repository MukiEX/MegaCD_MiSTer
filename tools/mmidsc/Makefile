# mmidsc: LaserActive MMI -> constant-bit-rate DSC converter
#   make                native build (Linux/macOS)
#   make windows        cross-compile mmidsc.exe (needs an x86_64-w64-mingw32 toolchain, e.g. llvm-mingw)
#   make test           run the codec round-trip tests
#   make rtl-test       run the FPGA decoder against the C reference (needs Verilator)
#   make bench-check    check dsc.c reproduces the reference bitstreams exactly, and time it
#   make CFLAGS="-O3 -march=native"   faster build for this machine only (about 10% more with AVX2)
CC      ?= cc
CFLAGS  ?= -O3 -Wall -Wextra
CPPFLAGS += -D_FILE_OFFSET_BITS=64 -D_GNU_SOURCE
LDLIBS  += -lpthread -lm
WINCC   ?= x86_64-w64-mingw32-gcc

SRC = src/main.c src/dsc.c src/dscv.c src/mmi.c src/util.c src/zipw.c
HDR = src/dsc.h src/dscv.h src/mmi.h src/util.h src/zipw.h src/qon.h src/qoi2.h

all: mmidsc

mmidsc: $(SRC) $(HDR)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $(SRC) $(LDLIBS)

windows: mmidsc.exe

mmidsc.exe: $(SRC) $(HDR)
	$(WINCC) $(CPPFLAGS) -O3 -static -o $@ $(SRC) -lpthread

dsctest: tests/dsctest.c src/dsc.c src/dsc.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ tests/dsctest.c src/dsc.c -lm

dscbench: tests/dscbench.c src/dsc.c src/dsc.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -Isrc -o $@ tests/dscbench.c src/dsc.c -lm

bench-check: dscbench
	./dscbench > dscbench.out && cmp dscbench.out tests/dscbench.golden && echo "bench-check: bit-exact"
	./dscbench quick > /dev/null

gen_test_qon: tests/gen_test_qon.c
	$(CC) -O2 -o $@ tests/gen_test_qon.c -lm

test: dsctest
	./dsctest

rtl-test:
	cd rtl/tb && ./build.sh && ./obj/Vdsc_decoder

clean:
	rm -f mmidsc mmidsc.exe dsctest dscbench dscbench.out gen_test_qon
	rm -rf rtl/tb/obj rtl/tb/build

.PHONY: all windows test bench-check rtl-test clean
