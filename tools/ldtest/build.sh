#!/bin/sh
# Builds the harness against a Main_MiSTer tree (default ../Main_MiSTer).
# Needs a Linux host with g++. Then:
#   python3 make_test_disc.py SOURCE.8bpp.dscmmi test_ld.dscmmi
#   ./harness test_ld.dscmmi
set -e
M=${1:-../Main_MiSTer}
g++ -std=gnu++14 -O1 -g -Wall -Wno-unused-result -Wno-class-memaccess -I$M -I$M/lib/libchdr/include -D_FILE_OFFSET_BITS=64 \
	-funsigned-char harness.cpp $M/support/megacd/megacdd.cpp $M/support/megacd/megald.cpp \
	$M/support/megacd/megald_mmi.cpp $M/support/megacd/megald_video.cpp $M/support/megacd/megald_audio.cpp $M/support/megacd/megald_crash.cpp -lpthread -o harness
echo built ./harness
