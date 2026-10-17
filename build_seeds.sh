#!/usr/bin/env bash
# build_seeds.sh - build the MegaCD core with several Quartus fitter seeds,
# in parallel, each in its own copy of the project.
#
#   ./build_seeds.sh [options]
#
#   -s SEEDS   seeds to try: a list and/or ranges, e.g. "1,7,42" or "1-8"
#              (default: 1)
#   -j N       builds at the same time (default: 1)
#   -t N       CPU threads per build (default: all CPUs / N builds)
#   -o DIR     where the copies, logs and results go (default: ./seed_builds)
#   -f         stop the other builds as soon as one fits and meets timing
#   -c         delete DIR (through docker, since the container writes as root)
#   -h         this help
#
# Each seed is built in DIR/seed_<n>/, a copy of this folder without the
# Quartus databases, so parallel builds never share files. The copy's
# MegaCD.qsf gets "SEED <n>" and "NUM_PARALLEL_PROCESSORS <t>"; your own
# MegaCD.qsf is never touched. Each build runs in its own container, named
# megacd_seed_<n>, and logs to DIR/seed_<n>/build.log.
#
# A build that produces an .rbf has it copied to DIR/MegaCD_seed<n>.rbf.
# At the end, a table shows each seed's result, its RAM blocks and logic,
# and its worst setup slack (negative = timing not met).
#
# The Quartus image is theypsilon/quartus-lite-c5:17.0; set QUARTUS_IMAGE to
# use another. The volume is mounted with :z for SELinux.
#
# Note: the number of RAM blocks the design needs is decided by synthesis,
# not by the seed. Seeds help with placement, routing and timing; if the
# fitter reports more M10Ks needed than the chip has, every seed will fail
# the same way.

set -u

[ "${BASH_VERSINFO[0]}" -gt 5 ] || { [ "${BASH_VERSINFO[0]}" -eq 5 ] && [ "${BASH_VERSINFO[1]}" -ge 1 ]; } || { echo "needs bash 5.1 or newer" >&2; exit 2; }

IMAGE=${QUARTUS_IMAGE:-theypsilon/quartus-lite-c5:17.0}
DOCKER=${DOCKER:-docker}
SEEDS_ARG=1
JOBS=1
THREADS=
OUT=seed_builds
FIRST=0
CLEAN=0

usage() { sed -n '2,/^$/{s/^# \{0,1\}//;p}' "$0"; exit "${1:-0}"; }

while getopts "s:j:t:o:fch" opt; do
	case $opt in
	s) SEEDS_ARG=$OPTARG ;;
	j) JOBS=$OPTARG ;;
	t) THREADS=$OPTARG ;;
	o) OUT=$OPTARG ;;
	f) FIRST=1 ;;
	c) CLEAN=1 ;;
	h) usage 0 ;;
	*) usage 2 ;;
	esac
done

SRC=$(cd "$(dirname "$0")" && pwd)
case $OUT in /*) ;; *) OUT=$SRC/$OUT ;; esac

if [ "$CLEAN" = 1 ]; then
	[ -d "$OUT" ] || { echo "nothing to clean ($OUT)"; exit 0; }
	parent=$(dirname "$OUT")
	$DOCKER run --rm -v "$parent":/w:z "$IMAGE" rm -rf "/w/$(basename "$OUT")"
	echo "removed $OUT"
	exit 0
fi

# ---------------------------------------------------------------- seeds
SEEDS=()
IFS=',' read -ra parts <<< "$SEEDS_ARG"
for p in "${parts[@]}"; do
	p=${p// /}
	if [[ $p =~ ^([0-9]+)-([0-9]+)$ ]]; then
		for ((n = BASH_REMATCH[1]; n <= BASH_REMATCH[2]; n++)); do SEEDS+=("$n"); done
	elif [[ $p =~ ^[0-9]+$ ]]; then
		SEEDS+=("$p")
	elif [ -n "$p" ]; then
		echo "bad seed list: $SEEDS_ARG" >&2; exit 2
	fi
done
[ ${#SEEDS[@]} -gt 0 ] || { echo "no seeds" >&2; exit 2; }
[[ $JOBS =~ ^[1-9][0-9]*$ ]] || { echo "bad -j: $JOBS" >&2; exit 2; }
if [ -z "$THREADS" ]; then
	THREADS=$(( $(nproc) / JOBS ))
	[ "$THREADS" -ge 1 ] || THREADS=1
fi
[[ $THREADS =~ ^[1-9][0-9]*$ ]] || { echo "bad -t: $THREADS" >&2; exit 2; }
[ -f "$SRC/MegaCD.qsf" ] || { echo "MegaCD.qsf not found next to this script" >&2; exit 2; }

mkdir -p "$OUT"
RESULTS=$OUT/results.txt
: > "$RESULTS"
echo "seeds: ${SEEDS[*]}; $JOBS at a time, $THREADS threads each; image $IMAGE"
echo "work: $OUT"

# ------------------------------------------------------------ one build
build_one() {
	local seed=$1 dir=$OUT/seed_$1 name=megacd_seed_$1
	local log=$dir/build.log status

	$DOCKER rm -f "$name" >/dev/null 2>&1
	if [ -e "$dir" ]; then
		# an earlier run's copy (root-owned): remove it through docker
		$DOCKER run --rm -v "$OUT":/w:z "$IMAGE" rm -rf "/w/seed_$seed" >/dev/null 2>&1
	fi
	mkdir -p "$dir"
	# (anything else big in this folder gets copied too: keep old build
	#  folders out of it, or name them seed_builds*)
	# copy the project without databases, outputs and other seed builds
	tar -C "$SRC" -cf - \
		--exclude=./.git --exclude=./db --exclude=./incremental_db \
		--exclude=./output_files --exclude=./greybox_tmp --exclude=./simulation \
		--exclude=./releases --exclude="./$(basename "$OUT")" --exclude='./seed_builds*' \
		--exclude='./output_files*' --exclude='./db.*' --exclude='*.rbf' \
		. | tar -C "$dir" -xf -

	# this copy's seed and thread count (the .qsf may have CRLF line ends)
	sed -i -E \
		-e "s/^(set_global_assignment -name SEED )[0-9]+/\1$seed/" \
		-e "s/^(set_global_assignment -name NUM_PARALLEL_PROCESSORS )[A-Za-z0-9]+/\1$THREADS/" \
		"$dir/MegaCD.qsf"
	grep -q "name SEED $seed" "$dir/MegaCD.qsf" || \
		printf 'set_global_assignment -name SEED %s\n' "$seed" >> "$dir/MegaCD.qsf"

	echo "[seed $seed] building"
	$DOCKER run --rm --name "$name" -v "$dir":/project:z -w /project "$IMAGE" \
		quartus_sh --flow compile MegaCD.qpf > "$log" 2>&1
	status=$?

	# summarize
	local fs=$dir/output_files/MegaCD.fit.summary
	local sta=$dir/output_files/MegaCD.sta.summary
	local ram="-" logic="-" slack="-" result
	if [ -f "$fs" ]; then
		ram=$(sed -n 's/^Total RAM Blocks : *//p' "$fs" | tr -d '\r' | head -1)
		logic=$(sed -n 's/^Logic utilization (in ALMs) : *//p' "$fs" | tr -d '\r' | head -1)
	fi
	if [ -f "$sta" ]; then
		# worst setup slack over all clocks and corners
		slack=$(awk '/^Type *: .*Setup/ {s=1; next} s && /^Slack *:/ {print $3; s=0}' "$sta" | tr -d '\r' | sort -g | head -1)
	fi
	if [ -f "$dir/output_files/MegaCD.rbf" ]; then
		cp "$dir/output_files/MegaCD.rbf" "$OUT/MegaCD_seed$seed.rbf"
		if [ -n "$slack" ] && [ "$slack" != "-" ] && awk "BEGIN{exit !($slack < 0)}"; then
			result="built, timing NOT met"
		else
			result="built, timing met"
		fi
	elif grep -qs "^Fitter Status : Failed" "$fs" || grep -qiE "can't fit|cannot fit|Error \(17[0-9]+\)" "$log"; then
		result="did not fit"
	elif [ $status -eq 137 ] || [ $status -eq 143 ]; then
		result="stopped"
	else
		result="failed (exit $status)"
	fi

	# one line per seed; flock keeps parallel writers from interleaving
	{
		flock 9
		printf '%-6s %-24s %-26s %-30s %s\n' "$seed" "$result" "$ram" "$logic" "$slack" >> "$RESULTS"
	} 9>> "$RESULTS.lock"
	echo "[seed $seed] $result (RAM $ram, slack $slack)"
	[ "$result" = "built, timing met" ]
}

# ------------------------------------------------------ run them all
declare -A SEED_OF=()      # pid -> seed
stop_all() {
	for pid in "${!SEED_OF[@]}"; do $DOCKER kill "megacd_seed_${SEED_OF[$pid]}" >/dev/null 2>&1; done
}
trap 'echo; echo "interrupted: stopping builds"; stop_all; wait; exit 130' INT TERM

found=
# wait for one build to end; remember it if it fits and meets timing
reap_one() {
	local pid st
	wait -n -p pid
	st=$?
	[ -n "${pid:-}" ] || return
	[ $st -eq 0 ] && [ -z "$found" ] && found=${SEED_OF[$pid]}
	unset "SEED_OF[$pid]"
}

for seed in "${SEEDS[@]}"; do
	while [ ${#SEED_OF[@]} -ge "$JOBS" ]; do reap_one; done
	if [ "$FIRST" = 1 ] && [ -n "$found" ]; then break; fi
	build_one "$seed" &
	SEED_OF[$!]=$seed
done
while [ ${#SEED_OF[@]} -gt 0 ]; do
	if [ "$FIRST" = 1 ] && [ -n "$found" ] && [ -z "${stopped:-}" ]; then
		echo "seed $found fits and meets timing: stopping the others"
		stop_all
		stopped=1
	fi
	reap_one
done
rm -f "$RESULTS.lock"

echo
echo "seed   result                   RAM blocks                 logic                          worst setup slack"
sort -n "$RESULTS"
echo
echo "logs: $OUT/seed_<n>/build.log; .rbf files: $OUT/MegaCD_seed<n>.rbf"
grep -q "timing met" "$RESULTS"
