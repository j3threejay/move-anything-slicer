#!/usr/bin/env bash
# Host-native test run. Builds Bungee and the DSP for this machine (not the
# Move) so the detector and parameter API can be exercised without hardware.
#
#   ./tests/run.sh            # generate fixtures, build, run everything
#   ./tests/run.sh <file>...  # also report detected BPM for real samples
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD="$PROJECT_DIR/build/tests"
BUNGEE="$PROJECT_DIR/src/bungee"

mkdir -p "$BUILD/obj" "$BUILD/fixtures"

# --- Bungee, built for the host ---
if [[ ! -f "$BUILD/obj/libbungee.a" ]]; then
    echo "==> Building Bungee for host ..."
    cc -O2 -fPIC -c "$BUNGEE/submodules/pffft/pffft.c"   -o "$BUILD/obj/pffft.o"
    cc -O2 -fPIC -c "$BUNGEE/submodules/pffft/fftpack.c" -o "$BUILD/obj/fftpack.o"
    for src in "$BUNGEE"/src/*.cpp; do
        c++ -O2 -fPIC -std=c++20 \
            -I"$BUNGEE/submodules/eigen" -I"$BUNGEE/submodules" -I"$BUNGEE" \
            '-DBUNGEE_VISIBILITY=__attribute__((visibility("default")))' \
            -DBUNGEE_SELF_TEST=0 -Deigen_assert=BUNGEE_ASSERT1 \
            -DEIGEN_DONT_PARALLELIZE=1 '-DBUNGEE_VERSION="0.0.0"' \
            -c "$src" -o "$BUILD/obj/$(basename "$src" .cpp).o"
    done
    ar rcs "$BUILD/obj/libbungee.a" "$BUILD/obj"/*.o
fi

build_test() {
    c++ -O2 -std=c++20 -I"$PROJECT_DIR/src" -I"$BUNGEE" \
        "$SCRIPT_DIR/$1.cpp" "$BUILD/obj/libbungee.a" -o "$BUILD/$1"
}

echo "==> Building tests ..."
build_test feature_test
build_test bpm_test

echo "==> Generating fixtures ..."
( cd "$BUILD/fixtures" && python3 "$SCRIPT_DIR/gen_fixtures.py" >/dev/null )

echo ""
echo "==> Feature tests"
( cd "$BUILD/fixtures" && "$BUILD/feature_test" )

echo ""
echo "==> BPM detection fixtures"
# beat_090 is a known miss: a 90 BPM loop with straight 8th hats is genuinely
# ambiguous against 180, which is what the Sync page's ratio knob is for.
cd "$BUILD/fixtures"
pass=0; total=0
for case in beat_090:90 beat_120:120 beat_140:140 beat_174:174 \
            beat_100_nohat:100 beat_128_swing:128 beat_160_2bar:160 melodic_110:110; do
    name="${case%%:*}"; want="${case##*:}"
    total=$((total + 1))
    if "$BUILD/bpm_test" "$name.wav" "$want" | head -1 | grep -q " OK$"; then
        pass=$((pass + 1))
    fi
done
echo "    $pass/$total exact (expected: 7/8)"

if [[ $# -gt 0 ]]; then
    echo ""
    echo "==> Real samples"
    for f in "$@"; do "$BUILD/bpm_test" "$f" | head -1; done
fi
