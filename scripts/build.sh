#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_DIR"

in_container() {
    [[ -n "${CROSS_PREFIX:-}" ]] || [[ -f /.dockerenv ]]
}

if ! in_container; then
    IMAGE_NAME="slicer-builder"

    echo "==> Building Docker image '${IMAGE_NAME}' ..."
    docker build -t "$IMAGE_NAME" -f "$PROJECT_DIR/scripts/Dockerfile" "$PROJECT_DIR"

    echo "==> Running cross-compilation inside container ..."
    docker run --rm \
        -v "$PROJECT_DIR":/build \
        "$IMAGE_NAME" \
        ./scripts/build.sh

    echo ""
    echo "==> Build complete. Artifacts:"
    ls -lh "$PROJECT_DIR/dist/slicer/"
    echo ""
    ls -lh "$PROJECT_DIR/dist/slicer-module.tar.gz"
    exit 0
fi

# === INSIDE CONTAINER ===
CC="${CC:-${CROSS_PREFIX}gcc}"
CXX="${CXX:-${CROSS_PREFIX}g++}"
BUNGEE_DIR="src/bungee"

rm -rf dist/slicer
mkdir -p dist/slicer build/bungee

# --- Phase 1: Build Bungee static library ---

echo "==> Compiling pffft ..."
$CC -O3 -fPIC -ffast-math -fno-finite-math-only -fno-exceptions \
    -c "$BUNGEE_DIR/submodules/pffft/pffft.c" -o build/bungee/pffft.o
$CC -O3 -fPIC -ffast-math -fno-finite-math-only -fno-exceptions \
    -c "$BUNGEE_DIR/submodules/pffft/fftpack.c" -o build/bungee/fftpack.o

echo "==> Compiling Bungee C++ sources ..."
for src in $BUNGEE_DIR/src/*.cpp; do
    obj="build/bungee/$(basename "$src" .cpp).o"
    $CXX -O3 -fPIC -std=c++20 -fwrapv \
        -I"$BUNGEE_DIR/submodules/eigen" \
        -I"$BUNGEE_DIR/submodules" \
        -I"$BUNGEE_DIR" \
        '-DBUNGEE_VISIBILITY=__attribute__((visibility("default")))' \
        -DBUNGEE_SELF_TEST=0 \
        -Deigen_assert=BUNGEE_ASSERT1 \
        -DEIGEN_DONT_PARALLELIZE=1 \
        '-DBUNGEE_VERSION="0.0.0"' \
        -c "$src" -o "$obj"
done

echo "==> Creating libbungee.a ..."
${CROSS_PREFIX}ar rcs build/bungee/libbungee.a build/bungee/*.o

# --- Phase 2: Build dsp.so ---

echo "==> Compiling dsp.so ..."
$CXX \
    -O3 -g -shared -fPIC -std=c++20 \
    -Isrc -I"$BUNGEE_DIR" \
    src/dsp/dsp.cpp \
    build/bungee/libbungee.a \
    -lm \
    -o dist/slicer/dsp.so

echo "==> Copying module.json, ui_chain.js, and help.json ..."
cp module.json dist/slicer/
cp ui_chain.js dist/slicer/
[ -f help.json ] && cp help.json dist/slicer/

# Create tarball
echo "==> Creating tarball ..."
tar -czf dist/slicer-module.tar.gz -C dist slicer

echo "==> Verifying dsp.so ..."
file dist/slicer/dsp.so

echo "==> Done."
