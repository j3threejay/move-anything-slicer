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

rm -rf dist/slicer
mkdir -p dist/slicer

echo "==> Compiling dsp.so ..."
$CC \
    -O3 -g -shared -fPIC \
    -Isrc \
    src/dsp/dsp.c \
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
