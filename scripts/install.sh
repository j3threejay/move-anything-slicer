#!/usr/bin/env bash
set -euo pipefail

MOVE_HOST="${MOVE_HOST:-move.local}"
DEST="/data/UserData/move-anything/modules/sound_generators/slicer"

SSH_OPTS="-o ConnectTimeout=10 -o StrictHostKeyChecking=no"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
DIST_DIR="$PROJECT_DIR/dist/slicer"

if [[ ! -d "$DIST_DIR" ]]; then
    echo "ERROR: dist/slicer/ not found. Run scripts/build.sh first."
    exit 1
fi

echo "==> Deploying to ${MOVE_HOST} ..."

echo "==> Creating destination directory ..."
ssh $SSH_OPTS "ableton@${MOVE_HOST}" "mkdir -p ${DEST}"

echo "==> Copying dsp.so ..."
scp $SSH_OPTS "$DIST_DIR/dsp.so" "ableton@${MOVE_HOST}:${DEST}/dsp.so"

echo "==> Copying module.json ..."
scp $SSH_OPTS "$DIST_DIR/module.json" "ableton@${MOVE_HOST}:${DEST}/module.json"

echo "==> Copying ui_chain.js ..."
scp $SSH_OPTS "$DIST_DIR/ui_chain.js" "ableton@${MOVE_HOST}:${DEST}/ui_chain.js"

echo "==> Restarting Move service ..."
ssh $SSH_OPTS "root@${MOVE_HOST}" "/etc/init.d/move stop; sleep 1; /etc/init.d/move start"

echo "==> Deploy complete."
