#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

JOBS="${PX4_BUILD_JOBS:-$(nproc)}"

echo "[1/2] Removing old build products..."
make distclean

echo "[2/2] Building px4_fmu-v6x_default with ${JOBS} jobs..."
make -j"${JOBS}" px4_fmu-v6x_default

FIRMWARE="$ROOT_DIR/build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4"
if [[ ! -s "$FIRMWARE" ]]; then
    echo "ERROR: firmware package was not created: $FIRMWARE" >&2
    exit 1
fi

sha256sum "$FIRMWARE"
echo "Firmware ready: $FIRMWARE"
echo "Upload by QGroundControl, or connect Pixhawk 6X by USB and run:"
echo "  make px4_fmu-v6x_default upload"
