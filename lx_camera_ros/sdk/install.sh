#!/bin/bash
# Install MRDVS SDK to /opt/MRDVS (required before building lx_camera_ros)

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DST="/opt/MRDVS"

if [ "$(id -u)" != "0" ]; then
    echo "Run with sudo: sudo bash $0"
    exit 1
fi

rm -rf "$DST"
mkdir -p "$DST/lib"
cp -r "$SCRIPT_DIR/include" "$DST/"

ARCH=$(uname -m)
echo "Architecture: $ARCH"
if [[ "$ARCH" == *"aarch"* ]]; then
    cp "$SCRIPT_DIR/lib/linux_aarch64/"* "$DST/lib/"
elif [[ "$ARCH" == *"x86_64"* ]]; then
    cp "$SCRIPT_DIR/lib/linux_x64/"* "$DST/lib/"
else
    echo "Unsupported architecture: $ARCH"
    exit 1
fi

# Add to LD_LIBRARY_PATH if not present
BASHRC="$HOME/.bashrc"
if ! grep -q "/opt/MRDVS/lib" "$BASHRC" 2>/dev/null; then
    echo 'export LD_LIBRARY_PATH=/opt/MRDVS/lib/:$LD_LIBRARY_PATH' >> "$BASHRC"
fi

# Set socket buffer size
bash "$SCRIPT_DIR/set_socket_buffer_size.sh" 2>/dev/null || true

echo "SDK installed to $DST"
ls -la "$DST/lib/"
