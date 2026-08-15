#!/bin/bash
# Install detect.py as a boot service on Raspberry Pi OS.
# Run on the Pi:  bash systemd/install-wro-detect-service.sh
set -euo pipefail

if [ "$(id -u)" -eq 0 ]; then
  echo "Run this as user pi, not root (it will sudo when needed)."
  exit 1
fi

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
DEFAULT_DIR="$HOME/Documents/Test2_Round2"

if [ -f "$DEFAULT_DIR/detect.py" ]; then
  WRO_DIR="$DEFAULT_DIR"
else
  WRO_DIR="$REPO_DIR"
fi

if [ ! -f "$WRO_DIR/detect.py" ]; then
  echo "detect.py not found in $WRO_DIR"
  echo "Copy detect.py (and best_ncnn.onnx) into $DEFAULT_DIR or run this from the repo."
  exit 1
fi

if [ -x "$WRO_DIR/venv/bin/python" ]; then
  PY="$WRO_DIR/venv/bin/python"
else
  PY="$(command -v python3)"
  echo "No venv at $WRO_DIR/venv — using $PY"
fi

UNIT=/tmp/wro-detect.service
cat > "$UNIT" <<EOF
[Unit]
Description=WRO Future Engineers ONNX block detector
After=local-fs.target
StartLimitIntervalSec=0

[Service]
Type=simple
User=$(id -un)
Group=$(id -gn)
SupplementaryGroups=video dialout render plugdev
WorkingDirectory=$WRO_DIR
Environment=PYTHONUNBUFFERED=1
Environment=WRO_HEADLESS=1
EnvironmentFile=-/etc/default/wro-detect
ExecStart=$PY -u $WRO_DIR/detect.py
Restart=always
RestartSec=5
Nice=5

[Install]
WantedBy=multi-user.target
EOF

echo "Installing unit with:"
echo "  dir: $WRO_DIR"
echo "  py:  $PY"
sudo cp "$UNIT" /etc/systemd/system/wro-detect.service
sudo cp "$REPO_DIR/systemd/wro-detect.env.example" /etc/default/wro-detect
sudo usermod -aG video,dialout "$(id -un)" || true
sudo systemctl daemon-reload
sudo systemctl enable wro-detect.service
sudo systemctl restart wro-detect.service
sleep 1
sudo systemctl --no-pager --full status wro-detect.service || true
echo
echo "Logs:    journalctl -u wro-detect -f"
echo "Stop:    sudo systemctl stop wro-detect"
echo "Disable: sudo systemctl disable --now wro-detect"
echo "Do not also run: python detect.py   (one process only)"
