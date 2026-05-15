#!/bin/bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cp "$SCRIPT_DIR/uarts.service"        /etc/systemd/system/uarts.service
cp "$SCRIPT_DIR/uros.service"         /etc/systemd/system/uros.service
cp "$SCRIPT_DIR/henry_launch.service" /etc/systemd/system/henry_launch.service

systemctl daemon-reload
systemctl enable uarts.service
systemctl enable uros.service
systemctl enable henry_launch.service

echo "Done. Services will start on next boot."
echo "To start them now without rebooting:"
echo "  sudo systemctl start uarts.service"
echo "  sudo systemctl start uros.service"
echo "  sudo systemctl start henry_launch.service"
