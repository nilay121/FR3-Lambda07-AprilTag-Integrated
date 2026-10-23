#!/usr/bin/env bash
set -euo pipefail

RULE_FILE="/etc/udev/rules.d/10-force-dimension-lambda.rules"
RULE='SUBSYSTEM=="usb", ATTR{idVendor}=="1451", ATTR{idProduct}=="040c", MODE="0660", GROUP="plugdev", TAG+="uaccess"'

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run with sudo:"
  echo "  sudo $0"
  exit 1
fi

install -d -m 0755 /etc/udev/rules.d
printf '%s\n' "${RULE}" > "${RULE_FILE}"
udevadm control --reload-rules
udevadm trigger

echo "Installed ${RULE_FILE}"
echo "Unplug and reconnect the Lambda.07 before testing without sudo."

