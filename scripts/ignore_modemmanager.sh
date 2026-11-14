#!/usr/bin/env bash
# Tell ModemManager to leave the Pico alone. It probes new /dev/ttyACM* ports with
# AT commands; those lines reach the firmware as non-JSON ("bad_json").
# Needs sudo. Run once on the Raspberry Pi, then replug the Pico.
set -euo pipefail
rule=/etc/udev/rules.d/99-lift-pico-no-modemmanager.rules
echo 'ATTRS{idVendor}=="2e8a", ENV{ID_MM_DEVICE_IGNORE}="1"' | sudo tee "$rule" >/dev/null
sudo udevadm control --reload-rules
echo "Installed $rule. Unplug and replug the Pico."
