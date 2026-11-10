#!/usr/bin/env bash
# Install UI files only. Does not open serial, flash Pico, or start the motor/UI.
set -euo pipefail
repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
prefix="$HOME/lift"
use_venv=1
while (($#)); do
    case "$1" in
        --prefix) prefix=${2:?--prefix requires a path}; shift 2 ;;
        --no-venv) use_venv=0; shift ;;
        -h|--help)
            echo 'Usage: scripts/install_pi.sh [--prefix ~/lift] [--no-venv]'
            exit 0 ;;
        *) echo "Unknown option: $1" >&2; exit 2 ;;
    esac
done
python3 -c 'import tkinter' || {
    echo 'Install python3-tk and python3-venv first; see README.md.' >&2
    exit 1
}
mkdir -p "$prefix"
if ((use_venv)); then
    python3 -m venv --system-site-packages "$prefix/.venv"
    "$prefix/.venv/bin/python" -m pip install -r "$repo_root/requirements.txt"
else
    python3 -c 'import serial' || {
        echo '--no-venv requires pyserial already installed.' >&2; exit 1;
    }
fi
backup=$(mktemp -d "$prefix/backups.install.XXXXXXXX")
for rel in start_position.sh position/lift_position.py position/lift_joystick.py position/firmware/main.py; do
    if [[ -f "$prefix/$rel" ]]; then
        mkdir -p "$backup/$(dirname "$rel")"
        cp -- "$prefix/$rel" "$backup/$rel"
    fi
done
mkdir -p "$prefix/position/firmware"
install -m 644 "$repo_root/raspberry_pi/lift/position/lift_position.py" "$prefix/position/"
install -m 644 "$repo_root/raspberry_pi/lift/position/lift_joystick.py" "$prefix/position/"
install -m 644 "$repo_root/pico/main.py" "$prefix/position/firmware/main.py"
install -m 755 "$repo_root/raspberry_pi/lift/start_position.sh" "$prefix/start_position.sh"
echo "Installed UI to $prefix; prior files: $backup"
echo 'Pico firmware and calibration were not changed. UI was not started.'
