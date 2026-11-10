#!/usr/bin/env python3
"""Back up, verify, and replace Pico main.py. Never arm, jog, or move the lift."""
import argparse
from datetime import datetime
import glob
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
PORT_PATTERN = '/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_*-if00'
EXTEND_WATCHDOG = 'import machine; machine.WDT(timeout=8000).feed()'


def find_port(explicit):
    if explicit:
        return explicit
    ports = glob.glob(PORT_PATTERN)
    if len(ports) != 1:
        raise RuntimeError('Specify --port: expected exactly one MicroPython Pico, found %d' % len(ports))
    return ports[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', help='Linux by-id path, /dev/ttyACM0, or Windows COM port')
    parser.add_argument('--firmware', type=Path, default=ROOT/'pico/main.py')
    parser.add_argument('--backup-dir', type=Path, default=ROOT/'backups')
    parser.add_argument('--motor-power-off', action='store_true',
                        help='Confirm motor 24V is OFF; keep Pi/Pico USB powered and close UI')
    args = parser.parse_args()
    if not args.motor_power_off:
        parser.error('Turn motor 24V OFF, close the UI, then pass --motor-power-off. USB stays on.')
    firmware = args.firmware.resolve()
    if not firmware.is_file():
        parser.error('Firmware file not found: %s' % firmware)
    port = find_port(args.port)
    runner = shutil.which('mpremote')
    command = [runner] if runner else [sys.executable, '-m', 'mpremote']

    def remote(*parts):
        return subprocess.run(command + ['connect', port] + list(parts),
                              check=True, text=True, capture_output=True, timeout=45).stdout

    args.backup_dir.mkdir(parents=True, exist_ok=True)
    backup = Path(tempfile.mkdtemp(prefix=datetime.now().strftime('%Y%m%d-%H%M%S-'),
                                   dir=str(args.backup_dir.resolve())))
    print('Backup:', backup, flush=True)
    # Raise the existing watchdog only while performing the verified replacement.
    probe = remote('exec', EXTEND_WATCHDOG + '; import os,json; print(json.dumps(os.listdir()))')
    files = None
    for line in probe.splitlines():
        try:
            value = json.loads(line)
        except ValueError:
            continue
        if isinstance(value, list):
            files = value
    if files is None:
        raise RuntimeError('Could not read Pico file list; active firmware was not replaced')
    parts = ['exec', EXTEND_WATCHDOG]
    for name in ('main.py', 'lift_calibration.json'):
        if name in files:
            parts += ['+', 'fs', 'cp', ':' + name, str(backup/name)]
    parts += ['+', 'fs', 'cp', str(firmware), ':main.next.py',
              '+', 'fs', 'cp', ':main.next.py', str(backup/'candidate-verify.py')]
    remote(*parts)
    if (backup/'candidate-verify.py').read_bytes() != firmware.read_bytes():
        raise RuntimeError('Upload readback differs; active main.py was not replaced')
    remote('exec', EXTEND_WATCHDOG + '; import os; os.rename("main.next.py", "main.py")',
           '+', 'fs', 'cp', ':main.py', str(backup/'active-verify.py'))
    if (backup/'active-verify.py').read_bytes() != firmware.read_bytes():
        raise RuntimeError('Active firmware verification failed; keep motor power OFF')
    remote('reset')
    print('Verified and restarted Pico. No motor command was sent.')
    print('Calibration file was preserved. Reconfirm the physical lower reference before use.')


if __name__ == '__main__':
    try:
        main()
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        print('Flash failed:', error, file=sys.stderr)
        if isinstance(error, subprocess.CalledProcessError) and error.stderr:
            print(error.stderr, file=sys.stderr)
        sys.exit(1)
