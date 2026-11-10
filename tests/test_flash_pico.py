"""Check firmware helper refusal paths without touching real serial/GPIO."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('flash_helper', ROOT/'scripts/flash_pico.py')
flash = importlib.util.module_from_spec(spec)
spec.loader.exec_module(flash)


class FlashRefusalTests(unittest.TestCase):
    def test_no_hardware_access_without_motor_power_off_confirmation(self):
        result = subprocess.run([sys.executable, str(ROOT/'scripts/flash_pico.py')],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn('--motor-power-off', result.stderr)

    def test_multiple_or_missing_boards_require_explicit_selection(self):
        for ports in ([], ['/dev/a', '/dev/b']):
            with patch.object(flash.glob, 'glob', return_value=ports):
                with self.assertRaises(RuntimeError):
                    flash.find_port(None)
        self.assertEqual(flash.find_port('COM4'), 'COM4')

    def exercise_flash(self, corrupt=False):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            firmware = root/'new.py'
            firmware.write_bytes(b'# new verified firmware\n')
            board = {'main.py': b'# previous firmware\n', 'lift_calibration.json': b'{"calibration":"preserve"}'}
            calls = []

            def fake_run(argv, **kwargs):
                parts = argv[argv.index('connect')+2:]
                output = ''
                while parts:
                    split = parts.index('+') if '+' in parts else len(parts)
                    segment, parts = parts[:split], parts[split+1:]
                    calls.append(segment)
                    if segment[0] == 'exec':
                        if 'os.listdir()' in segment[1]:
                            output += json.dumps(list(board)) + '\n'
                        elif 'os.rename(' in segment[1]:
                            board['main.py'] = board.pop('main.next.py')
                    elif segment[:2] == ['fs', 'cp']:
                        source, dest = segment[2:]
                        if source.startswith(':'):
                            content = board[source[1:]]
                            if corrupt and source == ':main.next.py':
                                content = b'corrupt readback'
                            Path(dest).write_bytes(content)
                        else:
                            board[dest[1:]] = Path(source).read_bytes()
                    elif segment != ['reset']:
                        raise AssertionError(segment)
                return subprocess.CompletedProcess(argv, 0, stdout=output, stderr='')

            argv = ['flash_pico.py', '--motor-power-off', '--port', 'SIMULATED',
                    '--firmware', str(firmware), '--backup-dir', str(root/'backups')]
            with patch.object(sys, 'argv', argv), patch.object(flash.subprocess, 'run', side_effect=fake_run):
                if corrupt:
                    with self.assertRaisesRegex(RuntimeError, 'readback differs'):
                        flash.main()
                    self.assertEqual(board['main.py'], b'# previous firmware\n')
                    self.assertNotIn(['reset'], calls)
                else:
                    flash.main()
                    self.assertEqual(board['main.py'], firmware.read_bytes())
                    self.assertIn(['reset'], calls)
                self.assertEqual(board['lift_calibration.json'], b'{"calibration":"preserve"}')
                self.assertEqual(next((root/'backups').glob('*/main.py')).read_bytes(), b'# previous firmware\n')
                self.assertFalse(any('"cmd"' in str(call) for call in calls))

    def test_verified_flash_preserves_calibration_and_previous_firmware(self):
        self.exercise_flash()

    def test_corrupt_upload_readback_never_replaces_active_firmware(self):
        self.exercise_flash(corrupt=True)
