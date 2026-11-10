#!/bin/sh
set -eu
lift_home=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
if [ -n "${LIFT_PYTHON:-}" ]; then
    python_bin=$LIFT_PYTHON
elif [ -x "$lift_home/.venv/bin/python" ]; then
    python_bin=$lift_home/.venv/bin/python
else
    python_bin=python3
fi
exec "$python_bin" "$lift_home/position/lift_position.py" "$@"
