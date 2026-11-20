#!/bin/sh
cd "$(dirname "$0")" || exit 1
printf 'Open http://127.0.0.1:8766 in your browser. Press Ctrl+C to stop.\n'
# Pair the HC-06 in the Bluetooth settings of the Mac first. The port name comes from the module name.
PORT=$(ls /dev/cu.HC-0* 2>/dev/null | head -n 1)
if [ -z "$PORT" ]; then
  printf 'No /dev/cu.HC-0* port. Pair the module, or start: python3 dashboard.py --port /dev/cu.<name>\n'
  exit 1
fi
printf 'Port: %s\n' "$PORT"
exec python3 dashboard.py --port "$PORT" --placeholder-geometry
