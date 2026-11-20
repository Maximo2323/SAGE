#!/bin/sh
cd "$(dirname "$0")" || exit 1
printf 'Open http://127.0.0.1:8766 in your browser. Press Ctrl+C to stop.\n'
# Find the ST-LINK serial port. The number changes with the USB socket.
PORT=$(ls /dev/cu.usbmodem* 2>/dev/null | head -n 1)
if [ -z "$PORT" ]; then
  printf 'No /dev/cu.usbmodem* port. Connect the Nucleo board and try again.\n'
  exit 1
fi
printf 'Port: %s\n' "$PORT"
exec python3 dashboard.py --port "$PORT" --placeholder-geometry
