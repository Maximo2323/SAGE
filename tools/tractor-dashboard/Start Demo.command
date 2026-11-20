#!/bin/sh
cd "$(dirname "$0")" || exit 1
printf 'Open http://127.0.0.1:8766 on browser. Press Ctrl+C to stop.\n'
exec python3 dashboard.py --demo
