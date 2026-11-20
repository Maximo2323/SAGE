#!/bin/sh
# macOS and Linux: double-click or run. The port name is found automatically.
cd "$(dirname "$0")" || exit 1
exec python3 dashboard.py --demo
