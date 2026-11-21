#!/usr/bin/env bash
set -eo pipefail
exec python3 /app/scripts/start_detector.py "$@"
