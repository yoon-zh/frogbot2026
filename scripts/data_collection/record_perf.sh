#!/usr/bin/env bash
set -euo pipefail

OUTPUT_PATH="${1:-$HOME/frogbot2026/runtime-data/perf/tegrastats_$(date +%Y%m%d_%H%M%S).log}"
INTERVAL_MS="${TEGRASTATS_INTERVAL_MS:-1000}"

mkdir -p "$(dirname "$OUTPUT_PATH")"
echo "Recording tegrastats to $OUTPUT_PATH (interval=${INTERVAL_MS}ms)"
tegrastats --interval "$INTERVAL_MS" | tee "$OUTPUT_PATH"
