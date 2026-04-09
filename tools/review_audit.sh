#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

echo "[audit] scanning tracked files for common anonymization leaks"
PATTERN='jin@example.com|jin@todo.todo|Jin Hyun Kim|kim2026|/home/jin|IEEE Robotics and Automation Letters \(RA-L\), 2026|Failure Analysis and Curvature-Aware Redesign'

if rg -n "$PATTERN" "$ROOT" \
    --glob '!tools/review_audit.sh' \
    --glob '!src/f1tenth_gym_ros/LICENSE' \
    --glob '!**/*.png' \
    --glob '!**/*.pgm'; then
    echo "[audit] findings detected"
    exit 1
fi

echo "[audit] checking recent git author metadata"
AUTHORS="$(git -C "$ROOT" log --format='%an <%ae>' -n 5 || true)"
printf '%s\n' "$AUTHORS"
if printf '%s\n' "$AUTHORS" | rg -qv '^anonymous <anonymous@example.com>$'; then
    echo "[audit] warning: existing commit history still contains non-anonymous author metadata"
fi

echo "[audit] done"
