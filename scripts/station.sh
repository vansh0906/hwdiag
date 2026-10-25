#!/usr/bin/env bash
# Production station wrapper: runs a batch, archives the logs for this run,
# appends a one-line entry to the station history and gates on yield.
#
#   scripts/station.sh [-s SPEC] [-n UNITS] [-j THREADS] [-y MIN_YIELD] [-b BINARY]
#
# Exit status: 0 = yield gate passed, 1 = yield below gate, 2 = setup error.

set -euo pipefail

SPEC="specs/sensor_board.spec"
UNITS=100
THREADS=8
MIN_YIELD=80
BIN="./build/hwdiag"

usage() {
    sed -n '2,8p' "$0" | sed 's/^# \{0,1\}//'
}

while getopts "s:n:j:y:b:h" opt; do
    case "$opt" in
        s) SPEC="$OPTARG" ;;
        n) UNITS="$OPTARG" ;;
        j) THREADS="$OPTARG" ;;
        y) MIN_YIELD="$OPTARG" ;;
        b) BIN="$OPTARG" ;;
        h) usage; exit 0 ;;
        *) usage; exit 2 ;;
    esac
done

[[ -x "$BIN" ]] || { echo "error: $BIN not found - build first (cmake --build build)" >&2; exit 2; }
[[ -f "$SPEC" ]] || { echo "error: spec $SPEC not found" >&2; exit 2; }

RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
OUT="reports/runs/${RUN_ID}"
mkdir -p "$OUT"

echo "== hwdiag station run ${RUN_ID}: ${UNITS} units, ${THREADS} threads, gate ${MIN_YIELD}% =="

# Capture the binary's exit status even though its output is piped through tee.
set +e
"$BIN" batch --spec "$SPEC" --units "$UNITS" --threads "$THREADS" \
    --out "$OUT" --min-yield "$MIN_YIELD" | tee "$OUT/summary.txt"
status=${PIPESTATUS[0]}
set -e

# Quick cross-check straight from the CSV log with standard tools.
passed=$(awk -F, 'NR > 1 && $4 == "PASS"' "$OUT/test_log.csv" | wc -l | tr -d ' ')
total=$(( $(wc -l < "$OUT/test_log.csv") - 1 ))
echo "CSV check: ${passed}/${total} units passed"

tar -czf "reports/runs/${RUN_ID}.tar.gz" -C reports/runs "$RUN_ID"
printf '%s\tunits=%s\tpassed=%s\tgate=%s\texit=%s\n' "$RUN_ID" "$total" "$passed" "$MIN_YIELD" "$status" \
    >> reports/station_history.tsv
echo "Archived: reports/runs/${RUN_ID}.tar.gz"

exit "$status"
