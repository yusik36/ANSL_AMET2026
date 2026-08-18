#!/usr/bin/env bash
# One benchmark run: reset the car, bring the stack up, score a lap, tear down.
#
# Simulator time is metered, so the point of this script is that a run costs
# one command and always happens the same way -- same reset, same settle,
# same teardown -- which is what makes two runs comparable at all.
#
#   tools/YS_run.sh baseline
#   tools/YS_run.sh sign-flip  avoid_steer_sign:=-1.0
#   tools/YS_run.sh faster     forward_speed:=1.0 avoid_speed:=0.5
#
# Everything after the label is passed straight to ros2 launch. Overriding
# at launch rather than editing defaults is what keeps a run reproducible
# from its own label -- and it is the only thing that works, since the nodes
# read their parameters once at construction, so `ros2 param set` on a
# running node is silently ignored.
#
# Results append to tools/YS_runs.log, raw samples land in tools/runs/.
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/.." && pwd)"
API="${YS_API:-http://localhost/sim/api}"
RUNS="$HERE/runs"
LOG="$HERE/YS_runs.log"
LAUNCH="${YS_LAUNCH:-physicar_bringup real_autonomy_launch.py}"
SETTLE="${YS_SETTLE:-6}"      # seconds to let nodes come up before scoring
TIMEOUT="${YS_TIMEOUT:-150}"  # give up on a lap after this long

label="${1:-run}"; shift || true
overrides=("$@")          # name:=value pairs, forwarded to ros2 launch

command -v ros2 >/dev/null || { echo "ros2 not on PATH -- source install/setup.bash first" >&2; exit 1; }
curl -sf "$API/status" >/dev/null || { echo "simulator not reachable at $API" >&2; exit 1; }

mkdir -p "$RUNS"
stamp="$(date +%Y%m%d-%H%M%S)"
slug="$(printf '%s' "$label" | tr -c 'A-Za-z0-9._-' '-')"
csv="$RUNS/${stamp}_${slug}.csv"
out="$RUNS/${stamp}_${slug}.txt"

stack_pid=""
cleanup() {
  if [ -n "$stack_pid" ] && kill -0 "$stack_pid" 2>/dev/null; then
    kill "$stack_pid" 2>/dev/null
    sleep 2
    kill -9 "$stack_pid" 2>/dev/null
  fi
  # Leave the car stopped rather than coasting into scenery after teardown.
  ros2 topic pub --once /speed std_msgs/msg/Float64 '{data: 0.0}' >/dev/null 2>&1
}
trap cleanup EXIT INT TERM

echo "=== run: $label"
echo "--- reset"
curl -sf -X POST "$API/reset" -H 'Content-Type: application/json' -d '{}' >/dev/null \
  || echo "    (reset failed -- car may not be at the start line)"
sleep 2

echo "--- stack up: $LAUNCH ${overrides[*]:-}"
# ${overrides[@]+...} rather than ${overrides[@]:-}: the latter expands an
# empty array to one empty-string argument, which ros2 launch rejects.
# shellcheck disable=SC2086
ros2 launch $LAUNCH ${overrides[@]+"${overrides[@]}"} \
  > "$RUNS/${stamp}_${slug}.stack.log" 2>&1 &
stack_pid=$!
sleep "$SETTLE"

if ! kill -0 "$stack_pid" 2>/dev/null; then
  echo "!!! stack died on startup -- see $RUNS/${stamp}_${slug}.stack.log" >&2
  tail -20 "$RUNS/${stamp}_${slug}.stack.log" >&2
  exit 1
fi

echo "--- scoring (timeout ${TIMEOUT}s)"
python3 "$HERE/YS_bench.py" --base "$API" --laps 1 --label "$label" \
        --csv "$csv" --timeout "$TIMEOUT" | tee "$out"

{
  echo "### $stamp  $label"
  if [ ${#overrides[@]} -gt 0 ] && [ -n "${overrides[0]:-}" ]; then
    echo "    launch: ${overrides[*]}"
  fi
  sed 's/^/    /' "$out"
  echo
} >> "$LOG"

echo
echo "--- saved: $out"
echo "--- appended to: $LOG"
