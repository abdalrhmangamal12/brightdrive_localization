#!/usr/bin/env bash
# One command per scenario (task PDF: "Each scenario shall be reproducible with a documented command").
#
#   sim/scripts/run.sh                       list the scenarios (the task's required set first, extras after)
#   sim/scripts/run.sh nominal               run it at real time (carla-server started if needed)
#   sim/scripts/run.sh nominal --record      ... and record every topic into BAGS_DIR (.env) as mcap
#   sim/scripts/run.sh nominal --speed 0     as fast as possible (quick checks); --speed 0.5 while recording
#   sim/scripts/run.sh nominal --param Speed:12 --param TimeLimit:60    change .xosc parameters for this run
#   sim/scripts/run.sh nominal --tick 0.05   world tick for this run (default: tick_s in sim/config/sim.yaml)
#   sim/scripts/run.sh stop                  stop every container (docker compose down)
#
# Nodes started elsewhere (estimator, RViz) see the topics live: same ROS domain, host network.

# Repo root (this file is sim/scripts/run.sh); docker compose reads ./.env there.
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$REPO"
SIM_YAML=sim/config/sim.yaml
RUN_INSIDE=/ws/src/brightdrive_localization/sim/scripts/run_inside.sh

# No argument: the scenario names from sim.yaml, required (`scenarios:`) then `extras:`.
if [ $# -eq 0 ]; then
  names() { sed -n "/^$1:/,/^[a-z]/p" "$SIM_YAML" | grep -E '^  [a-z_]+:' | sed 's/:.*//; s/^ *//'; }
  echo "required (the task's scenarios, in order):"; names scenarios | sed 's/^/  /'
  echo "extras:"; names extras | sed 's/^/  /'
  exit 0
fi
if [ "$1" = stop ]; then docker compose down; exit 0; fi

# Options.
SCENARIO=$1; shift
SPEED=1; RECORD=0; EXTRA=()
while [ $# -gt 0 ]; do
  case "$1" in
    --record) RECORD=1 ;;
    --speed) SPEED=$2; shift ;;
    --tick) EXTRA+=(--tick "$2"); shift ;;
    --param) EXTRA+=(--param "$2"); shift ;;
    *) echo "unknown option $1"; exit 1 ;;
  esac
  shift
done

# The .xosc files are generated from sim.yaml, sensors.yaml and the routes: regenerate them when any of those is
# newer than this scenario's file (or the file is missing), so an edit is never run with a stale scenario.
XOSC=sim/scenarios/xosc/$SCENARIO.xosc
if [ ! -f "$XOSC" ] || [ -n "$(find "$SIM_YAML" sim/config/sensors.yaml sim/scenarios/routes -name '*.yaml' -newer "$XOSC")" ]; then
  python3 sim/scenarios/make_scenarios.py > /dev/null || { echo "make_scenarios.py failed"; exit 1; }
  echo "regenerated .xosc files"
fi

# The server, if it is not running yet, then wait until its RPC port answers (healthy).
if [ -z "$(docker compose ps -q --status running carla-server)" ]; then
  echo "starting carla-server ..."
  docker compose up -d carla-server
fi
until [ "$(docker inspect -f '{{.State.Health.Status}}' brightdrive-carla-server-1 2>/dev/null)" = healthy ]; do sleep 2; done

# Only one scenario can drive the world at a time: stop a run that is still going (e.g. its terminal was closed).
if [ -n "$(docker ps -aq --filter name=^bd_run$)" ]; then
  echo "stopping the previous run ..."; docker rm -f bd_run > /dev/null
fi
# Everything else runs in one sim container (-T: no terminal inside, output still shows here).
# Ctrl-C must not reach docker compose: it would remove the container before the run has cleaned up (cars left in
# the world, bag not closed). So compose gets its own session (setsid), and on Ctrl-C we pass it to the container
# ourselves; run_inside.sh then stops ScenarioRunner first and the rest after it.
trap 'echo "stopping (cleaning up) ..."; docker kill -s INT bd_run > /dev/null 2>&1' INT TERM
setsid docker compose run -T --rm --no-deps --name bd_run sim \
  bash "$RUN_INSIDE" "$SCENARIO" "$SPEED" "$RECORD" "${EXTRA[@]}" &
RUN=$!
# wait returns early when Ctrl-C arrives: keep waiting until the container has finished.
while kill -0 "$RUN" 2>/dev/null; do wait "$RUN"; STATUS=$?; done
exit $STATUS
