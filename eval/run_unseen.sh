#!/usr/bin/env bash
# Unseen variants, live, without code edits: the localizer and the scenarios are unchanged; the variant inputs come
# from eval/variant_inputs.py on their own topics and the localizer is pointed at them by a parameter file.
#
#   eval/run_unseen.sh [gnss_outage_45s] [init_8m_90deg] [speed_plus20] [traffic_x2]   (default: all four)
#
# gnss_outage_45s  gnss_outage scenario, the localizer reads /gnss/fix_variant: raw fixes with a 45 s outage from 60 s
# init_8m_90deg    relocalization scenario, the localizer reads /initialpose_variant: start + 8 m at 45 deg, +90 deg
# speed_plus20     nominal at Speed 9.6 and TurnSpeed 9.6 m/s (+20 %), .xosc parameters through run.sh
# traffic_x2       dense_traffic + 40 more cars (eval/spawn_traffic.py, Traffic Manager autopilot)
# Recorded with --outputs-only (wall clock, no /points_raw); results in eval/results/unseen_<variant>/.
set -u
REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO"
HOST_RUNS="$(grep -E '^BAGS_DIR=' .env | cut -d= -f2)/loc_runs"
VARIANTS=("$@"); [ ${#VARIANTS[@]} -eq 0 ] && VARIANTS=(gnss_outage_45s init_8m_90deg speed_plus20 traffic_x2)

params() {  # params <name> <key> <value>...: localization.yaml with top-level keys replaced -> /bags/loc_runs/<name>.yaml
  local name=$1; shift
  cp localization/config/localization.yaml "$HOST_RUNS/$name.yaml"
  while [ $# -gt 0 ]; do sed -i "s|^    $1: .*|    $1: $2|" "$HOST_RUNS/$name.yaml"; shift 2; done
}
helper_start() { docker compose exec -d loc bash -c "source /opt/ros/humble/setup.bash; cd /ws/src/brightdrive_localization; \
  exec $1 > /bags/loc_runs/$2.helper.log 2>&1"; }
helper_stop() { docker compose exec -T loc pkill -INT -f "[v]ariant_inputs.py" > /dev/null 2>&1; \
  docker ps -q --filter name=^bd_traffic$ | xargs -r docker kill -s INT > /dev/null; }

for v in "${VARIANTS[@]}"; do
  name=unseen_$v
  echo "=== $v $(date +%T)"
  case $v in
    gnss_outage_45s)
      params "$name" gnss_topic /gnss/fix_variant
      docker compose up -d loc > /dev/null
      helper_start "python3 -u eval/variant_inputs.py --outage-start 60 --outage-duration 45" "$name"
      EXTRA_TOPICS="/gnss/fix_variant /eval/gnss_outage_variant" METRICS_ARGS="--outage-topic /eval/gnss_outage_variant" \
        eval/run_live.sh gnss_outage "$name" --outputs-only --node-arg "params_file:=/bags/loc_runs/$name.yaml" --speed 1
      helper_stop ;;
    init_8m_90deg)
      params "$name" initialpose_topic /initialpose_variant
      docker compose up -d loc > /dev/null
      helper_start "python3 -u eval/variant_inputs.py --init-scenario relocalization --init-offset-m 8 --init-dir-deg 45 \
        --init-heading-deg 90" "$name"
      EXTRA_TOPICS="/initialpose_variant /eval/init_offset_variant" \
      METRICS_ARGS="--initialpose-topic /initialpose_variant --init-offset-topic /eval/init_offset_variant" \
        eval/run_live.sh relocalization "$name" --outputs-only --node-arg "params_file:=/bags/loc_runs/$name.yaml" --speed 1
      helper_stop ;;
    speed_plus20)
      eval/run_live.sh nominal "$name" --outputs-only --speed 1 --param Speed:9.6 --param TurnSpeed:9.6 ;;
    traffic_x2)
      ( until docker ps -q --filter name=^bd_run$ | grep -q .; do sleep 1; done; sleep 15
        docker compose run -T --rm --no-deps --name bd_traffic sim python3 -u /ws/src/brightdrive_localization/eval/spawn_traffic.py --count 40 \
          > "$HOST_RUNS/$name.traffic.log" 2>&1 ) &
      eval/run_live.sh dense_traffic "$name" --outputs-only --speed 1
      helper_stop; wait ;;
    *) echo "unknown variant $v" ;;
  esac
done
