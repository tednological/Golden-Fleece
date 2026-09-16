#!/usr/bin/env bash
# Shared runner for the radar presets (run_preset_a.sh, run_preset_b.sh, run_preset_c.sh).
#
#   tools/run_preset.sh NAME "DESCRIPTION" [override ...] [-- [--thof N] run_pipeline.py args ...]
#
# --thof N (or --thof=N) replaces the preset's threshold offset for this run: an integer 10..60 dB (datasheet
# range; lower = more sensitive = longer range, more false and clutter targets). It is applied after the preset's
# own THOF, so it wins, and the change list printed below shows the value that was used.
#
# Builds /tmp/goldenfleece_preset_NAME from config/ plus the overrides (tools/config_overlay.py validates it),
# stops the goldenfleece service (it owns the radar port and the IMU), runs the pipeline in the foreground
# with that config, and starts the service again when the pipeline exits (Ctrl-C included).
# The field web app runs alongside on port 8080 with the SAME preset config: the goldenfleece-web service (which
# replays with the repo config) is stopped for the run and started again afterwards, so the page draws the tracks
# the preset pipeline computed (presets change the blind band, which changes tracking).
#
# Environment: PRESET_DRY_RUN=1 builds and validates the config, then exits without touching any service.
#              PRESET_WEB_PORT (default 8080) changes the web app's port.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="$ROOT/.venv/bin/python"
SERVICE=goldenfleece
WEB_SERVICE=goldenfleece-web
WEB_PORT="${PRESET_WEB_PORT:-8080}"

if [[ $# -lt 2 ]]; then
    echo "usage: tools/run_preset.sh NAME \"DESCRIPTION\" [override ...] [-- run_pipeline.py args ...]" >&2
    exit 2
fi
NAME="$1"
DESCRIPTION="$2"
shift 2
OVERRIDES=()
while [[ $# -gt 0 && "$1" != "--" ]]; do
    OVERRIDES+=("$1")
    shift
done
[[ "${1:-}" == "--" ]] && shift
PIPELINE_ARGS=()
THOF=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --thof)
            [[ $# -ge 2 ]] || { echo "--thof needs a value (10..60)" >&2; exit 2; }
            THOF="$2"
            shift 2
            ;;
        --thof=*)
            THOF="${1#--thof=}"
            shift
            ;;
        *)
            PIPELINE_ARGS+=("$1")
            shift
            ;;
    esac
done
if [[ -n "$THOF" ]]; then
    if ! [[ "$THOF" =~ ^[0-9]+$ ]] || (( 10#$THOF < 10 || 10#$THOF > 60 )); then
        echo "--thof must be an integer from 10 to 60 dB (got '$THOF')" >&2
        exit 2
    fi
    THOF=$((10#$THOF))
    OVERRIDES+=("radar.params.THOF=${THOF}")
    DESCRIPTION="${DESCRIPTION}, THOF override ${THOF} dB"
fi
CONFIG_DIR="/tmp/goldenfleece_preset_${NAME}"

echo "=== Golden Fleece preset ${NAME}: ${DESCRIPTION} ==="
"$PY" "$ROOT/tools/config_overlay.py" "$CONFIG_DIR" "${OVERRIDES[@]}"

rrai="$("$PY" -c 'import sys, yaml; print(yaml.safe_load(open(sys.argv[1]))["params"]["RRAI"])' "$CONFIG_DIR/radar.yaml")"
if [[ "$rrai" == "3" ]]; then
    cat <<'EOF'
  WARNING: RRAI 3 (100 m, 1 m range resolution). The tracker is still tuned for 30 cm resolution
  (tracker.sigma_r_meas_m, tracker.consistency_tol_mps), so real cars may be marked inconsistent and not warn.
  Use this preset for bench tests and recordings, not as a safety device on a ride.
EOF
fi

if [[ "${PRESET_DRY_RUN:-0}" == "1" ]]; then
    echo "  dry run: not starting the pipeline"
    exit 0
fi

restart_service=0
restart_web_service=0
web_child=""
restore() {
    if [[ -n "$web_child" ]] && kill -0 "$web_child" 2>/dev/null; then
        echo "=== stopping the preset web app ==="
        kill -TERM "$web_child" 2>/dev/null || true
        wait "$web_child" 2>/dev/null || true
    fi
    if [[ "$restart_service" == "1" ]]; then
        echo "=== starting the ${SERVICE} service again (repo config) ==="
        sudo systemctl start "$SERVICE" || echo "  could not start ${SERVICE}: run 'sudo systemctl start ${SERVICE}'" >&2
    fi
    if [[ "$restart_web_service" == "1" ]]; then
        echo "=== starting the ${WEB_SERVICE} service again (repo config) ==="
        sudo systemctl start "$WEB_SERVICE" || echo "  could not start ${WEB_SERVICE}: run 'sudo systemctl start ${WEB_SERVICE}'" >&2
    fi
}
trap restore EXIT

if systemctl is-active --quiet "$SERVICE"; then
    echo "=== stopping the ${SERVICE} service so the preset can use the radar and IMU ==="
    sudo systemctl stop "$SERVICE"
    restart_service=1
fi
if systemctl is-active --quiet "$WEB_SERVICE"; then
    echo "=== stopping the ${WEB_SERVICE} service so the web app can run with the preset config ==="
    sudo systemctl stop "$WEB_SERVICE"
    restart_web_service=1
fi

# Two pipelines on one radar corrupt each other's serial exchanges, and the second cannot claim the IMU pins.
if others="$(pgrep -af '^[^ ]*python[^ ]* [^ ]*tools/run_pipeline\.py')"; then
    echo "  another pipeline is still running; stop it first:" >&2
    echo "$others" | sed 's/^/    /' >&2
    exit 1
fi
if others="$(pgrep -af '^[^ ]*python[^ ]* [^ ]*tools/web_app\.py')"; then
    echo "  another web app is still running (it would hold port ${WEB_PORT}); stop it first:" >&2
    echo "$others" | sed 's/^/    /' >&2
    exit 1
fi

echo "=== starting the web app on port ${WEB_PORT} with the preset config (log: ${CONFIG_DIR}/web_app.log) ==="
"$PY" "$ROOT/tools/web_app.py" --config "$CONFIG_DIR" --port "$WEB_PORT" > "$CONFIG_DIR/web_app.log" 2>&1 &
web_child=$!
web_ok=0
for _ in $(seq 50); do
    if curl -s --max-time 1 -o /dev/null "http://127.0.0.1:${WEB_PORT}/state"; then web_ok=1; break; fi
    kill -0 "$web_child" 2>/dev/null || break
    sleep 0.1
done
if [[ "$web_ok" == "1" ]]; then
    echo "  open http://$(hostname).local:${WEB_PORT}/  or  http://$(hostname -I | awk '{print $1}'):${WEB_PORT}/"
else
    echo "  WARNING: the web app did not come up; the pipeline still runs and records. Log:" >&2
    tail -5 "$CONFIG_DIR/web_app.log" | sed 's/^/    /' >&2
fi

echo "=== running the pipeline (Ctrl-C to stop); the recording goes to the usual recordings directory ==="
# The pipeline runs as a child so this script outlives it: Ctrl-C or kill is passed on, the pipeline shuts down
# cleanly, and only then does the EXIT trap start the service again.
"$PY" "$ROOT/tools/run_pipeline.py" --config "$CONFIG_DIR" "${PIPELINE_ARGS[@]}" &
child=$!
trap 'kill -INT "$child" 2>/dev/null || true' INT TERM HUP
status=0
while kill -0 "$child" 2>/dev/null; do
    wait "$child" && status=0 || status=$?
done
trap - INT TERM HUP
exit "$status"
