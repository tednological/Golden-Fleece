#!/usr/bin/env bash
# Preset A: 100 m range scale, stock sensitivity.
# Cars the radar already sees beyond 30 m are measured at their true range instead of folding back to a false
# short range. Sensitivity is unchanged; range resolution drops from 30 cm to 1 m.
# Also starts the field web app on port 8080 with this preset's config (http://raspberrypi.local:8080/).
# Extra arguments go to tools/run_pipeline.py (e.g. --no-imu, --no-record).
exec "$(dirname "$0")/tools/run_preset.sh" A "100 m range, stock sensitivity" \
    radar.params.RRAI=3 \
    radar.params.RSPI=3 \
    radar.params.THOF=30 \
    radar.params.DEDI=2 \
    radar.params.MISP=0 \
    radar.params.MASP=100 \
    -- "$@"
