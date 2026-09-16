#!/usr/bin/env bash
# Preset C: long range, busy road (EXPERIMENTAL).
# As preset B, but the sensor reports approaching targets only (DEDI 1), so receding roadside clutter cannot fill
# the 12-target list. Cost: the pipeline's own-speed estimate needs receding clutter and never becomes valid, so
# stationary-clutter rejection and receding movers are gone. A team decision before any ride.
# Also starts the field web app on port 8080 with this preset's config (http://raspberrypi.local:8080/).
# Extra arguments go to tools/run_pipeline.py (e.g. --no-imu, --no-record).
exec "$(dirname "$0")/tools/run_preset.sh" C "long range, busy road (approaching only, experimental)" \
    radar.params.RRAI=3 \
    radar.params.RSPI=3 \
    radar.params.THOF=20 \
    radar.params.DEDI=1 \
    radar.params.MISP=2 \
    radar.params.MASP=100 \
    pipeline.clutter.doppler_blind_band_mps=0.6 \
    -- "$@"
