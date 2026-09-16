#!/usr/bin/env bash
# Preset B: long range, open road.
# Threshold 10 dB lower (roughly 1.8x the range if noise rather than clutter limits it). MISP 2 (~0.56 m/s at
# RSPI 3) keeps fidgeting and very slow movers out of the 12-target list; the tracker's blind band is raised
# to 0.6 m/s to match, so it does not assume it can see speeds the sensor now filters out.
# Watch the page for CAP HIT / PDAT_SATURATED: on busy streets the 12-target limit fills sooner.
# Also starts the field web app on port 8080 with this preset's config (http://raspberrypi.local:8080/).
# --thof N overrides the threshold offset for one run (10..60 dB), e.g. ./run_preset_b.sh --thof 15.
# Extra arguments go to tools/run_pipeline.py (e.g. --no-imu, --no-record).
exec "$(dirname "$0")/tools/run_preset.sh" B "long range, open road" \
    radar.params.RRAI=3 \
    radar.params.RSPI=3 \
    radar.params.THOF=20 \
    radar.params.DEDI=2 \
    radar.params.MISP=2 \
    radar.params.MASP=100 \
    pipeline.clutter.doppler_blind_band_mps=0.6 \
    -- "$@"
