#!/bin/bash
# ublox_gps_node's ACK timeout (1s) is hardcoded in gps.hpp and not
# configurable. At 19200 baud the config sequence occasionally misses an
# ACK and the node dies with FATAL. Since successful config gets saved to
# the receiver's EEPROM (save.mask/save.device in ublox_rover.yaml), we
# just need ONE clean pass. This retries ublox_gps_node alone until it
# survives past the config phase.

set -u
MAX_TRIES=20

for i in $(seq 1 "$MAX_TRIES"); do
    echo "=== attempt $i/$MAX_TRIES ==="
    ros2 run ublox_gps ublox_gps_node --ros-args \
        --params-file /home/a/ros2_ws/src/f9p_bringup/config/ublox_rover.yaml &
    pid=$!

    # give it 5s to get through the config phase; FATAL config errors
    # happen within ~1-2s of startup.
    sleep 5

    if kill -0 "$pid" 2>/dev/null; then
        echo "=== attempt $i survived config phase, node is running (pid $pid) ==="
        echo "=== leaving it running - Ctrl+C to stop, config should now be saved to EEPROM ==="
        wait "$pid"
        exit 0
    else
        wait "$pid" 2>/dev/null
        echo "=== attempt $i died, retrying ==="
    fi
done

echo "=== gave up after $MAX_TRIES attempts ==="
exit 1
