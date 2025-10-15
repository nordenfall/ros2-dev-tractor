#!/usr/bin/env bash
set -euo pipefail

#extract username
USERNAME="$(id -un)"
if grep -q '^USERNAME=' .env 2>/dev/null; then
  sed -i "s/^USERNAME=.*/USERNAME=${USERNAME}/" .env
else
  printf 'USERNAME=%s\n' "$USERNAME" >> .env
fi

#correcting config
CFG="./ws/src/livox_ros_driver2/config/MID360_config.json"  
if [[ -f ".env" ]]; then
  set -a
  . ./.env
  set +a
fi

TMP="${CFG}.tmp.$$"
trap 'rm -f "$TMP"' EXIT
jq \
  --arg host  "$HOST_IP" \
  --arg lidar "$LIDAR_IP" \
  '
    .MID360.host_net_info.cmd_data_ip    = $host |
    .MID360.host_net_info.push_msg_ip    = $host |
    .MID360.host_net_info.point_data_ip  = $host |
    .MID360.host_net_info.imu_data_ip    = $host |

    # всем лидарам:
    (.lidar_configs[]? | .ip) = $lidar
  ' "$CFG" > "$TMP"
mv "$TMP" "$CFG"
trap - EXIT