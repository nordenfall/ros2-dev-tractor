#!/usr/bin/env bash
set -euo pipefail

./get-env.sh
git submodule update --init --recursive

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_ROOT"

ENV_FILE="$REPO_ROOT/.env"
if [[ -f "$ENV_FILE" ]]; then
  set -a
  source "$ENV_FILE"
  set +a
else
  echo "Missing .env file in $REPO_ROOT" >&2
  exit 1
fi

COMPOSE="docker compose --env-file $ENV_FILE"

$COMPOSE build base
$COMPOSE run --rm base bash -lc "source /opt/ros/\$ROS_DISTRO/setup.bash && cd /home/\$(whoami)/ws && colcon build --packages-select cloud2seg"
