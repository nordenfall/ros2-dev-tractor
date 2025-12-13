#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_ROOT"

docker compose up -d base
docker compose exec base bash -lc 'source /opt/ros/$ROS_DISTRO/setup.bash && if [ -f /home/$(whoami)/ws/install/setup.bash ]; then source /home/$(whoami)/ws/install/setup.bash; fi; exec bash'
