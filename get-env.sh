#!/usr/bin/env bash
set -euo pipefail

USERNAME="$(id -un)"
if grep -q '^USERNAME=' .env 2>/dev/null; then
  sed -i "s/^USERNAME=.*/USERNAME=${USERNAME}/" .env
else
  printf 'USERNAME=%s\n' "$USERNAME" >> .env
fi


if [[ -f ".env" ]]; then
  set -a
  . ./.env
  set +a
fi
