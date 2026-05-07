#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

lidar_model="jt128"
pass_args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --mid360-params)
      lidar_model="jt128-mid360"
      shift
      ;;
    *)
      pass_args+=("$1")
      shift
      ;;
  esac
done

exec "$ROOT_DIR/script/run_superodom.sh" mapping-bag --lidar "$lidar_model" "${pass_args[@]}"
