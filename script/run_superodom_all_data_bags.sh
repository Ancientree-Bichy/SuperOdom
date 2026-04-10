#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
DATA_BAG_DIR=${DATA_BAG_DIR:-"$ROOT_DIR/data_bag"}
OUTPUT_BASE_DIR=${OUTPUT_BASE_DIR:-"$ROOT_DIR/mapping_output"}
BATCH_NAME=${BATCH_NAME:-"test_$(date +%Y%m%d_%H%M%S)"}
SKIP_BUILD=0
FORCE_RUN=0

usage() {
  cat <<EOF
Usage: $(basename "$0") [--skip-build] [--force] [--batch-name <name>]

Batch-run offline mapping for every ROS 2 bag under:
  $DATA_BAG_DIR

Outputs:
  - one batch directory at $OUTPUT_BASE_DIR/<batch_name>
  - one run directory per bag at $OUTPUT_BASE_DIR/<batch_name>/<bag_name>
  - summary file at $OUTPUT_BASE_DIR/<batch_name>/data_bag_offline_mapping_summary.txt

Options:
  --skip-build   Do not rebuild before running bags.
  --force        Re-run bags even if output PLY already exists.
  --batch-name   Override batch directory name. Default: $BATCH_NAME
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-build)
      SKIP_BUILD=1
      shift
      ;;
    --force)
      FORCE_RUN=1
      shift
      ;;
    --batch-name)
      if [[ $# -lt 2 ]]; then
        echo "--batch-name requires a value" >&2
        exit 1
      fi
      BATCH_NAME=$2
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 1
      ;;
  esac
done

BATCH_OUTPUT_DIR="$OUTPUT_BASE_DIR/$BATCH_NAME"
SUMMARY_FILE=${SUMMARY_FILE:-"$BATCH_OUTPUT_DIR/data_bag_offline_mapping_summary.txt"}

if [[ ! -d "$DATA_BAG_DIR" ]]; then
  echo "data_bag directory not found: $DATA_BAG_DIR" >&2
  exit 1
fi

mapfile -t bag_dirs < <(find "$DATA_BAG_DIR" -mindepth 1 -maxdepth 1 -type d | sort)

if [[ ${#bag_dirs[@]} -eq 0 ]]; then
  echo "No bag directories found under: $DATA_BAG_DIR" >&2
  exit 1
fi

mkdir -p "$BATCH_OUTPUT_DIR"

if [[ $SKIP_BUILD -eq 0 ]]; then
  "$ROOT_DIR/script/build_superodom_local.sh"
fi

{
  echo "timestamp=$(date -Iseconds)"
  echo "data_bag_dir=$DATA_BAG_DIR"
  echo "output_base_dir=$OUTPUT_BASE_DIR"
  echo "batch_name=$BATCH_NAME"
  echo "batch_output_dir=$BATCH_OUTPUT_DIR"
  echo "skip_build=$SKIP_BUILD"
  echo "force_run=$FORCE_RUN"
  echo
  printf '%-20s %-10s %s\n' "bag_name" "status" "saved_scans_ply"
} >"$SUMMARY_FILE"

for bag_dir in "${bag_dirs[@]}"; do
  bag_name=$(basename "$bag_dir")
  metadata_file="$bag_dir/metadata.yaml"
  output_dir="$BATCH_OUTPUT_DIR/$bag_name"
  ply_path="$output_dir/saved_scans.ply"

  if [[ ! -f "$metadata_file" ]]; then
    printf '%-20s %-10s %s\n' "$bag_name" "skipped" "missing metadata.yaml" | tee -a "$SUMMARY_FILE"
    continue
  fi

  if [[ $FORCE_RUN -eq 0 && -f "$ply_path" ]]; then
    printf '%-20s %-10s %s\n' "$bag_name" "cached" "$ply_path" | tee -a "$SUMMARY_FILE"
    continue
  fi

  echo "=== Running offline mapping for $bag_name ==="
  OUTPUT_BASE_DIR="$BATCH_OUTPUT_DIR" \
    bash "$ROOT_DIR/script/run_superodom.sh" mapping-bag --skip-build "$bag_dir" "$bag_name"

  if [[ -f "$ply_path" ]]; then
    printf '%-20s %-10s %s\n' "$bag_name" "ok" "$ply_path" | tee -a "$SUMMARY_FILE"
  else
    printf '%-20s %-10s %s\n' "$bag_name" "failed" "$output_dir" | tee -a "$SUMMARY_FILE"
  fi
done

echo
echo "Batch offline mapping summary:"
cat "$SUMMARY_FILE"
