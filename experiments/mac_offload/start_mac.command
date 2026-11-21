#!/bin/zsh
set -eu
TASK_DIR="${0:A:h}"
mkdir -p "$TASK_DIR/logs"
exec "$TASK_DIR/.venv-inference/bin/python" "$TASK_DIR/scripts/mac_offload_server.py" --model "$TASK_DIR/models/ckpt_jrdb_ann_dr_spaam_e20.pth" --log "$TASK_DIR/logs/mac.jsonl" "$@"
