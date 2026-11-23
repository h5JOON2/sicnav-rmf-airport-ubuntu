#!/bin/zsh
set -eu
TASK_DIR="${0:A:h}"
MP_ROOT="${SICNAV_MP_ROOT:?Set SICNAV_MP_ROOT to your existing controller environment root}"
BASE_PYTHON="$MP_ROOT/work/venv/bin/python"
"$BASE_PYTHON" -m venv --system-site-packages "$TASK_DIR/.venv-inference"
SHARED_SITE=$("$BASE_PYTHON" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')
LOCAL_SITE=$("$TASK_DIR/.venv-inference/bin/python" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')
printf '%s\n' "$SHARED_SITE" > "$LOCAL_SITE/sicnav_shared.pth"
"$TASK_DIR/.venv-inference/bin/python" -m pip install --no-deps -r "$TASK_DIR/requirements-inference-extra.txt"
"$TASK_DIR/.venv-inference/bin/python" -c 'import torch,cv2; import scipy; print("MPS available:",torch.backends.mps.is_available())'

"$TASK_DIR/.venv-inference/bin/python" "$TASK_DIR/scripts/prepare_dr_spaam.py"
