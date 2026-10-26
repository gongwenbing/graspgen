#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$TASK_ROOT"
# Never replace the system ROS Python, driver or CUDA installation.
python3 -m venv --system-site-packages --without-pip .venv-vision
.venv-vision/bin/python -m pip install --timeout 120 --retries 5 --cache-dir "$TASK_ROOT/.cache/pip" torch==2.7.1 torchvision==0.22.1 --index-url https://download.pytorch.org/whl/cpu
.venv-vision/bin/python -m pip install --timeout 120 --retries 5 --cache-dir "$TASK_ROOT/.cache/pip" 'numpy==1.24.4' 'opencv-python<4.12' 'ultralytics==8.3.221'
mkdir -p models .cache/ultralytics/Ultralytics .cache/matplotlib
YOLO_CONFIG_DIR="$TASK_ROOT/.cache/ultralytics" MPLCONFIGDIR="$TASK_ROOT/.cache/matplotlib" .venv-vision/bin/python - <<'PY'
from pathlib import Path
import urllib.request
from ultralytics import YOLO
p=Path('models/yolov8n-seg.pt')
if not p.exists():
    temporary=p.with_suffix('.download')
    urllib.request.urlretrieve('https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n-seg.pt',temporary)
    temporary.replace(p)
model=YOLO(str(p))
assert model.task=='segment'
print('Segmentation weights ready:',p.resolve())
PY
