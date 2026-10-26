"""Explicit shared TCP translation; no inferred calibration from a grasp."""
import json
import math
from pathlib import Path
DEFAULT_PATH=Path.home()/'GraspGen/calibration/tcp_config.json'


def load_tcp(path=DEFAULT_PATH):
    path=Path(path)
    data=json.loads(path.read_text()) if path.exists() else {'translation_m':[0.,0.,.1358],'translation_verified':False,'orientation_verified':False,'method':'provisional_measured_length'}
    t=data['translation_m']
    if len(t)!=3 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in t) or not .05<=math.sqrt(sum(v*v for v in t))<=.25:
        raise ValueError('Invalid TCP translation')
    return data
