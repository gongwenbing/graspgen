import argparse,json
from pathlib import Path
import yaml
from .core import solve_samples


def main():
    p=argparse.ArgumentParser();p.add_argument('samples');p.add_argument('--output',required=True)
    args=p.parse_args();data=json.loads(Path(args.samples).read_text())
    if data.get('schema')!=1:raise ValueError('Unknown samples schema')
    if data.get('purpose'):raise ValueError('Independent validation records must not be used to refit calibration')
    result=solve_samples(data['samples'])
    result.update(schema=1,base_frame=data['base_frame'],gripper_frame=data['gripper_frame'],camera_frame=data['camera_frame'],
                  marker_id=data['marker_id'],marker_length_m=data['marker_length_m'],dictionary=data['dictionary'])
    path=Path(args.output).expanduser();path.parent.mkdir(parents=True,exist_ok=True)
    if path.exists():raise ValueError(f'Refusing to overwrite {path}; choose another output')
    path.write_text(yaml.safe_dump(result,sort_keys=False))
    print(yaml.safe_dump(result,sort_keys=False))
    if not result['accepted']:raise SystemExit('Quality check failed: report saved, do not publish this calibration')
    print('Quality gates passed. Independently validate fixed-target consistency before grasping.')
