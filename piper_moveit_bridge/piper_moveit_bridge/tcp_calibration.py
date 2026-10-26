"""Read-only fixed-point TCP sample capture and offline translation fitting."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from .grasp_runtime import Runtime,ROOT,atomic_json
from .grasp_calibration import pose_array,solve_pivot


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    modes=parser.add_mutually_exclusive_group(required=True)
    modes.add_argument('--capture',action='store_true');modes.add_argument('--solve',action='store_true')
    parser.add_argument('--fixed-point-contact',action='store_true',help='The intended TCP remains physically at the same fixed fixture point for every sample')
    parser.add_argument('--apply',action='store_true',help='With --solve, write shared TCP translation config; requires restarting preview and recapturing grasp profile')
    parser.add_argument('--samples',default=str(ROOT/'calibration/tcp_pivot_samples.json'))
    parser.add_argument('--output',default=str(ROOT/'calibration/tcp_pivot_result.json'))
    args=parser.parse_args();path=Path(args.samples)
    samples=json.loads(path.read_text()) if path.exists() else []
    if args.solve:
        try:
            result=solve_pivot(samples);result['automatically_applied']=bool(args.apply)
            if args.apply:
                from .tool_config import DEFAULT_PATH
                atomic_json(DEFAULT_PATH,{**result,'source_samples':str(path.resolve()),'orientation_verified':False})
            atomic_json(args.output,result);print(json.dumps(result,ensure_ascii=False,indent=2))
            print('Translation fit only. Verify fixture/contact independently; changing TCP requires recapturing empirical grasp profile. Restart preview and recapture the grasp profile if --apply was used.')
        except (ValueError,KeyError,TypeError) as exc:parser.error(str(exc))
        return
    if not args.fixed_point_contact:parser.error('--capture requires --fixed-point-contact; arbitrary arm poses cannot calibrate a TCP')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('tcp_pivot_capture');rt=None
    try:
        rt=Runtime(node);start,q=rt.inputs();pose=rt.pose(start);p,r=pose_array(pose)
        until=time.monotonic()+.5;rt.wait(lambda:time.monotonic()>=until,2)
        _,after=rt.inputs()
        if max(abs(q[n]-after[n]) for n in q)>.001:raise ValueError('Arm/gripper not stationary')
        samples.append({'position_m':p.tolist(),'quaternion_xyzw':r.as_quat().tolist(),'captured_unix':time.time()})
        atomic_json(path,samples);print(f'Saved fixed-point sample {len(samples)}; require >=8 diverse orientations about the SAME contact point.')
    except (RuntimeError,ValueError,KeyError,TypeError) as exc:node.get_logger().error(str(exc));raise SystemExit(1)
    finally:
        if rt:rt.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
