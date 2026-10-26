"""URDF limits, exact units, and position-trajectory validation shared by both gates."""
import math
import xml.etree.ElementTree as ET
from bisect import bisect_right
from pathlib import Path

ARM = tuple(f'joint{i}' for i in range(1, 7))
JOINTS = ARM + ('joint7',)
# Engineering measurement allowance, not a manufacturer accuracy specification.
# Applies only to feedback; command and trajectory bounds remain exact.
ARM_FEEDBACK_TOLERANCE = math.radians(0.05)


class Model:
    def __init__(self, path):
        root = ET.parse(Path(path)).getroot()
        self.limits = {}
        for name in JOINTS:
            joint = root.find(f"joint[@name='{name}']")
            if joint is None or joint.find('limit') is None:
                raise ValueError(f'Missing URDF limits for {name}')
            l = joint.find('limit')
            bounds = tuple(float(l.get(k)) for k in ('lower', 'upper', 'velocity'))
            if not all(math.isfinite(v) for v in bounds) or bounds[0] >= bounds[1] or bounds[2] <= 0:
                raise ValueError(f'Invalid URDF limit for {name}')
            self.limits[name] = bounds
        self.max_opening = 2 * self.limits['joint7'][1]

    def check(self, positions):
        for name in JOINTS:
            value = positions[name]
            low, high, _ = self.limits[name]
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f'{name}={value:.6f} outside [{low:.6f}, {high:.6f}]')

    def feedback(self, msg, gripper_tolerance=0.0005):
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            raise ValueError('Feedback names/positions mismatch or duplicates')
        values = dict(zip(msg.name, msg.position))
        if any(not math.isfinite(v) for v in values.values()):
            raise ValueError('Nonfinite feedback')
        result = {name: values[name] for name in ARM}
        width = values['gripper'] if 'gripper' in values else 2 * values['joint7']
        if not -gripper_tolerance <= width <= self.max_opening + gripper_tolerance:
            raise ValueError('Gripper feedback exceeds configured measurement tolerance')
        # Raw measurements remain on /joint_states_single and hardware/status.
        # Normalize only sub-0.05-degree boundary excursions for planning.
        for name in ARM:
            low, high, _ = self.limits[name]
            if not low-ARM_FEEDBACK_TOLERANCE <= result[name] <= high+ARM_FEEDBACK_TOLERANCE:
                raise ValueError(f'{name}={result[name]:.6f} outside measurement allowance around [{low:.6f}, {high:.6f}]')
            result[name] = min(high, max(low, result[name]))
        result['joint7'] = min(self.max_opening, max(0.0, width)) / 2
        self.check(result)
        return result


def seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


def validate_trajectory(msg, names, current, model, arm_speed, jaw_speed, start_tolerance):
    if len(msg.joint_names) != len(names) or set(msg.joint_names) != set(names):
        raise ValueError('Trajectory must contain exactly the controller joint names')
    if not msg.points or len(msg.points) > 10000:
        raise ValueError('Trajectory needs 1..10000 points')
    order = [msg.joint_names.index(n) for n in names]
    times, points = [0.0], [[current[n] for n in names]]
    for index, point in enumerate(msg.points):
        if len(point.positions) != len(names) or not all(math.isfinite(v) for v in point.positions):
            raise ValueError('Missing/nonfinite trajectory positions')
        if point.effort:
            raise ValueError('Effort trajectories are unsupported')
        for derivative in (point.velocities, point.accelerations):
            if derivative and (len(derivative) != len(names) or not all(math.isfinite(v) for v in derivative)):
                raise ValueError('Invalid trajectory derivatives')
        t = seconds(point.time_from_start)
        values = [point.positions[i] for i in order]
        whole = dict(current)
        whole.update(zip(names, values))
        model.check(whole)
        if t < 0 or t > 120 or (index > 0 and t <= times[-1]):
            raise ValueError('Trajectory times must increase within 120 seconds')
        if index == 0 and t == 0:
            if max(abs(a-b) for a,b in zip(values, points[0])) > start_tolerance:
                raise ValueError('Trajectory starts too far from measured state')
            # Never jump to a time-zero point, even if within the start tolerance.
            # Interpolation begins at the actual measured state.
            continue
        if t <= times[-1]:
            raise ValueError('Invalid segment duration')
        for j, name in enumerate(names):
            cap = min(model.limits[name][2], jaw_speed if name == 'joint7' else arm_speed)
            if abs(values[j]-points[-1][j]) / (t-times[-1]) > cap + 1e-6:
                raise ValueError(f'Trajectory segment exceeds {name} velocity limit {cap}')
            if point.velocities and abs(point.velocities[order[j]]) > cap + 1e-6:
                raise ValueError(f'Supplied velocity exceeds {name} limit')
        times.append(t)
        points.append(values)
    if len(times) < 2:
        raise ValueError('Trajectory must contain a positive-duration point')
    return times, points


def interpolate(times, points, t):
    i = min(len(times)-2, max(0, bisect_right(times, t)-1))
    fraction = min(1.0, max(0.0, (t-times[i])/(times[i+1]-times[i])))
    return [a + fraction*(b-a) for a,b in zip(points[i],points[i+1])]
