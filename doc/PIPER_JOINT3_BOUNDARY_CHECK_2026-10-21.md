# joint3 boundary check, 2026-10-21

Passive SocketCAN receive for five seconds; no SDK initialization or transmit calls.
Host CAN TX count remained 1232 -> 1232.
Each of six joints had 1000 samples.

| Joint | Minimum (degrees) | Maximum (degrees) |
|---|---:|---:|
| joint1 | 0.540 | 0.540 |
| joint2 | 0.000 | 0.000 |
| joint3 | 0.047 | 0.047 |
| joint4 | 0.000 | 0.000 |
| joint5 | 0.000 | 0.000 |
| joint6 | 0.000 | 0.000 |

The raw standard CAN 0x2A6 signed big-endian first 32-bit angle was decoded in
0.001 degrees, consistent with the local official SDK joint feedback definition.
This is a stable reported boundary offset in this observation, not observed jitter.
It does not establish physical zero calibration error or a manufacturer-approved tolerance.
Current and older official URDF both set joint3 upper limit to zero.
MoveIt accepted the original slightly out-of-bounds start, then the trajectory
bridge rejected it under strict bounds. Planner start tolerance was tightened in
moveit_hardware.launch.py; this improves consistency but does not recover the pose.
No zero calibration, feedback clipping, command-limit expansion, reset or motion
was performed. A separately reviewed recovery to an interior state is required
before ordinary strict-bound trajectory execution can resume.
