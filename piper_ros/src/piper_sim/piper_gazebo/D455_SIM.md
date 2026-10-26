# Gazebo Classic RGB-D camera

## Current smaller bottle (2026-10-26)

The default `worlds/bottle.sdf` is now uniformly scaled to 0.7 of the original:
36.4mm body diameter, 131.6mm total height and 61.74g mass. All visual/collision
dimensions and xyz offsets scale by 0.7, mass by 0.7^3 and inertia by 0.7^5
(constant density). The original model was replaced; no model backup is retained.
Older measurements below describe that original model, not this smaller bottle.

The shared `scripts/bottle_model.py` reads dimensions from SDF. Observation surface
sampling, collision geometry, hover clearance, default grasp height and closure
starting width now follow those dimensions. The current default pad-center height
is 1mm below the top of the cylinder body, about 90mm. The earlier 72.8mm target
collided with the gripper base near the end of descent; 86mm and 90mm endpoints
were collision-free in the 2026-10-26 diagnostic. Do not pass the old `grasp_height:=0.15`
to this model. Robot TCP, finger-pad dimensions, camera pose and depth limits do
not scale. Prior grasp reports are rejected for resume when dimensions differ.

An already-spawned entity does not change when its SDF file is edited. Stop the
old Gazebo launch, restart the scene with the installed environment and generate
the new bottle (world coordinates, bottle origin at its bottom):

```bash
ros2 launch piper_gazebo piper_pick_scene.launch.py moveit:=true enable_camera:=true
# In another terminal with the same environment/domain:
ros2 run gazebo_ros spawn_entity.py -entity vision_bottle \
  -file ~/GraspGen/piper_ros/src/piper_sim/piper_gazebo/worlds/bottle.sdf \
  -reference_frame world -x 0.40 -y 0.10 -z 1.017
```

Reacquire RGB-D/segmentation, replan observation and hover, and validate contact
again. This model change has passed geometry tests but has not been physically
grasp-tested in Gazebo yet. A smaller bottle is not itself proof of a stable grasp.

This adds an ideal registered RGB-D sensor for ROS 2 Humble / Gazebo Classic.
It does not model D455 stereo matching, depth noise or real optical calibration.
The 640x480, 30 Hz, 1.211 rad horizontal field of view and 0.35–1.5 m range are
simulation settings. Gazebo publishes its generated CameraInfo; do not substitute
real-camera intrinsics. Color and depth share one sensor projection and viewpoint.

## Build this source tree

The existing `piper_ros/install` may contain symlinks to `/home/zlq/piper_ros`.
Use a separate build/install pair to avoid old CMake paths and source links:

```bash
cd ~/GraspGen/piper_ros
source /opt/ros/humble/setup.bash
source ~/GraspGen/install/setup.bash
colcon --log-base log_d455 build --base-paths src \
  --build-base build_d455 --install-base install_d455 \
  --packages-select piper_description piper_gazebo piper_with_gripper_moveit \
  --allow-overriding piper_description piper_with_gripper_moveit \
  --symlink-install --executor sequential --cmake-args -DBUILD_TESTING=OFF
```

## Start simulation

Use the same environment in every simulation terminal. Use a separate ROS domain
from the real robot. Do not start the real camera driver, handeye mount publisher
or hardware bridge in this simulation domain: robot_state_publisher owns camera TF.

```bash
source /opt/ros/humble/setup.bash
source ~/GraspGen/install/setup.bash
source ~/GraspGen/piper_ros/install_d455/local_setup.bash
export ROS_DOMAIN_ID=188
ros2 pkg prefix piper_gazebo
ros2 launch piper_gazebo piper_pick_scene.launch.py \
  moveit:=true enable_camera:=true
```

For an empty world use `piper_gazebo.launch.py enable_camera:=true` instead.
The camera follows link6; at the initial robot posture it may not see the table.
Move to an observation posture using MoveIt and inspect the image first.

## Inspect output

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/aligned_depth_to_color/image_raw
ros2 topic echo /camera/color/camera_info --once
ros2 run tf2_ros tf2_echo link6 camera_color_optical_frame
```

In RViz add Image using `/camera/color/image_raw` and Best Effort reliability.
All four image/info topics use `camera_color_optical_frame`:

- `/camera/color/image_raw`
- `/camera/color/camera_info`
- `/camera/aligned_depth_to_color/image_raw` (32FC1, metres)
- `/camera/aligned_depth_to_color/camera_info`

The depth is genuinely registered in this ideal model because it comes from the
same sensor as color. Remapping two separate camera outputs alone would not align
their images. Depth bounds do not guarantee a black background outside the range;
the perception node also filters invalid/out-of-range samples.

## Existing perception

`graspgen_vision/launch/vision.launch.py` now accepts `use_sim_time`. If its installed
launch file is a copy, rebuild that package from the GraspGen root first:

```bash
cd ~/GraspGen
colcon build --base-paths graspgen_vision --packages-select graspgen_vision
```

Then in a terminal with the simulation environment above:

```bash
ros2 launch graspgen_vision vision.launch.py use_sim_time:=true
```

The current cube world is for camera/geometry validation. Default YOLO classes do
not include arbitrary cubes. Add a recognizable bottle/cup model or implement cube
segmentation before expecting grasp candidates. This change does not connect the
hardware-specific auto_grasp execution checks to simulation.

## Plan a vision pregrasp in simulation

If the near bottle has left the camera view, find an observation pose without
dragging RViz markers or requiring any current vision candidate:

```bash
ros2 run piper_gazebo observe_bottle.py
# After inspecting the displayed path, request a fresh plan and simulation motion:
ros2 run piper_gazebo observe_bottle.py --ros-args -p execute:=true
```

This uses the supplied upright bottle's Gazebo position to select camera viewpoints
only. It screens sampled surfaces of the supplied bottle against the ideal image with
2% margins and depth above 0.35 m, then checks collisions and plans a joint
trajectory. It first tries fixed viewpoints, then searches joint configurations
and optimizes framing when fixed viewpoints have no IK solution. This does not
change the camera or perception depth limits; it relaxes the search's former
extra 0.38 m/20% margins.
Gazebo, camera TF, joint feedback and MoveIt/table scene are required; vision is
not. Existing gripper opening is preserved. Execution is opt-in and checks that
the arm has not moved during planning. Do not move the bottle during the command;
its collision geometry is a snapshot. The search is finite and can fail, and
geometric visibility does not guarantee YOLO recognition or absence of occlusion.
Check actual images and fresh perception after moving. This is a simulation-only
ground-truth assisted observation tool, not an autonomous real-camera search.

Keep Gazebo, MoveIt, vision and the preview node running in domain 188. The bottle
must be the supplied `bottle.sdf` (entity `vision_bottle`), and scene synchronization
must have added `pick_table`. With the environment above loaded:

```bash
ros2 run piper_gazebo plan_vision_pregrasp.py
```

If the vertical pose has no IK due to wrist limits, explicitly request a tilted
hover while preserving the same TCP position:

```bash
ros2 run piper_gazebo plan_vision_pregrasp.py --ros-args -p tilt_deg:=30.0
```

Simulation `tilt_deg` accepts 0..45 degrees, default 0; hardware tools retain their
existing limits. `yaw_offset_deg` optionally rotates the wrist about base Z at the
fixed TCP (-180..180 degrees, default 0). These parameters change the hover pose
only; they do not validate descent or contact. The planner checks start validity,
collision-aware IK and plans to the resulting joint solution. If IK fails, an
additional collision-disabled query is used only to distinguish no IK from a
colliding goal; it is never used to bypass collision validation. The actual tilted
path appears in Planned Path and its target is recorded in the JSON. The existing
preview markers continue to show the original vertical hover, not the tilted pose.

By default this command only plans. To replan and execute the hover in Gazebo:

```bash
ros2 run piper_gazebo plan_vision_pregrasp.py --ros-args -p tilt_deg:=30.0 -p execute:=true
```

Execution uses the fresh in-memory trajectory, checks for arm/bottle movement
since planning, and verifies the final joint feedback. It does not descend or
send gripper commands. `HOVER REACHED` means feedback matches the trajectory;
RViz Planned Path animation by itself does not move Gazebo.

It uses simulation time, checks recent joint
feedback and valid preview input, adds a conservative bottle collision cylinder
at its current Gazebo link pose, and requests `/plan_kinematic_path`. The goal is
the vision-derived `/grasp_preview/pregrasp_link6`, not Gazebo ground truth. On
success it publishes `/display_planned_path` and writes
`~/GraspGen/calibration/sim_pregrasp_plan.json`. View Planned Path in the existing
RViz MotionPlanning display. A planning failure may mean an unreachable target
or collision; read move_group diagnostics. No gripper command or descent occurs.

The bottle collision is a snapshot: rerun after moving it. Planning now mirrors
the supplied SDF's body cylinder, shoulder sphere and neck/cap cylinder, rather
than a full-width cylinder around the neck. Other assets need their own geometry.
Do not reuse the saved trajectory for execution. A successful
plan validates the modeled scene only, not real contact or physical calibration.

## Validate contact and a small lift

The supplied original bottle is 52mm wide and 188mm high. With the camera mounted
on the wrist and a 30-degree hover, body-center approaches collide with the camera
and gripper base. The verification command therefore targets the shoulder at
150mm above the bottle bottom. It preserves wrist orientation and positions the
finger-pad midpoint at this height; it does not blindly descend by the hover offset.

First actually execute the fresh hover (RViz animation alone is not execution).
Then inspect the verification geometry without moving:

```bash
ros2 run piper_gazebo verify_bottle_grasp.py
```

Explicitly request the one-shot physical simulation test:

```bash
ros2 run piper_gazebo verify_bottle_grasp.py --ros-args -p execute:=true -p grasp_height:=0.15
```

It checks actual hover position and bottle geometry, opens the jaws, requires a
complete collision-checked Cartesian approach, gradually closes until stable
bilateral contacts are reported, and then attempts a 10mm lift. It uses the native
Gazebo contact stream via `bottle_contact_monitor` (no Gazebo restart needed).
Before descent, the all-phase workflow levels the gripper's local +Y jaw-opening
axis into the table plane while keeping the TCP fixed. This preserves a tilted
approach rather than requiring the whole tool to point vertically down. Any
needed adjustment is separately collision-planned and verified against feedback
before the subsequent descent target is calculated. In the 2026-10-25 live test,
leveling corrected a 0.663-degree jaw-axis error to approximately 0.000015 degrees
with 0.0082mm TCP drift. The subsequent 101mm descent and bilateral contact passed.
The 10mm lift still slipped: the bottle rose only 0.063mm and relative drift was
9.96mm. Leveling is verified, but this shoulder grasp is not physically retained.
See `calibration/sim_leveling_grasp_test.json`. The prior tilted fixture was
released, the robot retracted to hover, and the bottle was reset upright before
the test; that fixture reset is separately recorded in
`calibration/sim_leveling_fixture_reset.json` and is not part of the lift result.
Only bottle/finger contacts and the initial supported bottle/table contact are
allowed during the corresponding phases. Camera, gripper-base and robot/table
collisions remain checked. The held object's geometry is attached in MoveIt's
planning scene for the lift; no physical Gazebo joint or suction is created.

The command writes `~/GraspGen/calibration/sim_bottle_grasp_validation.json`,
stops at the first failed stage, and never automatically retries or releases the
bottle. A lift succeeds only if the object physically rises 6–15mm, remains within
5mm of its relative wrist position, and still has fresh bilateral contacts.
After a successful test the jaws stay closed. Do not rerun at the lowered/held
pose: preflight requires the hover. The test uses known simulation geometry to
verify contact behavior, not a general vision-only grasp policy or real calibration.

Measured run on 2026-10-25: the shoulder approach descended approximately 101mm,
and gradual closure obtained stable bilateral contacts at a commanded 52mm opening.
The 10mm arm lift did not lift the bottle: measured bottle rise was approximately
-0.019mm and relative wrist/bottle drift was 10.019mm. The validator correctly
reported failure and stopped without releasing, retrying or creating a physical
constraint. Contact alone is insufficient evidence of holding force. Closing
preload/friction/contact geometry need further tuning before claiming a successful
physical grasp. The current posture is the post-test posture, not the hover.

For a run that stopped before requesting any lift, `phase:=lift execute:=true`
can resume only after checking the prior bilateral-contact report and fresh live
contacts. It does not reopen, descend or close again. It refuses a second lift
after a prior lift request or a successful run. Do not use it to repeat this failed
physical-lift experiment blindly.

Follow-up preload experiment on 2026-10-25: total jaw opening was reduced by 1mm
in two slow steps (about 52.095mm to 51.095mm). The bottle shifted about 1.04mm
and tilted 3.38 degrees. The first lift plan was blocked by contact with the
unrelated red cube; this cube was moved to the actual table corner (-0.20,0.40),
with its original pose recorded in `calibration/sim_bottle_obstacle_move.json`.
After clearing that obstruction, a 2mm trial lift executed. The bottle still
remained on the table (measured rise about -0.031mm), relative drift was 2.011mm,
and bilateral contact was lost. The remaining 8mm lift was not requested.
No physical attachment was introduced. The result remains a failed physical
grasp, not a planning failure. Do not increase closure or repeat motion blindly.

The preload commands (`phase:=preload`, then guarded `phase:=preloaded_lift` for
an initially blocked lift) retain previous report history and refuse repeat
preload/lift attempts once the corresponding experiment has executed. They are
diagnostic tools for this scene, not a reusable production grasp controller.

## Coordinate and geometry details

`d455_gazebo.xacro` defines a reusable macro. `piper_description_gazebo.xacro`
includes it conditionally; the real robot description is unchanged. The simulation
MoveIt launch loads this same Gazebo description, including the housing collision.

The default mount was derived from this workspace's accepted
`calibration/handeye_result.yaml` at implementation time:

`T_link6_camera_link = T_link6_color_optical * inverse(T_camera_link_color_optical)`

The nominal color origin is `(0, -0.059, 0)` in camera_link and its optical rotation
is RPY `(-pi/2, 0, -pi/2)`, following the locally installed RealSense D455 description.
Defaults are a snapshot: a later calibration edit does not automatically update
the mount. Update `camera_mount_xyz` / `camera_mount_rpy` defaults consistently.

Housing size is 26 x 124 x 29 mm; mass/inertia are approximations. Bracket and cable
are not included. Sensor resolution, rate, range and FOV are macro parameters.
For transfer to hardware verify actual intrinsics, mount, TCP and contact behavior.
