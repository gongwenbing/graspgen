# ROS 2 vision migration and verification — 2026-10-21

## Replacement

- Removed ROS 1 tensorrtx_yolov8_ros (catkin/C++ TensorRT8 custom plugin, engines,
  old weights, roslaunch and rospy automatic arm_action/item_confer motion scripts).
- Added ament_python graspgen_vision: instance masks, metric per-object RGB clouds,
  PCA observed-surface OBB, candidate poses, width checks, selection and RViz.
- Uses official YOLOv8n-seg COCO weights, Ultralytics 8.3.221, PyTorch2.7.1+cpu,
  torchvision0.22.1+cpu, NumPy1.24.4, OpenCV4.11 in project .venv-vision.
- CPU backend replaces original TensorRT backend; no GPU/TensorRT acceleration claimed.
- Root README now describes current Humble workflow. VSCode perception build task added.
- No hand-eye or robot-TCP transform is invented; no hardware motion topics/services used.

## Checks

1. Three metric geometry tests passed in both system and inference Python: invalid depth,
   masked projection, RGB order, rotated-plane PCA axes/quaternion, gripper width,
   oversized/degenerate rejection.
2. Isolated ROS_DOMAIN_ID194 synthetic RGB-D + mock-inference integration passed:
   synchronization, RGB clouds, candidate pose/status, clearing on detection miss and
   camera timeout, no replay of selected pose. No camera or CAN opened.
3. Actual official YOLO inference on official sample RGB passed. Synthetic depth supplied
   only for the geometry part; those positions are not real scene measurements.
4. Actual D455 stream, existing camera on domain187, 20-second perception-only observation:
   75 image/cloud/pose/status outputs, 68 with valid candidates. Detected mouse at
   confidence0.594, 625 filtered points, observed extent ~71.6x38.9x12.5 mm,
   opening estimate43.9 mm (including5 mm clearance), camera-frame center
   [-0.2350,0.0554,0.7481] m. Sample processing age0.1035s.
   Detailed JSON: ROS2_VISION_LIVE_RESULT_2026-10-21.json.
5. Perception launch shut down cleanly. Existing camera and other user nodes left running.

The cloud is an observed surface: backside, thickness, contact stability, physical
collision clearance, robot reachability, hand-eye/TCP calibration and actual picking
are NOT verified by these checks. Candidate widths do not prove grasp success.

Image/status matching is timestamp-based; selected_grasp is published only for a
fresh valid candidate, never latched. Downstream consumers must check status and age.
Frame-local ids are not trackers. Default classes exclude people and table backgrounds.

## Final filtered-class verification

After restricting inference/overlay to configured target classes and fixing clean
SIGINT shutdown, repeated D455 test produced 27 image/cloud/pose/status outputs,
all 27 with candidates. Last frame included mouse (score0.729,715 points) and
bottle (score0.480,4179 points); combined cloud4894 points, two candidate poses,
processing age0.139s. Width estimates varied with visible surface/depth, reinforcing
that these are candidates rather than calibrated grasp dimensions.
See ROS2_VISION_FINAL_RESULT_2026-10-21.json. Launch exited cleanly without errors.
The final mocked ROS regression also passed. No arm motion was requested.
