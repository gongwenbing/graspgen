# ROS 2 camera validation, 2026-10-21

Added graspgen_camera, using installed realsense2_camera 4.58.4 and librealsense 2.58.4.
Legacy realsense-ros 2.3.2 catkin tree removed after successful ROS 2 device validation.
Root README marked as historical ROS 1 instructions with ROS 2 entry links.

Real-device test on isolated ROS_DOMAIN_ID=193, no arm processes started:
- D455, USB 3.2, firmware 5.15.1.55.
- Color RGB8 and depth Z16: 640x480 at 30 FPS configured.
- Received /camera/color/image_raw, /camera/color/camera_info,
  /camera/aligned_depth_to_color/image_raw and camera_info.
- check_rgbd exit 0; optical frame camera_color_optical_frame; intrinsics valid;
  dimensions agree; sampled timestamp delta 0.0333 s.
- Sample valid depth fraction 54.7%, median valid depth 0.439 m.
- Driver stopped cleanly after check. No persistent camera test instance left running.

This verifies reception and metadata, not pixel-level calibration accuracy,
object recognition, hand-eye calibration or automatic grasping.
