#!/usr/bin/env bash
# Start the complete stack once. No motor enable, gate enable or motion commands.
set -e
GRASPGEN_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$GRASPGEN_ROOT"
source /opt/ros/humble/setup.bash
source "$GRASPGEN_ROOT/install/setup.bash"
export ROS_DOMAIN_ID=187
unset PYTHONNOUSERSITE
# Use the system ROS CLI; vision launch selects its own isolated Python.
export PATH="/usr/bin:/bin:$PATH"
if ! GRASPGEN_LINK="$(ip -o link show can0 2>/dev/null)"; then
    echo '找不到can0。请检查USB-CAN连接，并执行 ip -brief link 查看接口名称。'
    exit 1
fi
GRASPGEN_FLAGS="${GRASPGEN_LINK#*<}"
GRASPGEN_FLAGS="${GRASPGEN_FLAGS%%>*}"
case ",$GRASPGEN_FLAGS," in
    *,UP,*) ;;
    *)
        echo 'can0存在但未启用。先执行：sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up'
        exit 1
        ;;
esac
if ! GRASPGEN_NODES="$(ros2 node list --no-daemon 2>/dev/null)"; then
    echo '无法读取ROS节点列表。请检查ROS环境后重试；本次未启动节点。'
    exit 1
fi
GRASPGEN_START_CAMERA=true
while IFS= read -r GRASPGEN_NODE; do
    case "$GRASPGEN_NODE" in
        /piper_can_gateway|/piper_trajectory_bridge|/move_group|/graspgen_vision|/graspgen_grasp_generator|/grasp_preview|/handeye_mount)
            echo '发现已有机械臂/识别/手眼节点。请在原启动终端Ctrl+C停止旧流程后再启动，避免同名节点。'
            exit 1
            ;;
        /camera) GRASPGEN_START_CAMERA=false ;;
    esac
done <<< "$GRASPGEN_NODES"
if [ "$GRASPGEN_START_CAMERA" = false ]; then
    echo '复用已运行的/camera，相机不会重复启动。'
fi
for GRASPGEN_FILE in calibration/handeye_result.yaml calibration/grasp_profile.json models/yolov8n-seg.pt .venv-vision/bin/python; do
    if [ ! -e "$GRASPGEN_FILE" ]; then
        echo "缺少必要文件：$GRASPGEN_FILE"
        exit 1
    fi
done
echo '启动相机、识别、手眼TF、预览、MoveIt和真机桥接；随后加入桌面/相机碰撞模型。'
echo '运动门控保持关闭，不自动使能、回零、开闭夹爪或执行抓取。Ctrl+C停止本次启动的节点。'
exec /usr/bin/python3 /opt/ros/humble/bin/ros2 launch piper_moveit_bridge grasp_system.launch.py "start_camera:=$GRASPGEN_START_CAMERA" "$@"
