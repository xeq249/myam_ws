#!/usr/bin/env bash
# 从 MoveIt 用 URDF 生成 Gazebo 参考 SDF（不直接覆盖 PX4 的 myam.sdf）。
# 用法：在已 source 的 ROS 环境中执行；输出到 stdout 或 OUT 文件。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
URDF="${URDF:-${WS_ROOT}/src/myam_sys_moveit/config/gazebo_myam_sys.urdf}"
OUT="${OUT:-}"

if [[ ! -f "${URDF}" ]]; then
  echo "URDF not found: ${URDF}" >&2
  exit 1
fi

# 使 package://myam_description 与 model:// 可解析
if [[ -f /opt/ros/noetic/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/noetic/setup.bash
fi
if [[ -f "${WS_ROOT}/devel/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${WS_ROOT}/devel/setup.bash"
fi

PX4_ROOT="${PX4_ROOT:-/home/demo/PX4-Autopilot}"
if [[ -d "${PX4_ROOT}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models" ]]; then
  export GAZEBO_MODEL_PATH="${PX4_ROOT}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:${GAZEBO_MODEL_PATH:-}"
fi

echo "# Reference SDF from: ${URDF}" >&2
echo "# Merge manually into: .../sitl_gazebo-classic/models/myam/myam.sdf (add PX4 motor/mavlink + D435i + home initial_position)" >&2

if [[ -n "${OUT}" ]]; then
  gz sdf -p "${URDF}" > "${OUT}"
  echo "Wrote ${OUT}" >&2
else
  gz sdf -p "${URDF}"
fi
