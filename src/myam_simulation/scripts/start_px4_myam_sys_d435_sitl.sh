#!/usr/bin/env bash
# 在 PX4 源码树中启动 SITL：机架 gazebo-classic_myam_sys_d435，世界由 PX4_SITL_WORLD 指定（默认 apriltag）。
# apriltag.world 内已含 myam_sys_d435 与 AprilTag，无需再 spawn。
set -euo pipefail

PX4_DIR="${1:-${HOME}/PX4-Autopilot}"
SITL_WORLD="${2:-apriltag}"

export PX4_GAZEBO_START_PAUSED="${PX4_GAZEBO_START_PAUSED:-0}"
export PX4_SITL_WORLD="${SITL_WORLD}"

if [[ ! -d "${PX4_DIR}" ]]; then
  echo "[myam_simulation] PX4 目录不存在: ${PX4_DIR}" >&2
  echo "  请传入 launch 参数 px4_dir:=/你的路径/PX4-Autopilot" >&2
  exit 1
fi

cd "${PX4_DIR}"
exec make px4_sitl gazebo-classic_myam_sys_d435
