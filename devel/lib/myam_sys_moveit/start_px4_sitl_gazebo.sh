#!/usr/bin/env bash
set -euo pipefail

MODEL="${1:-myam}"
WORLD="${2:-none}"
PX4_DIR="${3:-/home/demo/PX4-Autopilot}"
WAIT_PARAM="${WAIT_PARAM:-/myam_arm_description}"
# 暂停时 Gazebo 不步进、传感器不更新，PX4 易 poll timeout；联合调臂默认不暂停启动。
# 需要「先暂停再解暂停」时: export PX4_GAZEBO_START_PAUSED=1
export PX4_GAZEBO_START_PAUSED="${PX4_GAZEBO_START_PAUSED:-0}"

case "${MODEL}" in
  myam_sys_d435|myam_arm_d435)
    MODEL="myam"
    ;;
  myam_arm)
    MODEL="myam_roarm"
    ;;
esac

# 略抬高生成，减少解暂停后起落架/臂触地穿模；可用 PX4_GAZEBO_SPAWN_Z 覆盖
if [[ -z "${PX4_GAZEBO_SPAWN_Z:-}" ]] && [[ "${MODEL}" == "myam" ]]; then
  export PX4_GAZEBO_SPAWN_Z="1.1"
fi

if [[ ! -d "${PX4_DIR}" ]]; then
  echo "PX4 directory not found: ${PX4_DIR}" >&2
  exit 1
fi

cd "${PX4_DIR}"

if command -v rosparam >/dev/null 2>&1; then
  echo "Waiting for ROS parameter ${WAIT_PARAM} before starting PX4 SITL..."
  for _ in $(seq 1 120); do
    if rosparam get "${WAIT_PARAM}" >/dev/null 2>&1; then
      break
    fi
    sleep 0.5
  done
fi

if [[ "${WORLD}" == "none" || -z "${WORLD}" ]]; then
  exec make px4_sitl "gazebo-classic_${MODEL}"
else
  exec make px4_sitl "gazebo-classic_${MODEL}__${WORLD}"
fi
