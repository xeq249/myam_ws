#!/usr/bin/env bash
# 先 sleep，再启动 controller_manager spawner，避免与 spawn_model 的 set_model_configuration 竞态。
set -euo pipefail
DELAY="${1:?usage: delayed_controller_spawner.sh DELAY_SEC controller [controller ...]}"
shift
sleep "$DELAY"
exec rosrun controller_manager spawner "$@"
