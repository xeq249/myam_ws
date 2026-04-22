#!/usr/bin/env python3
"""墙钟 sleep 后 exec rosrun controller_manager spawner，延后加载控制器（先让 spawn_model 完成 -J / set_model_configuration）。"""
import os
import shutil
import sys
import time


def main():
    if len(sys.argv) < 3:
        print(
            "usage: delayed_controller_spawner.py <sleep_sec> <spawner args...>",
            file=sys.stderr,
        )
        sys.exit(1)
    delay = float(sys.argv[1])
    spawner_args = sys.argv[2:]
    time.sleep(delay)
    rosrun = shutil.which("rosrun")
    if not rosrun:
        print("rosrun not in PATH", file=sys.stderr)
        sys.exit(1)
    argv0 = ["rosrun", "controller_manager", "spawner"] + spawner_args
    os.execv(rosrun, argv0)


if __name__ == "__main__":
    main()
