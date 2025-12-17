#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time

from booster_robotics_sdk_python import (
    B1HandIndex,
    B1LocoClient,
    ChannelFactory,
    GetModeResponse,
    Orientation,
    Position,
    Posture,
    RobotMode,
)

NIC = "127.0.0.1"   # DO NOT CHANGE
DOMAIN = 0          # DO NOT CHANGE

GETMODE_TRIES = 60
GETMODE_SLEEP = 0.25
MODE_SETTLE_S = 0.8
HOLD_S = 0.15

# Each point: (x, y, z, roll_deg, pitch_deg, yaw_deg, t_ms)
POINTS = [
    (0.25, -0.20, 0.1, 0.0, 0.0, 0.0, 800),
    (0.3, -0.18, 0.2, 0.0, 0.0, 0.0, 800),
    (0.25, -0.20, 0.1, 0.0, 0.0, 0.0, 800),
    (0.3, -0.18, 0.2, 0.0, 0.0, 0.0, 800),
    (0.25, -0.20, 0.1, 0.0, 0.0, 0.0, 800),
    (0.3, -0.18, 0.2, 0.0, 0.0, 0.0, 800),
    (0.25, -0.20, 0.1, 0.0, 0.0, 0.0, 800),
    (0.3, -0.18, 0.2, 0.0, 0.0, 0.0, 800),
    (0.25, -0.20, 0.1, 0.0, 0.0, 0.0, 800),
    (0.3, -0.18, 0.2, 0.0, 0.0, 0.0, 800),
    (0.25, -0.20, 0.1, 0.0, 0.0, 0.0, 800),
    (0.3, -0.18, 0.2, 0.0, 0.0, 0.0, 800),
    (0.25, -0.20, 0.1, 0.0, 0.0, 0.0, 800),
    (0.3, -0.18, 0.2, 0.0, 0.0, 0.0, 800),
    (0.25, -0.20, 0.1, 0.0, 0.0, 0.0, 800),
    # (0.38, -0.20, 0.1, 0.0, 0.0, 0.0, 800),
    # (0.25, -0.20, 0.1, 0.2, 0.0, 0.0, 800),
    # (0.22, -0.20, 0.1, 0.0, 0.0, 0.0, 800),

    # (0.40, -0.22, 0.2, 0.0, 0.0, 0.0, 800),

    # (0.25, -0.20, 0.1, 0.2, 0.0, 0.0, 800),
    # (0.40, -0.22, 0.1, 0.0, 0.0, 0.0, 800),
]
"""

x = 0.28：手端目标位置在 X 方向 0.28（单位通常是米，取决于 SDK 定义；在 T1 手册里一般是以 body 为参考的米制坐标）

y = -0.25：手端目标位置在 Y 方向 -0.25

负号通常表示朝机器人的右侧（右手常用负 y）

z = 0.08：手端目标位置在 Z 方向 0.08

z 越大抬得越高；0.08 属于比较低、偏“手放下”的高度

姿态（角度）部分是欧拉角，单位是“度”：

roll_deg = 0.0：绕 X 轴旋转 0°（手腕不侧翻）

pitch_deg = 0.0：绕 Y 轴旋转 0°（不前后俯仰）

yaw_deg = 0.0：绕 Z 轴旋转 0°（不左右转向）

时间：

t_ms = 800：让控制器用 800 毫秒完成这次移动（越大越慢越平滑，太小可能被忽略/抖动）

"""


def mk_posture(x, y, z, roll_deg, pitch_deg, yaw_deg):
    p = Posture()
    p.position = Position(x, y, z)
    p.orientation = Orientation(
        math.radians(roll_deg),
        math.radians(pitch_deg),
        math.radians(yaw_deg),
    )
    return p


def wait_getmode_ok(client):
    gm = GetModeResponse()
    last = None
    for i in range(GETMODE_TRIES):
        rc = client.GetMode(gm)
        if rc == 0:
            return gm.mode
        last = rc
        print(f"[GetMode {i+1}/{GETMODE_TRIES}] rc={rc}")
        time.sleep(GETMODE_SLEEP)
    raise RuntimeError(f"GetMode timeout: last_rc={last}")


def change_mode_and_confirm(client, target_mode, name):
    rc = client.ChangeMode(target_mode)
    print(f"ChangeMode({name}) rc={rc}")
    time.sleep(MODE_SETTLE_S)

    gm = GetModeResponse()
    for _ in range(20):
        rc2 = client.GetMode(gm)
        if rc2 == 0 and gm.mode == target_mode:
            return True
        time.sleep(0.25)

    print(f"⚠️ confirm {name} failed, current={gm.mode}")
    return False


def main():
    ChannelFactory.Instance().Init(DOMAIN, NIC)
    print(f"DDS init ok domain={DOMAIN} nic={NIC}")

    client = B1LocoClient()
    client.Init()
    print("client Init() ok")

    mode = wait_getmode_ok(client)
    print("current mode =", mode)

    # Prepare -> (wait 3s) -> Walking
    ok_prepare = change_mode_and_confirm(client, RobotMode.kPrepare, "kPrepare")
    if not ok_prepare:
        raise RuntimeError("Cannot confirm kPrepare.")
    print("✅ in kPrepare, wait 3 seconds ...")
    time.sleep(3.0)

    ok_walking = change_mode_and_confirm(client, RobotMode.kWalking, "kWalking")
    if not ok_walking:
        raise RuntimeError("Cannot confirm kWalking.")

    # Execute points in order
    for i, (x, y, z, r, p, yaw, t_ms) in enumerate(POINTS, start=1):
        target = mk_posture(x, y, z, r, p, yaw)
        print(f"[{i}/{len(POINTS)}] Move to ({x:.3f},{y:.3f},{z:.3f}) rpy=({r:.1f},{p:.1f},{yaw:.1f}) t={t_ms}ms")
        rc = client.MoveHandEndEffector(target, int(t_ms), B1HandIndex.kRightHand)

        gm = GetModeResponse()
        rc_m = client.GetMode(gm)
        print(f"  rc={rc} | GetMode rc={rc_m} mode={gm.mode}")

        if rc != 0:
            raise RuntimeError(f"MoveHandEndEffector failed: rc={rc}")

        time.sleep(t_ms / 1000.0 + HOLD_S)

    print("✅ done")


if __name__ == "__main__":
    main()
