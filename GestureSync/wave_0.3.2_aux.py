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

JOINT2_PITCH_OFFSET_RAD = 0.0

BASE = {"x": 0.28, "y": -0.25, "z": 0.05, "r": 0.0, "p": 0.0, "yaw": 0.0}
DELTA_Y = 0.03
T_MS = 600
HOLD_S = 0.15

# 仅使用 MoveHandEndEffectorWithAux，不做旧 API 兜底。

GETMODE_TRIES = 60
GETMODE_SLEEP = 0.25
MODE_SETTLE_S = 0.8


def mk_posture(x, y, z, r_deg, p_deg, yaw_deg):
    p = Posture()
    p.position = Position(x, y, z)
    p.orientation = Orientation(
        math.radians(r_deg),
        math.radians(p_deg) + JOINT2_PITCH_OFFSET_RAD,
        math.radians(yaw_deg),
    )
    return p


def get_mode(client):
    gm = GetModeResponse()
    rc = client.GetMode(gm)
    return rc, gm.mode


def wait_getmode_ok(client, tries=GETMODE_TRIES, sleep_s=GETMODE_SLEEP):
    gm = GetModeResponse()
    last_rc = None
    for i in range(tries):
        rc = client.GetMode(gm)
        if rc == 0:
            return gm.mode
        last_rc = rc
        print(f"[GetMode {i+1}/{tries}] rc={rc} (100=RPC timeout)")
        time.sleep(sleep_s)
    raise RuntimeError(f"GetMode timeout: last_rc={last_rc}")


def change_mode_and_confirm(client, target, label, settle_s=MODE_SETTLE_S):
    rc = client.ChangeMode(target)
    print(f"ChangeMode({label}) rc={rc}")
    if rc != 0:
        return False
    time.sleep(settle_s)
    for i in range(20):
        gm = GetModeResponse()
        rc2 = client.GetMode(gm)
        print(f"[Confirm {label} {i+1}/20] GetMode rc={rc2} mode={gm.mode}")
        if rc2 == 0 and gm.mode == target:
            return True
        time.sleep(0.25)
    return False


def fmt_pose(tag, p: Posture):
    return f"{tag} pos=({p.position.x:.3f},{p.position.y:.3f},{p.position.z:.3f})"


def move_withaux(client, target, aux, t_ms, hand):
    print(" ", fmt_pose("target", target), f"t={t_ms}ms")
    print(" ", fmt_pose("aux   ", aux))
    rc = client.MoveHandEndEffectorWithAux(target, aux, t_ms, hand)
    rc_m, mode = get_mode(client)
    print(f"  [WithAux] rc={rc} | GetMode rc={rc_m} mode={mode}")
    time.sleep(t_ms / 1000.0 + HOLD_S)
    return rc


def main():
    # DO NOT CHANGE
    nic = "127.0.0.1"
    domain = 0

    ChannelFactory.Instance().Init(domain, nic)
    print(f"DDS init ok domain={domain} nic={nic}")

    client = B1LocoClient()
    client.Init()
    print("client Init() ok")

    mode = wait_getmode_ok(client)
    print("current mode =", mode)

    print("\n=== Startup: kPrepare -> kWalking ===")
    _ = change_mode_and_confirm(client, RobotMode.kPrepare, "kPrepare")
    ok_walking = change_mode_and_confirm(client, RobotMode.kWalking, "kWalking")
    print("kWalking ok =", ok_walking)
    if not ok_walking:
        raise RuntimeError("Cannot confirm kWalking.")

    def target_at(y):
        return mk_posture(BASE["x"], y, BASE["z"], BASE["r"], BASE["p"], BASE["yaw"])

    targets = [
        ("BASE",  target_at(BASE["y"])),
        ("LEFT",  target_at(BASE["y"] - DELTA_Y)),
        ("RIGHT", target_at(BASE["y"] + DELTA_Y)),
        ("BASE2", target_at(BASE["y"])),
    ]

    fixed_aux = mk_posture(0.25, -0.18, 0.28, 0.0, 0.0, 0.0)

    print("\n=== Test A: aux == target (should behave like no-aux) ===")
    for name, t in targets:
        print(f"\nTarget {name}:")
        move_withaux(client, t, t, T_MS, B1HandIndex.kRightHand)

    print("\n=== Test B: fixed aux point (your style) ===")
    for name, t in targets:
        print(f"\nTarget {name}:")
        move_withaux(client, t, fixed_aux, T_MS, B1HandIndex.kRightHand)

    print("\n✅ done. If Test A still doesn't move at all while rc==0, WithAux path is being ignored/blocked by controller.")

if __name__ == "__main__":
    main()
