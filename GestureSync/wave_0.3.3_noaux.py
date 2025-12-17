#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import time
from typing import Dict, NamedTuple

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

HOLD_S = 0.12
MIN_PHASE_MS = 400  # 每段最短时间，避免过快导致控制器忽略
# 默认姿态缩放和偏移；位置会再被 clamping 到安全盒范围
POSE_SCALE = float(os.getenv("POSE_SCALE", "1.0"))  # 如需更小可设 0.6~0.8
POSE_Z_OFFSET = float(os.getenv("POSE_Z_OFFSET", "-0.05"))

GETMODE_TRIES = 60
GETMODE_SLEEP = 0.25
MODE_SETTLE_S = 0.8

WIGGLE = {"x": 0.28, "y": 0.25, "z": 0.08, "delta_y": 0.04, "t_ms": 600}
SAFE_BOX = {
    "x_min": 0.24,
    "x_max": 0.40,
    "y_abs_max": 0.25,
    "z_min": 0.05,
    "z_max": 0.35,
}


class HandPose(NamedTuple):
    x: float
    y: float
    z: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


class Gesture(NamedTuple):
    target: HandPose  # 最终位姿
    aux: HandPose     # 辅助点（手动两段路径）
    time_ms: int
    desc: str
    time_hint: str


def deg_pose(x, y, z, roll_deg, pitch_deg, yaw_deg) -> HandPose:
    return HandPose(x, y, z, roll_deg, pitch_deg, yaw_deg)


GESTURES: Dict[str, Gesture] = {
    "OPEN_ARMS_GREETING": Gesture(
        target=deg_pose(0.32, 0.18, 0.22, 0.0, 12.0, 0.0),
        aux=deg_pose(0.30, 0.12, 0.26, 0.0, 5.0, 0.0),
        time_ms=1000,
        time_hint="900-1100",
        desc='适合打招呼/开场白/欢迎客人/自我介绍前，先上扬再微张双臂',
    ),
    "POINTING_FORWARD": Gesture(
        target=deg_pose(0.55, 0.05, 1.00, 0.0, 0.0, 25.0),
        aux=deg_pose(0.40, 0.05, 1.10, 0.0, 0.0, 0.0),
        time_ms=900,
        time_hint="800-1000",
        desc='手向前伸直，轨迹先上后前，形成自然"指路"动作',
    ),
    "POINTING_SIDE": Gesture(
        target=deg_pose(0.30, 0.45, 0.95, 0.0, 0.0, 30.0),
        aux=deg_pose(0.28, 0.30, 1.05, 0.0, 0.0, 0.0),
        time_ms=1050,
        time_hint="900-1200",
        desc="手画一个向侧边展开的大弧线，最终指向侧方",
    ),
    "EMPHASIS_BEAT": Gesture(
        target=deg_pose(0.32, 0.18, 0.95, 0.0, 10.0, 0.0),
        aux=deg_pose(0.30, 0.18, 0.90, 0.0, -5.0, 0.0),
        time_ms=400,
        time_hint="350-450",
        desc='轻微"点一下"的上下节拍动作，可重复使用',
    ),
    "NARRATION_SWEEP": Gesture(
        target=deg_pose(0.40, 0.40, 1.05, 0.0, 15.0, 20.0),
        aux=deg_pose(0.38, 0.25, 1.20, 0.0, 10.0, 0.0),
        time_ms=1500,
        time_hint="1400-1600",
        desc="大幅 sweeping 动作，像在解释或描述概念",
    ),
    "THINKING_POSE": Gesture(
        target=deg_pose(0.20, 0.15, 1.10, 0.0, 30.0, 0.0),
        aux=deg_pose(0.25, 0.12, 1.20, 0.0, 0.0, 0.0),
        time_ms=1000,
        time_hint="800-1200",
        desc='手慢慢抬到胸前/下巴旁，呈现"思考中"轨迹',
    ),
    "REST_NEUTRAL": Gesture(
        target=deg_pose(0.25, 0.15, 0.70, 0.0, 0.0, 0.0),
        aux=deg_pose(0.25, 0.15, 0.70, 0.0, 0.0, 0.0),
        time_ms=600,
        time_hint="600",
        desc="自然回落到默认休息位，轨迹平滑收回",
    ),
    "HAPPY_BOUNCE": Gesture(
        target=deg_pose(0.33, 0.20, 1.05, 0.0, 20.0, 0.0),
        aux=deg_pose(0.30, 0.20, 0.90, 0.0, 0.0, 0.0),
        time_ms=400,
        time_hint="~400 per cycle",
        desc='手上下快速弹跳，两点之间往返形成"开心跳动"效果',
    ),
}


def mk_posture(x, y, z, r_deg, p_deg, yaw_deg):
    p = Posture()
    p.position = Position(x, y, z)
    p.orientation = Orientation(
        math.radians(r_deg),
        math.radians(p_deg) + JOINT2_PITCH_OFFSET_RAD,
        math.radians(yaw_deg),
    )
    return p


def pose_to_posture(pose: HandPose, mirror_sign: int) -> Posture:
    """Convert HandPose (deg) to Posture (rad), mirroring y/yaw for right hand."""
    return mk_posture(
        pose.x,
        pose.y * mirror_sign,
        pose.z,
        pose.roll_deg,
        pose.pitch_deg,
        pose.yaw_deg * mirror_sign,
    )


def fmt_hand_pose(label: str, pose: HandPose, sign: int) -> str:
    y = pose.y * sign
    yaw = pose.yaw_deg * sign
    return (
        f"{label} x={pose.x:.2f} y={y:+.2f} z={pose.z:.2f} "
        f"r={pose.roll_deg:+.1f} p={pose.pitch_deg:+.1f} yaw={yaw:+.1f} (deg)"
    )


def hand_sign(hand: B1HandIndex) -> int:
    return 1 if hand == B1HandIndex.kLeftHand else -1


def scale_pose(pose: HandPose) -> HandPose:
    """Apply global scale/offset, then clamp into a safe reachable box."""
    x = pose.x * POSE_SCALE
    y = pose.y * POSE_SCALE
    z = pose.z * POSE_SCALE + POSE_Z_OFFSET

    x = min(max(x, SAFE_BOX["x_min"]), SAFE_BOX["x_max"])
    y = max(-SAFE_BOX["y_abs_max"], min(SAFE_BOX["y_abs_max"], y))
    z = min(max(z, SAFE_BOX["z_min"]), SAFE_BOX["z_max"])

    return HandPose(
        x,
        y,
        z,
        pose.roll_deg,
        pose.pitch_deg,
        pose.yaw_deg,
    )


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


def move_noaux(client, target, t_ms, hand):
    print(" ", fmt_pose("target", target), f"t={t_ms}ms")
    rc = client.MoveHandEndEffector(target, t_ms, hand)
    gm = GetModeResponse()
    rc_m = client.GetMode(gm)
    print(f"  [NoAux] rc={rc} | GetMode rc={rc_m} mode={gm.mode}")
    time.sleep(t_ms / 1000.0 + HOLD_S)
    return rc


def run_gesture(client: B1LocoClient, name: str, gesture: Gesture, hand: B1HandIndex):
    """模拟无 aux 版：先到 aux，再到 target，分两段 MoveHandEndEffector。"""
    sign = hand_sign(hand)
    scaled_aux = scale_pose(gesture.aux)
    scaled_target = scale_pose(gesture.target)
    aux_posture = pose_to_posture(scaled_aux, sign)
    target_posture = pose_to_posture(scaled_target, sign)

    phase1 = max(MIN_PHASE_MS, gesture.time_ms // 2)
    phase2 = max(MIN_PHASE_MS, gesture.time_ms - phase1)

    hand_label = getattr(hand, "name", str(hand))
    print(f"\n-- {name} ({hand_label}) {gesture.desc}")
    print(f"   time_millis={gesture.time_ms} (建议 {gesture.time_hint}) -> phase1={phase1}ms phase2={phase2}ms")
    print(f"   scale={POSE_SCALE} z_offset={POSE_Z_OFFSET}")
    print("  ", fmt_hand_pose("aux    ", scaled_aux, sign))
    move_noaux(client, aux_posture, phase1, hand)
    print("  ", fmt_hand_pose("target ", scaled_target, sign))
    move_noaux(client, target_posture, phase2, hand)


def run_wiggle(client: B1LocoClient, hand: B1HandIndex):
    """简单 BASE/LEFT/RIGHT/BASE2 小摆动，用于快速确认通道可动."""
    sign = hand_sign(hand)
    y_base = WIGGLE["y"] * sign
    y_left = (WIGGLE["y"] + WIGGLE["delta_y"]) * sign
    y_right = (WIGGLE["y"] - WIGGLE["delta_y"]) * sign
    pts = [
        ("BASE", mk_posture(WIGGLE["x"], y_base, WIGGLE["z"], 0, 0, 0)),
        ("LEFT", mk_posture(WIGGLE["x"], y_left, WIGGLE["z"], 0, 0, 0)),
        ("RIGHT", mk_posture(WIGGLE["x"], y_right, WIGGLE["z"], 0, 0, 0)),
        ("BASE2", mk_posture(WIGGLE["x"], y_base, WIGGLE["z"], 0, 0, 0)),
    ]
    hand_label = getattr(hand, "name", str(hand))
    print(f"\n=== Wiggle test ({hand_label}) ===")
    for name, p in pts:
        print(f"\nTarget {name}:")
        move_noaux(client, p, WIGGLE["t_ms"], hand)


def print_gesture_library():
    print("\n=== 手势库（no-aux，两段路径：aux -> target） ===")
    for name, g in GESTURES.items():
        print(f"{name}: time_millis={g.time_ms} (参考 {g.time_hint}) | {g.desc}")
        print(f"  target: x={g.target.x:.2f}, y=+/-{g.target.y:.2f}, z={g.target.z:.2f}, "
              f"r={g.target.roll_deg:.1f}, p={g.target.pitch_deg:.1f}, yaw={g.target.yaw_deg:.1f}")
        print(f"  aux   : x={g.aux.x:.2f}, y=+/-{g.aux.y:.2f}, z={g.aux.z:.2f}, "
              f"r={g.aux.roll_deg:.1f}, p={g.aux.pitch_deg:.1f}, yaw={g.aux.yaw_deg:.1f}")


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

    print_gesture_library()

    # 先跑一个小幅 wiggle 测试，确认通道可动
    # run_wiggle(client, B1HandIndex.kRightHand)
    # run_wiggle(client, B1HandIndex.kLeftHand)

    print("\n=== Move both hands to REST_NEUTRAL ===")
    for hand in (B1HandIndex.kLeftHand, B1HandIndex.kRightHand):
        run_gesture(client, "REST_NEUTRAL", GESTURES["REST_NEUTRAL"], hand)

    sequence = [
        "OPEN_ARMS_GREETING",
        # "REST_NEUTRAL",
    ]
    # 保留全量手势顺序的注释，方便随时切回：
    # sequence = [
    #     "OPEN_ARMS_GREETING",
    #     "POINTING_FORWARD",
    #     "POINTING_SIDE",
    #     "EMPHASIS_BEAT",
    #     "NARRATION_SWEEP",
    #     "THINKING_POSE",
    #     "HAPPY_BOUNCE",
    #     "HAPPY_BOUNCE",
    #     "REST_NEUTRAL",
    # ]

    print("\n=== Greeting-only demo: OPEN_ARMS_GREETING -> REST_NEUTRAL ===")
    for name in sequence:
        g = GESTURES[name]
        for hand in (B1HandIndex.kLeftHand, B1HandIndex.kRightHand):
            run_gesture(client, name, g, hand)

    print("\n✅ done. 手势完成，已回到 REST_NEUTRAL。")

if __name__ == "__main__":
    main()
