#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Call the built-in Dance API (>= v1.2.0.2) with a selected DanceId.
Usage examples:
    python dance.py nezha
    python dance.py 2
    python dance.py stop
"""

import argparse
import json
import sys
import time

from booster_robotics_sdk_python import (
    B1LocoClient,
    B1LocoApiId,
    ChannelFactory,
    GetModeResponse,
    RobotMode,
)

NIC = "127.0.0.1"  # DO NOT CHANGE
DOMAIN = 0         # DO NOT CHANGE

GETMODE_TRIES = 60
GETMODE_SLEEP = 0.25
MODE_SETTLE_S = 0.8

# Map SDK enum names to DanceId (case-insensitive)
DANCE_IDS = {
    "knewyear": 0,
    "knezha": 1,
    "ktowardsfuture": 2,
    "kpogbaguesture": 3,
    "kultramanguesture": 4,
    "kchinesegreetingguesture": 5,
    "kcheeringguesture": 6,
    "kmanekiguesture": 7,
    "kstop": 1000,
}

# LocoApiId::kDance (not exposed with a named constant in the binding, so we construct by value)
LOCO_API_ID_DANCE = B1LocoApiId(2016)


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


def change_mode_and_confirm(client, target_mode, name, retries=2):
    gm = GetModeResponse()
    rc_chk = client.GetMode(gm)
    if rc_chk == 0 and gm.mode == target_mode:
        print(f"Already in {name}")
        return True

    for attempt in range(1, retries + 1):
        rc = client.ChangeMode(target_mode)
        print(f"ChangeMode({name}) rc={rc} (attempt {attempt}/{retries})")
        time.sleep(MODE_SETTLE_S)

        for _ in range(20):
            rc2 = client.GetMode(gm)
            if rc2 == 0 and gm.mode == target_mode:
                return True
            time.sleep(0.25)

        print(f"⚠️ confirm {name} failed (attempt {attempt}) rc={rc} current={gm.mode}")

    return False


def parse_args():
    parser = argparse.ArgumentParser(description="Trigger built-in dance action.")
    parser.add_argument(
        "dance",
        nargs="?",
        default="knewyear",
        help=(
            "Dance name or id. SDK names only: knewyear, knezha, ktowardsfuture, "
            "kpogbaguesture, kultramanguesture, kchinesegreetingguesture, "
            "kcheeringguesture, kmanekiguesture, kstop."
        ),
    )
    parser.add_argument("--id", type=int, dest="dance_id", help="DanceId number (overrides name)")
    return parser.parse_args()


def resolve_dance_id(args):
    if args.dance_id is not None:
        return args.dance_id
    key = str(args.dance).lower()
    if key in DANCE_IDS:
        return DANCE_IDS[key]
    if key.isdigit():
        return int(key)
    raise ValueError(f"Unknown dance '{args.dance}'. Valid names: {', '.join(sorted(DANCE_IDS))}")


def call_dance(client, dance_id):
    payload = json.dumps({"dance_id": dance_id})
    rc = client.SendApiRequest(LOCO_API_ID_DANCE, payload)
    print(f"Dance(dance_id={dance_id}) rc={rc}")
    if rc != 0:
        raise RuntimeError(f"Dance API failed with rc={rc}")


def main():
    args = parse_args()
    dance_id = resolve_dance_id(args)

    ChannelFactory.Instance().Init(DOMAIN, NIC)
    print(f"DDS init ok domain={DOMAIN} nic={NIC}")

    client = B1LocoClient()
    client.Init()
    print("client Init() ok")

    mode = wait_getmode_ok(client)
    print("current mode =", mode)

    if mode != RobotMode.kWalking:
        ok_prepare = change_mode_and_confirm(client, RobotMode.kPrepare, "kPrepare")
        if not ok_prepare:
            raise RuntimeError("Cannot confirm kPrepare.")
        print("✅ in kPrepare, wait 3 seconds ...")
        time.sleep(3.0)

        ok_walking = change_mode_and_confirm(client, RobotMode.kWalking, "kWalking")
        if not ok_walking:
            raise RuntimeError("Cannot confirm kWalking.")
    else:
        print("Already in kWalking, skipping kPrepare")

    call_dance(client, dance_id)
    print("✅ Dance command sent")

    ok_walking_after = change_mode_and_confirm(client, RobotMode.kWalking, "kWalking")
    if not ok_walking_after:
        raise RuntimeError("Cannot return to kWalking after dance.")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:  # noqa: BLE001
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)
