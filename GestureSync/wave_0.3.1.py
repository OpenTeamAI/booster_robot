#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import time
import socket
import subprocess

import booster_robotics_sdk_python as sdk
from booster_robotics_sdk_python import (
    B1LocoClient,
    ChannelFactory,
    GetModeResponse,
    RobotMode,
)

VERSION_FILE = "/opt/booster/version.txt"


def read_versions(path: str):
    if not os.path.isfile(path):
        return []
    out = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = re.search(r"Version:\s*(\S+)", line.strip())
            if m:
                out.append(m.group(1))
    return out


def get_mode(client: B1LocoClient):
    gm = GetModeResponse()
    rc = client.GetMode(gm)
    return rc, getattr(gm, "mode", None)


def wait_get_mode_ok(client: B1LocoClient, tries=60, sleep_s=0.25):
    """
    Wait until GetMode returns rc==0.
    rc=100 in your manual => RPC timeout (service not responding).
    """
    last = None
    for i in range(tries):
        rc, mode = get_mode(client)
        print(f"[GetMode try {i+1}/{tries}] rc={rc} mode={mode}")
        if rc == 0:
            return mode
        last = rc
        time.sleep(sleep_s)
    raise RuntimeError(
        f"GetMode never returned rc=0. last_rc={last}. "
        "This is RPC timeout (service not responding)."
    )


def wait_mode(client: B1LocoClient, target: RobotMode, tries=40, sleep_s=0.25):
    last = None
    for _ in range(tries):
        rc, mode = get_mode(client)
        if rc == 0 and mode == target:
            return True
        last = (rc, mode)
        time.sleep(sleep_s)
    print("wait_mode last:", last)
    return False


def change_mode_safe(client: B1LocoClient, target: RobotMode, label: str, settle_s=0.8):
    rc = client.ChangeMode(target)
    print(f"ChangeMode({label}) rc={rc}")
    if rc != 0:
        return False
    time.sleep(settle_s)
    ok = wait_mode(client, target)
    rc2, mode2 = get_mode(client)
    print(f"After ChangeMode({label}) => GetMode rc={rc2} mode={mode2} ok={ok}")
    return ok


def print_local_debug():
    # Purely prints debug info; does NOT change anything.
    print("\n=== Local debug (no changes) ===")
    print("sdk.__file__ =", getattr(sdk, "__file__", None))
    print("sdk.__version__ =", repr(getattr(sdk, "__version__", None)))

    # List network interfaces from /sys (no shell needed)
    try:
        ifaces = sorted(os.listdir("/sys/class/net"))
        print("interfaces:", ifaces)
    except Exception as e:
        print("interfaces: (failed to read /sys/class/net)", e)

    # Confirm loopback resolves
    try:
        print("localhost ->", socket.gethostbyname("localhost"))
    except Exception as e:
        print("localhost resolve failed:", e)

    # Optional: show brief `ip addr` if available (read-only). If you don't want subprocess, delete this.
    try:
        out = subprocess.check_output(["ip", "-brief", "addr"], text=True, stderr=subprocess.STDOUT)
        print("\nip -brief addr:\n", out.strip())
    except Exception:
        pass


def main():
    print("=== Booster system version (from file) ===")
    vs = read_versions(VERSION_FILE)
    print("version.txt entries:", vs)
    if vs:
        print("candidate current (last entry):", vs[-1])
    print()

    # !!! DO NOT CHANGE THESE TWO LINES (per your requirement) !!!
    nic = "127.0.0.1"
    domain = 0
    ChannelFactory.Instance().Init(domain, nic)
    print(f"DDS init ok domain={domain} nic={nic}")

    client = B1LocoClient()
    client.Init()
    print("client Init() ok")

    # If GetMode is timing out, print local debug and keep retrying
    try:
        mode = wait_get_mode_ok(client, tries=60, sleep_s=0.25)
        print("\n✅ GetMode connected. current mode =", mode)
    except RuntimeError as e:
        print_local_debug()
        raise

    # Startup mode sequence: Prepare -> Walking
    print("\n=== Startup mode sequence: kPrepare -> kWalking ===")
    _ = change_mode_safe(client, RobotMode.kPrepare, "kPrepare")
    ok = change_mode_safe(client, RobotMode.kWalking, "kWalking")
    if not ok:
        rc, mode = get_mode(client)
        raise RuntimeError(f"Failed to enter kWalking. GetMode rc={rc} mode={mode}")
    print("\n✅ Now in RobotMode.kWalking")

    # Optional: WaveHand 2s then stop (no enums)
    if hasattr(client, "WaveHand"):
        print("\n=== WaveHand (2s then stop) ===")
        def try_wave(open_val, close_val):
            rc1 = client.WaveHand(open_val)
            print(f"WaveHand({open_val}) rc:", rc1)
            if rc1 != 0:
                return False
            time.sleep(2.0)
            rc2 = client.WaveHand(close_val)
            print(f"WaveHand({close_val}) rc:", rc2)
            return rc2 == 0

        ok = try_wave(0, 1) or try_wave(1, 0)
        print("wave ok =", ok)

    print("\n✅ done")


if __name__ == "__main__":
    main()
