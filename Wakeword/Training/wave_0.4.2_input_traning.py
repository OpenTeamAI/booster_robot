#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Wake-word listener (hi vesta) using the local ONNX model in wakeword/Wakeword_listen.py."""

import os
import sys
from pathlib import Path

# Prefer stdlib wave instead of local wave.py in this folder.
_SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
sys.path = [p for p in sys.path if p not in ("", _SCRIPT_DIR)]

# Make sure we can import the wakeword module alongside this script.
WAKEWORD_DIR = Path(__file__).parent / "wakeword"
if str(WAKEWORD_DIR) not in sys.path:
    sys.path.insert(0, str(WAKEWORD_DIR))

from Wakeword_listen import listen_for_wakeword  # type: ignore

MAX_LISTEN_SECONDS = float(os.getenv("WAVE_MAX_LISTEN_SECONDS", "15"))
# 0.4.2: no greeting playback to avoid extra deps.
PLAY_GREETING = False


def main() -> None:
    detected = listen_for_wakeword(
        max_listen_seconds=MAX_LISTEN_SECONDS,
        play_greeting=PLAY_GREETING,
    )
    if detected:
        print("Wake word detected.")
    else:
        print("Wake word not detected within time limit.")
        sys.exit(1)


if __name__ == "__main__":
    main()
