#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Record audio immediately on startup and save it locally (hardcoded settings)."""

import os
import sys
from datetime import datetime
from pathlib import Path

# Prefer stdlib wave instead of local wave.py in this folder.
_SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
sys.path = [p for p in sys.path if p not in ("", _SCRIPT_DIR)]
import wave
import sounddevice as sd
import numpy as np

OUTPUT_DIR = Path(__file__).parent / "recordings"
SECONDS = 5.0
SAMPLE_RATE = 16000
CHANNELS = 1
DEVICE = 1  # 固定使用的麦克风设备索引，可用麦克风列表: [1] XFM-DP-V0.0.18: USB Audio (hw:1,0) in=1, [33]/[38] pulse/default 路由到默认源
GAIN = 5.0  # 线性增益
NORMALIZE = True
NORMALIZE_TARGET = 30000  # int16 peak headroom


def record_once() -> Path:
    """Record a single clip and return the path to the saved wav file."""
    OUTPUT_DIR.mkdir(exist_ok=True)
    frames = int(SECONDS * SAMPLE_RATE)
    print(f"Recording {SECONDS:.2f}s @ {SAMPLE_RATE} Hz ({CHANNELS}ch) using device={DEVICE} ...")

    audio = sd.rec(frames, samplerate=SAMPLE_RATE, channels=CHANNELS, dtype="int16", device=DEVICE)
    sd.wait()
    raw_peak = int(np.abs(audio).max()) if audio.size else 0

    # Apply gain/normalization
    if audio.size:
        audio_f = audio.astype(np.float32)
        if GAIN != 1.0:
            audio_f *= GAIN
        if NORMALIZE and raw_peak > 0:
            norm_factor = NORMALIZE_TARGET / raw_peak
            audio_f *= norm_factor
        audio = np.clip(audio_f, -32768, 32767).astype(np.int16)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = OUTPUT_DIR / f"input_{timestamp}.wav"
    with wave.open(str(out_path), "wb") as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(2)  # int16
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(audio.tobytes())

    return out_path


def main() -> None:
    try:
        out_path = record_once()
    except Exception as exc:
        print(f"Recording failed: {exc}", file=sys.stderr)
        sys.exit(1)
    print(f"Saved recording to {out_path}")


if __name__ == "__main__":
    main()
