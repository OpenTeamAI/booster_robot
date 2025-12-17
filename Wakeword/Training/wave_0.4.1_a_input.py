#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Wake-word listener (hi vesta) using openWakeWord, hardcoded settings."""

import os
import sys
import time
from pathlib import Path

# Prefer stdlib wave instead of local wave.py in this folder.
_SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
sys.path = [p for p in sys.path if p not in ("", _SCRIPT_DIR)]

import numpy as np
import sounddevice as sd

try:
    import openwakeword as oww
    from openwakeword import Model as WakeModel
except Exception:
    oww = None
    WakeModel = None

# === Hardcoded settings ===
SAMPLE_RATE = 16000
CHANNELS = 1
DEVICE = 1  # USB mic index; adjust if hardware changes.
CHUNK_SECONDS = 0.5
MAX_LISTEN_SECONDS = 15
SCORE_THRESHOLD = 0.5

# If you have a custom hi-vesta model (.tflite), put it here; otherwise falls back to built-in.
CUSTOM_MODEL = Path(__file__).parent / "wake_models" / "hi_vesta.tflite"
WAKE_NAME = "hi_vesta"


def load_wake_model() -> WakeModel:
    if WakeModel is None or oww is None:
        print("openwakeword not installed. Install: pip install --user openwakeword", file=sys.stderr)
        sys.exit(1)

    if CUSTOM_MODEL.exists():
        print(f"Loading custom wakeword model: {CUSTOM_MODEL}")
        return WakeModel(wakeword_models=[str(CUSTOM_MODEL)])

    # No custom model; fail fast with guidance.
    builtins = ", ".join(getattr(oww, "MODELS", {}).keys() or [])
    print(
        "Custom model wake_models/hi_vesta.tflite not found.\n"
        "Please provide the custom model to detect 'hi vesta'.\n"
        f"Built-in models available: {builtins}\n"
        "If you want to test built-ins, change WAKE_NAME and CUSTOM_MODEL accordingly."
    )
    sys.exit(1)


def listen_for_wake(model: WakeModel) -> bool:
    """Stream audio and return True if wake word is detected."""
    chunk_frames = int(CHUNK_SECONDS * SAMPLE_RATE)
    max_chunks = int(MAX_LISTEN_SECONDS / CHUNK_SECONDS)

    with sd.InputStream(
        samplerate=SAMPLE_RATE,
        channels=CHANNELS,
        dtype="float32",
        blocksize=chunk_frames,
        device=DEVICE,
    ) as stream:
        print(f"Listening for wake word '{WAKE_NAME}' for up to {MAX_LISTEN_SECONDS}s ...")
        for i in range(max_chunks):
            audio, _overflow = stream.read(chunk_frames)
            # openwakeword expects mono float32 array
            scores = model.predict(audio[:, 0])
            score = scores.get(WAKE_NAME, 0.0)
            print(f"[chunk {i+1}] score={score:.3f}")
            if score >= SCORE_THRESHOLD:
                print("success")
                return True
    print("Wake word not detected within time limit.")
    return False


def main() -> None:
    model = load_wake_model()
    listen_for_wake(model)


if __name__ == "__main__":
    main()
