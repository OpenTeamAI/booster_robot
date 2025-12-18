import os
import collections
from io import BytesIO

import numpy as np
import sounddevice as sd
import soundfile as sf
import webrtcvad
from elevenlabs.client import ElevenLabs
import time
import threading


# =====================
# CONFIG
# =====================
SAMPLE_RATE = 16000
CHANNELS = 1
FRAME_DURATION_MS = 30            # 10, 20, or 30 only (WebRTC requirement)
FRAME_SIZE = int(SAMPLE_RATE * FRAME_DURATION_MS / 1000)

VAD_AGGRESSIVENESS = 3             # 0–3 (higher = stricter)
MAX_SILENCE_FRAMES = 10            # ~300 ms silence to stop
MIC_DEVICE_INDEX = 1               # audio input device index
GAIN = 1.5                       # 1.0 = no change; boost signal if too quiet
NOISE_GATE = 500                   # zero samples with |amp| below this (basic suppression)
DC_REMOVE = True                   # remove DC offset per frame


# =====================
# MAIN FUNCTION
# =====================
def listen_once() -> str:
    """
    Blocks until user speaks, auto-stops on silence,
    sends audio to ElevenLabs STT, returns text.
    """
    done_event = threading.Event()

    vad = webrtcvad.Vad(VAD_AGGRESSIVENESS)
    ring_buffer = collections.deque(maxlen=MAX_SILENCE_FRAMES)

    voiced_frames = []
    triggered = False

    api_key = "sk_2d51c10b9617fe9fcca1860b83c5773aed7cc0f048a8edfb"
    client = ElevenLabs(api_key=api_key)

    def preprocess_frame(indata: np.ndarray) -> np.ndarray:
        """
        Light noise suppression: remove DC offset and gate low-level hum before VAD.
        Returns int16 audio with the same shape as input.
        """
        audio = indata.astype(np.int32)
        if DC_REMOVE:
            audio = audio - audio.mean(axis=0, keepdims=True)
        if NOISE_GATE > 0:
            audio = np.where(np.abs(audio) < NOISE_GATE, 0, audio)
        return np.clip(audio, -32768, 32767).astype(np.int16)

    def audio_callback(indata, frames, time, status):
        nonlocal triggered, voiced_frames, ring_buffer

        if status:
            print(status)

        cleaned = preprocess_frame(indata)
        pcm16 = cleaned[:, 0].tobytes()
        is_speech = vad.is_speech(pcm16, SAMPLE_RATE)

        if not triggered:
            ring_buffer.append((pcm16, is_speech))
            speech_ratio = sum(1 for _, s in ring_buffer if s) / ring_buffer.maxlen

            if speech_ratio > 0.8:
                triggered = True
                print("[VAD] Speech start detected")
                voiced_frames.extend(f for f, _ in ring_buffer)
                ring_buffer.clear()
        else:
            voiced_frames.append(pcm16)
            ring_buffer.append((pcm16, is_speech))

            silence_ratio = sum(1 for _, s in ring_buffer if not s) / ring_buffer.maxlen
            if silence_ratio > 0.8:
                print("[VAD] Speech end detected")
                done_event.set()
                raise sd.CallbackStop()

    print("Listening (speak now)...")

    with sd.InputStream(
        samplerate=SAMPLE_RATE,
        channels=CHANNELS,
        dtype="int16",
        blocksize=FRAME_SIZE,
        device=MIC_DEVICE_INDEX,
        callback=audio_callback,
    ):
        done_event.wait(timeout=10.0)  # safety timeout

    if not voiced_frames:
        return ""

    # =====================
    # Convert to WAV (memory)
    # =====================
    print("[AUDIO] Converting PCM to WAV...")
    t_wav_start = time.time()

    wav_io = BytesIO()
    audio = np.frombuffer(b"".join(voiced_frames), dtype=np.int16).astype(np.float32)
    audio = np.clip(audio * GAIN, -32768, 32767).astype(np.int16)
    sf.write(wav_io, audio, SAMPLE_RATE, format="WAV", subtype="PCM_16")
    wav_io.seek(0)

    t_wav_end = time.time()
    # print(f"[TIMING] WAV conversion: {t_wav_end - t_wav_start:.3f}s")

    # =====================
    # ElevenLabs STT
    # =====================
    t_stt_start = time.time()

    result = client.speech_to_text.convert(
        file=wav_io,
        model_id="scribe_v1",
        language_code="eng",
    )

    t_stt_end = time.time()
    # print(f"[TIMING] ElevenLabs STT request: {t_stt_end - t_stt_start:.3f}s")

    return result.text.strip()


# =====================
# TEST
# =====================
if __name__ == "__main__":
    text = listen_once()
    print("\nFINAL TRANSCRIPT:")
    print(text)
