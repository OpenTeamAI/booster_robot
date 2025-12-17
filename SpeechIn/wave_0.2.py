import time
import os
import shutil
import subprocess
from pathlib import Path
import threading

import booster_robotics_sdk_python as booster_sdk
from booster_robotics_sdk_python import B1LocoClient, ChannelFactory, GetModeResponse
from elevenlabs import ElevenLabs

ELEVENLABS_API_KEY = "sk_4dc221c541f0f3f4ea1677ecb67ec807f24fe172bc7580f2"
ELEVENLABS_VOICE_ID = "9lHjugDhwqoxA5MhX0az"
MODEL_ID = "eleven_multilingual_v2"

# TTS_TEXT = """你好呀，欢迎来到 Vesta technology！有什么可以帮到你的吗？我很乐意帮您挑选最适合您厨房的抽油烟机！为了给您提供最合适的建议，我需要了解您的具体需求和厨房布局。
# 让我问你几个问题：
# 您的烹饪空间属于哪种类型？是在公寓（可能存在通风限制）、独立式住宅还是开放式厨房？"""
TTS_TEXT = """你好呀，欢迎来到 Vesta technology！"""

OUTPUT_DIR = Path(__file__).parent / "voices"
OUTPUT_DIR.mkdir(exist_ok=True)
MP3_FILE = OUTPUT_DIR / "wave_hello.mp3"
WAV_FILE = OUTPUT_DIR / "wave_hello.wav"

ALSA_DEVICE = os.getenv("ALSA_DEVICE", "hw:0,0")
WAV_SAMPLE_RATE = os.getenv("WAV_SAMPLE_RATE", "48000")
WAV_CHANNELS = os.getenv("WAV_CHANNELS", "2")

_eleven_client = None


def require_cmd(cmd: str, install_hint: str) -> None:
    if not shutil.which(cmd):
        raise RuntimeError(f"Missing command: {cmd}\nInstall via: {install_hint}")


def get_eleven_client() -> ElevenLabs:
    global _eleven_client
    if _eleven_client is None:
        if not ELEVENLABS_API_KEY:
            raise RuntimeError(
                "Missing ELEVENLABS_API_KEY environment variable.\n"
                'Set it first: export ELEVENLABS_API_KEY="your_key"'
            )
        _eleven_client = ElevenLabs(api_key=ELEVENLABS_API_KEY)
    return _eleven_client


def synthesize_to_mp3(text: str) -> Path:
    client = get_eleven_client()
    audio_iter = client.text_to_speech.convert(
        voice_id=ELEVENLABS_VOICE_ID,
        model_id=MODEL_ID,
        text=text,
        output_format="mp3_44100_128",
    )
    audio_bytes = b"".join(audio_iter)
    MP3_FILE.write_bytes(audio_bytes)
    return MP3_FILE


def mp3_to_wav(mp3_path: Path) -> Path:
    require_cmd("ffmpeg", "sudo apt update && sudo apt install -y ffmpeg")
    subprocess.run(
        [
            "ffmpeg",
            "-y",
            "-i",
            str(mp3_path),
            "-ar",
            WAV_SAMPLE_RATE,
            "-ac",
            WAV_CHANNELS,
            "-c:a",
            "pcm_s16le",
            str(WAV_FILE),
        ],
        check=True,
    )
    return WAV_FILE


def play_wav_alsa(wav_path: Path) -> None:
    require_cmd("aplay", "sudo apt update && sudo apt install -y alsa-utils")
    subprocess.run(["aplay", "-D", ALSA_DEVICE, "-v", str(wav_path)], check=False)


def ensure_hello_wav() -> Path:
    mp3 = synthesize_to_mp3(TTS_TEXT)
    return mp3_to_wav(mp3)


def speak_hello_world():
    """Speak a short greeting once (best-effort; does not interrupt robot control)."""
    try:
        wav_path = ensure_hello_wav()
        play_wav_alsa(wav_path)
    except Exception as exc:
        print(f"Warning: failed to speak hello world via ElevenLabs: {exc}")


def wave_hand_until_stop(client: B1LocoClient, kHandOpen, kHandClose, stop_event: threading.Event) -> None:
    """Send wave open, keep it active until stop_event is set, then send wave close."""
    opened = False
    try:
        rc = client.WaveHand(kHandOpen)
        if rc != 0:
            print(f"WaveHand(kHandOpen) failed: err={rc}")
            return
        opened = True
        while not stop_event.is_set():
            time.sleep(0.1)
    finally:
        if opened:
            rc_close = client.WaveHand(kHandClose)
            if rc_close != 0:
                print(f"WaveHand(kHandClose) failed: err={rc_close}")

nic = "127.0.0.1"
domain = 0

ChannelFactory.Instance().Init(domain, nic)
print(f"DDS init ok domain={domain} nic={nic}")

client = B1LocoClient()
client.Init()  # 先用无参
print("client Init() ok")

last = None
for i in range(12):
    resp = GetModeResponse()
    rc = client.GetMode(resp)
    if rc == 0:
        print("mode =", resp.mode)

        # Resolve HandAction enums from the module (bindings expose them here)
        kHandOpen = getattr(booster_sdk.B1HandAction, "kHandOpen", None)
        kHandClose = getattr(booster_sdk.B1HandAction, "kHandClose", None)
        if kHandOpen is None or kHandClose is None:
            kHandOpen = getattr(booster_sdk, "kHandOpen", None)
            kHandClose = getattr(booster_sdk, "kHandClose", None)

        if kHandOpen is None or kHandClose is None:
            raise RuntimeError("Cannot find HandAction enums: kHandOpen/kHandClose in this python binding.")

        # 语音和挥手各自线程，主线程只负责同步
        stop_wave = threading.Event()
        wave_thread = threading.Thread(
            target=wave_hand_until_stop,
            args=(client, kHandOpen, kHandClose, stop_wave),
            daemon=True,
        )
        wave_thread.start()

        speech_thread = threading.Thread(target=speak_hello_world, daemon=True)
        speech_thread.start()

        # 等语音结束后停止挥手
        speech_thread.join(timeout=30.0)
        if speech_thread.is_alive():
            print("Warning: speech playback did not finish within timeout; forcing wave stop")

        stop_wave.set()
        wave_thread.join(timeout=5.0)
        if wave_thread.is_alive():
            print("Warning: wave thread did not exit cleanly")
        else:
            print("✅ waved while speaking and stopped")
        break
    last = rc
    time.sleep(0.25)
else:
    raise RuntimeError(f"GetMode timeout: err={last} (per T1 manual, 100 = RPC timeout)")
