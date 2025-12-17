import math
import os
import shutil
import subprocess
import threading
import time
from pathlib import Path
from typing import NamedTuple

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
from elevenlabs import ElevenLabs

# --- TTS settings ---
ELEVENLABS_API_KEY = "sk_4dc221c541f0f3f4ea1677ecb67ec807f24fe172bc7580f2"
ELEVENLABS_VOICE_ID = "9lHjugDhwqoxA5MhX0az"
MODEL_ID = "eleven_multilingual_v2"

TTS_TEXT = """你好呀，我是 Vesta 小助手，欢迎来到体验区。我会帮你挑选最合适的厨房抽油烟机，也会问你几个问题，了解你的烹饪空间、通风条件和安装习惯"""

OUTPUT_DIR = Path(__file__).parent / "voices"
OUTPUT_DIR.mkdir(exist_ok=True)
MP3_FILE = OUTPUT_DIR / "wave_hello.mp3"
WAV_FILE = OUTPUT_DIR / "wave_hello.wav"

ALSA_DEVICE = os.getenv("ALSA_DEVICE", "hw:0,0")
WAV_SAMPLE_RATE = os.getenv("WAV_SAMPLE_RATE", "48000")
WAV_CHANNELS = os.getenv("WAV_CHANNELS", "2")

# Shoulder joint-2 calibration offset: hardware zero differs by ~30°
# Positive means command pitch is lifted by this amount; flip the sign if needed on-site.
JOINT2_PITCH_OFFSET_RAD = math.radians(30.0)

_eleven_client = None


class HandPose(NamedTuple):
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    t_ms: int


class Gesture(NamedTuple):
    target: HandPose
    aux: HandPose


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
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return WAV_FILE


def get_wav_duration(path: Path) -> float:
    require_cmd("ffprobe", "sudo apt update && sudo apt install -y ffmpeg")
    try:
        out = subprocess.check_output(
            [
                "ffprobe",
                "-v",
                "error",
                "-show_entries",
                "format=duration",
                "-of",
                "default=noprint_wrappers=1:nokey=1",
                str(path),
            ],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
        return float(out)
    except Exception as exc:
        print(f"Warning: failed to read wav duration via ffprobe: {exc}")
        return 0.0


def play_wav_alsa(wav_path: Path) -> None:
    require_cmd("aplay", "sudo apt update && sudo apt install -y alsa-utils")
    subprocess.run(
        ["aplay", "-D", ALSA_DEVICE, "-v", str(wav_path)],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def ensure_hello_wav() -> Path:
    mp3 = synthesize_to_mp3(TTS_TEXT)
    return mp3_to_wav(mp3)


def move_hand(client: B1LocoClient, hand_index: B1HandIndex, pose: HandPose) -> int:
    """Move a single hand to target pose with an auxiliary waypoint, compensating joint-2 offset."""
    tar = Posture()
    tar.position = Position(pose.x, pose.y, pose.z)
    tar.orientation = Orientation(pose.roll, pose.pitch + JOINT2_PITCH_OFFSET_RAD, pose.yaw)

    aux = Posture()
    # Aux path: very close to target to avoid jittery arcs (small upward bias only).
    aux.position = Position(pose.x, pose.y, max(0.02, pose.z + 0.01))
    aux.orientation = tar.orientation

    rc = client.MoveHandEndEffectorWithAux(tar, aux, pose.t_ms, hand_index)
    if rc != 0:
        print(f"MoveHandEndEffectorWithAux failed rc={rc} (hand={hand_index}); fallback to V2")
        rc = client.MoveHandEndEffectorV2(tar, pose.t_ms, hand_index)
    return rc


def gesture_loop(client: B1LocoClient, stop_event: threading.Event) -> None:
    """Human-like dual-hand gestures while talking."""
    def deg_pose(x, y, z, roll_deg, pitch_deg, yaw_deg, t_ms) -> HandPose:
        return HandPose(x, y, z, math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg), t_ms)

    def mirror(pose: HandPose, sign: int) -> HandPose:
        return HandPose(
            pose.x,
            pose.y * sign,
            pose.z,
            pose.roll,
            pose.pitch,
            pose.yaw * sign,
            pose.t_ms,
        )

    # Gesture library from provided table
    gestures = {
        "OPEN_ARMS_GREETING": Gesture(
            target=deg_pose(0.35, 0.25, 0.95, 0, 20, 0, 1200),
            aux=deg_pose(0.30, 0.15, 1.05, 0, 0, 0, 1200),
        ),
        "POINTING_FORWARD": Gesture(
            target=deg_pose(0.55, 0.05, 1.00, 0, 0, 25, 900),
            aux=deg_pose(0.40, 0.05, 1.10, 0, 0, 0, 900),
        ),
        "POINTING_SIDE": Gesture(
            target=deg_pose(0.30, 0.45, 0.95, 0, 0, 30, 1000),
            aux=deg_pose(0.28, 0.30, 1.05, 0, 0, 0, 1000),
        ),
        "EMPHASIS_BEAT": Gesture(
            target=deg_pose(0.32, 0.18, 0.95, 0, -10, 0, 400),
            aux=deg_pose(0.30, 0.18, 0.90, 0, -5, 0, 400),
        ),
        "NARRATION_SWEEP": Gesture(
            target=deg_pose(0.40, 0.40, 1.05, 0, 15, 20, 1500),
            aux=deg_pose(0.38, 0.25, 1.20, 0, 10, 0, 1500),
        ),
        "THINKING_POSE": Gesture(
            target=deg_pose(0.20, 0.15, 1.10, 0, 30, 0, 1000),
            aux=deg_pose(0.25, 0.12, 1.20, 0, 0, 0, 1000),
        ),
        "REST_NEUTRAL": Gesture(
            target=deg_pose(0.25, 0.15, 0.70, 0, 0, 0, 600),
            aux=deg_pose(0.25, 0.15, 0.70, 0, 0, 0, 600),
        ),
        "HAPPY_BOUNCE": Gesture(
            target=deg_pose(0.33, 0.20, 1.05, 0, 20, 0, 400),
            aux=deg_pose(0.30, 0.20, 0.90, 0, 0, 0, 400),
        ),
    }

    def apply_pair(left_g: Gesture, right_g: Gesture, hold_s: float | None = None) -> None:
        move_hand(client, B1HandIndex.kLeftHand, mirror(left_g.target, 1))
        move_hand(client, B1HandIndex.kRightHand, mirror(right_g.target, -1))
        time.sleep(hold_s or max(left_g.target.t_ms, right_g.target.t_ms) / 1000.0)

    # Initial neutral
    apply_pair(gestures["REST_NEUTRAL"], gestures["REST_NEUTRAL"], 0.6)

    seq = [
        ("greet", gestures["OPEN_ARMS_GREETING"], gestures["OPEN_ARMS_GREETING"], 1.0),
        ("beat1", gestures["EMPHASIS_BEAT"], gestures["EMPHASIS_BEAT"], 0.45),
        ("sweep", gestures["NARRATION_SWEEP"], gestures["NARRATION_SWEEP"], 1.2),
        ("point_fwd", gestures["POINTING_FORWARD"], gestures["REST_NEUTRAL"], 0.9),
        ("beat2", gestures["EMPHASIS_BEAT"], gestures["EMPHASIS_BEAT"], 0.45),
        ("point_side", gestures["POINTING_SIDE"], gestures["REST_NEUTRAL"], 1.0),
        ("bounce1", gestures["HAPPY_BOUNCE"], gestures["HAPPY_BOUNCE"], 0.5),
        ("bounce2", gestures["HAPPY_BOUNCE"], gestures["HAPPY_BOUNCE"], 0.5),
        ("rest", gestures["REST_NEUTRAL"], gestures["REST_NEUTRAL"], 0.8),
    ]

    idx = 0
    try:
        while not stop_event.is_set():
            _, left_g, right_g, hold_s = seq[idx % len(seq)]
            apply_pair(left_g, right_g, hold_s)
            idx += 1
    finally:
        apply_pair(gestures["REST_NEUTRAL"], gestures["REST_NEUTRAL"], 0.8)


def main() -> None:
    nic = "127.0.0.1"
    domain = 0

    ChannelFactory.Instance().Init(domain, nic)
    print(f"DDS init ok domain={domain} nic={nic}")

    client = B1LocoClient()
    client.Init()
    print("client Init() ok")

    gm = GetModeResponse()
    last_rc = None
    for _ in range(12):
        rc = client.GetMode(gm)
        if rc == 0:
            break
        last_rc = rc
        time.sleep(0.25)
    else:
        raise RuntimeError(f"GetMode timeout: err={last_rc} (per T1 manual, 100 = RPC timeout)")
    print("current mode =", gm.mode)

    stop_gesture = threading.Event()
    gesture_thread = None
    try:
        wav_path = ensure_hello_wav()
        gesture_thread = threading.Thread(target=gesture_loop, args=(client, stop_gesture), daemon=True)
        gesture_thread.start()

        play_thread = threading.Thread(target=play_wav_alsa, args=(wav_path,), daemon=True)
        play_thread.start()

        play_thread.join(timeout=max(5.0, get_wav_duration(wav_path) + 1.0))
    finally:
        stop_gesture.set()
        if gesture_thread:
            gesture_thread.join(timeout=5.0)
        print("✅ gesture + speech session done")


if __name__ == "__main__":
    main()
