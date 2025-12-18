import collections
import json
import os
import queue
import re
import shutil
import subprocess
import tempfile
import threading
from io import BytesIO
from pathlib import Path

import numpy as np
import requests
import sounddevice as sd
import soundfile as sf
import webrtcvad
from elevenlabs import ElevenLabs


API_BASE_URL = "https://www.openteam.ai/api/agent/v1/vesta_agent/chat"
API_NEW_URL = f"{API_BASE_URL}/new"
AUTH_TOKEN = (
    "Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9."
    "eyJ1c2VyX2lkIjoidXNlci04ZThhYzk2OC02YjhkLTQxNGEt"
    "YTI2YS1kNjE2NmM4ZGRhNmMiLCJpYXQiOjE3NTkxNjgyNjAs"
    "InNjb3BlIjoiYWRtaW4iLCJhZ2VudCI6InZlc3RhX2FnZW50"
    "IiwiZXhwIjoxODgzNTg0MjYwfQ.5KpLao9lVwHWBXGUfU_"
    "ptNMzwHi8MWsoPWMIuL0aqrU"
)

ELEVENLABS_API_KEY = "sk_4dc221c541f0f3f4ea1677ecb67ec807f24fe172bc7580f2"
ELEVENLABS_VOICE_ID = "9lHjugDhwqoxA5MhX0az"
ELEVENLABS_MODEL_ID = "eleven_multilingual_v2"

# STT config
SAMPLE_RATE = 16000
CHANNELS = 1
FRAME_DURATION_MS = 30
FRAME_SIZE = int(SAMPLE_RATE * FRAME_DURATION_MS / 1000)
VAD_AGGRESSIVENESS = 3
MAX_SILENCE_FRAMES = 10
MIC_DEVICE_INDEX = 1
GAIN = 1.5
NOISE_GATE = 500
DC_REMOVE = True
STT_MODEL_ID = "scribe_v1"
STT_LANGUAGE_CODE = "zho"
LISTEN_TIMEOUT_SEC = 10.0

# TTS config
OUTPUT_DIR = Path("./voices")
ALSA_DEVICE = os.getenv("ALSA_DEVICE", "hw:0,0")
WAV_SAMPLE_RATE = "48000"
WAV_CHANNELS = "2"
STREAM_TTS_MIN_CHARS_STRONG = 12
STREAM_TTS_MAX_CHARS = 160
STRONG_ENDINGS = set("\u3002\uff01\uff1f!?")
LOG_PATH = Path("./conversation_log.jsonl")


def require_cmd(cmd: str, install_hint: str) -> None:
    if not shutil.which(cmd):
        raise RuntimeError(f"Missing command: {cmd}\nInstall: {install_hint}")


def remove_button_tags(text: str) -> str:
    return text.replace("[[[BUTTON]]]", "").replace("[[[/BUTTON]]]", "")


def sanitize_text(text: str) -> str:
    text = remove_button_tags(text)
    text = re.sub(r"\s+\n", "\n", text)
    return text.strip()


def has_incomplete_button_tag(text: str) -> bool:
    start = text.rfind("[[[")
    if start == -1:
        return False
    return text.find("]]]", start) == -1


def find_cut_index(text: str, max_len: int) -> int:
    cut = -1
    for ch in ("\uff0c", ",", "\uff1b", ";", " ", "\n"):
        idx = text.rfind(ch, 0, max_len)
        if idx > cut:
            cut = idx
    if cut == -1:
        return max_len
    return cut + 1


def split_tts_chunks(buffer: str) -> tuple[list[str], str]:
    if has_incomplete_button_tag(buffer):
        return [], buffer
    buffer = remove_button_tags(buffer)
    chunks = []
    start = 0
    for idx, ch in enumerate(buffer):
        if ch in STRONG_ENDINGS and idx - start + 1 >= STREAM_TTS_MIN_CHARS_STRONG:
            chunks.append(buffer[start : idx + 1])
            start = idx + 1
    remaining = buffer[start:]
    if len(remaining) >= STREAM_TTS_MAX_CHARS:
        cut = find_cut_index(remaining, STREAM_TTS_MAX_CHARS)
        chunks.append(remaining[:cut])
        remaining = remaining[cut:]
    return chunks, remaining


def preprocess_frame(indata: np.ndarray) -> np.ndarray:
    audio = indata.astype(np.int32)
    if DC_REMOVE:
        audio = audio - audio.mean(axis=0, keepdims=True)
    if NOISE_GATE > 0:
        audio = np.where(np.abs(audio) < NOISE_GATE, 0, audio)
    return np.clip(audio, -32768, 32767).astype(np.int16)


def append_log(entry: dict) -> None:
    with LOG_PATH.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(entry, ensure_ascii=False) + "\n")


def listen_once() -> str:
    done_event = threading.Event()
    audio_event_re = re.compile(
        r"\s*(\((?:[a-z][a-z\s'/-]{0,30})\)|\[(?:[a-z][a-z\s'/-]{0,30})\])\s*",
        re.IGNORECASE,
    )

    def strip_audio_events(text: str) -> str:
        if not text:
            return text
        cleaned = audio_event_re.sub(" ", text)
        cleaned = re.sub(r"\s{2,}", " ", cleaned).strip()
        return cleaned

    vad = webrtcvad.Vad(VAD_AGGRESSIVENESS)
    ring_buffer = collections.deque(maxlen=MAX_SILENCE_FRAMES)
    voiced_frames = []
    triggered = False
    client = ElevenLabs(api_key=ELEVENLABS_API_KEY)

    def audio_callback(indata, frames, time_info, status):
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
        done_event.wait(timeout=LISTEN_TIMEOUT_SEC)

    if not voiced_frames:
        return ""

    wav_io = BytesIO()
    audio = np.frombuffer(b"".join(voiced_frames), dtype=np.int16).astype(np.float32)
    audio = np.clip(audio * GAIN, -32768, 32767).astype(np.int16)
    sf.write(wav_io, audio, SAMPLE_RATE, format="WAV", subtype="PCM_16")
    wav_io.seek(0)

    stt_kwargs = {
        "file": wav_io,
        "model_id": STT_MODEL_ID,
        "tag_audio_events": False,
    }
    if STT_LANGUAGE_CODE:
        stt_kwargs["language_code"] = STT_LANGUAGE_CODE

    try:
        result = client.speech_to_text.convert(**stt_kwargs)
    except Exception as exc:
        print(f"STT failed: {exc}")
        return ""
    return strip_audio_events(result.text.strip())


def synthesize_to_mp3_bytes(client: ElevenLabs, text: str) -> bytes:
    audio_iter = client.text_to_speech.convert(
        voice_id=ELEVENLABS_VOICE_ID,
        model_id=ELEVENLABS_MODEL_ID,
        text=text,
        output_format="mp3_44100_128",
    )
    return b"".join(audio_iter)


def mp3_bytes_to_wav(mp3_bytes: bytes) -> Path:
    require_cmd("ffmpeg", "sudo apt update && sudo apt install -y ffmpeg")
    OUTPUT_DIR.mkdir(exist_ok=True)
    fd, wav_path = tempfile.mkstemp(dir=OUTPUT_DIR, suffix=".wav")
    os.close(fd)
    subprocess.run(
        [
            "ffmpeg",
            "-loglevel",
            "error",
            "-y",
            "-i",
            "pipe:0",
            "-ar",
            WAV_SAMPLE_RATE,
            "-ac",
            WAV_CHANNELS,
            "-c:a",
            "pcm_s16le",
            wav_path,
        ],
        input=mp3_bytes,
        check=True,
    )
    return Path(wav_path)


def play_wav_alsa(wav_path: Path) -> None:
    require_cmd("aplay", "sudo apt update && sudo apt install -y alsa-utils")
    subprocess.run(["aplay", "-D", ALSA_DEVICE, "-v", str(wav_path)], check=False)


def speak_text(text: str) -> None:
    text = sanitize_text(text)
    if not text:
        return
    try:
        client = ElevenLabs(api_key=ELEVENLABS_API_KEY)
        mp3_bytes = synthesize_to_mp3_bytes(client, text)
        wav_path = mp3_bytes_to_wav(mp3_bytes)
        play_wav_alsa(wav_path)
        wav_path.unlink(missing_ok=True)
    except Exception as exc:
        print(f"TTS failed: {exc}")


def tts_worker(text_queue: queue.Queue, audio_queue: queue.Queue) -> None:
    client = ElevenLabs(api_key=ELEVENLABS_API_KEY)
    while True:
        chunk = text_queue.get()
        if chunk is None:
            audio_queue.put(None)
            break
        chunk = sanitize_text(chunk)
        if not chunk:
            continue
        try:
            mp3_bytes = synthesize_to_mp3_bytes(client, chunk)
            wav_path = mp3_bytes_to_wav(mp3_bytes)
            audio_queue.put(wav_path)
        except Exception as exc:
            print(f"TTS failed: {exc}")


def audio_worker(audio_queue: queue.Queue) -> None:
    while True:
        wav_path = audio_queue.get()
        if wav_path is None:
            break
        play_wav_alsa(wav_path)
        try:
            wav_path.unlink(missing_ok=True)
        except OSError:
            pass


def request_and_speak(
    user_text: str, conversation_id: str | None
) -> tuple[str | None, str]:
    payload = {"messages": [{"role": "user", "content": "给我口语化简短回答：" + user_text}]}
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    headers = {
        "Content-Type": "application/json",
        "Authorization": AUTH_TOKEN,
        "Content-Length": str(len(body)),
    }
    url = API_NEW_URL if not conversation_id else f"{API_BASE_URL}/{conversation_id}"

    resp = requests.post(url, data=body, headers=headers, timeout=30, stream=True)
    resp.raise_for_status()
    if not resp.encoding or resp.encoding.lower() == "iso-8859-1":
        resp.encoding = "utf-8"

    content_type = resp.headers.get("Content-Type", "")
    assistant_parts = []
    updated_conversation_id = conversation_id

    if "application/json" in content_type and "text/event-stream" not in content_type:
        data = resp.json()
        print(data)
        if isinstance(data, dict):
            if data.get("conversationId") and not updated_conversation_id:
                updated_conversation_id = data["conversationId"]
            if data.get("role") == "assistant" and data.get("content"):
                assistant_parts.append(data["content"])
        elif isinstance(data, list):
            for item in data:
                if isinstance(item, dict):
                    if item.get("conversationId") and not updated_conversation_id:
                        updated_conversation_id = item["conversationId"]
                    if item.get("role") == "assistant" and item.get("content"):
                        assistant_parts.append(item["content"])
        speak_text("".join(assistant_parts))
        return updated_conversation_id, "".join(assistant_parts)

    if "text/event-stream" in content_type or "stream" in content_type:
        text_queue = queue.Queue()
        audio_queue = queue.Queue()
        tts_thread = threading.Thread(target=tts_worker, args=(text_queue, audio_queue))
        audio_thread = threading.Thread(target=audio_worker, args=(audio_queue,))
        tts_thread.start()
        audio_thread.start()

        assistant_buffer = ""
        for raw_line in resp.iter_lines(decode_unicode=False):
            if not raw_line:
                continue
            line = raw_line.decode("utf-8", errors="replace").strip()
            if line.startswith("data:"):
                line = line[5:].strip()
            if line == "[DONE]":
                break
            try:
                data = json.loads(line)
                print(data)
                if isinstance(data, dict) and data.get("conversationId"):
                    if not updated_conversation_id:
                        updated_conversation_id = data["conversationId"]
                        print(f"Conversation ID: {updated_conversation_id}")
                if (
                    isinstance(data, dict)
                    and data.get("role") == "assistant"
                    and data.get("content")
                ):
                    assistant_parts.append(data["content"])
                    assistant_buffer += data["content"]
                    chunks, assistant_buffer = split_tts_chunks(assistant_buffer)
                    for chunk in chunks:
                        text_queue.put(chunk)
            except json.JSONDecodeError:
                print(line)

        if assistant_buffer.strip():
            text_queue.put(assistant_buffer)
        text_queue.put(None)
        tts_thread.join()
        audio_thread.join()
        return updated_conversation_id, "".join(assistant_parts)

    print(resp.text)
    return updated_conversation_id, ""


def listener_loop(
    text_queue: queue.Queue,
    stop_event: threading.Event,
    playback_active: threading.Event,
) -> None:
    def on_speech_start():
        if playback_active.is_set():
            stop_event.set()

    while True:
        text = listen_once(
            on_speech_start=on_speech_start, playback_active=playback_active
        )
        if not text:
            print("No speech detected.")
            continue
        text_queue.put(text)


def main() -> None:
    conversation_id = None
    turn_index = 0
    print("Voice chat ready. Press Ctrl+C to stop.")
    playback_stop_event = threading.Event()
    playback_active_event = threading.Event()
    utterance_queue = queue.Queue()
    listener_thread = threading.Thread(
        target=listener_loop,
        args=(utterance_queue, playback_stop_event, playback_active_event),
        daemon=True,
    )
    listener_thread.start()
    while True:
        user_text = utterance_queue.get()
        playback_stop_event.clear()
        turn_index += 1
        print(f"User: {user_text}")
        updated_conversation_id, assistant_text = request_and_speak(
            user_text, conversation_id, playback_stop_event, playback_active_event
        )
        if updated_conversation_id and updated_conversation_id != conversation_id:
            conversation_id = updated_conversation_id

        append_log(
            {
                "turn": turn_index,
                "conversationId": conversation_id,
                "role": "user",
                "content": user_text,
            }
        )
        if assistant_text:
            append_log(
                {
                    "turn": turn_index,
                    "conversationId": conversation_id,
                    "role": "assistant",
                    "content": assistant_text,
                }
            )


if __name__ == "__main__":
    main()
