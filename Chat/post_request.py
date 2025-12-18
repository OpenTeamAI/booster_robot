import json
import os
import queue
import re
import shutil
import subprocess
import tempfile
import threading
from pathlib import Path

import requests


ELEVENLABS_API_KEY = "sk_4dc221c541f0f3f4ea1677ecb67ec807f24fe172bc7580f2"
ELEVENLABS_VOICE_ID = "9lHjugDhwqoxA5MhX0az"
ELEVENLABS_MODEL_ID = "eleven_multilingual_v2"
OUTPUT_DIR = Path("./voices")
ALSA_DEVICE = os.getenv("ALSA_DEVICE", "hw:0,0")
WAV_SAMPLE_RATE = "48000"
WAV_CHANNELS = "2"
STREAM_TTS_MIN_CHARS_STRONG = 12
STREAM_TTS_MAX_CHARS = 160
STRONG_ENDINGS = set("。！？!?")


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
    for ch in ("，", ",", "；", ";", " ", "\n"):
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


def synthesize_to_mp3_bytes(text: str, api_key: str) -> bytes:
    from elevenlabs import ElevenLabs

    OUTPUT_DIR.mkdir(exist_ok=True)
    client = ElevenLabs(api_key=api_key)
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
        mp3_bytes = synthesize_to_mp3_bytes(text, ELEVENLABS_API_KEY)
        wav_path = mp3_bytes_to_wav(mp3_bytes)
        play_wav_alsa(wav_path)
        wav_path.unlink(missing_ok=True)
    except Exception as exc:
        print(f"TTS failed: {exc}")


def tts_worker(text_queue: queue.Queue, audio_queue: queue.Queue) -> None:
    while True:
        chunk = text_queue.get()
        if chunk is None:
            audio_queue.put(None)
            break
        chunk = sanitize_text(chunk)
        if not chunk:
            continue
        try:
            mp3_bytes = synthesize_to_mp3_bytes(chunk, ELEVENLABS_API_KEY)
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


def main() -> None:
    url = "https://www.openteam.ai/api/agent/v1/vesta_agent/chat/new"
    payload = {
        "messages": [
            {"role": "user", "content": "你好"}
        ]
    }
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    headers = {
        "Content-Type": "application/json",
        "Authorization": (
            "Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9."
            "eyJ1c2VyX2lkIjoidXNlci04ZThhYzk2OC02YjhkLTQxNGEt"
            "YTI2YS1kNjE2NmM4ZGRhNmMiLCJpYXQiOjE3NTkxNjgyNjAs"
            "InNjb3BlIjoiYWRtaW4iLCJhZ2VudCI6InZlc3RhX2FnZW50"
            "IiwiZXhwIjoxODgzNTg0MjYwfQ.5KpLao9lVwHWBXGUfU_"
            "ptNMzwHi8MWsoPWMIuL0aqrU"
        ),
        "Content-Length": str(len(body)),
    }

    resp = requests.post(url, data=body, headers=headers, timeout=30, stream=True)
    resp.raise_for_status()
    if not resp.encoding or resp.encoding.lower() == "iso-8859-1":
        resp.encoding = "utf-8"
    content_type = resp.headers.get("Content-Type", "")
    assistant_parts = []
    if "application/json" in content_type and "text/event-stream" not in content_type:
        data = resp.json()
        print(data)
        if isinstance(data, dict):
            if data.get("role") == "assistant" and data.get("content"):
                assistant_parts.append(data["content"])
        elif isinstance(data, list):
            for item in data:
                if (
                    isinstance(item, dict)
                    and item.get("role") == "assistant"
                    and item.get("content")
                ):
                    assistant_parts.append(item["content"])
        speak_text("".join(assistant_parts))
        return

    if "text/event-stream" in content_type or "stream" in content_type:
        text_queue = queue.Queue()
        audio_queue = queue.Queue()
        tts_thread = threading.Thread(
            target=tts_worker, args=(text_queue, audio_queue)
        )
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
        return

    print(resp.text)


if __name__ == "__main__":
    main()
