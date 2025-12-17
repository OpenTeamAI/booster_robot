"""
最简单的 ElevenLabs 直出语音示例（Ubuntu/Jetson 友好）。
固定文本 -> ElevenLabs -> 保存 mp3 -> ffmpeg 转 WAV(48k/2ch/pcm_s16le) -> aplay 指定声卡 hw:0,0 播放

依赖安装：
  sudo apt update
  sudo apt install -y ffmpeg alsa-utils
  python3 -m pip install -U elevenlabs

强烈建议：不要把 API Key 写死在代码里！
运行前：
  export ELEVENLABS_API_KEY="你的key"
  source .venv/bin/activate && python elevenlabs_demo.py
"""

import os
import shutil
import subprocess
from pathlib import Path

from elevenlabs import ElevenLabs


# ===== 配置区 =====
ELEVENLABS_API_KEY = "sk_4dc221c541f0f3f4ea1677ecb67ec807f24fe172bc7580f2"
ELEVENLABS_VOICE_ID = "9lHjugDhwqoxA5MhX0az"
MODEL_ID = "eleven_multilingual_v2"

TEXT = """你好呀，欢迎来到 Vesta technology！有什么可以帮到你的吗？我很乐意帮您挑选最适合您厨房的抽油烟机！为了给您提供最合适的建议，我需要了解您的具体需求和厨房布局。
让我问你几个问题：
您的烹饪空间属于哪种类型？是在公寓（可能存在通风限制）、独立式住宅还是开放式厨房？"""

OUTPUT_DIR = Path("./voices")
OUTPUT_DIR.mkdir(exist_ok=True)
MP3_FILE = OUTPUT_DIR / "eleven_demo.mp3"
WAV_FILE = OUTPUT_DIR / "eleven_demo.wav"

# 你的机器上：card 0 device 0（USB Audio Device / C-Media），且 hw:0,0 不支持 mono，所以必须 2ch
ALSA_DEVICE = "hw:0,0"

# 输出为设备更兼容的 wav 规格
WAV_SAMPLE_RATE = "48000"
WAV_CHANNELS = "2"
# =================


def require_cmd(cmd: str, install_hint: str) -> None:
    if not shutil.which(cmd):
        raise RuntimeError(f"缺少命令：{cmd}\n请安装：{install_hint}")


def synthesize_to_mp3() -> Path:
    if not ELEVENLABS_API_KEY:
        raise RuntimeError(
            "缺少 ELEVENLABS_API_KEY 环境变量。\n"
            '请先：export ELEVENLABS_API_KEY="你的key"'
        )

    client = ElevenLabs(api_key=ELEVENLABS_API_KEY)

    audio_iter = client.text_to_speech.convert(
        voice_id=ELEVENLABS_VOICE_ID,
        model_id=MODEL_ID,
        text=TEXT,
        output_format="mp3_44100_128",
    )
    audio_bytes = b"".join(audio_iter)

    MP3_FILE.write_bytes(audio_bytes)
    print(f"已生成 MP3：{MP3_FILE}")
    return MP3_FILE


def mp3_to_wav(mp3_path: Path) -> Path:
    require_cmd("ffmpeg", "sudo apt update && sudo apt install -y ffmpeg")

    # 关键：强制输出为 pcm_s16le + 48k + 2ch（你的 hw:0,0 不支持 mono）
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
    print(f"已转换 WAV（pcm_s16le/{WAV_SAMPLE_RATE}Hz/{WAV_CHANNELS}ch）：{WAV_FILE}")
    return WAV_FILE


def play_wav_alsa(wav_path: Path) -> None:
    require_cmd("aplay", "sudo apt update && sudo apt install -y alsa-utils")

    # 不要吞掉输出，方便你看到 ALSA 实际在用什么参数/报什么错
    print(f"开始播放：{wav_path}  -> ALSA {ALSA_DEVICE}")
    subprocess.run(["aplay", "-D", ALSA_DEVICE, "-v", str(wav_path)], check=False)


def main() -> None:
    mp3 = synthesize_to_mp3()
    wav = mp3_to_wav(mp3)
    play_wav_alsa(wav)


if __name__ == "__main__":
    main()
