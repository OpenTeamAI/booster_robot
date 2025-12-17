import sounddevice as sd
import soundfile as sf
from faster_whisper import WhisperModel

DEVICE = 38
DURATION = 5
SAMPLERATE = 16000

model = WhisperModel("base")  # or "base"

def record_audio():
    print("Recording 5 seconds...")
    audio = sd.rec(int(DURATION * SAMPLERATE),
                   samplerate=SAMPLERATE,
                   channels=1,
                   dtype='float32',
                   device=DEVICE)
    sd.wait()
    sf.write("recorded.wav", audio, SAMPLERATE)
    print("Saved recorded.wav")

def play_audio():
    print("Playing recorded.wav...")
    data, sr = sf.read("recorded.wav")
    sd.play(data, sr)
    sd.wait()

def transcribe_audio():
    print("Transcribing recorded.wav with Whisper...")
    segments, info = model.transcribe("recorded.wav")

    # Combine all segment texts
    text = "".join(seg.text for seg in segments).strip()

    print("Transcription:", text)
    return text

if __name__ == "__main__":
    record_audio()
    play_audio()
    result = transcribe_audio()
    print("Final Output:", result)
