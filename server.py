# server.py  (merged with transcription functionality)
import os
import datetime
import sys
import wave
import argparse
import socket
import struct
import shutil
import json
from pathlib import Path
import threading
import numpy as np

# Whisper imports (from server_v1)
import whisper
from datetime import datetime as dt

if sys.version_info.major == 3:
    from urllib import parse
    from http.server import HTTPServer, BaseHTTPRequestHandler
else:
    import urlparse
    from BaseHTTPServer import HTTPServer, BaseHTTPRequestHandler

# -----------------------
# Config / storage layout (same as server_v1.py)
# -----------------------
BASE = Path(__file__).resolve().parent
ESPDOWN = BASE / "espdownloads"
TRANS_DIR = ESPDOWN / "transcripts"
USED_DIR = ESPDOWN / "usedFiles"
MASTER_LOG = TRANS_DIR / "all_transcripts.txt"


# Whisper model config (same behavior as server_v1)
MODEL_NAME = "small"
MODELS_DIR = Path(__file__).resolve().parent / "models"

_lock = threading.Lock()
_model = None

def get_model():
    print(f"Loading Whisper model '{MODEL_NAME}' ...")
    
    global _model
    with _lock:
        if _model is None:
            print(f"Loading Whisper model '{MODEL_NAME}' ...")
            _model = whisper.load_model(
                MODEL_NAME,
                download_root=str(MODELS_DIR)
            )
            print("Model loaded.")
    return _model


model = get_model()

# Ensure required folders exist
for p in (ESPDOWN, TRANS_DIR, USED_DIR):
    p.mkdir(parents=True, exist_ok=True)

# ================= PASTE START =================

def normalize_audio(wav_path):
    """
    Reads the WAV file, boosts the volume to a standard level (-3dB),
    and overwrites the file. This helps Whisper hear quiet commands from the INMP441.
    """
    try:
        # 1. Read the audio file
        with wave.open(str(wav_path), 'rb') as f:
            params = f.getparams()
            n_channels, sampwidth, framerate, n_frames, comptype, compname = params
            raw_data = f.readframes(n_frames)
        
        # 2. Convert to numpy array
        audio_data = np.frombuffer(raw_data, dtype=np.int16)
        
        # 3. Calculate peak amplitude
        max_val = np.max(np.abs(audio_data))
        
        if max_val == 0:
            return # Silence, nothing to do
            
        # 4. Normalize to target peak (approx 90% of max volume)
        target_peak = 30000 
        scale_factor = target_peak / max_val
        
        # Only amplify if it's too quiet (scale > 1) but not empty noise (scale < 50)
        if scale_factor > 1.0 and scale_factor < 50.0: 
            normalized_data = (audio_data * scale_factor).astype(np.int16)
            
            # 5. Write back to file
            with wave.open(str(wav_path), 'wb') as f:
                f.setparams(params)
                f.writeframes(normalized_data.tobytes())
            print(f"Audio normalized (Scale factor: {scale_factor:.2f}x)")
            
    except Exception as e:
        print(f"Warning: Audio normalization failed: {e}")

def transcribe_file(wav_path: str, print_progress: bool = False):
    """
    Transcribe file using Whisper with normalization and context prompting.
    """
    start_now = dt.now().strftime("%H_%M_%S")
    print(f"start Time:   {start_now}")

    # 1. Normalize Audio first
    normalize_audio(wav_path)

    wav_path = str(wav_path)

    # 2. Transcribe with Context Prompt
    # This helps Whisper distinguish "light 1" from "light on"
    initial_prompt = "James, the name of ai assistant listenting the commads. Voice commands for home automation like, Turn on light 1, Turn off light 2, Turn on light 3 or Turn off light 4."
    
    # We force English ('en') to prevent it from guessing random languages on noise
    result = model.transcribe(
        wav_path, 
        language="en", 
        fp16=False, 
        initial_prompt=initial_prompt
    )

    detected_text = result["text"].strip()
    
    if print_progress:
        print(f"Transcript by server: {detected_text}")

    # 3. Robust Command Parsing
    a = detected_text.lower()
    # Remove punctuation
    for char in [",", ".", "!"]:
        a = a.replace(char, "")
    
    sendText = ""
    uniqueWords = set(a.split())

    # Check for keywords (removed strict "james" requirement for reliability)
    if ("turn" in a) and (("on" in a) or ("off" in a)):
        
        # Determine Action
        action = "on" if "on" in uniqueWords else "off"
        
        # Determine Target (Light 1, 2, 3, 4)
        target = None
        if "one" in uniqueWords or "1" in uniqueWords: target = "1"
        elif "two" in uniqueWords or "2" in uniqueWords: target = "2"
        elif "three" in uniqueWords or "3" in uniqueWords: target = "3"
        elif "four" in uniqueWords or "4" in uniqueWords: target = "4"
        
        if target:
            sendText = f"light {target} {action}"
    
    stop_now = dt.now().strftime("%H:%M:%S")
    print(f"End Time:   {stop_now}")

    return {
        "success": True,
        "message": "transcribed",
        "language": "en",
        "lang_confidence": 1.0,
        "text": sendText
    }

# ================= PASTE END =================


    # return {
    #         "success": True,
    #         "message": "transcribed",
    #         "language": detected_lang,
    #         "lang_confidence": lang_conf,
    #         "text": "no command",
    #     }

# Helper: sanitize filename (copied pattern)
def safe_filename(name: str) -> str:
    return "".join(c for c in name if c.isalnum() or c in (" ", ".", "_", "-")).rstrip()

# -----------------------
# Original server.py constants
# -----------------------
ipADD = "10.253.94.16"
portADD = 8000
PORT = 8000

def get_timestamp():
    """Returns the current time in HH:MM:SS:millis format."""
    return datetime.datetime.now().strftime("%H_%M_%S_%f")[:-3]

class Handler(BaseHTTPRequestHandler):
    def _set_headers(self, length):
        self.send_response(200)
        if length > 0:
            self.send_header('Content-length', str(length))
        self.end_headers()

    def _get_chunk_size(self):

        data = self.rfile.read(2)
        while data[-2:] != b"\r\n":
            data += self.rfile.read(1)
        return int(data[:-2], 16)

    def _get_chunk_data(self, chunk_size):

        data = self.rfile.read(chunk_size)
        self.rfile.read(2)  # consume trailing \r\n
        return data

    def _write_wav(self, data, rates, bits, ch):
    
        """
        Write WAV into the espdownloads folder, matching server_v1 storage.
        Returns the full Path of the saved wav (Path object).
        """
        t = datetime.datetime.utcnow()
        time = t.strftime('%Y%m%dT%H%M%SZ')
        filename = f'{time}_{rates}_{bits}_{ch}.wav'
        save_path = ESPDOWN / filename

        # 'data' is a list of ints (0-255) because we did data += bytes
        raw_bytes = bytearray(data)

        if bits == 32:
            # Interpret as signed 32-bit little-endian samples
            num_samples = len(raw_bytes) // 4
            if num_samples == 0:
                print("No samples, nothing to write")
                return save_path

            ints32 = struct.unpack('<' + 'i' * num_samples, raw_bytes)

            # Downconvert 32 -> 16 bit by shifting right.
            # INMP441 effectively gives 24-bit into 32-bit slots;
            # shifting by 16 keeps the most significant 16 bits.
            ints16 = [x >> 16 for x in ints32]

            raw_bytes = struct.pack('<' + 'h' * num_samples, *ints16)
            bits = 16  # final WAV is 16-bit

        # Now write standard PCM WAV to the espdownloads folder
        with open(save_path, 'wb') as wavfile:
            # use wave module to set params
            wf = wave.open(wavfile, 'wb')
            wf.setnchannels(ch)
            wf.setsampwidth(bits // 8)
            wf.setframerate(rates)
            wf.writeframes(raw_bytes)
            wf.close()

        print(f"Wrote WAV: {save_path} (rate={rates}, bits={bits}, ch={ch})")
        return save_path

    def do_POST(self):

        if sys.version_info.major == 3:
            urlparts = parse.urlparse(self.path)
        else:
            urlparts = urlparse.urlparse(self.path)
        request_file_path = urlparts.path.strip('/')
        total_bytes = 0
        sample_rates = 0
        bits = 0
        channel = 0

        print("Do POST......")
        if (request_file_path == 'upload'
            and self.headers.get('Transfer-Encoding', '').lower() == 'chunked'):

            data = []
            sample_rates = self.headers.get('x-audio-sample-rates', '').lower()
            bits        = self.headers.get('x-audio-bits', '').lower()
            channel     = self.headers.get('x-audio-channel', '').lower()

            sample_rates = int(sample_rates) if sample_rates else 16000
            bits        = int(bits) if bits else 32
            channel     = int(channel) if channel else 1

            print("Audio information, sample rates: {}, bits: {}, channel(s): {}".format(
                sample_rates, bits, channel))

            while True:
                chunk_size = self._get_chunk_size()
                if chunk_size == 0:
                    break
                chunk_data = self._get_chunk_data(chunk_size)
                total_bytes += chunk_size
                # extend 'data' list with bytes
                data += chunk_data
                sys.stdout.write("\rTotal bytes received: {}".format(total_bytes))
                sys.stdout.flush()
            print("\nFinished receiving audio")

            # Save WAV into espdownloads (same layout as server_v1)
            try:
                save_path = self._write_wav(data, sample_rates, bits, channel)
            except Exception as e:
                print(f"Error writing wav: {e}")
                self.send_response(500)
                self.send_header("Content-type", "application/json;charset=utf-8")
                body = json.dumps({"status":"error","message":f"write_error: {e}"})
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body.encode('utf-8'))
                return

            # Now call transcription (Whisper)
            try:
                res = transcribe_file(str(save_path), print_progress=True)
            except Exception as e:
                print(f"Transcription error: {e}")
                res = {
                    "success": False,
                    "message": f"transcription failed: {e}",
                    "language": None,
                    "lang_confidence": 0.0,
                    "text": ""
                }

            # Extract transcript safely (mimic server_v1 behavior)
            transcript_text = (res.get("text") if res.get("success") else res.get("message"))
            safe_text = transcript_text


            # Save transcript file
            ts2 = datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
            transcript_file = TRANS_DIR / f"{save_path.stem}_{ts2}.txt"

            try:
                with open(transcript_file, "w", encoding="utf-8") as f:
                    f.write(f"Filename: {save_path.name}\n")
                    f.write(f"Processed: {ts2} UTC\n")
                    f.write(f"Detected language: {res.get('language')}\n")
                    f.write(f"Lang confidence: {res.get('lang_confidence')}\n\n")
                    f.write(transcript_text)
            except Exception as e:
                print(f"Failed to write transcript file: {e}")

            # Append to master log
            try:
                with open(MASTER_LOG, "a", encoding="utf-8") as f:
                    f.write(f"---\n{ts2} | {save_path.name} | {res.get('language')} | {res.get('lang_confidence')}\n")
                    f.write(transcript_text + "\n\n")
            except Exception as e:
                print(f"Failed to append to master log: {e}")

            # Move WAV to usedFiles
            try:
                final_dst = USED_DIR / save_path.name
                shutil.move(save_path, final_dst)
                print(f"[SERVER] Moved WAV to usedFiles: {final_dst}")
            except Exception as e:
                print(f"Failed to move wav to usedFiles: {e}")
                final_dst = save_path  # fallback to original path

            # Final JSON sent to ESP32 (same fields as server_v1)
            response = {
                "status": "ok" if res.get("success") else "error",
                "filename": save_path.name,
                "language": res.get("language"),
                "lang_confidence": res.get("lang_confidence"),
                "transcript": transcript_text 
            }

            body = json.dumps(response)
            body_bytes = body.encode('utf-8')

            self.send_response(200)
            self.send_header("Content-type", "application/json;charset=utf-8")
            self.send_header("Content-Length", str(len(body_bytes)))
            self.end_headers()
            self.wfile.write(body_bytes)

        else:
            # Not matched endpoint / not chunked
            print("POST to unknown path or not chunked")
            self.send_response(400)
            self.send_header('Content-type', "application/json;charset=utf-8")
            self.end_headers()
            self.wfile.write(json.dumps({"status":"error","message":"bad request"}).encode('utf-8'))


    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', "text/html;charset=utf-8")
        self.end_headers()
        self.wfile.write(b"OK")


def get_host_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    finally:
        s.close()
    return ip

parser = argparse.ArgumentParser(description='HTTP Server to save ESP32 raw I2S audio as WAV and transcribe')
parser.add_argument('--ip', default=0,type=str)
parser.add_argument('--port', default=0, type=int)
args = parser.parse_args()
if not args.ip:
    args.ip = get_host_ip()
if not args.port:
    args.port = PORT

httpd = HTTPServer((args.ip, args.port), Handler)

print("Serving HTTP on {} port {}".format(args.ip, args.port))
httpd.serve_forever()