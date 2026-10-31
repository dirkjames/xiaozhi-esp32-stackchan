#!/usr/bin/env python3
"""Generate the built-in UI sound effects (16 kHz mono WAV -> Opus OGG).

The generated OGG files are embedded into the firmware by
`firmware/main/CMakeLists.txt` (`file(GLOB COMMON_SOUNDS ... assets/sfx/*.ogg)`),
so after adding/removing a sound you must run `idf.py reconfigure` (or a clean
build) before the symbols exist.

Usage:
    python gen_sfx.py [--ffmpeg PATH]

Only the Python standard library is required, plus ffmpeg with libopus.
"""
from __future__ import annotations

import argparse
import math
import os
import shutil
import struct
import subprocess
import tempfile
import wave

SR = 16000
PEAK = 0.35

DEFAULT_FFMPEG_CANDIDATES = [
    r"D:\software\ffmpeg-6.1.1-essentials_build\ffmpeg-6.1.1-essentials_build\bin\ffmpeg.exe",
    "ffmpeg",
]


def _envelope(n: int, attack_ms: float = 5.0, release_ms: float = 25.0) -> list[float]:
    a = max(1, int(SR * attack_ms / 1000.0))
    r = max(1, int(SR * release_ms / 1000.0))
    env = [1.0] * n
    for i in range(min(a, n)):
        env[i] = i / a
    for i in range(min(r, n)):
        env[n - 1 - i] = min(env[n - 1 - i], i / r)
    return env


def _tone(freq: float, ms: float, amp: float = 1.0, sweep_to: float | None = None,
          release_ms: float = 25.0, wave_fn=math.sin) -> list[float]:
    n = int(SR * ms / 1000.0)
    env = _envelope(n, release_ms=release_ms)
    out = []
    phase = 0.0
    for i in range(n):
        f = freq if sweep_to is None else freq + (sweep_to - freq) * (i / max(1, n - 1))
        phase += 2.0 * math.pi * f / SR
        out.append(wave_fn(phase) * env[i] * amp)
    return out


def _silence(ms: float) -> list[float]:
    return [0.0] * int(SR * ms / 1000.0)


def _mix(a: list[float], b: list[float]) -> list[float]:
    n = max(len(a), len(b))
    return [(a[i] if i < len(a) else 0.0) + (b[i] if i < len(b) else 0.0) for i in range(n)]


def _square(phase: float) -> float:
    return 1.0 if math.sin(phase) >= 0.0 else -1.0


def _seq(parts: list[list[float]]) -> list[float]:
    out: list[float] = []
    for p in parts:
        out.extend(p)
    return out


def build_sounds() -> dict[str, list[float]]:
    sounds: dict[str, list[float]] = {}

    # wake: two rising notes (attention chime)
    sounds["wake"] = _seq([_tone(880, 90), _tone(1320, 130, release_ms=40)])

    # listen_start / listen_end: short up/down sweeps
    sounds["listen_start"] = _tone(660, 150, sweep_to=990, release_ms=30)
    sounds["listen_end"] = _tone(990, 150, sweep_to=660, release_ms=30)

    # thinking: three soft pulses
    sounds["thinking"] = _seq([_tone(440, 60), _silence(60)] * 3)

    # happy: major arpeggio
    sounds["happy"] = _seq([_tone(f, 90) for f in (523.25, 659.25, 783.99, 1046.5)])

    # sad: descending, slow release
    sounds["sad"] = _seq([_tone(f, 170, release_ms=60) for f in (440.0, 392.0, 349.23, 293.66)])

    # error: low square buzz, two pulses
    sounds["error"] = _seq([_tone(196, 120, amp=0.7, wave_fn=_square, release_ms=20), _silence(40),
                            _tone(165, 150, amp=0.7, wave_fn=_square, release_ms=30)])

    # dance_start: rhythmic hook
    sounds["dance_start"] = _seq([
        _tone(523.25, 125), _tone(659.25, 125), _tone(523.25, 125), _tone(783.99, 250, release_ms=50),
    ])

    # beat_tick: very short click for beat sync
    sounds["beat_tick"] = _tone(1200, 25, amp=0.8, release_ms=10)

    # horn: two-tone honk
    sounds["horn"] = _mix(_tone(392, 260, amp=0.6, release_ms=60), _tone(494, 260, amp=0.5, release_ms=60))

    return sounds


def write_wav(path: str, samples: list[float]) -> None:
    peak = max((abs(s) for s in samples), default=1.0)
    scale = PEAK / peak if peak > 0 else 0.0
    with wave.open(path, "wb") as f:
        f.setnchannels(1)
        f.setsampwidth(2)
        f.setframerate(SR)
        frames = bytearray()
        for s in samples:
            v = int(max(-1.0, min(1.0, s * scale)) * 32767)
            frames += struct.pack("<h", v)
        f.writeframes(bytes(frames))


def find_ffmpeg(explicit: str | None) -> str:
    candidates = [explicit] if explicit else DEFAULT_FFMPEG_CANDIDATES
    for c in candidates:
        if not c:
            continue
        found = shutil.which(c) if os.path.basename(c) == c else (c if os.path.isfile(c) else None)
        if found:
            return found
    raise SystemExit("ffmpeg not found, pass --ffmpeg PATH")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ffmpeg", default=None)
    args = parser.parse_args()

    ffmpeg = find_ffmpeg(args.ffmpeg)
    out_dir = os.path.dirname(os.path.abspath(__file__))
    sounds = build_sounds()

    with tempfile.TemporaryDirectory() as tmp:
        for name, samples in sounds.items():
            wav_path = os.path.join(tmp, name + ".wav")
            ogg_path = os.path.join(out_dir, name + ".ogg")
            write_wav(wav_path, samples)
            subprocess.run(
                [ffmpeg, "-y", "-loglevel", "error", "-i", wav_path,
                 "-c:a", "libopus", "-b:a", "24k", "-ar", "16000", ogg_path],
                check=True,
            )
            print(f"{name}.ogg  {os.path.getsize(ogg_path)} bytes  {len(samples) / SR * 1000:.0f} ms")


if __name__ == "__main__":
    main()
