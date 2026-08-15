"""Generate known-BPM test WAVs for the slicer's tempo detector.

Each file is an exact whole number of bars so the bar-snap path is exercised
alongside plain autocorrelation.
"""
import math
import os
import random
import struct
import sys
import wave

SR = 44100
# Written to the current directory (or argv[1]) — never next to this script,
# so generated audio stays out of the repo.
OUT = sys.argv[1] if len(sys.argv) > 1 else os.getcwd()


def write_wav(name, samples):
    path = os.path.join(OUT, name)
    with wave.open(path, "wb") as w:
        w.setnchannels(2)
        w.setsampwidth(2)
        w.setframerate(SR)
        frames = bytearray()
        for v in samples:
            iv = max(-32768, min(32767, int(v * 30000)))
            frames += struct.pack("<hh", iv, iv)
        w.writeframes(bytes(frames))
    return path


def env(n, decay):
    return [math.exp(-i / decay) for i in range(n)]


def kick(n=4000):
    out = []
    for i in range(n):
        f = 120 * math.exp(-i / 3000.0) + 45
        out.append(math.sin(2 * math.pi * f * i / SR) * math.exp(-i / 2500.0))
    return out


def snare(n=4000):
    rnd = random.Random(7)
    return [(rnd.uniform(-1, 1) * 0.7 + math.sin(2 * math.pi * 190 * i / SR) * 0.3)
            * math.exp(-i / 1800.0) for i in range(n)]


def hat(n=1500):
    rnd = random.Random(11)
    return [rnd.uniform(-1, 1) * math.exp(-i / 400.0) * 0.5 for i in range(n)]


def mix(buf, src, at, gain=1.0):
    for i, v in enumerate(src):
        j = at + i
        if 0 <= j < len(buf):
            buf[j] += v * gain


def make_beat(bpm, bars=4, with_hats=True, swing=0.0):
    spb = SR * 60.0 / bpm          # samples per beat
    total = int(spb * 4 * bars)
    buf = [0.0] * total
    k, s, h = kick(), snare(), hat()
    for bar in range(bars):
        base = bar * spb * 4
        mix(buf, k, int(base))                       # 1
        mix(buf, s, int(base + spb))                 # 2
        mix(buf, k, int(base + spb * 2.5))           # & of 3
        mix(buf, s, int(base + spb * 3))             # 4
        if with_hats:
            for eighth in range(8):
                off = spb * eighth * 0.5
                if swing and eighth % 2 == 1:
                    off += spb * 0.5 * swing
                mix(buf, h, int(base + off), 0.6)
    return buf


def make_melodic(bpm, bars=4):
    """Sustained tones on the beat — weaker transients, harder case."""
    spb = SR * 60.0 / bpm
    total = int(spb * 4 * bars)
    buf = [0.0] * total
    notes = [220, 261.6, 329.6, 261.6]
    for beat in range(bars * 4):
        f = notes[beat % len(notes)]
        at = int(beat * spb)
        n = int(spb * 0.9)
        for i in range(n):
            j = at + i
            if j < total:
                buf[j] += math.sin(2 * math.pi * f * i / SR) * math.exp(-i / (spb * 0.35)) * 0.8
    return buf


cases = [
    ("beat_090.wav", make_beat(90), 90),
    ("beat_120.wav", make_beat(120), 120),
    ("beat_140.wav", make_beat(140), 140),
    ("beat_174.wav", make_beat(174), 174),
    ("beat_100_nohat.wav", make_beat(100, with_hats=False), 100),
    ("beat_128_swing.wav", make_beat(128, swing=0.25), 128),
    ("beat_160_2bar.wav", make_beat(160, bars=2), 160),
    ("melodic_110.wav", make_melodic(110), 110),
]

for name, buf, bpm in cases:
    peak = max(abs(v) for v in buf) or 1.0
    write_wav(name, [v / peak * 0.9 for v in buf])
    print(f"{name}\t{bpm}")
