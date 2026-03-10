# Sample Slicer - User Manual

A transient-detection sample slicer for **Ableton Move** via the
[move-anything](https://github.com/charlesvestal/move-anything) module system.

Load a WAV file, auto-detect transients, and play slices from the pads or
keyboard with per-pad envelopes, pitch, gain, and loop modes.

---

## Quick Start

1. **Browse** - jog wheel scrolls the sample browser. Samples preview on hover.
2. **Select** - click jog to load a WAV file.
3. **Adjust threshold** - turn jog wheel to set detection sensitivity.
4. **Scan** - click jog to detect transients.
5. **Play** - hit pads (1-32) to trigger slices.

---

## Controls

### Bank A - Knobs 1-4 (Per-Pad Envelope)

| Knob | Parameter | Range |
|------|-----------|-------|
| 1 | Start trim | offset from slice start (ms) |
| 2 | End trim | offset from slice end (ms) |
| 3 | Attack | 5 - 500 ms |
| 4 | Decay | 0 - 5000 ms |

These are always per-pad - each pad stores its own envelope settings.

### Bank B - Knobs 5-8 (Global or Per-Pad)

Bank B has two edit scopes, toggled by **clicking jog** while in Bank B:

**Global [G]** (default):

| Knob | Parameter | Range |
|------|-----------|-------|
| 5 | Mode | gate / trigger |
| 6 | Pitch | -24 to +24 semitones |
| 7 | Gain | 0 - 100% (master volume) |
| 8 | Loop mode | off / loop / ping-pong |

**Per-Pad [P]** (click jog in Bank B to switch):

| Knob | Parameter | Range |
|------|-----------|-------|
| 5 | Mode override | --- (follow global) / TRIG / GATE |
| 6 | Pitch offset | -24 to +24 semitones (added to global) |
| 7 | Gain multiplier | 0 - 200% (multiplied by global gain) |
| 8 | Loop mode | off / loop / ping-pong |

The display shows `[G]` or `[P]` to indicate the current edit scope.

### Jog Wheel

- **Turn**: scroll browser / adjust threshold
- **Click (Bank A)**: open sensitivity screen / trigger scan
- **Click (Bank B)**: toggle Global / Per-Pad edit scope

### Knob Touch

Touching knobs 1-4 switches the display to Bank A.
Touching knobs 5-8 switches the display to Bank B.
The display persists across pad presses.

---

## Play Modes

| Mode | Behavior |
|------|----------|
| **Gate** | Plays while pad is held. Decay envelope on release. |
| **Trigger** | Plays full slice regardless of how long pad is held. |

Per-pad mode override lets individual pads behave differently from the global
setting. Set to `---` to follow the global mode.

## Loop Modes

| Mode | Behavior |
|------|----------|
| **Off** | Plays once, stops at slice end. |
| **Loop** | Loops forward within slice bounds until released. |
| **Ping-Pong** | Bounces back and forth within slice bounds until released. |

---

## MIDI Mapping

| Input | Notes | Mapping |
|-------|-------|---------|
| Pads | 68 - 99 | Slices 0 - 31 (direct) |
| Keyboard | 36+ (C2 root) | Chromatic, up to 91 slices |

Velocity sensitivity can be toggled on/off.

---

## Global vs Per-Pad Parameters

The slicer combines global and per-pad values at voice start:

- **Pitch**: `global_pitch + pad_pitch_offset` (additive)
- **Gain**: `global_gain * pad_gain` (multiplicative)
- **Mode**: pad override wins if set, otherwise global mode applies

This means you can set a global pitch/gain/mode and then fine-tune individual
pads as needed.

---

## Building

```bash
# Cross-compile for Move (ARM64) via Docker
./scripts/build.sh

# Deploy to Move over SSH
MOVE_HOST=move.local ./scripts/install.sh
```

Requires Docker for cross-compilation. The build produces `dist/slicer/`
containing `dsp.so`, `module.json`, `ui_chain.js`, and `help.json`.
