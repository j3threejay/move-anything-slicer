# Slicer - working notes

## 2026-08-15 - v0.3.0: automatic tempo sync + random slicing

Shipped two Serato Sample style features, built and deployed to hardware.
Hardware listening test still pending.

### What landed

**Automatic tempo sync.** The sample's BPM is detected on load and Bungee
stretches it to the Move's tempo. The manual speed knob is gone; `speed` is now
read-only and reports the derived ratio. Sync is on by default.

Host tempo discovery, in priority order (shown as `src:` on the Sync page):

| Tag | Source |
|-----|--------|
| CLK | Live MIDI clock (external clock only, see below) |
| SET | `active_set.txt` -> `Sets/<uuid>/<name>/Song.abl` -> `"tempo"` |
| CFG | `move-anything/settings.txt` -> `tempo_bpm` |
| DEF | Fallback 120 |

The plugin API carries no tempo at all, hence the file reads. Cable-0 realtime
(the Move's own sequencer clock) is consumed by the shim and deliberately not
broadcast to chain slots, so MIDI clock only helps when slaved externally.
Verified on device: `active_set.txt` -> `Set 24` -> `"tempo": 128.0`. There is
no `settings.txt` on the test unit, so CFG never fires in practice.

**Random slicing.** N random start points snapped to a 16th grid at the detected
BPM, so a random pick still lands in time. Applies instantly with no scan step;
knob 6 re-rolls. Selecting it turns on playthrough, where a held pad ignores its
own slice end and runs on through the sample. Switching back to Transient turns
it off; knob 5 overrides either way.

**Third UI scope.** Jog click now cycles `[P] -> [G] -> [S]` instead of toggling
two. Sync page: K1 sync, K2 BPM ratio, K3 algo, K4 slice count, K5 playthrough,
K6 re-roll. All 16 previous knob slots were full, hence the new page.

### Decisions worth remembering

**Stretcher pool 4 -> 8.** With sync on the ratio is almost never exactly 1.0,
so nearly every voice claims a stretcher. At 4 the overflow voices silently fell
back to rate-based playback *at the wrong tempo*. Measured 2.46% of realtime for
8 sustained stretchers on an M4; the Move's A53s are far slower. This is the one
thing with no test coverage and the first suspect if audio misbehaves.

**BPM detector took four attempts.** Recorded so it is not re-litigated:

1. Onset envelope must be rise in **RMS amplitude**, not log energy. Log flux
   makes a quiet hat after silence outrank a loud kick landing on a decaying
   one, which flips offbeat/downbeat weighting and reads everything double time.
2. For material <= 30s, candidates come from **integer beat counts of the sample
   duration**. This alone eliminates the 2/3 and 3/2 errors, since a dotted
   period almost never divides into a whole beat count. Longer material falls
   back to autocorrelation peaks expanded by metrical ratios.
3. Scoring is a comb filter: `coverage^1.5 * strength * fill * prior`. Coverage
   must outweigh per-pulse strength, or a sparse half-time grid cherry-picks the
   loudest onsets via the phase search and wins.
4. An alternation test (comparing interleaved half-grids) was tried and
   **removed**. Measured ratios at true tempo (0.588-0.946) overlap those at
   true double time (0.727), so it cannot separate them. Do not re-add it
   without new evidence.

Known limit: a loop with straight 8th hats is genuinely ambiguous between N and
2N BPM. That is the one failing fixture (7/8) and what the K2 ratio knob exists
for. K2 spans /2, 2/3, x1, 3/2, x2 because a 3/2 error cannot be undone by
halving; ratios apply to the raw estimate so stepping back to x1 restores it.

### Tests

`./tests/run.sh` builds Bungee and dsp.cpp natively (no hardware), generates
known-BPM fixtures, and runs 29 behavioural checks plus the detector suite.
Expect `ALL PASS` and `7/8 exact`. Pass file paths as arguments to check real
samples. Fixtures generate into `build/tests/` which is gitignored.

Real-material spot checks: the ~4s house loops in the Demuir pack read
119.6 / 120.2 / 120.8, each landing on a whole beat count. A 403s track went
from a 3/2 error to exactly 125.00 once long-form candidates were widened.

### Deployment

v0.3.0 deployed 2026-08-15 over USB (`MOVE_HOST=172.16.254.1 ./scripts/install.sh`).
Verified on device: dsp.so, ui_chain.js and module.json checksums match the local
build, version reads 0.3.0, `move_plugin_init_v2` exported, all four required
libs present, no load errors in the log. `help.json` was copied but not
checksum-verified.

**Wi-Fi SSH does not work to this Move.** It is properly associated
(`wlan0 192.168.68.50/22`, SSID Dr.Jones, 5GHz, -54 dBm) and this Mac is on the
same subnet at .59, but ARP for .50 never resolves, so ping/SSH/HTTP all time
out. The Mac reaches the router and other devices fine. Signature of wireless
client isolation; the Move is associated to a different mesh node. USB is
currently the only path. If a USB window appears, set up a reverse SSH tunnel
out to a reachable host so remote work survives losing the cable.

### Next steps

1. **Hardware listening test** - the whole point of the deploy, not yet done.
   Load a loop, confirm the Sync page shows `src:SET` and Set 128.0, then stress
   8 simultaneous stretched voices with a tempo far from 128 and listen for
   dropouts.
2. Confirm random mode + playthrough feel right musically; the 16th-note grid
   snap is a guess that may want to be an option (8th / 16th / free).
3. If detection misreads real samples, the tuning constants are
   `BPM_COVERAGE_EXP` and `BPM_PRIOR_SIGMA` in dsp.cpp; re-run `tests/run.sh`
   against the offending file before changing anything.
4. Not yet released: no git tag, no GitHub release, `release.json` points at a
   v0.3.0 URL that does not exist yet. Do not push until hardware-verified, this
   repo serves the store installer.

### Open questions

- Does the Move ever emit clock on a cable modules can see, making CLK reachable
  without an external clock? Not investigated beyond reading the shim.
- 8-voice stretcher CPU headroom on the Move is unmeasured.
- The 220s Demuir file reads 183, which may be a 3/2 error against ~122. No
  ground truth available, and long-form is not the primary use case.
