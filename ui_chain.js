/*
 * Slicer — ui_chain.js v3
 *
 * Confirmed hardware MIDI mappings:
 *   Jog click  = CC3  (MoveMainButton)
 *   Jog rotate = CC14 (MoveMainKnob)
 *   Jog touch  = CC9  (capacitive, NOT the click)
 *   Knobs 1-8  = CC71-78
 *   Pads       = Notes 68-99
 *   Knob touch = Notes 0-7 — reach chain UI (confirmed), used for bank switching
 *
 * Note → slice mapping (mirrors DSP):
 *   Move pads (notes 68-99): slice_idx = note - 68  (0-31, direct pad mapping)
 *   All other notes:         slice_idx = note - 36  (C2 root, chromatic)
 *   Notes outside [0, slice_count_actual) are silently ignored.
 *
 * Per-pad params stored in JS padState[] — NOT round-tripped from DSP after
 * each pad hit. Knob edits write to DSP and update local state directly.
 *
 * READY view (bank A) shows chromatic range: C2–<endNote> (N slices)
 *   endNote = note name of (36 + slice_count_actual - 1)
 *   "*" appended if slice_count_actual > 91 (some slices unreachable via MIDI)
 */

import * as os from 'os';
import {
    MoveMainKnob, MoveMainButton,
    MoveKnob1, MoveKnob2, MoveKnob3, MoveKnob4,
    MoveKnob5, MoveKnob6, MoveKnob7, MoveKnob8
} from '/data/UserData/schwung/shared/constants.mjs';
import { decodeDelta } from '/data/UserData/schwung/shared/input_filter.mjs';

const SAMPLES_DIR      = '/data/UserData/UserLibrary/Samples';
const SCREEN_W         = 128;
const SCAN_FLASH_TICKS = 120;
const DBLCLICK_TICKS   = 10;   /* double-click window (tune to taste) */
const LOOP_LABELS      = ['Off', 'Loop', 'Ping'];
const MAX_SLICES       = 128;
const ROOT_NOTE        = 36;   /* C2 — chromatic mapping root, matches DSP */
const NOTE_NAMES       = ['C','C#','D','D#','E','F','F#','G','G#','A','A#','B'];

function noteName(n) {
    return NOTE_NAMES[n % 12] + (Math.floor(n / 12) - 1);
}

/* per-pad param cache — mirrors DSP pads[] array */
const padState = [];
for (let i = 0; i < MAX_SLICES; i++) {
    padState.push({ startTrim: 0.0, endTrim: 0.0, attack: 5.0, decay: 500.0, gain: 1.0, pitchOffset: 0.0, modeOverride: 1, loop: 0 });
}

const s = {
    view:             'browser',
    knobBank:         'A',
    dirty:            true,
    sampleName:       '',
    samplePath:       '',
    threshold:        0.5,
    pitch:            0.0,
    globalGain:       0.8,
    globalAttack:     5.0,
    globalDecay:      500.0,
    velSens:          1,
    mode:             'gate',
    editScope:        'P',       /* 'G'=global, 'P'=per-pad (jog click toggle) */
    sliceCountActual: 0,
    slicerState:      0,
    selectedSlice:    0,
    scanFlashTicks:   0,
    previewSlices:    0,
    browserPath:      SAMPLES_DIR,
    browserEntries:   [],
    browserCursor:    0,
    browserScroll:    0,
    jogClickTicks:    0,
    jogClickAction:   null,
};

function gp(key, fallback) {
    try { const v = host_module_get_param(key); return v != null ? v : fallback; }
    catch(e) { return fallback; }
}
function sp(key, val) { try { host_module_set_param(key, String(val)); } catch(e) {} }

function pad() { return padState[s.selectedSlice]; }

function syncGlobal() {
    s.samplePath       = gp('sample_path', '');
    s.threshold        = parseFloat(gp('threshold', 0.5));
    s.pitch            = parseFloat(gp('pitch', 0.0));
    s.globalGain       = parseFloat(gp('global_gain', 0.8));
    s.mode             = gp('mode', 'gate');
    s.velSens          = parseInt(gp('velocity_sens', 1));
    s.sliceCountActual = parseInt(gp('slice_count_actual', 0));
    s.slicerState      = parseInt(gp('slicer_state', 0));
    s.sampleName       = s.samplePath ? s.samplePath.split('/').pop().replace(/\.wav$/i, '') : '';
}

function resetPadState() {
    for (let i = 0; i < MAX_SLICES; i++) {
        padState[i] = { startTrim: 0.0, endTrim: 0.0, attack: 5.0, decay: 500.0, gain: 1.0, pitchOffset: 0.0, modeOverride: 1, loop: 0 };
    }
}

function selectSlice(idx) {
    s.selectedSlice = idx;
    sp('selected_slice', idx);
    s.dirty = true;
}

function isDir(path) {
    try { const [st, err] = os.stat(path); return !err && (st.mode & 0o170000) === 0o040000; }
    catch(e) { return false; }
}

function browserOpen(path) {
    s.browserPath = path; s.browserCursor = 0; s.browserScroll = 0; s.browserEntries = [];
    try {
        const [names, err] = os.readdir(path);
        if (err || !names) return;
        const entries = [];
        if (path !== SAMPLES_DIR) {
            const parts = path.split('/'); parts.pop();
            entries.push({ name: '..', path: parts.join('/') || '/', dir: true });
        }
        const dirs = [], files = [];
        for (const n of names) {
            if (n === '.' || n === '..') continue;
            const full = path + '/' + n;
            if (isDir(full)) dirs.push({ name: n, path: full, dir: true });
            else if (/\.wav$/i.test(n)) files.push({ name: n, path: full, dir: false });
        }
        dirs.sort((a,b) => a.name.localeCompare(b.name));
        files.sort((a,b) => a.name.localeCompare(b.name));
        s.browserEntries = entries.concat(dirs, files);
    } catch(e) {}
    /* trigger preview for the entry now under the cursor */
    const e0 = s.browserEntries[0];
    if (e0 && !e0.dir) sp('preview_path', e0.path);
    else               sp('preview_stop', '1');
    s.dirty = true;
}

function browserScrollBy(delta) {
    s.browserCursor = Math.max(0, Math.min(s.browserEntries.length - 1, s.browserCursor + delta));
    if (s.browserCursor < s.browserScroll) s.browserScroll = s.browserCursor;
    else if (s.browserCursor >= s.browserScroll + 4) s.browserScroll = s.browserCursor - 3;
    /* hover preview: auto-play WAV under cursor; stop on dir or empty */
    const e = s.browserEntries[s.browserCursor];
    if (e && !e.dir) sp('preview_path', e.path);
    else             sp('preview_stop', '1');
    s.dirty = true;
}

function browserSelect() {
    const e = s.browserEntries[s.browserCursor];
    if (!e) return;
    if (e.dir) { browserOpen(e.path); }
    else {
        sp('preview_stop', '1');
        sp('sample_path', e.path);
        s.samplePath = e.path; s.sampleName = e.name.replace(/\.wav$/i, '');
        s.slicerState = 0; s.sliceCountActual = 0;
        resetPadState();
        s.view = 'main'; s.dirty = true;
    }
}

function fmtMs(v)    { return (v >= 0 ? '+' : '') + Math.round(v) + 'ms'; }
function fmtPitch(v) { return (v >= 0 ? '+' : '') + v.toFixed(1) + 'st'; }

/* param adjusters — update local padState AND write to DSP */
function adjustStartTrim(d) { pad().startTrim += d*5;                                          sp('slice_start_trim', pad().startTrim.toFixed(1)); s.dirty=true; }
function adjustEndTrim(d)   { pad().endTrim   += d*5;                                          sp('slice_end_trim',   pad().endTrim.toFixed(1));   s.dirty=true; }
function adjustAttack(d)    { pad().attack = Math.max(5,Math.min(2000, pad().attack+d*5));     sp('slice_attack',     pad().attack.toFixed(1));    s.dirty=true; }
function adjustDecay(d)     { pad().decay  = Math.max(0,Math.min(5000, pad().decay+d*20));     sp('slice_decay',      pad().decay.toFixed(1));     s.dirty=true; }
function adjustLoop(d)      { pad().loop   = Math.max(0,Math.min(2,    pad().loop+(d>0?1:-1)));sp('slice_loop',       String(pad().loop));          s.dirty=true; }
/* global adjusters */
function adjustMode(d)      { s.mode = s.mode==='trigger'?'gate':'trigger'; sp('mode',s.mode); s.dirty=true; }
function adjustPitch(d)     { s.pitch = Math.max(-24,Math.min(24,s.pitch+d*0.5)); sp('pitch',s.pitch.toFixed(1)); s.dirty=true; }
function adjustGlobalGain(d){ s.globalGain = Math.max(0,Math.min(1,s.globalGain+d*0.05)); sp('global_gain',s.globalGain.toFixed(3)); s.dirty=true; }
function adjustGlobalAttack(d) {
    s.globalAttack = Math.max(5,Math.min(2000,s.globalAttack+d*5));
    sp('global_attack',s.globalAttack.toFixed(1));
    for (let i = 0; i < MAX_SLICES; i++) padState[i].attack = s.globalAttack;
    s.dirty=true;
}
function adjustGlobalDecay(d) {
    s.globalDecay = Math.max(0,Math.min(5000,s.globalDecay+d*20));
    sp('global_decay',s.globalDecay.toFixed(1));
    for (let i = 0; i < MAX_SLICES; i++) padState[i].decay = s.globalDecay;
    s.dirty=true;
}
function adjustVelSens(d)   { s.velSens = s.velSens ? 0 : 1; sp('velocity_sens',String(s.velSens)); s.dirty=true; }
function adjustThreshold(d) { s.threshold=Math.max(0,Math.min(1,s.threshold+d*0.05)); sp('threshold',s.threshold.toFixed(3)); s.slicerState=0; s.dirty=true; }
/* per-pad adjusters */
function adjustPadGain(d)   { pad().gain = Math.max(0,Math.min(2,pad().gain+d*0.05)); sp('slice_gain',pad().gain.toFixed(3)); s.dirty=true; }
function adjustPadPitch(d)  { pad().pitchOffset = Math.max(-24,Math.min(24,pad().pitchOffset+d*0.5)); sp('slice_pitch',pad().pitchOffset.toFixed(1)); s.dirty=true; }
function adjustPadMode(d)   { pad().modeOverride = pad().modeOverride === 1 ? 0 : 1; sp('slice_mode',String(pad().modeOverride)); s.dirty=true; }
function triggerScan()      { resetPadState(); sp('scan','1'); }

/* chromatic range string for READY display, e.g. "C2-D#4 (32sl)" */
function rangeStr() {
    if (s.sliceCountActual <= 0) return '';
    const endNote = ROOT_NOTE + s.sliceCountActual - 1;
    const flag = s.sliceCountActual > 91 ? '*' : '';
    return 'C2-' + noteName(endNote) + ' (' + s.sliceCountActual + 'sl)' + flag;
}

function drawSampleName() {
    print(0, 0, (s.sampleName||'-- no sample --').substring(0,21), 1);
    fill_rect(0, 10, SCREEN_W, 1, 1);
}
function drawIdle() {
    clear_screen(); drawSampleName();
    print(0, 20, 'Thresh:' + Math.round(s.threshold*100) + '%  ~' + s.previewSlices + ' slices', 1);
    print(0, 36, 'Jog: adjust thresh', 1);
    print(0, 50, 'Jog click: scan', 1);
}
function drawNoSlices() {
    clear_screen(); drawSampleName();
    print(0, 20, 'No slices found', 1);
    print(0, 32, 'Thresh:' + Math.round(s.threshold*100) + '%  ~' + s.previewSlices + ' slices', 1);
    print(0, 44, 'Jog: adjust/click:scan', 1);
}
function drawScanFlash() {
    clear_screen(); drawSampleName();
    print(0, 20, 'Detected:', 1);
    print(0, 32, s.sliceCountActual + ' slices', 1);
}
const MODE_LABELS = { '0': 'TRIG', '1': 'GATE' };
function drawBankA() {
    clear_screen(); drawSampleName();
    if (s.editScope === 'P') {
        const p = pad();
        print(0, 13, '[P] Pad ' + (s.selectedSlice + 1), 1);
        print(0, 23, 'Atk:'+Math.round(p.attack - 5)+'ms  Dec:'+Math.round(p.decay)+'ms', 1);
        print(0, 33, '', 1);
        print(0, 43, 'Str:'+fmtMs(p.startTrim)+'  End:'+fmtMs(p.endTrim), 1);
        print(0, 53, rangeStr(), 1);
    } else {
        print(0, 13, '[G] Global', 1);
        print(0, 23, 'Atk:'+Math.round(s.globalAttack-5)+'ms  Dec:'+Math.round(s.globalDecay)+'ms', 1);
        print(0, 33, '', 1);
        print(0, 43, '', 1);
        print(0, 53, rangeStr(), 1);
    }
}
function drawBankB() {
    const p = pad();
    clear_screen(); drawSampleName();
    if (s.editScope === 'P') {
        print(0, 13, '[P] Pad '+(s.selectedSlice+1), 1);
        print(0, 23, 'Mode:'+MODE_LABELS[String(p.modeOverride)], 1);
        print(0, 33, 'Pitch:'+fmtPitch(p.pitchOffset), 1);
        print(0, 43, 'Gain:x'+Math.round(p.gain*100)+'%', 1);
        print(0, 53, 'Loop:'+LOOP_LABELS[p.loop], 1);
    } else {
        print(0, 13, '[G] Global', 1);
        print(0, 23, 'Mode:'+s.mode.toUpperCase(), 1);
        print(0, 33, 'Pitch:'+fmtPitch(s.pitch), 1);
        print(0, 43, 'Gain:'+Math.round(s.globalGain*100)+'%', 1);
        print(0, 53, 'Loop:'+LOOP_LABELS[p.loop], 1);
    }
}
function drawBrowser() {
    clear_screen();
    print(0, 0, 'Browse Samples', 1);
    fill_rect(0, 10, SCREEN_W, 1, 1);
    s.browserEntries.slice(s.browserScroll, s.browserScroll+4).forEach((e,i) => {
        const idx = s.browserScroll+i;
        print(0, 14+i*12, ((idx===s.browserCursor?'>':' ')+(e.dir?'/':' ')+e.name).substring(0,21), idx===s.browserCursor?2:1);
    });
    if (!s.browserEntries.length) print(0, 26, 'No WAV files here', 1);
}
function drawSensitivity() {
    clear_screen(); drawSampleName();
    print(0, 14, 'Sensitivity', 1);
    fill_rect(0, 26, Math.round(s.threshold*100), 8, 1);
    print(0, 38, Math.round(s.threshold*100)+'%', 1);
    print(0, 50, 'Jog:adjust  Clk:scan', 1);
}

function tick() {
    const newState = parseInt(gp('slicer_state', 0));
    if (newState !== s.slicerState) {
        s.slicerState = newState;
        s.sliceCountActual = parseInt(gp('slice_count_actual', 0));
        if (newState === 1) { s.scanFlashTicks = SCAN_FLASH_TICKS; s.knobBank = 'A'; s.selectedSlice = 0; }
        s.dirty = true;
    }
    if (s.slicerState === 1) {
        const dspSlice = parseInt(gp('selected_slice', s.selectedSlice));
        if (dspSlice >= 0 && dspSlice < s.sliceCountActual && dspSlice !== s.selectedSlice) {
            s.selectedSlice = dspSlice;
            s.dirty = true;
        }
    }
    if (s.slicerState === 0 || s.slicerState === 2) {
        const pv = parseInt(gp('preview_slices', 0));
        if (pv !== s.previewSlices) { s.previewSlices = pv; s.dirty = true; }
    }
    if (s.jogClickTicks > 0) { s.jogClickTicks--; if (s.jogClickTicks === 0 && s.jogClickAction) { s.jogClickAction(); s.jogClickAction = null; } }
    if (s.scanFlashTicks > 0) { s.scanFlashTicks--; if (s.scanFlashTicks===0) s.dirty=true; }
    if (!s.dirty) return;
    s.dirty = false;

    if (s.view === 'browser')     { drawBrowser();     return; }
    if (s.view === 'sensitivity') { drawSensitivity(); return; }
    if (!s.samplePath)            { drawBrowser();     return; }
    if (s.slicerState === 0)      { drawIdle();        return; }
    if (s.slicerState === 2)      { drawNoSlices();    return; }
    if (s.scanFlashTicks > 0)     { drawScanFlash();   return; }
    if (s.knobBank === 'B')       { drawBankB();       return; }
    drawBankA();
}

function onMidiMessageInternal(data) {
    const status = data[0] & 0xF0;
    const byte1  = data[1];
    const byte2  = data[2];

    /* Knob touch (capacitive) — notes 0-7, switch bank display on touch */
    if (status === 0x90 && byte1 <= 7) {
        if (byte2 > 0 && s.view === 'main' && s.slicerState === 1) {
            const newBank = byte1 <= 3 ? 'A' : 'B';
            if (s.knobBank !== newBank) { s.knobBank = newBank; s.dirty = true; }
        }
        return;
    }

    /* Pad hit — notes 68-99, slice_idx = note - 68 (0-31, matches DSP pad mapping) */
    if (status === 0x90 && byte2 > 0 && byte1 >= 68 && byte1 <= 99) {
        if (s.slicerState === 1) {
            const slice = byte1 - 68;
            if (slice >= s.sliceCountActual) return;  /* pad out of range for this scan */
            selectSlice(slice);
            /* reinforce per-pad mode to DSP so voice_start picks it up */
            sp('slice_mode', String(padState[slice].modeOverride));
            s.dirty = true;
            if (s.view !== 'main') { s.view = 'main'; }
        }
        return;
    }

    if (status !== 0xB0) return;
    const cc = byte1, val = byte2;

    /* Jog rotate (CC14) */
    if (cc === MoveMainKnob) {
        const delta = decodeDelta(val);
        if (s.view === 'browser')     { browserScrollBy(delta); return; }
        if (s.view === 'sensitivity') { adjustThreshold(delta); return; }
        if (!s.samplePath) { s.view = 'browser'; s.dirty = true; return; }
        if (s.slicerState !== 1) { adjustThreshold(delta); return; }
        return;
    }

    /* Jog click (CC3 = MoveMainButton) */
    if (cc === MoveMainButton && val > 0) {
        if (s.view === 'browser')     { browserSelect(); return; }
        if (s.view === 'sensitivity') { triggerScan(); s.view = 'main'; s.dirty = true; return; }
        /* Double-click detection: second click within window → file browser */
        if (s.jogClickTicks > 0) {
            s.jogClickTicks = 0; s.jogClickAction = null;
            browserOpen(s.browserPath || SAMPLES_DIR);
            s.view = 'browser'; s.dirty = true;
            return;
        }
        /* First click — defer action, start double-click window */
        if (s.slicerState !== 1) {
            s.jogClickAction = () => { triggerScan(); };
        } else {
            s.jogClickAction = () => { s.editScope = s.editScope === 'G' ? 'P' : 'G'; s.dirty = true; };
        }
        s.jogClickTicks = DBLCLICK_TICKS;
        return;
    }

    /* Knobs 1-4: bank A (per-pad or global based on editScope) */
    if (cc===MoveKnob1||cc===MoveKnob2||cc===MoveKnob3||cc===MoveKnob4) {
        s.knobBank='A'; s.dirty=true;
        const d=decodeDelta(val);
        if (s.editScope === 'P') {
            if (s.slicerState!==1) return;
            if (cc===MoveKnob1) adjustAttack(d);
            if (cc===MoveKnob2) adjustDecay(d);
            if (cc===MoveKnob3) adjustStartTrim(d);
            if (cc===MoveKnob4) adjustEndTrim(d);
        } else {
            if (cc===MoveKnob1) adjustGlobalAttack(d);
            if (cc===MoveKnob2) adjustGlobalDecay(d);
            /* K3, K4 inactive in global scope */
        }
        return;
    }

    /* Knobs 5-8: bank B (global or per-pad based on editScope) */
    if (cc===MoveKnob5||cc===MoveKnob6||cc===MoveKnob7||cc===MoveKnob8) {
        s.knobBank='B'; s.dirty=true;
        if (s.slicerState!==1) return;
        const d=decodeDelta(val);
        if (s.editScope === 'G') {
            if (cc===MoveKnob5) adjustMode(d);
            if (cc===MoveKnob6) adjustPitch(d);
            if (cc===MoveKnob7) adjustGlobalGain(d);
            if (cc===MoveKnob8) adjustLoop(d);
        } else {
            if (cc===MoveKnob5) adjustPadMode(d);
            if (cc===MoveKnob6) adjustPadPitch(d);
            if (cc===MoveKnob7) adjustPadGain(d);
            if (cc===MoveKnob8) adjustLoop(d);
        }
        return;
    }
}

function init() {
    syncGlobal();
    resetPadState();
    browserOpen(SAMPLES_DIR);
    s.view  = s.samplePath ? 'main' : 'browser';
    s.dirty = true;
}

globalThis.chain_ui = { init, tick, onMidiMessageInternal };
