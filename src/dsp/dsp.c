/*
 * Slicer — move-anything DSP module
 * Transient-detection sample slicer, 128 slices, trigger/gate, A/D envelope
 * API v2, 44100Hz, stereo interleaved int16_t, 128 frames/block
 *
 * Per-pad params: start_offset_ms, end_offset_ms, attack_ms, decay_ms, gain, loop_mode
 * Global params:  pitch, mode_gate, threshold, slice_count, velocity_sens
 *
 * Note → slice mapping:
 *   Move pads (notes 68–99): slice_idx = note - 68  (0–31, direct pad mapping)
 *   All other notes:         slice_idx = note - 36  (C2 = root, chromatic)
 *   Notes outside [0, slice_count_actual) are silently ignored.
 *
 * Detection always finds up to MAX_SLICES (128) transients.
 * slice_count (8/16/32/64) is only used as fallback chunk size when no
 * transients are found.
 * Max chromatic reach via MIDI: note 127 - 36 = 91 slices.
 */

#include "host/plugin_api_v1.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>

#define SAMPLE_RATE      44100
#define BLOCK_SIZE       128
#define MAX_SLICES       128
#define MAX_VOICES       8
#define RELEASE_SAMPLES  64   /* ~1.5ms fade-out when slice end is reached */
#define ROOT_NOTE        36   /* C2: chromatic mapping root */
#define PAD_BASE         68   /* Move pad 1 = MIDI note 68 = slice 0 */
#define PAD_TOP          99   /* Move pad 32 = MIDI note 99 = slice 31 */

/* loop modes */
#define LOOP_OFF      0
#define LOOP_FORWARD  1
#define LOOP_PINGPONG 2

/* ── Envelope states ─────────────────────────────────────────────────────── */
typedef enum { ENV_IDLE, ENV_ATTACK, ENV_SUSTAIN, ENV_DECAY } env_state_t;

/* ── Voice ───────────────────────────────────────────────────────────────── */
typedef struct {
    int       active;
    int       note;
    int       slice_idx;
    int64_t   pos;           /* current read position (fixed point: pos >> 16) */
    float     rate;          /* playback rate (pitch shift) */
    int       direction;     /* +1 forward, -1 reverse (ping pong) */
    int32_t   slice_start;   /* effective start after trim offset */
    int32_t   slice_end;     /* effective end after trim offset */
    int       loop_mode;     /* per-voice snapshot of pad's loop_mode */
    env_state_t env_state;
    float     env_val;
    float     env_attack;    /* attack coefficient per sample */
    float     env_decay;     /* decay coefficient per sample */
    float     velocity;      /* velocity gain: vel/127 when sens on, 1.0 when off */
    float     pad_gain;      /* per-pad gain snapshot */
    int       release;       /* countdown for end-of-slice fade-out */
    int       released;      /* note has been released (gate/loop mode) */
} voice_t;

/* ── Per-pad parameters ──────────────────────────────────────────────────── */
typedef struct {
    float  start_offset_ms;  /* offset from detected slice start, ± ms */
    float  end_offset_ms;    /* offset from detected slice end, ± ms */
    float  attack_ms;
    float  decay_ms;
    float  gain;             /* 0.0–1.0 */
    int    loop_mode;        /* LOOP_OFF / LOOP_FORWARD / LOOP_PINGPONG */
} pad_params_t;

/* ── Main plugin state ───────────────────────────────────────────────────── */
typedef struct {
    /* sample data */
    int16_t  *sample_data;   /* stereo interleaved */
    int32_t   sample_frames; /* total frames */
    char      sample_path[512];

    /* slices */
    int32_t   slice_points[MAX_SLICES + 1];
    int       slice_count_actual;
    int       preview_slice_count; /* live preview during threshold adjust */

    /* global params */
    float     threshold;
    int       slice_count;   /* fallback chunk count: 8/16/32/64 */
    float     pitch;         /* semitones ±24 */
    int       mode_gate;     /* 0=trigger, 1=gate */
    int       velocity_sens; /* 0=off (fixed gain 1.0), 1=on (vel/127) */

    /* per-pad params */
    pad_params_t pads[MAX_SLICES];

    /* selected slice (for param get/set from UI) */
    int       selected_slice;

    /* state */
    int       slicer_state;  /* 0=IDLE, 1=READY, 2=NO_SLICES */

    /* voices */
    voice_t   voices[MAX_VOICES];

    /* preview playback (browser hover) */
    int16_t  *preview_data;
    int32_t   preview_frames;
    int64_t   preview_pos;
    int       preview_active;
} slicer_t;

/* ── Helpers ─────────────────────────────────────────────────────────────── */
static float semitones_to_rate(float semitones) {
    return powf(2.0f, semitones / 12.0f);
}

static inline int16_t clamp16(float v) {
    if (v >  32767.0f) return  32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}

static inline int32_t clampi(int32_t v, int32_t lo, int32_t hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static float ms_to_coeff(float ms, float target) {
    if (ms < 0.5f) return 1.0f;
    int samples = (int)(ms * SAMPLE_RATE / 1000.0f);
    return powf(target, 1.0f / (float)samples);
}

static inline float ms_to_frames(float ms) {
    return ms * SAMPLE_RATE / 1000.0f;
}

static void reset_pad(pad_params_t *p) {
    p->start_offset_ms = 0.0f;
    p->end_offset_ms   = 0.0f;
    p->attack_ms       = 5.0f;
    p->decay_ms        = 500.0f;
    p->gain            = 0.8f;
    p->loop_mode       = LOOP_OFF;
}

/* ── WAV loader (16-bit and 24-bit PCM, any chunk layout) ────────────────── */
/* Loads a WAV file into a newly malloc'd stereo int16 buffer.
   On success returns 1 and sets *buf_out / *frames_out; caller must free. */
static int load_wav_buf(const char *path, int16_t **buf_out, int32_t *frames_out) {
    FILE *f = fopen(path, "rb");
    if (!f) return 0;

    char tag[4];
    uint32_t u32;

    if (fread(tag, 1, 4, f) < 4 || memcmp(tag, "RIFF", 4) != 0) { fclose(f); return 0; }
    fread(&u32, 4, 1, f);
    if (fread(tag, 1, 4, f) < 4 || memcmp(tag, "WAVE", 4) != 0) { fclose(f); return 0; }

    uint16_t channels = 0, bits_per_sample = 0, audio_format = 0;
    uint32_t data_size = 0;
    long     data_offset = 0;
    int      found_fmt = 0, found_data = 0;

    char     chunk_id[4];
    uint32_t chunk_size;
    while (fread(chunk_id, 1, 4, f) == 4 && fread(&chunk_size, 4, 1, f) == 1) {
        if (memcmp(chunk_id, "fmt ", 4) == 0) {
            uint8_t fmt[16];
            uint32_t rd = chunk_size < 16 ? chunk_size : 16;
            if (fread(fmt, 1, rd, f) < rd) { fclose(f); return 0; }
            if (chunk_size > rd) fseek(f, (long)(chunk_size - rd), SEEK_CUR);
            audio_format    = fmt[0]  | (fmt[1] <<8);
            channels        = fmt[2]  | (fmt[3] <<8);
            bits_per_sample = fmt[14] | (fmt[15]<<8);
            found_fmt = 1;
        } else if (memcmp(chunk_id, "data", 4) == 0) {
            data_size   = chunk_size;
            data_offset = ftell(f);
            found_data  = 1;
            break;
        } else {
            fseek(f, (long)(chunk_size + (chunk_size & 1)), SEEK_CUR);
        }
    }

    if (!found_fmt || !found_data || channels == 0 || data_size == 0) { fclose(f); return 0; }
    if (audio_format != 1 && audio_format != 0xFFFE) { fclose(f); return 0; }
    if (bits_per_sample != 16 && bits_per_sample != 24) { fclose(f); return 0; }

    uint32_t bytes_per_smp = bits_per_sample / 8;
    int32_t  frames = (int32_t)(data_size / (channels * bytes_per_smp));
    if (frames <= 0) { fclose(f); return 0; }

    int16_t *buf = malloc((size_t)frames * 2 * sizeof(int16_t));
    if (!buf) { fclose(f); return 0; }

    fseek(f, data_offset, SEEK_SET);

    if (bits_per_sample == 16) {
        if (channels == 2) {
            fread(buf, sizeof(int16_t), (size_t)frames * 2, f);
        } else {
            int16_t *mono = malloc((size_t)frames * sizeof(int16_t));
            if (!mono) { free(buf); fclose(f); return 0; }
            fread(mono, sizeof(int16_t), (size_t)frames, f);
            for (int32_t i = 0; i < frames; i++) buf[i*2] = buf[i*2+1] = mono[i];
            free(mono);
        }
    } else {
        uint32_t raw_size = (uint32_t)frames * channels * 3;
        uint8_t *raw = malloc(raw_size);
        if (!raw) { free(buf); fclose(f); return 0; }
        fread(raw, 1, raw_size, f);
        for (int32_t i = 0; i < frames; i++) {
            for (int c = 0; c < 2; c++) {
                int src_c = (c < (int)channels) ? c : 0;
                int off   = (int)(i * channels * 3 + src_c * 3);
                int32_t v = ((int32_t)raw[off+2] << 24)
                          | ((int32_t)raw[off+1] << 16)
                          | ((int32_t)raw[off+0] << 8);
                buf[i*2+c] = (int16_t)(v >> 16);
            }
        }
        free(raw);
    }

    fclose(f);
    *buf_out    = buf;
    *frames_out = frames;
    return 1;
}

static int load_wav(slicer_t *s, const char *path) {
    int16_t *buf; int32_t frames;
    if (!load_wav_buf(path, &buf, &frames)) return 0;
    if (s->sample_data) free(s->sample_data);
    s->sample_data   = buf;
    s->sample_frames = frames;
    strncpy(s->sample_path, path, sizeof(s->sample_path)-1);
    return 1;
}

/* ── Transient detection ─────────────────────────────────────────────────── */
static void detect_slices(slicer_t *s) {
    if (!s->sample_data || s->sample_frames == 0) return;

    int32_t total_start = 0;
    int32_t total_end   = s->sample_frames;
    int32_t region      = total_end - total_start;
    if (region <= 0) return;

    int win = 512;
    float det_threshold = 1.5f + (1.0f - s->threshold) * 8.0f;

    int32_t markers[MAX_SLICES];
    int     nmarkers = 0;
    markers[nmarkers++] = total_start;

    float prev_rms = 0.001f;

    /* scan for transients up to MAX_SLICES — not capped by slice_count */
    for (int32_t i = total_start; i < total_end - win && nmarkers < MAX_SLICES; i += win/2) {
        float rms = 0.0f;
        for (int j = 0; j < win; j++) {
            int32_t idx = (i + j) * 2;
            float l = s->sample_data[idx]   / 32768.0f;
            float r = s->sample_data[idx+1] / 32768.0f;
            rms += l*l + r*r;
        }
        rms = sqrtf(rms / (win * 2));

        if (rms > prev_rms * det_threshold && rms > 0.01f) {
            int32_t min_gap = SAMPLE_RATE / 32;
            if (nmarkers == 0 || (i - markers[nmarkers-1]) > min_gap) {
                markers[nmarkers++] = i;
            }
        }
        prev_rms = rms * 0.3f + prev_rms * 0.7f;
    }

    /* fallback: no transients — divide evenly using slice_count as chunk count */
    if (nmarkers < 2) {
        nmarkers = 0;
        int n = (s->slice_count > 0) ? s->slice_count : 16;
        int32_t step = region / n;
        for (int i = 0; i < n; i++) {
            markers[nmarkers++] = total_start + i * step;
        }
    }

    s->slice_count_actual = nmarkers;
    for (int i = 0; i < nmarkers; i++) s->slice_points[i] = markers[i];
    s->slice_points[nmarkers] = total_end;

    /* reset all per-pad params on fresh scan */
    for (int i = 0; i < MAX_SLICES; i++) reset_pad(&s->pads[i]);
    s->preview_slice_count = s->slice_count_actual;
}

/* Count-only preview — same algorithm as detect_slices but doesn't
   touch slice_points, pad params, or slice_count_actual.             */
static void preview_slice_count(slicer_t *s) {
    if (!s->sample_data || s->sample_frames == 0) { s->preview_slice_count = 0; return; }

    int32_t total_end = s->sample_frames;
    int win = 512;
    float det_threshold = 1.5f + (1.0f - s->threshold) * 8.0f;
    int nmarkers = 1; /* first marker at start */
    int32_t last_marker = 0;
    float prev_rms = 0.001f;

    for (int32_t i = 0; i < total_end - win && nmarkers < MAX_SLICES; i += win/2) {
        float rms = 0.0f;
        for (int j = 0; j < win; j++) {
            int32_t idx = (i + j) * 2;
            float l = s->sample_data[idx]   / 32768.0f;
            float r = s->sample_data[idx+1] / 32768.0f;
            rms += l*l + r*r;
        }
        rms = sqrtf(rms / (win * 2));
        if (rms > prev_rms * det_threshold && rms > 0.01f) {
            int32_t min_gap = SAMPLE_RATE / 32;
            if ((i - last_marker) > min_gap) {
                nmarkers++;
                last_marker = i;
            }
        }
        prev_rms = rms * 0.3f + prev_rms * 0.7f;
    }
    if (nmarkers < 2) {
        int n = (s->slice_count > 0) ? s->slice_count : 16;
        nmarkers = n;
    }
    s->preview_slice_count = nmarkers;
}

/* ── Voice management ────────────────────────────────────────────────────── */
static voice_t* find_free_voice(slicer_t *s) {
    for (int i = 0; i < MAX_VOICES; i++) {
        if (!s->voices[i].active) return &s->voices[i];
    }
    memset(&s->voices[0], 0, sizeof(voice_t));
    return &s->voices[0];
}

static voice_t* find_voice_for_note(slicer_t *s, int note) {
    for (int i = 0; i < MAX_VOICES; i++) {
        if (s->voices[i].active && s->voices[i].note == note) return &s->voices[i];
    }
    return NULL;
}

/* note_to_slice: returns slice index for a MIDI note, or -1 if out of range.
   Move pads (notes 68–99) map directly to slices 0–31 (note - PAD_BASE).
   All other notes use chromatic mapping from ROOT_NOTE (C2=36): note 36 → 0.
   Notes outside [0, slice_count_actual) are silently ignored. */
static int note_to_slice(slicer_t *s, int note) {
    int idx;
    if (note >= PAD_BASE && note <= PAD_TOP)
        idx = note - PAD_BASE;   /* Move pads 68–99 → slices 0–31 */
    else
        idx = note - ROOT_NOTE;  /* chromatic from C2 */
    if (idx < 0 || idx >= s->slice_count_actual) return -1;
    return idx;
}

static void voice_start(slicer_t *s, int note, int velocity) {
    if (s->slice_count_actual == 0 || !s->sample_data) return;

    int slice_idx = note_to_slice(s, note);
    if (slice_idx < 0) return;  /* out of range — silent ignore */

    s->selected_slice = slice_idx;

    pad_params_t *p = &s->pads[slice_idx];

    /* apply per-pad offsets to detected boundaries, clamp to file */
    int32_t base_start = s->slice_points[slice_idx];
    int32_t base_end   = s->slice_points[slice_idx + 1];
    int32_t start = clampi(base_start + (int32_t)ms_to_frames(p->start_offset_ms), 0, s->sample_frames - 1);
    int32_t end   = clampi(base_end   + (int32_t)ms_to_frames(p->end_offset_ms),   1, s->sample_frames);
    if (end <= start) end = start + 1;

    voice_t *v = find_voice_for_note(s, note);
    if (v) {
        memset(v, 0, sizeof(voice_t));
    } else {
        v = find_free_voice(s);
    }

    v->active      = 1;
    v->note        = note;
    v->slice_idx   = slice_idx;
    v->pos         = (int64_t)start << 16;
    v->rate        = semitones_to_rate(s->pitch);
    v->direction   = 1;
    v->slice_start = start;
    v->slice_end   = end;
    v->loop_mode   = p->loop_mode;
    v->env_state   = ENV_ATTACK;
    v->env_val     = 0.0f;
    v->env_attack  = ms_to_coeff(p->attack_ms, 0.001f);
    v->env_decay   = ms_to_coeff(p->decay_ms,  0.001f);
    v->velocity    = s->velocity_sens ? (velocity / 127.0f) : 1.0f;
    v->pad_gain    = p->gain;
    v->release     = 0;
    v->released    = 0;
}

static void voice_release(voice_t *v) {
    if (v->active && v->env_state != ENV_IDLE) {
        v->released  = 1;
        v->env_state = ENV_DECAY;
    }
}

/* ── JSON helpers for state persistence ───────────────────────────────────── */
static int json_get_string(const char *json, const char *key, char *out, int out_len) {
    if (!json || !key || !out || out_len < 1) return 0;
    char search[64];
    snprintf(search, sizeof(search), "\"%s\"", key);
    const char *pos = strstr(json, search);
    if (!pos) return 0;
    const char *colon = strchr(pos, ':');
    if (!colon) return 0;
    while (*colon && (*colon == ':' || *colon == ' ' || *colon == '\t')) colon++;
    if (*colon != '"') return 0;
    colon++;
    const char *end = strchr(colon, '"');
    if (!end) return 0;
    int len = (int)(end - colon);
    if (len >= out_len) len = out_len - 1;
    strncpy(out, colon, len);
    out[len] = '\0';
    return len;
}

static int json_get_int(const char *json, const char *key, int *out) {
    if (!json || !key || !out) return 0;
    char search[64];
    snprintf(search, sizeof(search), "\"%s\"", key);
    const char *pos = strstr(json, search);
    if (!pos) return 0;
    const char *colon = strchr(pos, ':');
    if (!colon) return 0;
    colon++;
    while (*colon && (*colon == ' ' || *colon == '\t')) colon++;
    *out = atoi(colon);
    return 1;
}

static int json_get_float(const char *json, const char *key, float *out) {
    if (!json || !key || !out) return 0;
    char search[64];
    snprintf(search, sizeof(search), "\"%s\"", key);
    const char *pos = strstr(json, search);
    if (!pos) return 0;
    const char *colon = strchr(pos, ':');
    if (!colon) return 0;
    colon++;
    while (*colon && (*colon == ' ' || *colon == '\t')) colon++;
    *out = (float)atof(colon);
    return 1;
}

/* ── API callbacks ───────────────────────────────────────────────────────── */
static void* v2_create_instance(const char *module_dir, const char *json_defaults) {
    (void)module_dir; (void)json_defaults;
    slicer_t *s = calloc(1, sizeof(slicer_t));
    s->threshold      = 0.5f;
    s->slice_count    = 16;
    s->pitch          = 0.0f;
    s->mode_gate      = 1;
    s->velocity_sens  = 1;
    s->selected_slice = 0;
    s->slicer_state   = 0;
    for (int i = 0; i < MAX_SLICES; i++) reset_pad(&s->pads[i]);
    return s;
}

static void v2_destroy_instance(void *inst) {
    slicer_t *s = inst;
    if (s->sample_data)   free(s->sample_data);
    if (s->preview_data)  free(s->preview_data);
    free(s);
}

static void v2_set_param(void *inst, const char *key, const char *val) {
    slicer_t *s = inst;

    /* global params */
    if (strcmp(key, "threshold") == 0) {
        s->threshold    = atof(val);
        s->slicer_state = 0;
        preview_slice_count(s);
    } else if (strcmp(key, "slices") == 0) {
        int n = atoi(val);
        if (n==8||n==16||n==32||n==64) s->slice_count = n;
        s->slicer_state = 0;
    } else if (strcmp(key, "pitch") == 0) {
        s->pitch = atof(val);
    } else if (strcmp(key, "mode") == 0) {
        s->mode_gate = (strcmp(val, "gate") == 0) ? 1 : 0;
    } else if (strcmp(key, "velocity_sens") == 0) {
        s->velocity_sens = atoi(val) ? 1 : 0;
    } else if (strcmp(key, "selected_slice") == 0) {
        int n = atoi(val);
        if (n >= 0 && n < MAX_SLICES) s->selected_slice = n;

    /* per-pad params (operate on selected_slice) */
    } else if (strcmp(key, "slice_start_trim") == 0) {
        s->pads[s->selected_slice].start_offset_ms = atof(val);
    } else if (strcmp(key, "slice_end_trim") == 0) {
        s->pads[s->selected_slice].end_offset_ms = atof(val);
    } else if (strcmp(key, "slice_attack") == 0) {
        float a = atof(val); if (a < 5.0f) a = 5.0f;
        s->pads[s->selected_slice].attack_ms = a;
    } else if (strcmp(key, "slice_decay") == 0) {
        s->pads[s->selected_slice].decay_ms = atof(val);
    } else if (strcmp(key, "slice_gain") == 0) {
        s->pads[s->selected_slice].gain = atof(val);
    } else if (strcmp(key, "slice_loop") == 0) {
        int n = atoi(val);
        if (n >= LOOP_OFF && n <= LOOP_PINGPONG)
            s->pads[s->selected_slice].loop_mode = n;

    /* sample + scan */
    } else if (strcmp(key, "sample_path") == 0) {
        if (load_wav(s, val)) { s->slicer_state = 0; preview_slice_count(s); }
    } else if (strcmp(key, "scan") == 0) {
        detect_slices(s);
        s->slicer_state = (s->slice_count_actual > 0) ? 1 : 2;

    /* browser hover preview */
    } else if (strcmp(key, "preview_path") == 0) {
        s->preview_active = 0;
        int16_t *buf; int32_t frames;
        if (load_wav_buf(val, &buf, &frames)) {
            if (s->preview_data) free(s->preview_data);
            s->preview_data   = buf;
            s->preview_frames = frames;
            s->preview_pos    = 0;
            s->preview_active = 1;
        }
    } else if (strcmp(key, "preview_stop") == 0) {
        s->preview_active = 0;

    /* ── State persistence (Option B: save/restore slice boundaries) ───── */
    } else if (strcmp(key, "state") == 0) {
        float fval;
        int ival;
        char str[512];

        /* Scalar params */
        if (json_get_float(val, "threshold", &fval)) s->threshold = fval;
        if (json_get_int(val, "slices", &ival)) {
            if (ival==8||ival==16||ival==32||ival==64) s->slice_count = ival;
        }
        if (json_get_float(val, "pitch", &fval)) s->pitch = fval;
        if (json_get_string(val, "mode", str, sizeof(str)))
            s->mode_gate = (strcmp(str, "gate") == 0) ? 1 : 0;
        if (json_get_int(val, "vel_sens", &ival)) s->velocity_sens = ival ? 1 : 0;

        /* Load sample from saved path */
        if (json_get_string(val, "sample_path", str, sizeof(str)) && str[0]) {
            load_wav(s, str);
        }

        /* Restore slice boundaries directly (no re-scan) */
        if (json_get_int(val, "sca", &ival) && ival > 0 && ival <= MAX_SLICES) {
            int n = ival;
            s->slice_count_actual = n;

            char csv[4096];
            if (json_get_string(val, "sp", csv, sizeof(csv))) {
                const char *p = csv;
                for (int i = 0; i <= n && *p; i++) {
                    s->slice_points[i] = (int32_t)atoi(p);
                    const char *c = strchr(p, ',');
                    if (!c) break;
                    p = c + 1;
                }
            }

            /* Reset pads then restore saved per-pad params */
            for (int i = 0; i < MAX_SLICES; i++) reset_pad(&s->pads[i]);

            if (json_get_string(val, "ps", csv, sizeof(csv))) {
                const char *p = csv;
                for (int i = 0; i < n && *p; i++) {
                    s->pads[i].start_offset_ms = (float)atof(p);
                    const char *c = strchr(p, ','); if (!c) break; p = c + 1;
                }
            }
            if (json_get_string(val, "pe", csv, sizeof(csv))) {
                const char *p = csv;
                for (int i = 0; i < n && *p; i++) {
                    s->pads[i].end_offset_ms = (float)atof(p);
                    const char *c = strchr(p, ','); if (!c) break; p = c + 1;
                }
            }
            if (json_get_string(val, "pa", csv, sizeof(csv))) {
                const char *p = csv;
                for (int i = 0; i < n && *p; i++) {
                    float a = (float)atof(p);
                    if (a < 5.0f) a = 5.0f;
                    s->pads[i].attack_ms = a;
                    const char *c = strchr(p, ','); if (!c) break; p = c + 1;
                }
            }
            if (json_get_string(val, "pd", csv, sizeof(csv))) {
                const char *p = csv;
                for (int i = 0; i < n && *p; i++) {
                    s->pads[i].decay_ms = (float)atof(p);
                    const char *c = strchr(p, ','); if (!c) break; p = c + 1;
                }
            }
            if (json_get_string(val, "pg", csv, sizeof(csv))) {
                const char *p = csv;
                for (int i = 0; i < n && *p; i++) {
                    s->pads[i].gain = (float)atof(p);
                    const char *c = strchr(p, ','); if (!c) break; p = c + 1;
                }
            }
            if (json_get_string(val, "pl", csv, sizeof(csv))) {
                const char *p = csv;
                for (int i = 0; i < n && *p; i++) {
                    int lm = atoi(p);
                    if (lm >= LOOP_OFF && lm <= LOOP_PINGPONG) s->pads[i].loop_mode = lm;
                    const char *c = strchr(p, ','); if (!c) break; p = c + 1;
                }
            }

            s->preview_slice_count = n;
        }

        /* Set slicer_state based on restored data */
        if (s->sample_data && s->slice_count_actual > 0)
            s->slicer_state = 1;  /* READY */
        else if (s->sample_data)
            s->slicer_state = 2;  /* NO_SLICES */
        else
            s->slicer_state = 0;  /* IDLE */
    }
}

static int v2_get_param(void *inst, const char *key, char *buf, int buf_len) {
    slicer_t *s = inst;
    pad_params_t *p = &s->pads[s->selected_slice];

    /* global */
    if (strcmp(key, "threshold") == 0)         return snprintf(buf, buf_len, "%.3f", s->threshold);
    if (strcmp(key, "slices") == 0)             return snprintf(buf, buf_len, "%d",   s->slice_count);
    if (strcmp(key, "pitch") == 0)              return snprintf(buf, buf_len, "%.1f", s->pitch);
    if (strcmp(key, "mode") == 0)               return snprintf(buf, buf_len, "%s",   s->mode_gate ? "gate" : "trigger");
    if (strcmp(key, "velocity_sens") == 0)      return snprintf(buf, buf_len, "%d",   s->velocity_sens);
    if (strcmp(key, "sample_path") == 0)        return snprintf(buf, buf_len, "%s",   s->sample_path);
    if (strcmp(key, "slice_count_actual") == 0) return snprintf(buf, buf_len, "%d",   s->slice_count_actual);
    if (strcmp(key, "preview_slices") == 0)     return snprintf(buf, buf_len, "%d",   s->preview_slice_count);
    if (strcmp(key, "slicer_state") == 0)       return snprintf(buf, buf_len, "%d",   s->slicer_state);
    if (strcmp(key, "selected_slice") == 0)     return snprintf(buf, buf_len, "%d",   s->selected_slice);

    /* per-pad (for selected_slice) */
    if (strcmp(key, "slice_start_trim") == 0)   return snprintf(buf, buf_len, "%.1f", p->start_offset_ms);
    if (strcmp(key, "slice_end_trim") == 0)     return snprintf(buf, buf_len, "%.1f", p->end_offset_ms);
    if (strcmp(key, "slice_attack") == 0)       return snprintf(buf, buf_len, "%.1f", p->attack_ms);
    if (strcmp(key, "slice_decay") == 0)        return snprintf(buf, buf_len, "%.1f", p->decay_ms);
    if (strcmp(key, "slice_gain") == 0)         return snprintf(buf, buf_len, "%.3f", p->gain);
    if (strcmp(key, "slice_loop") == 0)         return snprintf(buf, buf_len, "%d",   p->loop_mode);

    /* Shadow UI param metadata — expose all key params for knob editing */
    if (strcmp(key, "chain_params") == 0) {
        const char *json =
            "["
            "{\"key\":\"threshold\",\"name\":\"Sensitivity\","
             "\"type\":\"float\",\"min\":0.0,\"max\":1.0,\"default\":0.5},"
            "{\"key\":\"slices\",\"name\":\"Slices\","
             "\"type\":\"enum\",\"options\":[\"8\",\"16\",\"32\",\"64\"],\"default\":\"16\"},"
            "{\"key\":\"pitch\",\"name\":\"Pitch\","
             "\"type\":\"float\",\"min\":-24.0,\"max\":24.0,\"default\":0.0},"
            "{\"key\":\"velocity_sens\",\"name\":\"Velocity\","
             "\"type\":\"enum\",\"options\":[\"Off\",\"On\"],\"default\":\"On\"},"
            "{\"key\":\"mode\",\"name\":\"Mode\","
             "\"type\":\"enum\",\"options\":[\"trigger\",\"gate\"],\"default\":\"gate\"},"
            "{\"key\":\"slice_attack\",\"name\":\"Attack\","
             "\"type\":\"float\",\"min\":5.0,\"max\":500.0,\"default\":5.0},"
            "{\"key\":\"slice_decay\",\"name\":\"Decay\","
             "\"type\":\"float\",\"min\":0.0,\"max\":5000.0,\"default\":500.0},"
            "{\"key\":\"slice_gain\",\"name\":\"Gain\","
             "\"type\":\"float\",\"min\":0.0,\"max\":1.0,\"default\":0.8}"
            "]";
        int len = (int)strlen(json);
        if (len < buf_len) { memcpy(buf, json, (size_t)len + 1); return len; }
        return -1;
    }

    /* ── State persistence ──────────────────────────────────────────────── */
    if (strcmp(key, "state") == 0) {
        char tmp[8192];
        int pos = 0;
        int n = s->slice_count_actual;

        pos += snprintf(tmp + pos, sizeof(tmp) - pos,
            "{\"threshold\":%.3f,\"slices\":%d,\"pitch\":%.1f,"
            "\"mode\":\"%s\",\"vel_sens\":%d,"
            "\"sample_path\":\"%s\",\"sca\":%d",
            s->threshold, s->slice_count, s->pitch,
            s->mode_gate ? "gate" : "trigger",
            s->velocity_sens, s->sample_path, n);

        if (n > 0 && pos < (int)sizeof(tmp) - 64) {
            /* slice_points: n+1 values */
            pos += snprintf(tmp + pos, sizeof(tmp) - pos, ",\"sp\":\"");
            for (int i = 0; i <= n && pos < (int)sizeof(tmp) - 16; i++)
                pos += snprintf(tmp + pos, sizeof(tmp) - pos, "%s%d",
                                i ? "," : "", (int)s->slice_points[i]);
            pos += snprintf(tmp + pos, sizeof(tmp) - pos, "\"");

            /* per-pad start offsets */
            pos += snprintf(tmp + pos, sizeof(tmp) - pos, ",\"ps\":\"");
            for (int i = 0; i < n && pos < (int)sizeof(tmp) - 16; i++)
                pos += snprintf(tmp + pos, sizeof(tmp) - pos, "%s%.1f",
                                i ? "," : "", s->pads[i].start_offset_ms);
            pos += snprintf(tmp + pos, sizeof(tmp) - pos, "\"");

            /* per-pad end offsets */
            pos += snprintf(tmp + pos, sizeof(tmp) - pos, ",\"pe\":\"");
            for (int i = 0; i < n && pos < (int)sizeof(tmp) - 16; i++)
                pos += snprintf(tmp + pos, sizeof(tmp) - pos, "%s%.1f",
                                i ? "," : "", s->pads[i].end_offset_ms);
            pos += snprintf(tmp + pos, sizeof(tmp) - pos, "\"");

            /* per-pad attack */
            pos += snprintf(tmp + pos, sizeof(tmp) - pos, ",\"pa\":\"");
            for (int i = 0; i < n && pos < (int)sizeof(tmp) - 16; i++)
                pos += snprintf(tmp + pos, sizeof(tmp) - pos, "%s%.0f",
                                i ? "," : "", s->pads[i].attack_ms);
            pos += snprintf(tmp + pos, sizeof(tmp) - pos, "\"");

            /* per-pad decay */
            pos += snprintf(tmp + pos, sizeof(tmp) - pos, ",\"pd\":\"");
            for (int i = 0; i < n && pos < (int)sizeof(tmp) - 16; i++)
                pos += snprintf(tmp + pos, sizeof(tmp) - pos, "%s%.0f",
                                i ? "," : "", s->pads[i].decay_ms);
            pos += snprintf(tmp + pos, sizeof(tmp) - pos, "\"");

            /* per-pad gain */
            pos += snprintf(tmp + pos, sizeof(tmp) - pos, ",\"pg\":\"");
            for (int i = 0; i < n && pos < (int)sizeof(tmp) - 16; i++)
                pos += snprintf(tmp + pos, sizeof(tmp) - pos, "%s%.2f",
                                i ? "," : "", s->pads[i].gain);
            pos += snprintf(tmp + pos, sizeof(tmp) - pos, "\"");

            /* per-pad loop mode */
            pos += snprintf(tmp + pos, sizeof(tmp) - pos, ",\"pl\":\"");
            for (int i = 0; i < n && pos < (int)sizeof(tmp) - 16; i++)
                pos += snprintf(tmp + pos, sizeof(tmp) - pos, "%s%d",
                                i ? "," : "", s->pads[i].loop_mode);
            pos += snprintf(tmp + pos, sizeof(tmp) - pos, "\"");
        }

        pos += snprintf(tmp + pos, sizeof(tmp) - pos, "}");

        int len = pos < buf_len ? pos : buf_len - 1;
        memcpy(buf, tmp, len);
        buf[len] = '\0';
        return len;
    }

    return -1;
}

static void v2_on_midi(void *inst, const uint8_t *msg, int len, int source) {
    (void)source; (void)len;
    slicer_t *s = inst;

    uint8_t status   = msg[0] & 0xF0;
    uint8_t note     = msg[1];
    uint8_t velocity = (len > 2) ? msg[2] : 0;

    if (status == 0x90 && velocity > 0) {
        voice_start(s, note, velocity);
    } else if (status == 0x80 || (status == 0x90 && velocity == 0)) {
        /* Gate mode: note-off triggers decay.
           Loop voices in trigger mode also need note-off to exit the loop.
           Non-looping trigger voices ignore note-off — slice plays to end. */
        voice_t *v = find_voice_for_note(s, note);
        if (v && (s->mode_gate || v->loop_mode != LOOP_OFF)) {
            voice_release(v);
        }
    }
}

static void v2_render_block(void *inst, int16_t *out_lr, int frames) {
    slicer_t *s = inst;
    if (!s->sample_data && !s->preview_active) { memset(out_lr, 0, frames * 2 * sizeof(int16_t)); return; }

    float mix_l[BLOCK_SIZE];
    float mix_r[BLOCK_SIZE];
    memset(mix_l, 0, frames * sizeof(float));
    memset(mix_r, 0, frames * sizeof(float));

    for (int vi = 0; vi < MAX_VOICES; vi++) {
        voice_t *v = &s->voices[vi];
        if (!v->active) continue;

        for (int i = 0; i < frames; i++) {
            /* envelope */
            if (v->env_state == ENV_IDLE) { v->active = 0; v->env_val = 0.0f; break; }
            float env = v->env_val;
            switch (v->env_state) {
                case ENV_ATTACK:
                    env = env + (1.0f - env) * (1.0f - v->env_attack);
                    if (env >= 0.999f) {
                        env = 1.0f;
                        if (v->loop_mode != LOOP_OFF || s->mode_gate)
                            v->env_state = ENV_SUSTAIN;
                        else
                            v->env_state = ENV_DECAY;
                    }
                    break;
                case ENV_SUSTAIN:
                    env = 1.0f;
                    break;
                case ENV_DECAY:
                    env *= v->env_decay;
                    if (env < 0.0001f) {
                        env = 0.0f;
                        v->env_state = ENV_IDLE;
                        v->active = 0;
                    }
                    break;
                case ENV_IDLE:
                    v->active = 0;
                    break;
            }
            v->env_val = env;
            if (!v->active) break;

            /* position */
            int32_t pos_int = (int32_t)(v->pos >> 16);

            /* loop / ping-pong boundary handling */
            if (v->loop_mode == LOOP_FORWARD && !v->released) {
                if (pos_int >= v->slice_end) {
                    v->pos = (int64_t)v->slice_start << 16;
                    pos_int = v->slice_start;
                }
                if (pos_int < v->slice_start) {
                    v->pos = (int64_t)v->slice_start << 16;
                    pos_int = v->slice_start;
                }
            } else if (v->loop_mode == LOOP_PINGPONG && !v->released) {
                if (pos_int >= v->slice_end) {
                    v->direction = -1;
                    v->pos = ((int64_t)(v->slice_end - 1) << 16);
                    pos_int = v->slice_end - 1;
                } else if (pos_int < v->slice_start) {
                    v->direction = 1;
                    v->pos = (int64_t)v->slice_start << 16;
                    pos_int = v->slice_start;
                }
            } else {
                if (pos_int >= v->slice_end) {
                    if (v->release == 0) v->release = RELEASE_SAMPLES;
                    pos_int = v->slice_end - 1;
                }
            }

            /* linear interpolation — clamp to valid sample range */
            float frac       = (uint32_t)(v->pos & 0xFFFF) / 65536.0f;
            int32_t pos_next = pos_int + v->direction;
            if (pos_next >= v->slice_end)   pos_next = pos_int;
            if (pos_next < v->slice_start)  pos_next = pos_int;

            /* guard against out-of-bounds sample access */
            if (pos_int < 0 || pos_int >= s->sample_frames ||
                pos_next < 0 || pos_next >= s->sample_frames) {
                v->active = 0; v->env_val = 0.0f; break;
            }

            float l = s->sample_data[pos_int*2]   * (1.0f - frac)
                    + s->sample_data[pos_next*2]   * frac;
            float r = s->sample_data[pos_int*2+1] * (1.0f - frac)
                    + s->sample_data[pos_next*2+1] * frac;

            float amp = v->velocity * v->pad_gain * env;
            if (v->release > 0) {
                amp *= (float)v->release / (float)RELEASE_SAMPLES;
                if (--v->release == 0) { v->active = 0; v->env_val = 0.0f; }
            }

            mix_l[i] += l * amp;
            mix_r[i] += r * amp;
            if (!v->active) break;

            /* advance position (direction-aware) */
            v->pos += (int64_t)(v->direction * v->rate * 65536.0f);
        }
    }

    /* browser hover preview — plays full file at unity, stops at end */
    if (s->preview_active && s->preview_data) {
        for (int i = 0; i < frames; i++) {
            int32_t pi = (int32_t)(s->preview_pos >> 16);
            if (pi >= s->preview_frames) { s->preview_active = 0; break; }
            mix_l[i] += (float)s->preview_data[pi*2]   * 0.7f;
            mix_r[i] += (float)s->preview_data[pi*2+1] * 0.7f;
            s->preview_pos += 65536LL;
        }
    }

    for (int i = 0; i < frames; i++) {
        out_lr[i*2]   = clamp16(mix_l[i]);
        out_lr[i*2+1] = clamp16(mix_r[i]);
    }
}

/* ── Plugin entry point ──────────────────────────────────────────────────── */
static plugin_api_v2_t g_api = {
    .api_version     = MOVE_PLUGIN_API_VERSION_2,
    .create_instance = v2_create_instance,
    .destroy_instance= v2_destroy_instance,
    .on_midi         = v2_on_midi,
    .set_param       = v2_set_param,
    .get_param       = v2_get_param,
    .render_block    = v2_render_block
};

plugin_api_v2_t* move_plugin_init_v2(const host_api_v1_t *host) {
    (void)host;
    return &g_api;
}
