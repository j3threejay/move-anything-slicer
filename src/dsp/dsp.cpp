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

extern "C" {
#include "host/plugin_api_v1.h"
}
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <cstdint>

#include <bungee/Bungee.h>

/* One stretcher per voice. Tempo sync means the speed ratio is almost never
   exactly 1.0, so nearly every voice needs a stretcher — a smaller pool would
   silently drop the overflow voices to rate-based playback at the wrong tempo. */
#define STRETCHER_POOL_SIZE 8

#define MINIMP3_IMPLEMENTATION
#include "dsp/minimp3.h"

#define DR_FLAC_IMPLEMENTATION
#define DR_FLAC_NO_STDIO
#include "dsp/dr_flac.h"

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
#define LOOP_REVERSE  3

/* slice algorithms */
#define SLICE_ALGO_TRANSIENT 0
#define SLICE_ALGO_RANDOM    1

/* tempo sources, in priority order — reported to the UI as a 3-char tag */
#define TEMPO_SRC_CLOCK   0   /* live MIDI clock */
#define TEMPO_SRC_SET     1   /* current Set's Song.abl */
#define TEMPO_SRC_CFG     2   /* move-anything settings.txt */
#define TEMPO_SRC_DEFAULT 3   /* nothing found — 120 */

/* Host tempo discovery. The plugin API carries no tempo, so we read the same
   files the shim maintains: active_set.txt is written on every set change
   (line 1 = UUID, line 2 = set name) and resolves to that Set's Song.abl. */
#define ACTIVE_SET_PATH  "/data/UserData/move-anything/active_set.txt"
#define SETS_DIR         "/data/UserData/UserLibrary/Sets"
#define MA_SETTINGS_PATH "/data/UserData/move-anything/settings.txt"

#define DEFAULT_BPM      120.0f
#define BPM_MIN          40.0f
#define BPM_MAX          400.0f

/* Sample BPM detection: onset envelope resolution */
#define BPM_HOP          256   /* ~172 envelope frames/sec at 44.1k */
#define BPM_WIN          512
#define BPM_SEARCH_MIN   60.0f
#define BPM_SEARCH_MAX   200.0f

/* Clock is considered live if a tick arrived within this many frames */
#define CLOCK_STALE_FRAMES (SAMPLE_RATE * 2)

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
    float     pad_gain;      /* effective gain: global * per-pad */
    int       mode_gate;     /* per-voice snapshot of effective mode */
    int       release;       /* countdown for end-of-slice fade-out */
    int       released;      /* note has been released (gate/loop mode) */

    /* Bungee time-stretch */
    int       stretcher_idx; /* index into stretcher pool, or -1 for rate-based */
    Bungee::Request bungee_req;
    float    *grain_out_buf;    /* L+R interleaved output cache (allocated per voice) */
    int       grain_out_pos;    /* read position in grain output buffer */
    int       grain_out_count;  /* frames available in grain output buffer */
} voice_t;

/* ── Per-pad parameters ──────────────────────────────────────────────────── */
typedef struct {
    float  start_offset_ms;  /* offset from detected slice start, ± ms */
    float  end_offset_ms;    /* offset from detected slice end, ± ms */
    float  attack_ms;
    float  decay_ms;
    float  gain;             /* per-pad gain multiplier (0.0–2.0, default 1.0) */
    float  pitch_offset;     /* per-pad pitch offset in semitones (default 0) */
    int    mode_override;    /* -1=follow global, 0=trigger, 1=gate */
    int    loop_mode;        /* LOOP_OFF / LOOP_FORWARD / LOOP_PINGPONG / LOOP_REVERSE */
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
    float     pitch;         /* global pitch in semitones ±24 */
    float     global_gain;   /* global gain 0.0–1.0 (default 0.8) */
    int       mode_gate;     /* global mode: 0=trigger, 1=gate */
    int       velocity_sens; /* 0=off (fixed gain 1.0), 1=on (vel/127) */
    int       mono_mode;     /* 0=poly (8 voices), 1=mono (1 voice) */

    /* tempo sync — Bungee speed is derived, never set by hand */
    float     sample_bpm;      /* working BPM of the loaded sample (after ÷2/×2) */
    float     sample_bpm_raw;  /* as detected, before octave correction */
    float     host_bpm;        /* Move set/session tempo */
    int       tempo_src;       /* TEMPO_SRC_* — where host_bpm came from */
    int       sync_enabled;    /* 1 = stretch sample_bpm → host_bpm */

    /* MIDI clock measurement (fed by realtime bytes in v2_on_midi) */
    int64_t   frame_clock;         /* frames rendered since instance creation */
    int64_t   clk_last_tick_frame;
    int64_t   clk_last_beat_frame;
    int       clk_ticks;           /* 0–23, ticks since last beat boundary */
    float     clk_bpm;

    /* slicing */
    int       slice_algo;    /* SLICE_ALGO_TRANSIENT / SLICE_ALGO_RANDOM */
    int       playthrough;   /* 1 = pads play on past the slice end while held */
    uint32_t  rng_state;     /* xorshift state for random slicing */

    /* per-pad params */
    pad_params_t pads[MAX_SLICES];

    /* selected slice (for param get/set from UI) */
    int       selected_slice;

    /* state */
    int       slicer_state;  /* 0=IDLE, 1=READY, 2=NO_SLICES */

    /* voices */
    voice_t   voices[MAX_VOICES];

    /* Bungee stretcher pool */
    Bungee::Stretcher<Bungee::Basic> *stretchers[STRETCHER_POOL_SIZE];
    int       stretcher_owner[STRETCHER_POOL_SIZE]; /* voice index or -1 */
    int       max_grain_frames; /* from Bungee maxInputFrameCount() */
    float    *grain_input;    /* shared input: L at [0], R at [max_grain_frames] */

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
    p->gain            = 1.0f;          /* multiplier on global_gain */
    p->pitch_offset    = 0.0f;
    p->mode_override   = 1;             /* 1 = gate (default) */
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

    int16_t *buf = (int16_t *)malloc((size_t)frames * 2 * sizeof(int16_t));
    if (!buf) { fclose(f); return 0; }

    fseek(f, data_offset, SEEK_SET);

    if (bits_per_sample == 16) {
        if (channels == 2) {
            fread(buf, sizeof(int16_t), (size_t)frames * 2, f);
        } else {
            int16_t *mono = (int16_t *)malloc((size_t)frames * sizeof(int16_t));
            if (!mono) { free(buf); fclose(f); return 0; }
            fread(mono, sizeof(int16_t), (size_t)frames, f);
            for (int32_t i = 0; i < frames; i++) buf[i*2] = buf[i*2+1] = mono[i];
            free(mono);
        }
    } else {
        uint32_t raw_size = (uint32_t)frames * channels * 3;
        uint8_t *raw = (uint8_t *)malloc(raw_size);
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

/* ── AIFF/AIF loader (8/16/24-bit PCM, big-endian) ───────────────────────── */
static inline uint16_t read_be16(const uint8_t *p) { return (uint16_t)((p[0]<<8)|p[1]); }
static inline uint32_t read_be32(const uint8_t *p) { return (uint32_t)((p[0]<<24)|(p[1]<<16)|(p[2]<<8)|p[3]); }

/* Convert 80-bit IEEE 754 extended to double (enough for sample rates) */
static double extended_to_double(const uint8_t *p) {
    int sign = (p[0] >> 7) & 1;
    int exp  = ((p[0] & 0x7F) << 8) | p[1];
    uint64_t mantissa = 0;
    for (int i = 0; i < 8; i++) mantissa = (mantissa << 8) | p[2+i];
    if (exp == 0 && mantissa == 0) return 0.0;
    double val = (double)mantissa / (1ULL << 63) * pow(2.0, exp - 16383);
    return sign ? -val : val;
}

static int load_aiff_buf(const char *path, int16_t **buf_out, int32_t *frames_out) {
    FILE *f = fopen(path, "rb");
    if (!f) return 0;

    uint8_t hdr[12];
    if (fread(hdr, 1, 12, f) < 12) { fclose(f); return 0; }
    if (memcmp(hdr, "FORM", 4) != 0) { fclose(f); return 0; }
    /* Accept both AIFF and AIFC (uncompressed) */
    if (memcmp(hdr+8, "AIFF", 4) != 0 && memcmp(hdr+8, "AIFC", 4) != 0) { fclose(f); return 0; }

    uint16_t channels = 0, bits_per_sample = 0;
    uint32_t num_frames = 0;
    double   sample_rate = 0.0;
    long     ssnd_offset = 0;
    uint32_t ssnd_size = 0;
    int      found_comm = 0, found_ssnd = 0;

    uint8_t chunk_hdr[8];
    while (fread(chunk_hdr, 1, 8, f) == 8) {
        uint32_t chunk_size = read_be32(chunk_hdr + 4);
        long chunk_start = ftell(f);

        if (memcmp(chunk_hdr, "COMM", 4) == 0) {
            uint8_t comm[26];
            uint32_t rd = chunk_size < 26 ? chunk_size : 26;
            if (fread(comm, 1, rd, f) < rd) { fclose(f); return 0; }
            channels        = read_be16(comm);
            num_frames      = read_be32(comm + 2);
            bits_per_sample = read_be16(comm + 6);
            sample_rate     = extended_to_double(comm + 8);
            (void)sample_rate; /* we resample to 44100 in the host */
            found_comm = 1;
        } else if (memcmp(chunk_hdr, "SSND", 4) == 0) {
            /* SSND has 8 bytes of offset+blockSize before PCM data */
            uint8_t ssnd_hdr[8];
            if (fread(ssnd_hdr, 1, 8, f) < 8) { fclose(f); return 0; }
            ssnd_offset = ftell(f);
            ssnd_size   = chunk_size - 8;
            found_ssnd  = 1;
        }

        /* skip to next chunk (pad to even) */
        fseek(f, chunk_start + (long)(chunk_size + (chunk_size & 1)), SEEK_SET);
    }

    if (!found_comm || !found_ssnd || channels == 0 || num_frames == 0) { fclose(f); return 0; }
    if (bits_per_sample != 8 && bits_per_sample != 16 && bits_per_sample != 24) { fclose(f); return 0; }

    int32_t frames = (int32_t)num_frames;
    int16_t *buf = (int16_t *)malloc((size_t)frames * 2 * sizeof(int16_t));
    if (!buf) { fclose(f); return 0; }

    fseek(f, ssnd_offset, SEEK_SET);
    uint32_t bytes_per_smp = bits_per_sample / 8;

    if (bits_per_sample == 16) {
        uint32_t raw_size = (uint32_t)frames * channels * 2;
        uint8_t *raw = (uint8_t *)malloc(raw_size);
        if (!raw) { free(buf); fclose(f); return 0; }
        fread(raw, 1, raw_size, f);
        for (int32_t i = 0; i < frames; i++) {
            for (int c = 0; c < 2; c++) {
                int src_c = (c < (int)channels) ? c : 0;
                int off = (int)(i * channels * 2 + src_c * 2);
                /* big-endian 16-bit signed */
                int16_t v = (int16_t)((raw[off] << 8) | raw[off+1]);
                buf[i*2+c] = v;
            }
        }
        free(raw);
    } else if (bits_per_sample == 24) {
        uint32_t raw_size = (uint32_t)frames * channels * 3;
        uint8_t *raw = (uint8_t *)malloc(raw_size);
        if (!raw) { free(buf); fclose(f); return 0; }
        fread(raw, 1, raw_size, f);
        for (int32_t i = 0; i < frames; i++) {
            for (int c = 0; c < 2; c++) {
                int src_c = (c < (int)channels) ? c : 0;
                int off = (int)(i * channels * 3 + src_c * 3);
                /* big-endian 24-bit signed → 16-bit */
                int32_t v = ((int32_t)(int8_t)raw[off] << 16)
                          | ((int32_t)raw[off+1] << 8)
                          | ((int32_t)raw[off+2]);
                buf[i*2+c] = (int16_t)(v >> 8);
            }
        }
        free(raw);
    } else { /* 8-bit */
        uint32_t raw_size = (uint32_t)frames * channels;
        uint8_t *raw = (uint8_t *)malloc(raw_size);
        if (!raw) { free(buf); fclose(f); return 0; }
        fread(raw, 1, raw_size, f);
        for (int32_t i = 0; i < frames; i++) {
            for (int c = 0; c < 2; c++) {
                int src_c = (c < (int)channels) ? c : 0;
                /* 8-bit AIFF is signed, scale to 16-bit */
                int16_t v = (int16_t)((int8_t)raw[i * channels + src_c]) << 8;
                buf[i*2+c] = v;
            }
        }
        free(raw);
    }

    fclose(f);
    *buf_out    = buf;
    *frames_out = frames;
    return 1;
}

/* ── MP3 loader (via minimp3) ────────────────────────────────────────────── */
static int load_mp3_buf(const char *path, int16_t **buf_out, int32_t *frames_out) {
    FILE *f = fopen(path, "rb");
    if (!f) return 0;
    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (file_size <= 0 || file_size > 200 * 1024 * 1024) { fclose(f); return 0; } /* 200MB cap */

    uint8_t *file_data = (uint8_t *)malloc((size_t)file_size);
    if (!file_data) { fclose(f); return 0; }
    fread(file_data, 1, (size_t)file_size, f);
    fclose(f);

    mp3dec_t mp3d;
    mp3dec_init(&mp3d);

    /* First pass: count total frames to size the output buffer */
    int32_t total_frames = 0;
    size_t offset = 0;
    mp3dec_frame_info_t info;
    int16_t pcm_tmp[MINIMP3_MAX_SAMPLES_PER_FRAME];

    while (offset < (size_t)file_size) {
        int samples = mp3dec_decode_frame(&mp3d, file_data + offset,
                                          (int)(file_size - (long)offset), pcm_tmp, &info);
        if (info.frame_bytes == 0) break;
        offset += (size_t)info.frame_bytes;
        total_frames += samples;
    }

    if (total_frames <= 0) { free(file_data); return 0; }

    int16_t *buf = (int16_t *)malloc((size_t)total_frames * 2 * sizeof(int16_t));
    if (!buf) { free(file_data); return 0; }

    /* Second pass: decode into output buffer */
    mp3dec_init(&mp3d);
    offset = 0;
    int32_t out_pos = 0;
    int channels = 0;

    while (offset < (size_t)file_size && out_pos < total_frames) {
        int samples = mp3dec_decode_frame(&mp3d, file_data + offset,
                                          (int)(file_size - (long)offset), pcm_tmp, &info);
        if (info.frame_bytes == 0) break;
        offset += (size_t)info.frame_bytes;
        channels = info.channels;

        for (int i = 0; i < samples && out_pos < total_frames; i++, out_pos++) {
            if (channels == 2) {
                buf[out_pos*2]   = pcm_tmp[i*2];
                buf[out_pos*2+1] = pcm_tmp[i*2+1];
            } else {
                buf[out_pos*2] = buf[out_pos*2+1] = pcm_tmp[i];
            }
        }
    }

    free(file_data);
    *buf_out    = buf;
    *frames_out = total_frames;
    return 1;
}

/* ── FLAC loader (via dr_flac) ───────────────────────────────────────────── */
static int load_flac_buf(const char *path, int16_t **buf_out, int32_t *frames_out) {
    FILE *f = fopen(path, "rb");
    if (!f) return 0;
    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (file_size <= 0 || file_size > 200 * 1024 * 1024) { fclose(f); return 0; }

    uint8_t *file_data = (uint8_t *)malloc((size_t)file_size);
    if (!file_data) { fclose(f); return 0; }
    fread(file_data, 1, (size_t)file_size, f);
    fclose(f);

    unsigned int channels, sample_rate;
    drflac_uint64 total_pcm_frames;
    drflac_int16 *pcm = drflac_open_memory_and_read_pcm_frames_s16(
        file_data, (size_t)file_size, &channels, &sample_rate, &total_pcm_frames, NULL);
    free(file_data);

    if (!pcm || total_pcm_frames == 0) { if (pcm) drflac_free(pcm, NULL); return 0; }

    int32_t frames = (int32_t)total_pcm_frames;
    int16_t *buf = (int16_t *)malloc((size_t)frames * 2 * sizeof(int16_t));
    if (!buf) { drflac_free(pcm, NULL); return 0; }

    if (channels >= 2) {
        for (int32_t i = 0; i < frames; i++) {
            buf[i*2]   = pcm[i * channels];
            buf[i*2+1] = pcm[i * channels + 1];
        }
    } else {
        for (int32_t i = 0; i < frames; i++) {
            buf[i*2] = buf[i*2+1] = pcm[i];
        }
    }

    drflac_free(pcm, NULL);
    *buf_out    = buf;
    *frames_out = frames;
    return 1;
}

/* ── Unified audio loader — dispatches by file extension ─────────────────── */
static const char* get_extension(const char *path) {
    const char *dot = strrchr(path, '.');
    return dot ? dot : "";
}

static int strcasecmp_ext(const char *a, const char *b) {
    while (*a && *b) {
        char ca = (*a >= 'A' && *a <= 'Z') ? *a + 32 : *a;
        char cb = (*b >= 'A' && *b <= 'Z') ? *b + 32 : *b;
        if (ca != cb) return ca - cb;
        a++; b++;
    }
    return (unsigned char)*a - (unsigned char)*b;
}

static int load_audio_buf(const char *path, int16_t **buf_out, int32_t *frames_out) {
    const char *ext = get_extension(path);
    if (strcasecmp_ext(ext, ".wav") == 0)
        return load_wav_buf(path, buf_out, frames_out);
    if (strcasecmp_ext(ext, ".aif") == 0 || strcasecmp_ext(ext, ".aiff") == 0)
        return load_aiff_buf(path, buf_out, frames_out);
    if (strcasecmp_ext(ext, ".mp3") == 0)
        return load_mp3_buf(path, buf_out, frames_out);
    if (strcasecmp_ext(ext, ".flac") == 0)
        return load_flac_buf(path, buf_out, frames_out);
    return 0; /* unsupported format */
}

static int load_sample(slicer_t *s, const char *path) {
    int16_t *buf; int32_t frames;
    if (!load_audio_buf(path, &buf, &frames)) return 0;
    if (s->sample_data) free(s->sample_data);
    s->sample_data   = buf;
    s->sample_frames = frames;
    strncpy(s->sample_path, path, sizeof(s->sample_path)-1);
    return 1;
}

/* ── Host tempo discovery ────────────────────────────────────────────────── */

/* Scan an open text file for `"tempo": <number>`. Song.abl is JSON but we only
   need one scalar, so a line scan beats pulling in a parser. */
static float scan_tempo_field(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) return 0.0f;

    float tempo = 0.0f;
    char line[512];
    while (fgets(line, sizeof(line), f)) {
        char *p = strstr(line, "\"tempo\":");
        if (!p) continue;
        p += 8;
        while (*p == ' ') p++;
        float t = strtof(p, NULL);
        if (t >= 20.0f && t <= 999.0f) { tempo = t; break; }
    }
    fclose(f);
    return tempo;
}

/* Tempo of the Set currently open on the Move, via the shim's active_set.txt.
   Returns 0 if the set can't be resolved. */
static float read_set_tempo(void) {
    FILE *f = fopen(ACTIVE_SET_PATH, "r");
    if (!f) return 0.0f;

    char uuid[128] = "", name[192] = "";
    int have_uuid = fgets(uuid, sizeof(uuid), f) != NULL;
    int have_name = fgets(name, sizeof(name), f) != NULL;
    fclose(f);
    if (!have_uuid || !have_name) return 0.0f;

    /* strip trailing newline/whitespace from both lines */
    for (char *e = uuid + strlen(uuid); e > uuid && (e[-1] == '\n' || e[-1] == '\r' || e[-1] == ' '); ) *--e = '\0';
    for (char *e = name + strlen(name); e > name && (e[-1] == '\n' || e[-1] == '\r' || e[-1] == ' '); ) *--e = '\0';
    if (!uuid[0] || !name[0]) return 0.0f;

    char path[512];
    snprintf(path, sizeof(path), "%s/%s/%s/Song.abl", SETS_DIR, uuid, name);
    return scan_tempo_field(path);
}

/* move-anything's own tempo setting: `tempo_bpm=<int>` in settings.txt */
static float read_settings_tempo(void) {
    FILE *f = fopen(MA_SETTINGS_PATH, "r");
    if (!f) return 0.0f;

    float bpm = 0.0f;
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        if (strncmp(line, "tempo_bpm=", 10) != 0) continue;
        float t = strtof(line + 10, NULL);
        if (t >= 20.0f && t <= 300.0f) bpm = t;
        break;
    }
    fclose(f);
    return bpm;
}

/* Resolve the tempo we sync to. Live MIDI clock wins when it's ticking;
   otherwise fall back to files. Does file IO — never call from render. */
static void refresh_host_bpm(slicer_t *s) {
    if (s->clk_bpm >= BPM_MIN &&
        (s->frame_clock - s->clk_last_tick_frame) < CLOCK_STALE_FRAMES) {
        s->host_bpm  = s->clk_bpm;
        s->tempo_src = TEMPO_SRC_CLOCK;
        return;
    }

    float t = read_set_tempo();
    if (t >= 20.0f) {
        s->host_bpm  = t;
        s->tempo_src = TEMPO_SRC_SET;
        return;
    }

    t = read_settings_tempo();
    if (t >= 20.0f) {
        s->host_bpm  = t;
        s->tempo_src = TEMPO_SRC_CFG;
        return;
    }

    s->host_bpm  = DEFAULT_BPM;
    s->tempo_src = TEMPO_SRC_DEFAULT;
}

/* ── Sample BPM detection ────────────────────────────────────────────────── */

/* How well does a metronome at `bpm` explain the onsets? Lays a pulse train
   over the onset envelope at the best-fitting phase and rewards grids whose
   every pulse lands on energy. Grids at 2/3 or double the real tempo score
   badly here — half their pulses fall in the gaps — which is exactly the
   ambiguity plain autocorrelation cannot resolve. */
/* How much the score favours explaining every onset over hitting only strong
   ones. Above 1 this outweighs a half-time grid's freedom to cherry-pick the
   loudest subset of onsets; too high and double-time grids win instead.
   Calibrated against a fixture set of known-tempo loops. */
#ifndef BPM_COVERAGE_EXP
#define BPM_COVERAGE_EXP 1.5f
#endif

/* Width of the tempo prior, in octaves. */
#ifndef BPM_PRIOR_SIGMA
#define BPM_PRIOR_SIGMA 0.8f
#endif

static float score_tempo(const float *flux, int n, float fps, float bpm,
                         float total_flux) {
    float period = 60.0f * fps / bpm;
    if (period < 2.0f || period > (float)n) return 0.0f;
    if ((float)n / period < 3.0f) return 0.0f;   /* too few pulses to judge */
    if (total_flux <= 0.0f) return 0.0f;

    int phase_steps = (int)period;
    if (phase_steps > 32) phase_steps = 32;
    if (phase_steps < 1)  phase_steps = 1;

    float best = 0.0f;
    for (int ps = 0; ps < phase_steps; ps++) {
        float phase = (float)ps * period / (float)phase_steps;
        float sum = 0.0f;
        int   cnt = 0, empty = 0;
        for (int i = 0; ; i++) {
            int idx = (int)(phase + (float)i * period + 0.5f);
            if (idx >= n) break;
            /* ±1 envelope frame (~6ms) of slack for imprecise onsets */
            float v = flux[idx];
            if (idx > 0     && flux[idx - 1] > v) v = flux[idx - 1];
            if (idx + 1 < n && flux[idx + 1] > v) v = flux[idx + 1];
            sum += v;
            cnt++;
            if (v <= 1e-6f) empty++;
        }
        if (cnt < 3) continue;

        float strength = sum / (float)cnt;               /* per-pulse energy */
        float coverage = sum / total_flux;               /* share of all onsets */
        if (coverage > 1.0f) coverage = 1.0f;
        float fill     = 1.0f - (float)empty / (float)cnt;

        float sc = powf(coverage, BPM_COVERAGE_EXP) * strength * fill;
        if (sc > best) best = sc;
    }

    /* Log-normal prior centred on 120 BPM. This is what settles a true octave
       tie: when a pattern's onsets fit the beat grid and the 8th grid equally
       well, the more usual tempo wins. */
    float lg = log2f(bpm / 120.0f) / BPM_PRIOR_SIGMA;
    return best * expf(-0.5f * lg * lg);
}

/* Build the onset-strength envelope: positive change in log energy per hop,
   whitened against a ~1s moving average so loud passages don't dominate.
   Returns the frame count and hands ownership of *out to the caller. */
static int build_onset_envelope(slicer_t *s, float **out) {
    *out = NULL;
    if (!s->sample_data || s->sample_frames < SAMPLE_RATE) return 0;

    int n_env = (int)((s->sample_frames - BPM_WIN) / BPM_HOP);
    if (n_env < 32) return 0;

    float *env = (float *)calloc((size_t)n_env, sizeof(float));
    if (!env) return 0;

    /* Rise in RMS amplitude, not in log energy: a log ratio makes a quiet hat
       after silence look bigger than a loud kick landing on top of a decaying
       one, which flips the weighting between offbeats and downbeats. */
    float prev_rms = 0.0f;
    for (int i = 0; i < n_env; i++) {
        int32_t base = (int32_t)i * BPM_HOP;
        float e = 0.0f;
        for (int j = 0; j < BPM_WIN; j++) {
            int32_t idx = (base + j) * 2;
            float l = s->sample_data[idx]     / 32768.0f;
            float r = s->sample_data[idx + 1] / 32768.0f;
            e += l * l + r * r;
        }
        float rms = sqrtf(e / (BPM_WIN * 2));
        env[i] = (i > 0 && rms > prev_rms) ? (rms - prev_rms) : 0.0f;
        prev_rms = rms;
    }

    int mean_win = (int)(SAMPLE_RATE / BPM_HOP);
    if (mean_win < 4) mean_win = 4;
    float *flux = (float *)calloc((size_t)n_env, sizeof(float));
    if (!flux) { free(env); return 0; }
    for (int i = 0; i < n_env; i++) {
        int lo = i - mean_win / 2; if (lo < 0) lo = 0;
        int hi = i + mean_win / 2; if (hi > n_env) hi = n_env;
        float sum = 0.0f;
        for (int k = lo; k < hi; k++) sum += env[k];
        float local_mean = sum / (float)(hi - lo);
        flux[i] = env[i] > local_mean ? env[i] - local_mean : 0.0f;
    }
    free(env);

    *out = flux;
    return n_env;
}

/* Onset-envelope tempo detection: autocorrelation proposes, a comb filter
   disposes. Returns 0 if the sample is too short to judge. */
static float detect_sample_bpm(slicer_t *s) {
    float *flux = NULL;
    int n_env = build_onset_envelope(s, &flux);
    if (!flux || n_env < 32) { free(flux); return 0.0f; }

    const float fps = (float)SAMPLE_RATE / (float)BPM_HOP;

    float total_flux = 0.0f;
    for (int i = 0; i < n_env; i++) total_flux += flux[i];
    if (total_flux <= 0.0f) { free(flux); return 0.0f; }

    int lag_min = (int)(60.0f * fps / BPM_SEARCH_MAX);
    int lag_max = (int)(60.0f * fps / BPM_SEARCH_MIN);
    if (lag_max >= n_env / 2) lag_max = n_env / 2 - 1;
    if (lag_min < 2 || lag_max <= lag_min) { free(flux); return 0.0f; }

    double dur = (double)s->sample_frames / (double)SAMPLE_RATE;

    /* Candidate tempos.

       For anything loop-shaped — which is nearly everything a slicer gets fed —
       only tempos that divide the sample into a whole number of beats are
       musically possible. Restricting to those kills the 2/3 and 3/2 readings
       that autocorrelation cannot tell apart from the beat, because a dotted
       period almost never yields an integer beat count. Beat counts that fill
       whole bars get a small edge over odd ones.

       Long material has no meaningful loop length (the integer beat counts sit
       fractions of a BPM apart), so it falls back to autocorrelation peaks
       expanded by the usual metrical ratios. */
    float cand[256];
    float weight[256];
    int   n_cand = 0;
    const int loop_shaped = (dur <= 30.0);

    if (loop_shaped) {
        for (int k = 1; k <= 256 && n_cand < 250; k++) {
            float b = (float)(k * 60.0 / dur);
            if (b < BPM_SEARCH_MIN) continue;
            if (b > BPM_SEARCH_MAX) break;
            cand[n_cand]   = b;
            weight[n_cand] = (k % 4 == 0) ? 1.15f : (k % 2 == 0) ? 1.05f : 1.0f;
            n_cand++;
        }
    }

    if (!loop_shaped || n_cand < 2) {
        for (int lag = lag_min + 1; lag < lag_max && n_cand < 250; lag++) {
            float ac  = 0.0f, acl = 0.0f, acr = 0.0f;
            for (int i = 0; i + lag + 1 < n_env; i++) {
                ac  += flux[i] * flux[i + lag];
                acl += flux[i] * flux[i + lag - 1];
                acr += flux[i] * flux[i + lag + 1];
            }
            if (ac <= acl || ac < acr) continue;       /* local maxima only */
            float base = 60.0f * fps / (float)lag;
            /* Long material has no loop length to pin the beat count, so a peak
               may well sit on a dotted or triplet period. Offer those readings
               too and let the coverage score choose. */
            static const float ratios[] = { 1.0f/3, 0.5f, 2.0f/3, 1.0f, 1.5f, 2.0f, 3.0f };
            for (unsigned r = 0; r < sizeof(ratios)/sizeof(ratios[0]) && n_cand < 250; r++) {
                float b = base * ratios[r];
                if (b < BPM_SEARCH_MIN || b > BPM_SEARCH_MAX) continue;
                cand[n_cand]   = b;
                weight[n_cand] = 1.0f;
                n_cand++;
            }
        }
    }
    if (n_cand == 0) { free(flux); return 0.0f; }

    float best_bpm = 0.0f, best_score = 0.0f;
    for (int i = 0; i < n_cand; i++) {
        float sc = score_tempo(flux, n_env, fps, cand[i], total_flux) * weight[i];
        if (sc > best_score) { best_score = sc; best_bpm = cand[i]; }
    }
    if (best_bpm <= 0.0f) { free(flux); return 0.0f; }

    /* Fine sweep around the winner for material that isn't an exact loop. */
    if (!loop_shaped) {
        float lo = best_bpm * 0.97f, hi = best_bpm * 1.03f;
        for (float b = lo; b <= hi; b += 0.1f) {
            if (b < BPM_SEARCH_MIN || b > BPM_SEARCH_MAX) continue;
            float sc = score_tempo(flux, n_env, fps, b, total_flux);
            if (sc > best_score) { best_score = sc; best_bpm = b; }
        }
    }
    free(flux);

    float bpm = best_bpm;

    /* Bar snap: loops and one-bar breaks are almost always an exact number of
       beats long. If the estimate is close to a whole-bar reading, take the
       exact one — that's what makes the stretch seamless rather than drifting. */
    double beats = dur * bpm / 60.0;
    double bars  = beats / 4.0;
    double nbars = floor(bars + 0.5);
    if (nbars >= 1.0 && fabs(bars - nbars) / nbars < 0.04) {
        bpm = (float)(nbars * 4.0 * 60.0 / dur);
    } else {
        double nbeats = floor(beats + 0.5);
        if (nbeats >= 1.0 && fabs(beats - nbeats) / nbeats < 0.03)
            bpm = (float)(nbeats * 60.0 / dur);
    }

    if (bpm < BPM_MIN || bpm > BPM_MAX) return 0.0f;
    return bpm;
}

/* Speed ratio handed to Bungee. Derived from the two tempos — with sync off,
   or either tempo unknown, playback stays at the original rate. */
static float effective_speed(slicer_t *s) {
    if (!s->sync_enabled) return 1.0f;
    if (s->sample_bpm < BPM_MIN || s->host_bpm < BPM_MIN) return 1.0f;
    float r = s->host_bpm / s->sample_bpm;
    if (r < 0.25f) r = 0.25f;
    if (r > 4.0f)  r = 4.0f;
    return r;
}

/* Load a sample and immediately work out its tempo. Every path that loads a
   sample goes through here so sync is never left holding a stale BPM. */
static int load_sample_synced(slicer_t *s, const char *path) {
    if (!load_sample(s, path)) return 0;
    s->sample_bpm_raw = detect_sample_bpm(s);
    s->sample_bpm     = s->sample_bpm_raw;
    refresh_host_bpm(s);
    /* re-seed so a fresh sample doesn't reproduce the previous one's slices */
    s->rng_state ^= (uint32_t)s->sample_frames * 2654435761u;
    if (!s->rng_state) s->rng_state = 0x9E3779B9u;
    return 1;
}

/* ── Transient detection (ranked) ────────────────────────────────────────── */

/* Transient candidate: position + strength score */
typedef struct { int32_t pos; float score; } transient_t;

/* Detect all transients at max sensitivity, return count.
   Stores position + strength score for each. */
static int detect_all_transients(slicer_t *s, transient_t *out, int max_out) {
    int32_t total_end = s->sample_frames;
    int win = 512;
    float det_threshold = 2.0f;  /* fixed low threshold to catch everything */
    int count = 0;
    float prev_rms = 0.001f;

    for (int32_t i = 0; i < total_end - win && count < max_out; i += win/2) {
        float rms = 0.0f;
        for (int j = 0; j < win; j++) {
            int32_t idx = (i + j) * 2;
            float l = s->sample_data[idx]   / 32768.0f;
            float r = s->sample_data[idx+1] / 32768.0f;
            rms += l*l + r*r;
        }
        rms = sqrtf(rms / (win * 2));

        float ratio = (prev_rms > 0.0001f) ? (rms / prev_rms) : 0.0f;
        if (ratio > det_threshold && rms > 0.01f) {
            int32_t min_gap = SAMPLE_RATE / 32;
            if (count == 0 || (i - out[count-1].pos) > min_gap) {
                out[count].pos = i;
                out[count].score = ratio * rms;  /* rank by spike strength */
                count++;
            }
        }
        prev_rms = rms * 0.3f + prev_rms * 0.7f;
    }
    return count;
}

/* Compare transients by score (descending) for qsort */
static int cmp_score_desc(const void *a, const void *b) {
    float sa = ((const transient_t *)a)->score;
    float sb = ((const transient_t *)b)->score;
    return (sb > sa) ? 1 : (sb < sa) ? -1 : 0;
}

/* Compare transients by position (ascending) for qsort */
static int cmp_pos_asc(const void *a, const void *b) {
    int32_t pa = ((const transient_t *)a)->pos;
    int32_t pb = ((const transient_t *)b)->pos;
    return (pa > pb) ? 1 : (pa < pb) ? -1 : 0;
}

/* ── Random slicing ──────────────────────────────────────────────────────── */

static uint32_t rng_next(slicer_t *s) {
    uint32_t x = s->rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s->rng_state = x ? x : 0x9E3779B9u;
    return s->rng_state;
}

/* Scatter slice_count start points across the sample instead of following
   transients. Points snap to a 16th-note grid when the sample's BPM is known,
   so a random pick still lands in time; free-running otherwise. */
static void make_random_slices(slicer_t *s) {
    if (!s->sample_data || s->sample_frames <= 0) return;

    int n = (s->slice_count > 0) ? s->slice_count : 16;
    if (n > MAX_SLICES) n = MAX_SLICES;

    int32_t min_gap = SAMPLE_RATE / 32;
    int32_t grid    = 0;
    if (s->sample_bpm >= BPM_MIN)
        grid = (int32_t)((float)SAMPLE_RATE * 60.0f / s->sample_bpm / 4.0f);
    if (grid < 1) grid = 0;

    /* Leave room for the last point to be audible. */
    int32_t span = s->sample_frames - min_gap;
    if (span <= 0) span = s->sample_frames;

    int32_t picks[MAX_SLICES];
    int count = 0;
    for (int attempt = 0; attempt < n * 8 && count < n; attempt++) {
        int32_t pos = (int32_t)(rng_next(s) % (uint32_t)span);
        if (grid) pos = (pos / grid) * grid;
        if (pos >= s->sample_frames) continue;

        /* reject duplicates and near-duplicates */
        int clash = 0;
        for (int i = 0; i < count; i++) {
            int32_t d = picks[i] - pos;
            if (d < 0) d = -d;
            if (d < min_gap) { clash = 1; break; }
        }
        if (!clash) picks[count++] = pos;
    }
    if (count == 0) { picks[count++] = 0; }

    /* sort ascending — slice i runs from picks[i] to picks[i+1] */
    for (int i = 1; i < count; i++) {
        int32_t key = picks[i];
        int j = i - 1;
        while (j >= 0 && picks[j] > key) { picks[j + 1] = picks[j]; j--; }
        picks[j + 1] = key;
    }

    for (int i = 0; i < count; i++) s->slice_points[i] = picks[i];
    s->slice_points[count] = s->sample_frames;
    s->slice_count_actual  = count;

    for (int i = 0; i < MAX_SLICES; i++) reset_pad(&s->pads[i]);
    s->preview_slice_count = count;
}

static void detect_slices(slicer_t *s) {
    if (!s->sample_data || s->sample_frames == 0) return;

    if (s->slice_algo == SLICE_ALGO_RANDOM) { make_random_slices(s); return; }

    int32_t total_start = 0;
    int32_t total_end   = s->sample_frames;
    int32_t region      = total_end - total_start;
    if (region <= 0) return;

    /* Step 1: detect all transients at max sensitivity */
    transient_t all[MAX_SLICES];
    int total_detected = detect_all_transients(s, all, MAX_SLICES);

    /* Step 2: threshold maps to how many to keep (1.0 = all, 0.0 = just 2) */
    int keep;
    if (total_detected < 2) {
        /* fallback: no transients — divide evenly */
        int n = (s->slice_count > 0) ? s->slice_count : 16;
        int32_t step = region / n;
        s->slice_count_actual = n;
        for (int i = 0; i < n; i++) s->slice_points[i] = total_start + i * step;
        s->slice_points[n] = total_end;
        for (int i = 0; i < MAX_SLICES; i++) reset_pad(&s->pads[i]);
        s->preview_slice_count = n;
        return;
    }

    keep = 2 + (int)((s->threshold) * (total_detected - 2) + 0.5f);
    if (keep > total_detected) keep = total_detected;
    if (keep < 2) keep = 2;

    /* Step 3: sort by score (descending), take top N */
    qsort(all, total_detected, sizeof(transient_t), cmp_score_desc);

    /* Step 4: re-sort the kept ones by position (ascending) */
    qsort(all, keep, sizeof(transient_t), cmp_pos_asc);

    /* Always include sample start as first slice point */
    s->slice_points[0] = total_start;
    int nmarkers = 1;
    for (int i = 0; i < keep && nmarkers < MAX_SLICES; i++) {
        /* Skip if too close to previous marker or to the start */
        if (all[i].pos > s->slice_points[nmarkers - 1] + SAMPLE_RATE / 32) {
            s->slice_points[nmarkers++] = all[i].pos;
        }
    }
    s->slice_points[nmarkers] = total_end;
    s->slice_count_actual = nmarkers;

    /* reset all per-pad params on fresh scan */
    for (int i = 0; i < MAX_SLICES; i++) reset_pad(&s->pads[i]);
    s->preview_slice_count = s->slice_count_actual;
}

/* Count-only preview — same ranked algorithm but doesn't
   touch slice_points, pad params, or slice_count_actual.  */
static void preview_slice_count(slicer_t *s) {
    if (!s->sample_data || s->sample_frames == 0) { s->preview_slice_count = 0; return; }

    /* Random mode gives exactly what was asked for — nothing to estimate. */
    if (s->slice_algo == SLICE_ALGO_RANDOM) {
        int n = (s->slice_count > 0) ? s->slice_count : 16;
        s->preview_slice_count = n > MAX_SLICES ? MAX_SLICES : n;
        return;
    }

    transient_t all[MAX_SLICES];
    int total_detected = detect_all_transients(s, all, MAX_SLICES);

    if (total_detected < 2) {
        int n = (s->slice_count > 0) ? s->slice_count : 16;
        s->preview_slice_count = n;
        return;
    }

    int keep = 2 + (int)((s->threshold) * (total_detected - 2) + 0.5f);
    if (keep > total_detected) keep = total_detected;
    if (keep < 2) keep = 2;

    /* Account for deduplication: sort by score, take top keep, sort by pos, dedup */
    qsort(all, total_detected, sizeof(transient_t), cmp_score_desc);
    qsort(all, keep, sizeof(transient_t), cmp_pos_asc);
    int nmarkers = 1;  /* sample start */
    int32_t last_pos = 0;
    for (int i = 0; i < keep; i++) {
        if (all[i].pos > last_pos + SAMPLE_RATE / 32) {
            nmarkers++;
            last_pos = all[i].pos;
        }
    }
    s->preview_slice_count = nmarkers;
}

/* ── Voice management ────────────────────────────────────────────────────── */
static void release_stretcher(slicer_t *s, int stretcher_idx);  /* forward decl */

static voice_t* find_free_voice(slicer_t *s) {
    for (int i = 0; i < MAX_VOICES; i++) {
        if (!s->voices[i].active) return &s->voices[i];
    }
    /* Steal voice 0 — preserve its grain buffer pointer */
    release_stretcher(s, s->voices[0].stretcher_idx);
    float *saved_buf = s->voices[0].grain_out_buf;
    memset(&s->voices[0], 0, sizeof(voice_t));
    s->voices[0].grain_out_buf = saved_buf;
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

/* ── Bungee stretcher pool helpers ───────────────────────────────────────── */

static int claim_stretcher(slicer_t *s, int voice_idx) {
    for (int i = 0; i < STRETCHER_POOL_SIZE; i++) {
        if (s->stretcher_owner[i] < 0) {
            s->stretcher_owner[i] = voice_idx;
            return i;
        }
    }
    return -1;  /* pool exhausted — fall back to rate-based */
}

static void release_stretcher(slicer_t *s, int stretcher_idx) {
    if (stretcher_idx >= 0 && stretcher_idx < STRETCHER_POOL_SIZE)
        s->stretcher_owner[stretcher_idx] = -1;
}

static bool needs_stretcher(slicer_t *s) {
    return effective_speed(s) != 1.0f || s->pitch != 0.0f;
}

/* Fill Bungee grain output buffer for a voice.
   Pulls grains from the stretcher until we have enough output frames.
   bufferStartPosition=0 because sample_data covers the entire file from frame 0.
   Handles negative speed for reverse playback; wraps position for LOOP_REVERSE/LOOP_FORWARD. */
static void fill_grain_output(slicer_t *s, voice_t *v, int frames_needed) {
    if (v->stretcher_idx < 0 || !s->sample_data) return;
    auto *st = s->stretchers[v->stretcher_idx];
    int mgf = s->max_grain_frames;

    v->grain_out_pos = 0;
    v->grain_out_count = 0;

    int max_grains = 8;  /* safety: don't loop forever */
    while (v->grain_out_count < frames_needed && max_grains-- > 0) {
        bool reverse = v->bungee_req.speed < 0.0;

        /* Boundary handling. LOOP_REVERSE is one-shot backward — falls
           through to break (release). LOOP_FORWARD wraps to start.
           LOOP_PINGPONG flips speed sign at either boundary. */
        if (reverse) {
            if (v->bungee_req.position <= (double)v->slice_start) {
                if (v->loop_mode == LOOP_PINGPONG && !v->released) {
                    v->bungee_req.position = (double)v->slice_start;
                    v->bungee_req.speed = -v->bungee_req.speed;
                    v->bungee_req.reset = true;
                    st->preroll(v->bungee_req);
                    v->bungee_req.reset = false;
                } else {
                    break;
                }
            }
        } else {
            if (v->bungee_req.position >= (double)v->slice_end) {
                if (v->loop_mode == LOOP_FORWARD && !v->released) {
                    v->bungee_req.position = (double)v->slice_start;
                    v->bungee_req.reset = true;
                    st->preroll(v->bungee_req);
                    v->bungee_req.reset = false;
                } else if (v->loop_mode == LOOP_PINGPONG && !v->released) {
                    v->bungee_req.position = (double)(v->slice_end - 1);
                    v->bungee_req.speed = -v->bungee_req.speed;
                    v->bungee_req.reset = true;
                    st->preroll(v->bungee_req);
                    v->bungee_req.reset = false;
                } else {
                    break;
                }
            }
        }

        Bungee::OutputChunk output;
        auto chunk = st->specifyGrain(v->bungee_req, 0.0);

        int grain_frames = chunk.end - chunk.begin;
        if (grain_frames <= 0) break;
        if (grain_frames > mgf) grain_frames = mgf;

        /* Convert int16 interleaved → float non-interleaved for Bungee.
           Layout: L at grain_input[0..], R at grain_input[mgf..] */
        float *in_l = s->grain_input;
        float *in_r = s->grain_input + mgf;
        for (int i = 0; i < grain_frames; i++) {
            int src_idx = chunk.begin + i;
            if (src_idx >= 0 && src_idx < s->sample_frames) {
                in_l[i] = s->sample_data[src_idx * 2]     / 32768.0f;
                in_r[i] = s->sample_data[src_idx * 2 + 1] / 32768.0f;
            } else {
                in_l[i] = 0.0f;
                in_r[i] = 0.0f;
            }
        }

        /* Mute samples outside slice boundaries */
        int mute_head = (chunk.begin < v->slice_start) ? (v->slice_start - chunk.begin) : 0;
        int mute_tail = (chunk.end > v->slice_end)     ? (chunk.end - v->slice_end)     : 0;
        if (mute_head < 0) mute_head = 0;
        if (mute_tail < 0) mute_tail = 0;

        st->analyseGrain(s->grain_input, (intptr_t)mgf, mute_head, mute_tail);
        st->synthesiseGrain(output);

        /* Copy output to voice grain buffer (interleaved float) */
        if (output.data && output.frameCount > 0) {
            for (int i = 0; i < output.frameCount && v->grain_out_count < mgf; i++) {
                v->grain_out_buf[v->grain_out_count * 2]     = output.data[i];
                v->grain_out_buf[v->grain_out_count * 2 + 1] = output.data[output.channelStride + i];
                v->grain_out_count++;
            }
        }

        st->next(v->bungee_req);
    }
}

static void voice_start(slicer_t *s, int note, int velocity) {
    if (s->slice_count_actual == 0 || !s->sample_data) return;

    int slice_idx = note_to_slice(s, note);
    if (slice_idx < 0) return;  /* out of range — silent ignore */

    s->selected_slice = slice_idx;

    pad_params_t *p = &s->pads[slice_idx];

    /* apply per-pad offsets to detected boundaries, clamp to file.
       Playthrough ignores the slice's own end so a held pad keeps running
       through everything that follows it, rather than stopping at the next
       slice point. */
    int32_t base_start = s->slice_points[slice_idx];
    int32_t base_end   = s->playthrough ? s->sample_frames
                                        : s->slice_points[slice_idx + 1];
    int32_t start = clampi(base_start + (int32_t)ms_to_frames(p->start_offset_ms), 0, s->sample_frames - 1);
    int32_t end   = clampi(base_end   + (int32_t)ms_to_frames(p->end_offset_ms),   1, s->sample_frames);
    if (end <= start) end = start + 1;

    /* mono mode: kill all active voices before starting new one */
    if (s->mono_mode) {
        for (int i = 0; i < MAX_VOICES; i++) {
            if (s->voices[i].active) {
                release_stretcher(s, s->voices[i].stretcher_idx);
                s->voices[i].stretcher_idx = -1;
                s->voices[i].active = 0;
                s->voices[i].env_state = ENV_IDLE;
                s->voices[i].env_val = 0.0f;
                s->voices[i].grain_out_pos = 0;
                s->voices[i].grain_out_count = 0;
            }
        }
    }

    voice_t *v = find_voice_for_note(s, note);
    if (v) {
        release_stretcher(s, v->stretcher_idx);
        float *saved_buf = v->grain_out_buf;
        memset(v, 0, sizeof(voice_t));
        v->grain_out_buf = saved_buf;
    } else {
        v = find_free_voice(s);
    }

    int voice_idx = (int)(v - s->voices);

    v->active      = 1;
    v->note        = note;
    v->slice_idx   = slice_idx;
    v->pos         = (p->loop_mode == LOOP_REVERSE)
                     ? (int64_t)(end - 1) << 16
                     : (int64_t)start << 16;
    v->rate        = semitones_to_rate(s->pitch + p->pitch_offset);
    v->direction   = (p->loop_mode == LOOP_REVERSE) ? -1 : 1;
    v->slice_start = start;
    v->slice_end   = end;
    v->loop_mode   = p->loop_mode;
    v->mode_gate   = (p->mode_override >= 0) ? p->mode_override : s->mode_gate;
    v->env_state   = ENV_ATTACK;
    v->env_val     = 0.0f;
    v->env_attack  = ms_to_coeff(p->attack_ms, 0.001f);
    v->env_decay   = ms_to_coeff(p->decay_ms,  0.001f);
    v->velocity    = s->velocity_sens ? (velocity / 127.0f) : 1.0f;
    v->pad_gain    = s->global_gain * p->gain;
    v->release     = 0;
    v->released    = 0;

    /* Bungee: claim stretcher if time-stretch or pitch-shift is active */
    v->stretcher_idx = -1;
    v->grain_out_pos = 0;
    v->grain_out_count = 0;
    if (needs_stretcher(s)) {
        int si = claim_stretcher(s, voice_idx);
        if (si >= 0) {
            v->stretcher_idx = si;
            float pitch_mult = powf(2.0f, (s->pitch + p->pitch_offset) / 12.0f);
            float speed = effective_speed(s);
            bool reverse = (p->loop_mode == LOOP_REVERSE);
            v->bungee_req.position = reverse ? (double)(end - 1) : (double)start;
            v->bungee_req.speed = reverse ? -(double)speed : (double)speed;
            v->bungee_req.pitch = (double)pitch_mult;
            v->bungee_req.reset = true;
            v->bungee_req.resampleMode = ::resampleMode_autoOut;
            s->stretchers[si]->preroll(v->bungee_req);
            v->bungee_req.reset = false;
            /* Pre-fill the grain output buffer */
            fill_grain_output(s, v, BLOCK_SIZE);
        }
        /* If claim failed, voice falls back to rate-based playback */
    }
}

static void voice_release(voice_t *v) {
    if (v->active && v->env_state != ENV_IDLE) {
        v->released  = 1;
        v->env_state = ENV_DECAY;
    }
}

/* Deactivate a voice and return its stretcher to the pool */
static void voice_kill(slicer_t *s, voice_t *v) {
    if (v->stretcher_idx >= 0) {
        release_stretcher(s, v->stretcher_idx);
        v->stretcher_idx = -1;
    }
    v->active = 0;
    v->env_val = 0.0f;
    v->env_state = ENV_IDLE;
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
    slicer_t *s = (slicer_t *)calloc(1, sizeof(slicer_t));
    s->threshold      = 0.5f;
    s->slice_count    = 16;
    s->pitch          = 0.0f;
    s->global_gain    = 0.8f;
    s->mode_gate      = 1;
    s->velocity_sens  = 1;
    s->selected_slice = 0;
    s->slicer_state   = 0;
    s->sync_enabled   = 1;
    s->slice_algo     = SLICE_ALGO_TRANSIENT;
    s->playthrough    = 0;
    s->rng_state      = 0x9E3779B9u;
    for (int i = 0; i < MAX_SLICES; i++) reset_pad(&s->pads[i]);

    refresh_host_bpm(s);

    /* Bungee stretcher pool */
    Bungee::SampleRates sr{SAMPLE_RATE, SAMPLE_RATE};
    for (int i = 0; i < STRETCHER_POOL_SIZE; i++) {
        s->stretchers[i] = new Bungee::Stretcher<Bungee::Basic>(sr, 2, 0);
        s->stretcher_owner[i] = -1;
    }
    s->max_grain_frames = s->stretchers[0]->maxInputFrameCount();
    if (s->max_grain_frames < 1024) s->max_grain_frames = 16384; /* safety fallback */
    s->grain_input = (float *)calloc(s->max_grain_frames * 2, sizeof(float));
    for (int i = 0; i < MAX_VOICES; i++) {
        s->voices[i].grain_out_buf = (float *)calloc(s->max_grain_frames * 2, sizeof(float));
        s->voices[i].stretcher_idx = -1;
    }

    return s;
}

static void v2_destroy_instance(void *inst) {
    slicer_t *s = (slicer_t *)inst;
    if (s->sample_data)   free(s->sample_data);
    if (s->preview_data)  free(s->preview_data);
    for (int i = 0; i < STRETCHER_POOL_SIZE; i++) {
        delete s->stretchers[i];
    }
    for (int i = 0; i < MAX_VOICES; i++) {
        free(s->voices[i].grain_out_buf);
    }
    free(s->grain_input);
    free(s);
}

static void v2_set_param(void *inst, const char *key, const char *val) {
    slicer_t *s = (slicer_t *)inst;

    /* global params */
    if (strcmp(key, "threshold") == 0) {
        s->threshold    = atof(val);
        s->slicer_state = 0;
        preview_slice_count(s);
    } else if (strcmp(key, "slices") == 0) {
        int n = atoi(val);
        if (n==8||n==16||n==32||n==64) s->slice_count = n;
        /* Random slicing needs no scan step — apply the new count immediately.
           Transient mode still re-scans on the user's command. */
        if (s->slice_algo == SLICE_ALGO_RANDOM && s->sample_data) {
            make_random_slices(s);
            s->slicer_state = 1;
        } else {
            s->slicer_state = 0;
        }
    } else if (strcmp(key, "pitch") == 0) {
        s->pitch = atof(val);
    } else if (strcmp(key, "mode") == 0) {
        s->mode_gate = (strcmp(val, "gate") == 0) ? 1 : 0;
    } else if (strcmp(key, "velocity_sens") == 0) {
        s->velocity_sens = atoi(val) ? 1 : 0;
    } else if (strcmp(key, "mono_mode") == 0) {
        s->mono_mode = atoi(val) ? 1 : 0;

    /* tempo sync — speed is always derived, so there is no manual speed key */
    } else if (strcmp(key, "sync") == 0) {
        s->sync_enabled = atoi(val) ? 1 : 0;
        if (s->sync_enabled) refresh_host_bpm(s);
    } else if (strcmp(key, "bpm_ratio") == 0) {
        /* Correction against the raw estimate rather than the working value, so
           stepping through ratios is reversible and never accumulates drift.
           Covers the errors detection actually makes: half/double time, and the
           dotted 3/2 and 2/3 readings that turn up on long material. */
        float r = (float)atof(val);
        if (r > 0.0f && s->sample_bpm_raw > 0.0f) {
            float b = s->sample_bpm_raw * r;
            if (b >= BPM_MIN && b <= BPM_MAX) s->sample_bpm = b;
        }
    } else if (strcmp(key, "detect_bpm") == 0) {
        s->sample_bpm_raw = detect_sample_bpm(s);
        s->sample_bpm     = s->sample_bpm_raw;
        refresh_host_bpm(s);
    } else if (strcmp(key, "refresh_tempo") == 0) {
        refresh_host_bpm(s);
    } else if (strcmp(key, "slice_algo") == 0) {
        int a = atoi(val);
        if (a == SLICE_ALGO_TRANSIENT || a == SLICE_ALGO_RANDOM) {
            int changed = (a != s->slice_algo);
            s->slice_algo = a;
            if (changed && s->sample_data) {
                /* Random slicing is instant, so switch straight into it.
                   Playthrough follows the mode: random points are start
                   points to run on from, transients are self-contained hits. */
                if (a == SLICE_ALGO_RANDOM) {
                    s->playthrough = 1;
                    make_random_slices(s);
                    s->slicer_state = 1;
                } else {
                    s->playthrough = 0;
                    s->slicer_state = 0;
                    preview_slice_count(s);
                }
            }
        }
    } else if (strcmp(key, "reroll") == 0) {
        if (s->slice_algo == SLICE_ALGO_RANDOM && s->sample_data) {
            make_random_slices(s);
            s->slicer_state = 1;
        }
    } else if (strcmp(key, "playthrough") == 0) {
        s->playthrough = atoi(val) ? 1 : 0;
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
    } else if (strcmp(key, "slice_pitch") == 0) {
        s->pads[s->selected_slice].pitch_offset = atof(val);
    } else if (strcmp(key, "slice_mode") == 0) {
        int n = atoi(val);
        if (n >= -1 && n <= 1) s->pads[s->selected_slice].mode_override = n;
    } else if (strcmp(key, "global_gain") == 0) {
        s->global_gain = atof(val);
    } else if (strcmp(key, "global_attack") == 0) {
        float a = atof(val); if (a < 5.0f) a = 5.0f;
        for (int i = 0; i < MAX_SLICES; i++) s->pads[i].attack_ms = a;
    } else if (strcmp(key, "global_decay") == 0) {
        float d = atof(val);
        for (int i = 0; i < MAX_SLICES; i++) s->pads[i].decay_ms = d;
    } else if (strcmp(key, "slice_loop") == 0) {
        int n = atoi(val);
        if (n >= LOOP_OFF && n <= LOOP_REVERSE)
            s->pads[s->selected_slice].loop_mode = n;

    /* sample + scan */
    } else if (strcmp(key, "sample_path") == 0) {
        if (load_sample_synced(s, val)) { s->slicer_state = 0; preview_slice_count(s); }
    } else if (strcmp(key, "reload_sample") == 0) {
        /* Retry loading the current sample_path — used after failed state
           restore (e.g. filesystem not ready at boot). If slice boundaries
           were already restored (slice_count_actual > 0), go straight to READY. */
        if (s->sample_path[0] && load_sample_synced(s, s->sample_path)) {
            s->slicer_state = (s->slice_count_actual > 0) ? 1
                            : (s->sample_data ? 2 : 0);
        }
    } else if (strcmp(key, "scan") == 0) {
        detect_slices(s);
        s->slicer_state = (s->slice_count_actual > 0) ? 1 : 2;

    /* browser hover preview */
    } else if (strcmp(key, "preview_path") == 0) {
        s->preview_active = 0;
        int16_t *buf; int32_t frames;
        if (load_audio_buf(val, &buf, &frames)) {
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
        if (json_get_int(val, "mono", &ival)) s->mono_mode = ival ? 1 : 0;
        /* global_gain: if absent (old save), use 1.0 so old per-pad gains work */
        if (json_get_float(val, "gg", &fval)) s->global_gain = fval;
        else s->global_gain = 1.0f;

        /* tempo sync — default to on for patches saved before sync existed */
        if (json_get_int(val, "sync", &ival)) s->sync_enabled = ival ? 1 : 0;
        else s->sync_enabled = 1;
        if (json_get_int(val, "algo", &ival))
            s->slice_algo = (ival == SLICE_ALGO_RANDOM) ? SLICE_ALGO_RANDOM
                                                        : SLICE_ALGO_TRANSIENT;
        if (json_get_int(val, "pt", &ival)) s->playthrough = ival ? 1 : 0;
        float restored_bpm = 0.0f;
        if (json_get_float(val, "sbpm", &fval) && fval >= BPM_MIN && fval <= BPM_MAX)
            restored_bpm = fval;

        /* Load sample from saved path. Preserve s->sample_path even on
           load failure so the UI can display the missing-file name and
           retry via "reload_sample" once the filesystem is ready. */
        if (json_get_string(val, "sample_path", str, sizeof(str)) && str[0]) {
            strncpy(s->sample_path, str, sizeof(s->sample_path) - 1);
            s->sample_path[sizeof(s->sample_path) - 1] = '\0';
            load_sample(s, str);
        }

        /* Prefer the saved BPM — it may carry a ÷2/×2 the user applied by hand.
           Only re-analyse when the patch predates sync or the value was junk. */
        if (restored_bpm > 0.0f) {
            s->sample_bpm     = restored_bpm;
            s->sample_bpm_raw = restored_bpm;
        } else if (s->sample_data) {
            s->sample_bpm_raw = detect_sample_bpm(s);
            s->sample_bpm     = s->sample_bpm_raw;
        }
        refresh_host_bpm(s);

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
                    if (lm >= LOOP_OFF && lm <= LOOP_REVERSE) s->pads[i].loop_mode = lm;
                    const char *c = strchr(p, ','); if (!c) break; p = c + 1;
                }
            }
            if (json_get_string(val, "pp", csv, sizeof(csv))) {
                const char *p = csv;
                for (int i = 0; i < n && *p; i++) {
                    s->pads[i].pitch_offset = (float)atof(p);
                    const char *c = strchr(p, ','); if (!c) break; p = c + 1;
                }
            }
            if (json_get_string(val, "pm", csv, sizeof(csv))) {
                const char *p = csv;
                for (int i = 0; i < n && *p; i++) {
                    int mo = atoi(p);
                    if (mo >= -1 && mo <= 1) s->pads[i].mode_override = mo;
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
    slicer_t *s = (slicer_t *)inst;
    pad_params_t *p = &s->pads[s->selected_slice];

    /* global */
    if (strcmp(key, "threshold") == 0)         return snprintf(buf, buf_len, "%.3f", s->threshold);
    if (strcmp(key, "slices") == 0)             return snprintf(buf, buf_len, "%d",   s->slice_count);
    if (strcmp(key, "pitch") == 0)              return snprintf(buf, buf_len, "%.1f", s->pitch);
    if (strcmp(key, "mode") == 0)               return snprintf(buf, buf_len, "%s",   s->mode_gate ? "gate" : "trigger");
    if (strcmp(key, "velocity_sens") == 0)      return snprintf(buf, buf_len, "%d",   s->velocity_sens);
    if (strcmp(key, "mono_mode") == 0)          return snprintf(buf, buf_len, "%d",   s->mono_mode);
    /* speed is read-only now — it reports the ratio sync arrived at */
    if (strcmp(key, "speed") == 0)              return snprintf(buf, buf_len, "%.3f", effective_speed(s));
    if (strcmp(key, "sync") == 0)               return snprintf(buf, buf_len, "%d",   s->sync_enabled);
    if (strcmp(key, "sample_bpm") == 0)         return snprintf(buf, buf_len, "%.1f", s->sample_bpm);
    if (strcmp(key, "host_bpm") == 0)           return snprintf(buf, buf_len, "%.1f", s->host_bpm);
    if (strcmp(key, "tempo_src") == 0) {
        const char *tag = s->tempo_src == TEMPO_SRC_CLOCK ? "CLK"
                        : s->tempo_src == TEMPO_SRC_SET   ? "SET"
                        : s->tempo_src == TEMPO_SRC_CFG   ? "CFG" : "DEF";
        return snprintf(buf, buf_len, "%s", tag);
    }
    if (strcmp(key, "slice_algo") == 0)         return snprintf(buf, buf_len, "%d",   s->slice_algo);
    if (strcmp(key, "playthrough") == 0)        return snprintf(buf, buf_len, "%d",   s->playthrough);
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
    if (strcmp(key, "slice_pitch") == 0)        return snprintf(buf, buf_len, "%.1f", p->pitch_offset);
    if (strcmp(key, "slice_mode") == 0)         return snprintf(buf, buf_len, "%d",   p->mode_override);
    if (strcmp(key, "global_gain") == 0)        return snprintf(buf, buf_len, "%.3f", s->global_gain);
    if (strcmp(key, "slice_loop") == 0)         return snprintf(buf, buf_len, "%d",   p->loop_mode);

    /* Shadow UI param metadata — expose all key params for knob editing */
    if (strcmp(key, "chain_params") == 0) {
        const char *json =
            "["
            "{\"key\":\"threshold\",\"name\":\"Sensitivity\","
             "\"type\":\"float\",\"min\":0.0,\"max\":1.0,\"default\":0.5},"
            "{\"key\":\"pitch\",\"name\":\"Pitch\","
             "\"type\":\"float\",\"min\":-24.0,\"max\":24.0,\"default\":0.0},"
            "{\"key\":\"velocity_sens\",\"name\":\"Velocity\","
             "\"type\":\"enum\",\"options\":[\"Off\",\"On\"],\"default\":\"On\"},"
            "{\"key\":\"mode\",\"name\":\"Mode\","
             "\"type\":\"enum\",\"options\":[\"trigger\",\"gate\"],\"default\":\"gate\"},"
            "{\"key\":\"global_gain\",\"name\":\"Gain\","
             "\"type\":\"float\",\"min\":0.0,\"max\":1.0,\"default\":0.8},"
            "{\"key\":\"mono_mode\",\"name\":\"Mono\","
             "\"type\":\"enum\",\"options\":[\"Off\",\"On\"],\"default\":\"Off\"},"
            "{\"key\":\"sync\",\"name\":\"Tempo Sync\","
             "\"type\":\"enum\",\"options\":[\"0\",\"1\"],\"labels\":[\"Off\",\"On\"],\"default\":\"1\"},"
            "{\"key\":\"slice_algo\",\"name\":\"Slice Mode\","
             "\"type\":\"enum\",\"options\":[\"0\",\"1\"],\"labels\":[\"Transient\",\"Random\"],\"default\":\"0\"},"
            "{\"key\":\"playthrough\",\"name\":\"Playthrough\","
             "\"type\":\"enum\",\"options\":[\"0\",\"1\"],\"labels\":[\"Off\",\"On\"],\"default\":\"0\"},"
            "{\"key\":\"global_attack\",\"name\":\"Global Attack\","
             "\"type\":\"float\",\"min\":5.0,\"max\":2000.0,\"default\":5.0,\"unit\":\"ms\"},"
            "{\"key\":\"global_decay\",\"name\":\"Global Decay\","
             "\"type\":\"float\",\"min\":0.0,\"max\":5000.0,\"default\":500.0,\"unit\":\"ms\"},"
            "{\"key\":\"slice_attack\",\"name\":\"Attack\","
             "\"type\":\"float\",\"min\":5.0,\"max\":2000.0,\"default\":5.0,\"unit\":\"ms\"},"
            "{\"key\":\"slice_decay\",\"name\":\"Decay\","
             "\"type\":\"float\",\"min\":0.0,\"max\":5000.0,\"default\":500.0,\"unit\":\"ms\"},"
            "{\"key\":\"slice_start_trim\",\"name\":\"Start Trim\","
             "\"type\":\"float\",\"min\":-1000.0,\"max\":1000.0,\"default\":0.0,\"unit\":\"ms\"},"
            "{\"key\":\"slice_end_trim\",\"name\":\"End Trim\","
             "\"type\":\"float\",\"min\":-1000.0,\"max\":1000.0,\"default\":0.0,\"unit\":\"ms\"},"
            "{\"key\":\"slice_pitch\",\"name\":\"Pad Pitch\","
             "\"type\":\"float\",\"min\":-24.0,\"max\":24.0,\"default\":0.0},"
            "{\"key\":\"slice_gain\",\"name\":\"Pad Gain\","
             "\"type\":\"float\",\"min\":0.0,\"max\":2.0,\"default\":1.0},"
            "{\"key\":\"slice_mode\",\"name\":\"Pad Mode\","
             "\"type\":\"enum\",\"options\":[\"0\",\"1\"],\"labels\":[\"Trig\",\"Gate\"],\"default\":\"1\"},"
            "{\"key\":\"slice_loop\",\"name\":\"Loop\","
             "\"type\":\"enum\",\"options\":[\"0\",\"1\",\"2\",\"3\"],\"labels\":[\"Off\",\"Loop\",\"Ping-Pong\",\"Reverse\"],\"default\":\"0\"}"
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
            "\"mode\":\"%s\",\"vel_sens\":%d,\"gg\":%.3f,"
            "\"mono\":%d,\"sync\":%d,\"sbpm\":%.1f,\"algo\":%d,\"pt\":%d,"
            "\"sample_path\":\"%s\",\"sca\":%d",
            s->threshold, s->slice_count, s->pitch,
            s->mode_gate ? "gate" : "trigger",
            s->velocity_sens, s->global_gain,
            s->mono_mode, s->sync_enabled, s->sample_bpm,
            s->slice_algo, s->playthrough,
            s->sample_path, n);

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

            /* per-pad pitch offset */
            pos += snprintf(tmp + pos, sizeof(tmp) - pos, ",\"pp\":\"");
            for (int i = 0; i < n && pos < (int)sizeof(tmp) - 16; i++)
                pos += snprintf(tmp + pos, sizeof(tmp) - pos, "%s%.1f",
                                i ? "," : "", s->pads[i].pitch_offset);
            pos += snprintf(tmp + pos, sizeof(tmp) - pos, "\"");

            /* per-pad mode override */
            pos += snprintf(tmp + pos, sizeof(tmp) - pos, ",\"pm\":\"");
            for (int i = 0; i < n && pos < (int)sizeof(tmp) - 16; i++)
                pos += snprintf(tmp + pos, sizeof(tmp) - pos, "%s%d",
                                i ? "," : "", s->pads[i].mode_override);
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
    (void)source;
    slicer_t *s = (slicer_t *)inst;
    if (len < 1) return;

    /* System realtime is a single byte — reading msg[1] here would overrun.
       Clock ticks give us the session tempo when we're slaved to an external
       clock; the Move's own clock is consumed by the shim and never reaches
       modules, so refresh_host_bpm() falls back to Song.abl in that case. */
    if (msg[0] >= 0xF8) {
        if (msg[0] == 0xF8) {
            s->clk_last_tick_frame = s->frame_clock;
            if (++s->clk_ticks >= 24) {
                int64_t span = s->frame_clock - s->clk_last_beat_frame;
                if (s->clk_last_beat_frame > 0 && span > 0) {
                    float bpm = 60.0f * (float)SAMPLE_RATE / (float)span;
                    if (bpm >= BPM_MIN && bpm <= BPM_MAX) s->clk_bpm = bpm;
                }
                s->clk_last_beat_frame = s->frame_clock;
                s->clk_ticks = 0;
            }
        } else if (msg[0] == 0xFA || msg[0] == 0xFC) {
            /* start / stop — drop the partial beat rather than mismeasure it */
            s->clk_ticks = 0;
            s->clk_last_beat_frame = s->frame_clock;
        }
        return;
    }

    if (len < 2) return;
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
        if (v && (v->mode_gate || v->loop_mode != LOOP_OFF)) {
            voice_release(v);
        }
    }
}

static void v2_render_block(void *inst, int16_t *out_lr, int frames) {
    slicer_t *s = (slicer_t *)inst;
    /* Frame counter drives MIDI clock timing — advance it even when silent. */
    s->frame_clock += frames;
    if (!s->sample_data && !s->preview_active) { memset(out_lr, 0, frames * 2 * sizeof(int16_t)); return; }

    float mix_l[BLOCK_SIZE];
    float mix_r[BLOCK_SIZE];
    memset(mix_l, 0, frames * sizeof(float));
    memset(mix_r, 0, frames * sizeof(float));

    for (int vi = 0; vi < MAX_VOICES; vi++) {
        voice_t *v = &s->voices[vi];
        if (!v->active) continue;

        /* === Bungee time-stretch path === */
        if (v->stretcher_idx >= 0) {
            /* Refill grain buffer if needed */
            if (v->grain_out_pos >= v->grain_out_count) {
                fill_grain_output(s, v, frames);
                if (v->grain_out_count == 0) {
                    /* End of slice — trigger release */
                    if (v->release == 0) v->release = RELEASE_SAMPLES;
                }
            }

            for (int i = 0; i < frames; i++) {
                /* envelope */
                if (v->env_state == ENV_IDLE) { voice_kill(s, v); break; }
                float env = v->env_val;
                switch (v->env_state) {
                    case ENV_ATTACK:
                        env = env + (1.0f - env) * (1.0f - v->env_attack);
                        if (env >= 0.999f) {
                            env = 1.0f;
                            if (v->loop_mode != LOOP_OFF || v->mode_gate)
                                v->env_state = ENV_SUSTAIN;
                            else
                                v->env_state = ENV_DECAY;
                        }
                        break;
                    case ENV_SUSTAIN: env = 1.0f; break;
                    case ENV_DECAY:
                        env *= v->env_decay;
                        if (env < 0.0001f) { env = 0.0f; voice_kill(s, v); }
                        break;
                    case ENV_IDLE: voice_kill(s, v); break;
                }
                v->env_val = env;
                if (!v->active) break;

                float l = 0.0f, r = 0.0f;
                if (v->grain_out_pos < v->grain_out_count) {
                    l = v->grain_out_buf[v->grain_out_pos * 2]     * 32768.0f;
                    r = v->grain_out_buf[v->grain_out_pos * 2 + 1] * 32768.0f;
                    v->grain_out_pos++;
                }

                float amp = v->velocity * v->pad_gain * env;
                if (v->release > 0) {
                    amp *= (float)v->release / (float)RELEASE_SAMPLES;
                    if (--v->release == 0) { voice_kill(s, v); }
                }

                mix_l[i] += l * amp;
                mix_r[i] += r * amp;
                if (!v->active) break;
            }
            continue;  /* skip rate-based path */
        }

        /* === Original rate-based path (no stretching) === */
        for (int i = 0; i < frames; i++) {
            /* envelope */
            if (v->env_state == ENV_IDLE) { voice_kill(s, v); break; }
            float env = v->env_val;
            switch (v->env_state) {
                case ENV_ATTACK:
                    env = env + (1.0f - env) * (1.0f - v->env_attack);
                    if (env >= 0.999f) {
                        env = 1.0f;
                        if (v->loop_mode != LOOP_OFF || v->mode_gate)
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
                    if (env < 0.0001f) { env = 0.0f; voice_kill(s, v); }
                    break;
                case ENV_IDLE:
                    voice_kill(s, v);
                    break;
            }
            v->env_val = env;
            if (!v->active) break;

            /* position */
            int32_t pos_int = (int32_t)(v->pos >> 16);

            /* loop / ping-pong boundary handling. LOOP_REVERSE is one-shot
               backward — falls through to the release branch on hitting
               slice_start, matching LOOP_OFF's behavior at slice_end. */
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
                if (pos_int < v->slice_start) {
                    if (v->release == 0) v->release = RELEASE_SAMPLES;
                    pos_int = v->slice_start;
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
                voice_kill(s, v); break;
            }

            float l = s->sample_data[pos_int*2]   * (1.0f - frac)
                    + s->sample_data[pos_next*2]   * frac;
            float r = s->sample_data[pos_int*2+1] * (1.0f - frac)
                    + s->sample_data[pos_next*2+1] * frac;

            float amp = v->velocity * v->pad_gain * env;
            if (v->release > 0) {
                amp *= (float)v->release / (float)RELEASE_SAMPLES;
                if (--v->release == 0) { voice_kill(s, v); }
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

extern "C" plugin_api_v2_t* move_plugin_init_v2(const host_api_v1_t *host) {
    (void)host;
    return &g_api;
}
