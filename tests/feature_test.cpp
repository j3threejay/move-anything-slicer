/* Behavioural tests for the sync + random-slice features, driving the real
   DSP through its parameter API the way the UI does. */
#include "../src/dsp/dsp.cpp"

static int failures = 0;

static void check(const char *what, bool ok, const char *detail = "") {
    printf("  [%s] %s %s\n", ok ? "PASS" : "FAIL", what, detail);
    if (!ok) failures++;
}

static void check_near(const char *what, float got, float want, float tol) {
    char d[128];
    snprintf(d, sizeof(d), "(got %.4f, want %.4f)", got, want);
    check(what, fabsf(got - want) <= tol, d);
}

static const char *SAMPLE = "beat_120.wav";

int main(void) {
    char buf[8192];

    printf("\n== tempo sync ==\n");
    {
        slicer_t *s = (slicer_t *)v2_create_instance("/tmp", NULL);
        v2_set_param(s, "sample_path", SAMPLE);
        check("sample loaded", s->sample_data != NULL);
        check_near("detected 120 BPM", s->sample_bpm, 120.0f, 1.0f);

        /* Sync ratio must be host/sample, and 1.0 when sync is off. */
        s->host_bpm = 90.0f;
        check_near("speed = host/sample", effective_speed(s), 0.75f, 0.001f);
        v2_set_param(s, "sync", "0");
        check_near("sync off -> no stretch", effective_speed(s), 1.0f, 0.0001f);
        v2_set_param(s, "sync", "1");
        s->host_bpm = 90.0f;

        /* A voice must actually claim a stretcher when synced, or it plays
           at the wrong tempo silently. */
        v2_set_param(s, "scan", "1");
        check("stretcher needed when synced", needs_stretcher(s));

        /* All 8 voices must get one — the pool used to be half that. */
        int claimed = 0;
        for (int i = 0; i < MAX_VOICES; i++)
            if (claim_stretcher(s, i) >= 0) claimed++;
        char d[64]; snprintf(d, sizeof(d), "(%d of %d)", claimed, MAX_VOICES);
        check("pool covers every voice", claimed == MAX_VOICES, d);

        v2_destroy_instance(s);
    }

    printf("\n== bpm correction ==\n");
    {
        slicer_t *s = (slicer_t *)v2_create_instance("/tmp", NULL);
        v2_set_param(s, "sample_path", SAMPLE);
        float raw = s->sample_bpm;

        v2_set_param(s, "bpm_ratio", "0.5");
        check_near("half time", s->sample_bpm, raw * 0.5f, 0.01f);
        v2_set_param(s, "bpm_ratio", "2.0");
        check_near("double time (from raw, not compounded)", s->sample_bpm, raw * 2.0f, 0.01f);
        v2_set_param(s, "bpm_ratio", "1.0");
        check_near("back to unity restores estimate", s->sample_bpm, raw, 0.01f);

        /* Out-of-range corrections must be refused, not clamped to nonsense */
        v2_set_param(s, "bpm_ratio", "0.01");
        check_near("absurd ratio rejected", s->sample_bpm, raw, 0.01f);
        v2_destroy_instance(s);
    }

    printf("\n== random slicing ==\n");
    {
        slicer_t *s = (slicer_t *)v2_create_instance("/tmp", NULL);
        v2_set_param(s, "sample_path", SAMPLE);
        v2_set_param(s, "slices", "16");
        v2_set_param(s, "slice_algo", "1");

        check("random mode is ready without a scan", s->slicer_state == 1);
        check("playthrough follows random mode", s->playthrough == 1);
        char d[64]; snprintf(d, sizeof(d), "(%d)", s->slice_count_actual);
        check("slice count honoured", s->slice_count_actual == 16, d);

        /* Re-roll must actually change the points */
        int32_t before[MAX_SLICES];
        memcpy(before, s->slice_points, sizeof(before));
        v2_set_param(s, "reroll", "1");
        check("reroll changes the slices",
              memcmp(before, s->slice_points, sizeof(int32_t) * 16) != 0);

        /* Points stay sorted and inside the file across many rolls */
        bool ok = true;
        for (int trial = 0; trial < 200; trial++) {
            v2_set_param(s, "reroll", "1");
            for (int i = 0; i < s->slice_count_actual; i++) {
                if (s->slice_points[i] < 0 || s->slice_points[i] >= s->sample_frames) ok = false;
                if (i && s->slice_points[i] <= s->slice_points[i-1]) ok = false;
            }
            if (s->slice_points[s->slice_count_actual] != s->sample_frames) ok = false;
        }
        check("200 rolls stay sorted and in range", ok);

        /* Playthrough: a held pad must run past its own slice to the file end */
        v2_set_param(s, "playthrough", "1");
        voice_start(s, PAD_BASE + 3, 100);
        voice_t *v = find_voice_for_note(s, PAD_BASE + 3);
        check("voice started", v != NULL && v->active);
        if (v) check("playthrough runs to end of sample", v->slice_end == s->sample_frames);

        v2_set_param(s, "playthrough", "0");
        voice_start(s, PAD_BASE + 3, 100);
        v = find_voice_for_note(s, PAD_BASE + 3);
        if (v) check("without playthrough, stops at next slice",
                     v->slice_end == s->slice_points[4]);

        /* Switching back to transients must restore scan-based behaviour */
        v2_set_param(s, "slice_algo", "0");
        check("transient mode asks for a re-scan", s->slicer_state == 0);
        check("playthrough off in transient mode", s->playthrough == 0);
        v2_destroy_instance(s);
    }

    printf("\n== midi clock ==\n");
    {
        slicer_t *s = (slicer_t *)v2_create_instance("/tmp", NULL);
        /* 24 ticks per beat; at 140 BPM a beat is 60/140 s */
        const float want = 140.0f;
        int frames_per_tick = (int)(SAMPLE_RATE * 60.0f / want / 24.0f);
        uint8_t tick[1] = { 0xF8 };
        int16_t out[BLOCK_SIZE * 2];

        for (int beat = 0; beat < 4; beat++) {
            for (int t = 0; t < 24; t++) {
                v2_on_midi(s, tick, 1, MOVE_MIDI_SOURCE_EXTERNAL);
                /* Advance the frame clock by exactly one tick's worth. Rounding
                   up to whole blocks here would slow the synthetic clock and
                   test the harness rather than the DSP. */
                int rendered = 0;
                while (rendered < frames_per_tick) {
                    int n = frames_per_tick - rendered;
                    if (n > BLOCK_SIZE) n = BLOCK_SIZE;
                    v2_render_block(s, out, n);
                    rendered += n;
                }
            }
        }
        /* The synthetic clock quantises to whole frames per tick */
        float expect = 60.0f * SAMPLE_RATE / (float)(frames_per_tick * 24);
        check_near("clock BPM measured", s->clk_bpm, expect, 0.5f);
        check_near("clock BPM close to intended tempo", s->clk_bpm, want, 1.0f);

        refresh_host_bpm(s);
        check("live clock wins over files", s->tempo_src == TEMPO_SRC_CLOCK);

        /* A single-byte realtime message must not be parsed as a note */
        uint8_t start[1] = { 0xFA };
        v2_on_midi(s, start, 1, MOVE_MIDI_SOURCE_EXTERNAL);
        int active = 0;
        for (int i = 0; i < MAX_VOICES; i++) if (s->voices[i].active) active++;
        check("realtime bytes never trigger voices", active == 0);
        v2_destroy_instance(s);
    }

    printf("\n== state round-trip ==\n");
    {
        slicer_t *a = (slicer_t *)v2_create_instance("/tmp", NULL);
        v2_set_param(a, "sample_path", SAMPLE);
        v2_set_param(a, "slices", "32");
        v2_set_param(a, "slice_algo", "1");
        v2_set_param(a, "bpm_ratio", "0.5");
        v2_set_param(a, "sync", "0");
        v2_set_param(a, "playthrough", "1");
        int len = v2_get_param(a, "state", buf, sizeof(buf));
        check("state serialised", len > 0);

        slicer_t *b = (slicer_t *)v2_create_instance("/tmp", NULL);
        v2_set_param(b, "state", buf);
        check("sync flag restored", b->sync_enabled == 0);
        check("algo restored", b->slice_algo == SLICE_ALGO_RANDOM);
        check("playthrough restored", b->playthrough == 1);
        check_near("corrected BPM restored", b->sample_bpm, a->sample_bpm, 0.2f);
        check("slice points restored", b->slice_count_actual == a->slice_count_actual);
        bool same = true;
        for (int i = 0; i <= b->slice_count_actual; i++)
            if (b->slice_points[i] != a->slice_points[i]) same = false;
        check("random slices survive reload", same);

        /* A patch saved before sync existed must come back with sync on */
        slicer_t *c = (slicer_t *)v2_create_instance("/tmp", NULL);
        v2_set_param(c, "state", "{\"threshold\":0.5,\"slices\":16,\"pitch\":0.0,\"mode\":\"gate\"}");
        check("legacy patch defaults to sync on", c->sync_enabled == 1);

        v2_destroy_instance(a); v2_destroy_instance(b); v2_destroy_instance(c);
    }

    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASS",
           failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
