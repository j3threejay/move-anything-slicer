/* Host-side harness for the slicer's BPM detector and random slicer.
   Includes dsp.cpp directly so it exercises the shipping code, not a copy. */
#include "../src/dsp/dsp.cpp"

int main(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr, "usage: bpm_test <file.wav> [expected_bpm]\n"); return 2; }

    slicer_t *s = (slicer_t *)v2_create_instance("/tmp", NULL);
    if (!s) { fprintf(stderr, "create_instance failed\n"); return 2; }

    if (!load_sample_synced(s, argv[1])) {
        fprintf(stderr, "load failed: %s\n", argv[1]);
        return 2;
    }

    double dur = (double)s->sample_frames / SAMPLE_RATE;
    printf("%-28s  %6.2fs  detected=%7.2f", argv[1], dur, s->sample_bpm);

    int rc = 0;
    if (argc > 2) {
        float want = strtof(argv[2], NULL);
        float got  = s->sample_bpm;
        /* accept the octave-ambiguous readings separately so we can see them */
        float err     = fabsf(got - want) / want;
        float err_half = fabsf(got * 2.0f - want) / want;
        float err_dbl  = fabsf(got * 0.5f - want) / want;
        if (err < 0.02f)            printf("  want=%6.2f  OK", want);
        else if (err_half < 0.02f)  printf("  want=%6.2f  HALF-TIME", want), rc = 1;
        else if (err_dbl  < 0.02f)  printf("  want=%6.2f  DOUBLE-TIME", want), rc = 1;
        else                        printf("  want=%6.2f  FAIL", want), rc = 1;
    }
    printf("\n");

    /* Exercise random slicing + playthrough boundaries at the detected grid */
    s->slice_algo = SLICE_ALGO_RANDOM;
    s->slice_count = 16;
    make_random_slices(s);
    printf("    random: %d slices, first=%d last=%d end=%d",
           s->slice_count_actual, (int)s->slice_points[0],
           (int)s->slice_points[s->slice_count_actual - 1],
           (int)s->slice_points[s->slice_count_actual]);
    int sorted = 1, in_range = 1;
    for (int i = 0; i < s->slice_count_actual; i++) {
        if (i && s->slice_points[i] <= s->slice_points[i-1]) sorted = 0;
        if (s->slice_points[i] < 0 || s->slice_points[i] >= s->sample_frames) in_range = 0;
    }
    printf("  sorted=%s in_range=%s\n", sorted ? "yes" : "NO", in_range ? "yes" : "NO");
    if (!sorted || !in_range) rc = 1;

    /* Sync ratio sanity */
    s->host_bpm = 120.0f;
    printf("    speed @120: %.4f  (sync=%d)\n", effective_speed(s), s->sync_enabled);

    v2_destroy_instance(s);
    return rc;
}
