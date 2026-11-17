/* la_unit.c - lookahead unit test: includes harness.c (lexer, setup) */
#define main harness_main_unused
#include "../harness.c"
#undef main

static double insig(int ch, long n, double fs)
{   /* deterministic worst cases */
    double t = n / fs, s = 0.0;
    uint64_t seed = (uint64_t)ch * 1000003ULL;
    double u = urand(seed, n) - 0.5;
    switch (ch) {
    case 0: case 1:   /* subs: 40 Hz bursts at 2x threshold, hard on/off */
        s = (fmod(t, 0.5) < 0.2) ? 0.8 * sin(2 * M_PI * 40 * t) : 0.1 * u; break;
    case 2: {         /* esll: 3 kHz hard-onset bursts +10 dB, clicks +20 dB, 80/30 Hz bursts, noise */
        double ph = fmod(t, 0.25);
        s = 0.05 * u;
        if (ph < 0.01) s += 1.58 * sin(2 * M_PI * 3000 * t);
        else if (ph > 0.1 && ph < 0.13) s += 0.4 * sin(2 * M_PI * 80 * t);
        else if (ph > 0.15 && ph < 0.2) s += 0.3 * sin(2 * M_PI * 30 * t);
        if (n % 9973 == 17) s += 5.0;
        if (n % 7919 == 3) s -= 5.0;
        break; }
    case 3:           /* eslr: low level, must pass unchanged (delayed) */
        s = 0.01 * sin(2 * M_PI * 1000 * t); break;
    case 4:           /* alkoven: unmapped, must be delayed only */
        s = 0.3 * u; break;
    }
    return s;
}

int main(int argc, char **argv)
{
    if (argc < 4) { fprintf(stderr, "la_unit CFG RATE SECONDS\n"); return 2; }
    if (getenv("ALRM")) alarm(3);
    int rate = atoi(argv[2]);
    long nblk = (long)(atof(argv[3]) * rate / BL);
    if (setup(argv[1], rate, 8) != 0) { printf("setup failed\n"); return 3; }
#ifdef NO_LA
    int D = 0;
#else
    int D = la_L > 0 ? la_L - 1 : 0;
#endif
    double thr_hard = groups[0].limits[0].threshold_lin;
    double thr_sclip = groups[1].limits[0].threshold_lin;
    struct limit_cfg *fx = &groups[0].limits[1];
    double fz = 0.0, fmax_out = 0.0, fmax_in = 0.0, fz_in = 0.0;
    double maxr_hard = 0, maxr_sub = 0, dly_err = 0, pass_err = 0, minr = 1;
    uint64_t h = 1469598103934665603ULL;
    static double buf[NCH][BL];
    for (long b = 0; b < nblk; b++) {
        for (int c = 0; c < NCH; c++) {
            for (int i = 0; i < BL; i++) buf[c][i] = insig(c, b * BL + i, rate);
            output_timed(buf[c], c);
            for (int i = 0; i < BL; i++) {
                long n = b * BL + i;
                double y = buf[c][i];
                unsigned char *p = (unsigned char *)&y;
                for (int k = 0; k < 8; k++) { h ^= p[k]; h *= 1099511628211ULL; }
                double x_d = (n - D >= 0) ? insig(c, n - D, rate) : 0.0;
                if (c == 2) {
                    if (fabs(y) / thr_hard > maxr_hard) maxr_hard = fabs(y) / thr_hard;
                    fz = (1 - fx->shelf_coeff) * y + fx->shelf_coeff * fz;
                    if (fabs(fz) > fmax_out) fmax_out = fabs(fz);
                    fz_in = (1 - fx->shelf_coeff) * x_d + fx->shelf_coeff * fz_in;
                    if (fabs(fz_in) > fmax_in) fmax_in = fabs(fz_in);
                    if (x_d != 0 && fabs(y / x_d) < minr) minr = fabs(y / x_d);
                }
                if (c == 0 && fabs(y) / thr_sclip > maxr_sub) maxr_sub = fabs(y) / thr_sclip;
                if (c == 3 && n > rate && fabs(y - x_d) > pass_err) pass_err = fabs(y - x_d);
                if (c == 4 && fabs(y - x_d) > dly_err) dly_err = fabs(y - x_d);
            }
        }
    }
    printf("rate=%d D=%d hash=%016llx\n", rate, D, (unsigned long long)h);
    printf("  hard (unweighted) max |y|/thr = %.9f  (%+.3f dB)\n", maxr_hard, 20 * log10(maxr_hard));
    printf("  sub sclip         max |y|/thr = %.9f  (%+.3f dB)\n", maxr_sub, 20 * log10(maxr_sub));
    printf("  flux detector out max/thr     = %.4f  (%+.2f dB), input %+.2f dB\n",
           fmax_out / fx->threshold_lin, 20 * log10(fmax_out / fx->threshold_lin),
           20 * log10(fmax_in / fx->threshold_lin));
    printf("  esll deepest gain %.2f dB\n", 20 * log10(minr));
    printf("  eslr low-level |y - x[n-D]| max = %.3g (after 1 s)\n", pass_err);
    printf("  alkoven unmapped |y - x[n-D]| max = %.3g\n", dly_err);
    return 0;
}
