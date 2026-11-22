/* integ_unit.c - offline test of weighting "integrator" (Cue-020).
 *
 * Includes harness.c (lexer, setup). One process = one config + one signal,
 * because the module state is static. The signal goes to "esll"; all other
 * outputs get silence.
 *
 *   integ_unit CFG RATE FREQ_HZ AMP_V SECONDS FADE_S DC_V TAIL_S
 *
 *   AMP_V   tone peak in volts at the limit's point (chain: 1000 V = 0 dBFS)
 *   FADE_S  raised-cosine fade-in; 0 = hard onset at zero phase
 *   DC_V    DC offset added for the whole tone
 *   TAIL_S  silence after the tone (integrator decay check)
 *
 * Environment (optional):
 *   REALSIZE=4     float buffers (S32/float path), default 8 (double)
 *   OFF_AT=<s>     "lmc off flux" before the block that contains time s
 *   ON_AT=<s>      "lmc on flux"  before the block that contains time s
 *
 * Invariants reported for EVERY run (checked by integ_tests.sh): the module
 * never makes a sample larger than its input (grow_V) and never flips its
 * sign (sign_flips).
 *
 * Prints one line "key=value ..." for tests/integ/integ_tests.sh.
 * Nothing here touches BruteFIR, holographer or an audio device.
 */
#define main harness_main_unused
#include "../harness.c"
#undef main

#define VFS 1000.0                    /* volts at 0 dBFS in cfg_integ_*.txt */

static double g_f, g_amp, g_sec, g_fade, g_dc;
static long g_ntone;
static int g_rs = 8;

static double insig_raw(long n, double fs)
{
    if (n < 0 || n >= g_ntone) return 0.0;
    double t = n / fs, env = 1.0;
    if (g_fade > 0.0 && t < g_fade) env = 0.5 - 0.5 * cos(M_PI * t / g_fade);
    return (env * g_amp * sin(2 * M_PI * g_f * t) + g_dc) / VFS;
}

/* the input exactly as the module sees it (float-rounded for REALSIZE=4) */
static double insig(int ch, long n, double fs)
{
    if (ch != ESLL) return 0.0;
    double x = insig_raw(n, fs);
    return g_rs == 4 ? (double)(float)x : x;
}

static double envd(const char *k, double dflt)
{
    const char *v = getenv(k);
    return (v && *v) ? atof(v) : dflt;
}

int main(int argc, char **argv)
{
    if (argc < 9) {
        fprintf(stderr, "integ_unit CFG RATE FREQ_HZ AMP_V SECONDS FADE_S DC_V TAIL_S\n");
        return 2;
    }
    int rate = atoi(argv[2]);
    g_f = atof(argv[3]); g_amp = atof(argv[4]); g_sec = atof(argv[5]);
    g_fade = atof(argv[6]); g_dc = atof(argv[7]);
    double tail = atof(argv[8]);
    g_rs = (int)envd("REALSIZE", 8);
    double off_at = envd("OFF_AT", -1), on_at = envd("ON_AT", -1);
    g_ntone = (long)(g_sec * rate);
    long nblk = (long)((g_sec + tail) * rate / BL) + 1;
    if (setup(argv[1], rate, g_rs) != 0) { printf("setup failed\n"); return 3; }
    int D = la_L > 0 ? la_L - 1 : 0;

    struct limit_cfg *lc = &groups[0].limits[0];
    if (!lc->integrator) { printf("limit 0 is not an integrator\n"); return 3; }
    const double a = lc->shelf_coeff, G = lc->integ_gain, thr = lc->threshold_lin;
    long off_blk = off_at >= 0 ? (long)(off_at * rate) / BL : -1;
    long on_blk = on_at >= 0 ? (long)(on_at * rate) / BL : -1;
    long after_n = on_at >= 0 ? (long)((on_at + 0.5) * rate) : 0;
    char cmd_msg[256] = "";

    /* steady window: a whole number of periods, about the last 0.25 s */
    int ncyc = (int)(0.25 * g_f + 0.5); if (ncyc < 1) ncyc = 1;
    long st1 = g_ntone, st0 = g_ntone - (long)(ncyc * (double)rate / g_f + 0.5);
    double ss = 0, sc = 0, sq = 0, sm = 0; long sn = 0;
    double zin = 0, zout = 0, det_out_max = 0, det_in_max = 0, det_after = 0;
    double det_out_st = 0, minr = 1.0, steady_pk = 0, steady_minr = 1.0, tail_z = 0, tone_z = 0;
    long identical = 1, nlim = 0, flips = 0;
    double maxdev = 0, stdev_max = 0, grow = 0, ymax = 0;
    static double buf[NCH][BL];
    static float fbuf[NCH][BL];
    for (long b = 0; b < nblk; b++) {
        if (b == off_blk) { bflogic_command("off flux"); snprintf(cmd_msg, sizeof(cmd_msg), "%s", bflogic_message()); }
        if (b == on_blk) bflogic_command("on flux");
        for (int c = 0; c < NCH; c++) {
            for (int i = 0; i < BL; i++) {
                double x = insig(c, b * BL + i, rate);
                if (g_rs == 4) fbuf[c][i] = (float)x; else buf[c][i] = x;
            }
            output_timed(g_rs == 4 ? (void *)fbuf[c] : (void *)buf[c], c);
            if (g_rs == 4) for (int i = 0; i < BL; i++) buf[c][i] = fbuf[c][i];
        }
        for (int i = 0; i < BL; i++) {
            long n = b * BL + i;
            long m = n - D;                    /* input time of this output */
            double y = buf[ESLL][i], x = insig(ESLL, m, rate);
            if (y != x) identical = 0;
            if (fabs(y - x) > maxdev) maxdev = fabs(y - x);
            if (fabs(y) - fabs(x) > grow) grow = fabs(y) - fabs(x);
            if (y * x < 0.0 || (x == 0.0 && y != 0.0)) flips++;
            if (fabs(y) > ymax) ymax = fabs(y);
            if (m >= st0 && m < st1 && fabs(y - x) > stdev_max) stdev_max = fabs(y - x);
            zout = (1 - a) * y + a * zout;
            zin = (1 - a) * x + a * zin;
            double dout = G * fabs(zout), din = G * fabs(zin);
            if (dout > det_out_max) det_out_max = dout;
            if (din > det_in_max) det_in_max = din;
            if (on_at >= 0 && n >= after_n && dout > det_after) det_after = dout;
            if (fabs(x) > 1e-9) {
                double r = fabs(y / x);
                if (r < minr) minr = r;
                if (r < 0.999999) nlim++;
                if (m >= st0 && m < st1 && r < steady_minr) steady_minr = r;
            }
            if (m >= st0 && m < st1 && fabs(y) > steady_pk) steady_pk = fabs(y);
            if (m >= st0 && m < st1 && dout > det_out_st) det_out_st = dout;
            if (m >= st0 && m < st1) {
                double ph = 2 * M_PI * g_f * m / rate;
                ss += y * sin(ph); sc += y * cos(ph); sq += y * y; sm += y; sn++;
            }
            if (m >= 0 && m < g_ntone && fabs(zout) > tone_z) tone_z = fabs(zout);
            if (m >= g_ntone) tail_z = fabs(zout);       /* last value */
        }
    }
    double A = sn ? 2.0 * sqrt(ss * ss + sc * sc) / sn : 0.0;     /* fundamental peak */
    double mean = sn ? sm / sn : 0.0;
    double resid = sn ? sq / sn - mean * mean - A * A / 2.0 : 0.0;
    double thdn = (A > 0 && resid > 0) ? sqrt(resid) / (A / sqrt(2.0)) : 0.0;
    printf("rate=%d D=%d rs=%d f=%g amp=%g fade=%g dc=%g G=%.6f thr_V=%.3f "
           "identical=%ld limited_samples=%ld max_red_dB=%.4f steady_red_dB=%.4f "
           "steady_peak_V=%.3f det_out_steady_ratio=%.6f det_out_max_ratio=%.6f det_in_max_ratio=%.4f "
           "tail_z_ratio=%.3g flux_clips=%ld fund_peak_V=%.3f thdn_pct=%.3f max_dev_V=%.4g steady_dev_V=%.4g "
           "grow_V=%.3g sign_flips=%ld y_max_V=%.3f det_after_on_ratio=%.6f off_refused=%d\n",
           rate, D, g_rs, g_f, g_amp, g_fade, g_dc, G, thr * VFS,
           identical, nlim, -20 * log10(minr), -20 * log10(steady_minr),
           steady_pk * VFS, det_out_st / thr, det_out_max / thr, det_in_max / thr,
           tone_z > 0 ? tail_z / tone_z : 0.0, ch_runtime[0][0].st[0].flux_clips, A * VFS, 100.0 * thdn,
           maxdev * VFS, stdev_max * VFS, grow * VFS, flips, ymax * VFS, det_after / thr,
           strstr(cmd_msg, " 0 refused") == NULL && off_blk >= 0);
    return 0;
}
