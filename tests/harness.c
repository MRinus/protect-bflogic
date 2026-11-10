/*
 * harness.c - offline test harness for bflogic_protect.c (no BruteFIR).
 *
 * Includes the module source directly (white box: fault injection into the
 * publish slots, block-wise gain snapshots). Emulates what BruteFIR 1.1.2
 * does around the module:
 *   - preinit with a token stream (get_config_token, ends with 0) built by a
 *     small lexer from a config text (the body of the "protect" { } block),
 *   - bflogic_init, then per block one output_timed(buf, ch) per channel,
 *   - three execution modes:
 *       seq : all channels in one thread, one after the other
 *       thr : one thread per channel, pthread barrier before output_timed
 *       sem : one thread per channel, semaphore barrier exactly like
 *             bfrun.c synch_filter_processes (post to all others, wait n-1)
 *
 * Usage (see tests.sh):
 *   harness preinit CFG                     exit 0 = accepted, 3 = rejected
 *   harness run CFG SCEN MODE RATE NBLK RS  prints "hash=... faults=..."
 *   harness t1a CFG_LINK CFG_NOLINK RATE NBLK      invariant (fork + pipe)
 *   harness t2|t3|t4|t5|t9|t10|t11 ...            see the test functions
 *   harness t8 CFG RATE NBLK                       hot path / timing
 *   harness lmc CFG RATE NBLK CMD                  lmc output after a run
 *
 * Outputs (fixed): subl subr esll eslr alkoven = intname 0..4.
 */
#define _GNU_SOURCE
#include <pthread.h>
#include <semaphore.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/wait.h>
#include <time.h>
#include <errno.h>

#include MODULE_SRC

#define NCH 5
#define BL  1024
static const char *out_names[NCH] = { "subl", "subr", "esll", "eslr", "alkoven" };
enum { SUBL, SUBR, ESLL, ESLR, ALK };

/* ---------------------------------------------------------------- lexer */
struct tok { int token; union bflexval v; };
static struct tok toks[8192];
static int ntok, tokpos;
static char strpool[1 << 20];
static size_t poolpos;

static char *pool_dup(const char *s, size_t n)
{
    char *d = strpool + poolpos;
    memcpy(d, s, n); d[n] = 0; poolpos += n + 1;
    return d;
}

static int lex(const char *t)
{
    ntok = 0; tokpos = 0; poolpos = 0;
    while (*t) {
        if (*t == '#') { while (*t && *t != '\n') t++; continue; }
        if (*t == ' ' || *t == '\t' || *t == '\n' || *t == '\r') { t++; continue; }
        struct tok *k = &toks[ntok++];
        if (*t == ';') { k->token = BF_LEX_EOS; t++; }
        else if (*t == '{') { k->token = BF_LEX_LBRACE; t++; }
        else if (*t == '}') { k->token = BF_LEX_RBRACE; t++; }
        else if (*t == ',') { k->token = BF_LEX_COMMA; t++; }
        else if (*t == '/') { k->token = BF_LEX_SLASH; t++; }
        else if (*t == '"') {
            const char *e = strchr(t + 1, '"');
            if (!e) return -1;
            k->token = BF_LEXVAL_STRING; k->v.string = pool_dup(t + 1, e - t - 1); t = e + 1;
        } else if ((*t >= '0' && *t <= '9') || *t == '-' || *t == '+' || *t == '.') {
            char *e; k->token = BF_LEXVAL_REAL; k->v.real = strtod(t, &e);
            if (e == t) return -1;
            t = e;
        } else {
            const char *s = t;
            while ((*t >= 'a' && *t <= 'z') || (*t >= 'A' && *t <= 'Z') || (*t >= '0' && *t <= '9') || *t == '_') t++;
            if (t == s) return -1;
            if (*t == ':') {
                k->token = BF_LEXVAL_FIELD; k->v.field = pool_dup(s, t - s); t++;
            } else if ((t - s == 4 && !strncmp(s, "true", 4)) || (t - s == 5 && !strncmp(s, "false", 5))) {
                k->token = BF_LEXVAL_BOOLEAN; k->v.boolean = (*s == 't');
            } else {
                return -1;
            }
        }
    }
    toks[ntok].token = 0;
    return 0;
}

static int get_tok(union bflexval *v)
{
    if (toks[tokpos].token == 0) return 0;
    *v = toks[tokpos].v;
    return toks[tokpos++].token;
}

static char *read_file(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); exit(2); }
    static char buf[1 << 20];
    size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    buf[n] = 0;
    return buf;
}

/* --------------------------------------------------------- module setup */
static struct bfchannel chs_out[NCH];
static struct bfchannel chs_in[2];
static struct bfaccess acc;
static struct bfevents ev;

static int setup(const char *cfgpath, int rate, int realsz)
{
    if (lex(read_file(cfgpath)) != 0) { fprintf(stderr, "HARNESS: lex error in %s\n", cfgpath); return -2; }
    for (int i = 0; i < NCH; i++) { strcpy(chs_out[i].name, out_names[i]); chs_out[i].intname = i; }
    strcpy(chs_in[0].name, "leftin"); chs_in[0].intname = 0;
    strcpy(chs_in[1].name, "rightin"); chs_in[1].intname = 1;
    const struct bfchannel *chp[2] = { chs_in, chs_out };
    int nch[2] = { 2, NCH };
    int vmaj = 0, vmin = 0, fork_mode = -1;
    memset(&ev, 0, sizeof(ev));
    int rc = bflogic_preinit(&vmaj, &vmin, get_tok, rate, BL, 64, 0, NULL, nch, chp, 0, NULL, &ev, &fork_mode, 0);
    if (rc != 0) return rc;
    memset(&acc, 0, sizeof(acc));
    acc.realsize = realsz;
    return bflogic_init(&acc, rate, BL, 64, 0, NULL, nch, chp, 0, NULL, -1, -1);
}

/* -------------------------------------------------------------- signals */
/* deterministic per (scenario, channel, sample) -> both processes of a
   fork comparison and all thread modes see identical input */
static uint64_t mix64(uint64_t x)
{
    x ^= x >> 33; x *= 0xff51afd7ed558ccdULL; x ^= x >> 33;
    x *= 0xc4ceb9fe1a85ec53ULL; x ^= x >> 33; return x;
}
static double urand(uint64_t a, uint64_t b) { return (mix64(a * 0x9E3779B97F4A7C15ULL ^ mix64(b)) >> 11) * (1.0 / 9007199254740992.0); }

static int g_rate;
static double g_fs = 1.0;          /* full scale in buffer units (output_scale) */
static int g_scen;
enum { SC_RANDOM, SC_T2, SC_T3, SC_T4, SC_T11, SC_QUIET, SC_CONST };
static double sc_level_db = 0;     /* scenario parameter */

static double sig(int ch, long n)
{
    double t = (double)n / g_rate;
    switch (g_scen) {
    case SC_RANDOM: {
        /* noise + sines, with random bursts (per 0.1 s segment) from
           -40 dBFS to +6 dBFS -> every threshold is crossed sometimes */
        long seg = n / (g_rate / 10);
        double lvl_db = -40.0 + 46.0 * urand(ch * 7 + 1, seg);
        double a = pow(10.0, lvl_db / 20.0);
        double f = 20.0 + 2000.0 * urand(ch * 7 + 2, seg);
        double x = 0.7 * sin(2 * M_PI * f * t) + 0.3 * (2.0 * urand(ch * 7 + 3, n) - 1.0);
        return a * x * g_fs;
    }
    case SC_T2: case SC_T4: {
        /* driver(s): 144 Hz burst from 1.0 s to 2.0 s at +sc_level_db over
           the -6 dBFS threshold, else -30 dBFS; followers: constant
           -20 dBFS (DC, so y/x = applied gain per sample exactly) */
        int driver = (ch == ESLL);
        if (driver) {
            double a = (t >= 1.0 && t < 2.0) ? pow(10.0, (-6.0 + sc_level_db) / 20.0) : pow(10.0, -30.0 / 20.0);
            return a * sin(2 * M_PI * 144.0 * t) * g_fs;
        }
        if (g_scen == SC_T4 && ch == ESLR) return pow(10.0, -20.0 / 20.0) * g_fs;
        return pow(10.0, -20.0 / 20.0) * g_fs;
    }
    case SC_T3:
        /* alkoven over its own limit, ESL quiet (-30 dBFS sine) */
        if (ch == ALK) return pow(10.0, 0.0 / 20.0) * sin(2 * M_PI * 144.0 * t) * g_fs;
        return pow(10.0, -30.0 / 20.0) * sin(2 * M_PI * 440.0 * t) * g_fs;
    case SC_T11:
        /* deep reduction on esll (+12 dB over threshold 1.0..1.5 s), then
           release; alkoven constant -20 dBFS */
        if (ch == ESLL) {
            double a = (t >= 1.0 && t < 1.5) ? pow(10.0, 6.0 / 20.0) : pow(10.0, -40.0 / 20.0);
            return a * sin(2 * M_PI * 144.0 * t) * g_fs;
        }
        return pow(10.0, -20.0 / 20.0) * g_fs;
    case SC_QUIET:
        return pow(10.0, -40.0 / 20.0) * sin(2 * M_PI * (100.0 + 50 * ch) * t) * g_fs;
    case SC_CONST:
    default:
        return pow(10.0, -20.0 / 20.0) * g_fs;
    }
}

/* -------------------------------------------------------------- runner */
static int g_realsize = 8;
static double xin[NCH][BL], yout[NCH][BL];
static float fbuf_[NCH][BL];

static void fill(int ch, long blk)
{
    for (int i = 0; i < BL; i++) xin[ch][i] = sig(ch, blk * BL + i);
}

static void run_ch(int ch)
{
    if (g_realsize == 4) {
        for (int i = 0; i < BL; i++) fbuf_[ch][i] = (float)xin[ch][i];
        output_timed(fbuf_[ch], ch);
        for (int i = 0; i < BL; i++) yout[ch][i] = fbuf_[ch][i];
    } else {
        memcpy(yout[ch], xin[ch], sizeof(yout[ch]));
        output_timed(yout[ch], ch);
    }
}

/* per-channel FNV-1a hash over the output and the channel's own limit
   gains, updated by whichever thread processed the channel */
static uint64_t g_hash[NCH];
static void hash_ch(int ch)
{
    uint64_t h = g_hash[ch];
    const unsigned char *p = (const unsigned char *)yout[ch];
    for (size_t i = 0; i < sizeof(yout[ch]); i++) { h ^= p[i]; h *= 1099511628211ULL; }
    int gi = chan_group[ch], ci = chan_idx[ch];
    if (gi >= 0)
        for (int l = 0; l < groups[gi].n_limits; l++) {
            const unsigned char *q = (const unsigned char *)&ch_runtime[gi][ci].st[l].gain;
            for (size_t i = 0; i < sizeof(double); i++) { h ^= q[i]; h *= 1099511628211ULL; }
        }
    g_hash[ch] = h;
}

/* per-block hook (called by the main thread between blocks) */
typedef void (*blk_hook)(long blk, void *ctx);

struct thr_ctx {
    int ch; long nblk; int mode; int faithful;
    pthread_barrier_t *b_start, *b_done;
    sem_t *sems;  /* sem mode: one per thread */
};
static volatile int hot = 0;   /* set while output_timed may run (T8) */

static void sem_barrier(sem_t *sems, int me, int n)
{
    /* bfrun.c synch_filter_processes: post to every other, wait n-1 */
    for (int j = 0; j < n; j++) if (j != me) sem_post(&sems[j]);
    for (int j = 0; j < n - 1; j++) while (sem_wait(&sems[me]) != 0 && errno == EINTR) ;
}

static void *thr_main(void *arg)
{
    struct thr_ctx *c = arg;
    for (long k = 0; k < c->nblk; k++) {
        fill(c->ch, k);
        if (c->mode == 1) pthread_barrier_wait(c->b_start);
        else sem_barrier(c->sems, c->ch, NCH);
        run_ch(c->ch);
        if (c->faithful) {
            /* like BruteFIR: no other synchronisation than the barrier
               above; each thread accounts for its own channel only */
            hash_ch(c->ch);
            continue;
        }
        pthread_barrier_wait(c->b_done);   /* main collects block k */
        pthread_barrier_wait(c->b_start + 1); /* main done collecting */
    }
    return NULL;
}

/* mode: 0 seq, 1 thr (pthread barrier), 2 sem (bfrun-like) */
static void run(int mode, long nblk, blk_hook hook, void *ctx)
{
    if (mode == 0) {
        for (long k = 0; k < nblk; k++) {
            for (int ch = 0; ch < NCH; ch++) fill(ch, k);
            for (int ch = 0; ch < NCH; ch++) { run_ch(ch); if (!hook) hash_ch(ch); }
            if (hook) hook(k, ctx);
        }
        return;
    }
    int faithful = (hook == NULL);
    pthread_t th[NCH];
    struct thr_ctx c[NCH];
    pthread_barrier_t b[2], done;
    sem_t sems[NCH];
    pthread_barrier_init(&b[0], NULL, NCH);
    pthread_barrier_init(&b[1], NULL, NCH + 1);
    pthread_barrier_init(&done, NULL, NCH + 1);
    for (int i = 0; i < NCH; i++) sem_init(&sems[i], 0, 0);
    for (int i = 0; i < NCH; i++) {
        c[i] = (struct thr_ctx){ i, nblk, mode, faithful, b, &done, sems };
        pthread_create(&th[i], NULL, thr_main, &c[i]);
    }
    for (long k = 0; k < nblk && !faithful; k++) {
        pthread_barrier_wait(&done);
        if (hook) hook(k, ctx);
        pthread_barrier_wait(&b[1]);
    }
    for (int i = 0; i < NCH; i++) pthread_join(th[i], NULL);
    pthread_barrier_destroy(&b[0]); pthread_barrier_destroy(&b[1]); pthread_barrier_destroy(&done);
    for (int i = 0; i < NCH; i++) sem_destroy(&sems[i]);
}

static long total_faults(void)
{
#ifndef REFERENCE_BUILD
    long s = 0;
    for (int i = 0; i < MAX_MAPPED_CHANNELS; i++) s += a_load_l(&link_faults[i]);
    return s;
#else
    return 0;
#endif
}

/* ---------------------------------------------------------- reporting */
static int n_pass, n_fail;
static void check(int ok, const char *fmt, ...)
{
    va_list ap; va_start(ap, fmt);
    printf("  %s ", ok ? "PASS" : "FAIL");
    vprintf(fmt, ap); printf("\n");
    va_end(ap);
    if (ok) n_pass++; else n_fail++;
    fflush(stdout);
}
static int finish(void) { printf("  [%d PASS, %d FAIL]\n", n_pass, n_fail); return n_fail ? 1 : 0; }

static int parse_mode(const char *s) { return !strcmp(s, "seq") ? 0 : !strcmp(s, "thr") ? 1 : !strcmp(s, "sem") ? 2 : -1; }
static int parse_scen(const char *s)
{
    if (!strcmp(s, "random")) return SC_RANDOM;
    if (!strcmp(s, "t2")) return SC_T2;
    if (!strcmp(s, "t3")) return SC_T3;
    if (!strcmp(s, "t4")) return SC_T4;
    if (!strcmp(s, "t11")) return SC_T11;
    if (!strcmp(s, "quiet")) return SC_QUIET;
    return SC_CONST;
}

static int cmd_run(int argc, char **argv)
{
    /* run CFG SCEN MODE RATE NBLK REALSIZE FS */
    if (argc < 9) return 2;
    g_scen = parse_scen(argv[3]);
    int mode = parse_mode(argv[4]);
    g_rate = atoi(argv[5]);
    long nblk = atol(argv[6]);
    g_realsize = atoi(argv[7]);
    g_fs = atof(argv[8]);
    if (setup(argv[2], g_rate, g_realsize) != 0) return 3;
    for (int ch = 0; ch < NCH; ch++) g_hash[ch] = 1469598103934665603ULL;
    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    run(mode, nblk, NULL, NULL);
    clock_gettime(CLOCK_MONOTONIC, &t1);
    uint64_t hc = 1469598103934665603ULL;
    for (int ch = 0; ch < NCH; ch++) { hc ^= g_hash[ch]; hc *= 1099511628211ULL; }
    printf("hash=%016llx faults=%ld blocks=%ld mode=%s rate=%d realsize=%d wall=%.2fs\n",
           (unsigned long long)hc, total_faults(), nblk, argv[4], g_rate, g_realsize,
           (t1.tv_sec - t0.tv_sec) + 1e-9 * (t1.tv_nsec - t0.tv_nsec));
    return 0;
}

/* lmc CFG RATE NBLK SCEN CMD... : output of bflogic_command after a run */
static int cmd_lmc(int argc, char **argv)
{
    if (argc < 7) return 2;
    g_rate = atoi(argv[3]);
    g_scen = parse_scen(argv[5]);
    g_fs = 2147483647.0;
    if (setup(argv[2], g_rate, 8) != 0) return 3;
    run(0, atol(argv[4]), NULL, NULL);
    for (int i = 6; i < argc; i++) {
        bflogic_command(argv[i]);
        printf("=== lmc '%s'\n%s", argv[i], bflogic_message());
    }
    return 0;
}

#ifndef REFERENCE_BUILD
/* ------------------------------------------------- T1a: invariant (fork) */
struct t1a_rec { double y[NCH][BL]; double g[MAX_GROUPS][MAX_CH_PER_GROUP][MAX_LIMITS_PER_GROUP]; };
static int pipefd[2];
static struct t1a_rec rec;

static void snap(struct t1a_rec *r)
{
    memcpy(r->y, yout, sizeof(yout));
    memset(r->g, 0, sizeof(r->g));
    for (int g = 0; g < n_groups; g++)
        for (int c = 0; c < groups[g].n_channels; c++)
            for (int l = 0; l < groups[g].n_limits; l++) r->g[g][c][l] = ch_runtime[g][c].st[l].gain;
}

static void child_hook(long blk, void *v)
{
    (void)blk; (void)v;
    snap(&rec);
    const char *p = (const char *)&rec; size_t left = sizeof(rec);
    while (left) { ssize_t w = write(pipefd[1], p, left); if (w <= 0) _exit(9); p += w; left -= (size_t)w; }
}

struct t1a_stat { long viol, det_diff, reduced, samples; double worst; long blocks; };
static void parent_hook(long blk, void *v)
{
    struct t1a_stat *s = v;
    static struct t1a_rec ref, me;
    char *p = (char *)&ref; size_t left = sizeof(ref);
    while (left) { ssize_t r = read(pipefd[0], p, left); if (r <= 0) { fprintf(stderr, "pipe short read at block %ld\n", blk); exit(9); } p += r; left -= (size_t)r; }
    snap(&me);
    if (memcmp(me.g, ref.g, sizeof(me.g)) != 0) s->det_diff++;
    for (int ch = 0; ch < NCH; ch++)
        for (int i = 0; i < BL; i++) {
            double a = fabs(me.y[ch][i]), b = fabs(ref.y[ch][i]);
            s->samples++;
            if (a > b) { s->viol++; if (a - b > s->worst) s->worst = a - b; }
            else if (a < b) s->reduced++;
        }
    s->blocks++;
}

static int cmd_t1a(int argc, char **argv)
{
    /* t1a CFG_LINK CFG_NOLINK RATE NBLK MODE */
    if (argc < 7) return 2;
    g_scen = SC_RANDOM; g_rate = atoi(argv[4]); g_fs = 2147483647.0;
    long nblk = atol(argv[5]);
    int mode = parse_mode(argv[6]);
    if (pipe(pipefd) != 0) return 2;
    pid_t pid = fork();
    if (pid == 0) {
        close(pipefd[0]);
        if (setup(argv[3], g_rate, 8) != 0) _exit(3);
        run(0, nblk, child_hook, NULL);
        _exit(0);
    }
    close(pipefd[1]);
    if (setup(argv[2], g_rate, 8) != 0) return 3;
    struct t1a_stat st = { 0 };
    run(mode, nblk, parent_hook, &st);
    int status; waitpid(pid, &status, 0);
    printf("T1a rate=%d mode=%s blocks=%ld samples=%ld\n", g_rate, argv[6], st.blocks, st.samples);
    check(WIFEXITED(status) && WEXITSTATUS(status) == 0, "reference run (no link) completed");
    check(st.det_diff == 0, "own detector gains bit-identical with/without link in every block (%ld blocks differ)", st.det_diff);
    check(st.viol == 0, "|y_link| <= |y_nolink| for every sample (%ld violations, worst %.3g)", st.viol, st.worst);
    check(st.reduced > 0, "link actually reduced %ld samples (test is not vacuous)", st.reduced);
    check(total_faults() == 0, "link_faults = 0 (%ld)", total_faults());
    return finish();
}

/* ---------------------------------------- per-block gain trace helpers */
#define MAXB 4096
static double tr_min[NCH][MAXB], tr_end[NCH][MAXB], tr_first[NCH][MAXB];
static double tr_maxstep[NCH][MAXB], tr_bstep[NCH][MAXB];
static double last_g[NCH];
static long tr_n;

/* gain per sample = y/x (x != 0 required: followers get constant input) */
static void trace_hook(long blk, void *v)
{
    (void)v;
    if (blk >= MAXB) return;
    for (int ch = 0; ch < NCH; ch++) {
        double mn = 1e9, ms = 0;
        double prev = (blk == 0) ? 1.0 : last_g[ch];
        double bstep = 0;
        for (int i = 0; i < BL; i++) {
            double g = (xin[ch][i] != 0.0) ? yout[ch][i] / xin[ch][i] : 1.0;
            if (g < mn) mn = g;
            double st = fabs(g - prev);
            if (i == 0) bstep = st; else if (st > ms) ms = st;
            prev = g;
        }
        tr_first[ch][blk] = yout[ch][0] / (xin[ch][0] != 0 ? xin[ch][0] : 1);
        tr_min[ch][blk] = mn; tr_end[ch][blk] = prev; tr_maxstep[ch][blk] = ms; tr_bstep[ch][blk] = bstep;
        last_g[ch] = prev;
    }
    tr_n = blk + 1;
}
static double db(double g) { return 20.0 * log10(g); }

/* T2 / T5 / T4: follower block-end gain(k) == driver own_min(k-1) mapped by
   ratio/offset. Drivers here have no partner reduction (L = 1), so their
   own_min = min over the block of y/x for a sine? -> use the published slot
   value directly (white box) recorded per block. */
static double pub_rec[NCH][MAXB];
static void trace_pub_hook(long blk, void *v)
{
    trace_hook(blk, v);
    if (blk < MAXB)
        for (int ch = 0; ch < NCH; ch++) pub_rec[ch][blk] = a_load_d(&pub[ch][(blk + 1) & 1].gmin);
}

static int follow_test(const char *name, int drv, int fol, double ratio, double offset, double tol_db, int rate)
{
    long first_d = -1, first_f = -1;
    double worst = 0; long checked = 0;
    for (long k = 1; k < tr_n; k++) {
        double a = -db(pub_rec[drv][k - 1]);
        double exp_db = ratio * fmax(0.0, a - offset);
        double got_db = -db(tr_end[fol][k]);
        double e = fabs(got_db - exp_db);
        if (e > worst) worst = e;
        checked++;
        if (first_d < 0 && ratio * (a - offset) > 0.1) first_d = k - 1;
        if (first_f < 0 && got_db > 0.1) first_f = k;
    }
    check(worst <= tol_db, "%s: follower block-end reduction = ratio*max(0,A_drv(k-1)-offset) within %.2f dB (worst %.2e dB over %ld blocks)",
          name, tol_db, worst, checked);
    if (first_d >= 0 && first_f >= 0)
        printf("    %s: driver first >0.1 dB in block %ld, follower in block %ld -> lag %ld block(s) = %.1f ms @ %d Hz\n",
               name, first_d, first_f, first_f - first_d, (first_f - first_d) * 1000.0 * BL / rate, rate);
    return worst <= tol_db;
}

static int cmd_follow(int argc, char **argv)
{
    /* follow TEST CFG RATE MODE RATIO OFFSET LEVEL_DB */
    if (argc < 9) return 2;
    const char *tn = argv[2];
    g_rate = atoi(argv[4]); int mode = parse_mode(argv[5]);
    double ratio = atof(argv[6]), offset = atof(argv[7]);
    sc_level_db = atof(argv[8]);
    g_scen = !strcmp(tn, "t4") ? SC_T4 : SC_T2;
    g_fs = 1.0;
    if (setup(argv[3], g_rate, 8) != 0) return 3;
    long nblk = (long)(3.0 * g_rate / BL);
    run(mode, nblk, trace_pub_hook, NULL);
    int fol = !strcmp(tn, "t4") ? ESLR : ALK;
    double maxred = 0;
    for (long k = 0; k < tr_n; k++) if (-db(pub_rec[ESLL][k]) > maxred) maxred = -db(pub_rec[ESLL][k]);
    printf("%s rate=%d mode=%s ratio=%.2f offset=%.1f: driver max reduction %.2f dB\n", tn, g_rate, argv[5], ratio, offset, maxred);
    if (maxred > offset) {
        check(maxred > offset + 1.0, "driver (esll) is really limiting beyond the dead zone (%.2f dB > offset+1)", maxred);
    } else {
        double minf = 1.0;
        for (long k = 0; k < tr_n; k++) if (tr_min[fol][k] < minf) minf = tr_min[fol][k];
        check(maxred > 0.5 && minf == 1.0, "driver limits %.2f dB, inside the %.1f dB dead zone -> follower untouched (min gain %.6f)", maxred, offset, minf);
    }
    follow_test(tn, ESLL, fol, ratio, offset, 1e-9, g_rate);
    /* followers never louder than input (own limits idle) */
    double maxg = 0;
    for (long k = 0; k < tr_n; k++) if (tr_end[fol][k] > maxg) maxg = tr_end[fol][k];
    check(maxg <= 1.0, "follower gain never > 1 (max %.6f)", maxg);
    check(total_faults() == 0, "link_faults = 0 (%ld)", total_faults());
    return finish();
}

/* T3: alkoven over its own limit, ESL quiet: esll identical with/without link */
static int cmd_t3(int argc, char **argv)
{
    /* t3 CFG_LINK CFG_NOLINK RATE */
    if (argc < 5) return 2;
    g_scen = SC_T3; g_rate = atoi(argv[4]); g_fs = 1.0;
    long nblk = (long)(2.0 * g_rate / BL);
    if (pipe(pipefd) != 0) return 2;
    pid_t pid = fork();
    if (pid == 0) {
        close(pipefd[0]);
        if (setup(argv[3], g_rate, 8) != 0) _exit(3);
        run(0, nblk, child_hook, NULL); _exit(0);
    }
    close(pipefd[1]);
    if (setup(argv[2], g_rate, 8) != 0) return 3;
    static struct t1a_rec ref; long diff_esl = 0, alk_red = 0;
    for (long k = 0; k < nblk; k++) {
        for (int ch = 0; ch < NCH; ch++) fill(ch, k);
        for (int ch = 0; ch < NCH; ch++) run_ch(ch);
        char *p = (char *)&ref; size_t left = sizeof(ref);
        while (left) { ssize_t r = read(pipefd[0], p, left); if (r <= 0) exit(9); p += r; left -= (size_t)r; }
        if (memcmp(ref.y[ESLL], yout[ESLL], sizeof(yout[ESLL])) != 0) diff_esl++;
        for (int i = 0; i < BL; i++) if (fabs(yout[ALK][i]) < fabs(xin[ALK][i])) { alk_red++; break; }
    }
    waitpid(pid, NULL, 0);
    printf("T3 rate=%d blocks=%ld\n", g_rate, nblk);
    check(alk_red > 0, "alkoven is limited by its own limit in %ld blocks", alk_red);
    check(diff_esl == 0, "esll bit-identical to the no-link run (%ld blocks differ) - alkoven does not drive", diff_esl);
    return finish();
}

/* T9: fault injection into esll's publish slot before block K */
static int inj_kind; static long inj_blk;
static void inject_hook(long blk, void *v)
{
    trace_pub_hook(blk, v);
    if (blk + 1 == inj_blk) {
        /* harness block b = module block k = b+1. Block inj_blk (k = inj_blk+1)
           reads slot (k-1)&1 = inj_blk&1 = (blk+1)&1, written just now */
        struct pub_slot *s = &pub[ESLL][(blk + 1) & 1];
        switch (inj_kind) {
        case 0: a_store_d(&s->gmin, NAN); break;
        case 1: a_store_d(&s->gmin, 0.0); break;
        case 2: a_store_d(&s->gmin, 2.0); break;
        case 3: a_store_l(&s->blk, blk - 5); a_store_d(&s->gmin, 0.5);
                a_store_d(&pub[ESLL][blk & 1].gmin, 0.25); break;
        }
    }
}
static int cmd_t9(int argc, char **argv)
{
    /* t9 CFG RATE */
    if (argc < 4) return 2;
    static const char *kinds[] = { "NaN", "0.0", "2.0", "wrong blk" };
    g_rate = atoi(argv[3]);
    int rc = 0;
    for (inj_kind = 0; inj_kind < 4; inj_kind++) {
        pid_t pid = fork();
        if (pid == 0) {
            g_scen = SC_T2; sc_level_db = 6.0; g_fs = 1.0;
            if (setup(argv[2], g_rate, 8) != 0) _exit(3);
            inj_blk = (long)(1.2 * g_rate / BL);   /* inside the burst */
            run(0, inj_blk + 4, inject_hook, NULL);
            long f = total_faults();
            double gend = tr_end[ALK][inj_blk];
            double gprev = tr_end[ALK][inj_blk - 1];
            int ok_f = f >= 1;
            int ok_inv = 1;
            for (long k = 0; k < tr_n; k++) if (tr_end[ALK][k] > 1.0 || tr_min[ALK][k] <= 0.0) ok_inv = 0;
            printf("T9 inject %-9s: faults=%ld alkoven gain block %ld end %.4f (prev %.4f)\n", kinds[inj_kind], f, inj_blk, gend, gprev);
            if (inj_kind == 3) {
                /* fallback = min of both slots. The other slot is the one the
                   partner writes in the current block, so it holds either
                   the injected 0.25 or (seq mode: esll runs before alkoven)
                   esll's new value -> expected min(0.5, that value) */
                double other = pub_rec[ESLL][inj_blk];
                double expv = other < 0.5 ? other : 0.5;
                check(ok_f && fabs(gend - expv) < 1e-12 && gend <= 0.5,
                      "inject %s: counted, conservative min of both slots used (%.4f = min(0.5 stale, %.4f current))",
                      kinds[inj_kind], gend, other);
            } else {
                /* partner ignored -> L_tgt = 1 (own limits only, which are idle) */
                check(ok_f && fabs(gend - 1.0) < 1e-12, "inject %s: counted, partner ignored, own limits only (gain %.6f)", kinds[inj_kind], gend);
            }
            check(ok_inv, "inject %s: follower gain stays in (0,1]", kinds[inj_kind]);
            _exit(n_fail ? 1 : 0);
        }
        int st; waitpid(pid, &st, 0);
        if (!WIFEXITED(st) || WEXITSTATUS(st) != 0) rc = 1;
    }
    return rc;
}

/* T10: lmc link off during the burst, on again later */
static long off_blk, on_blk;
static void lmc_hook(long blk, void *v)
{
    trace_pub_hook(blk, v);
    if (blk + 1 == off_blk) bflogic_command("link off alk_fill");
    if (blk + 1 == on_blk) bflogic_command("link on alk_fill");
}
static int cmd_t10(int argc, char **argv)
{
    /* t10 CFG RATE MODE */
    if (argc < 5) return 2;
    g_rate = atoi(argv[3]); int mode = parse_mode(argv[4]);
    g_scen = SC_T2; sc_level_db = 6.0; g_fs = 1.0;
    if (setup(argv[2], g_rate, 8) != 0) return 3;
    off_blk = (long)(1.3 * g_rate / BL); on_blk = (long)(1.6 * g_rate / BL);
    run(mode, (long)(2.5 * g_rate / BL), lmc_hook, NULL);
    printf("T10 rate=%d mode=%s off@%ld on@%ld\n", g_rate, argv[4], off_blk, on_blk);
    check(-db(tr_end[ALK][off_blk - 1]) > 1.0, "before off: alkoven follows (%.2f dB)", db(tr_end[ALK][off_blk - 1]));
    int imm = 1;
    for (int k = off_blk; k < on_blk; k++) if (tr_min[ALK][k] != 1.0) imm = 0;
    check(imm, "link off: from the very next block only own limits (gain exactly 1.0 in blocks %ld..%ld)", off_blk, on_blk - 1);
    double exp_end = pub_rec[ESLL][on_blk - 1];
    double step = (1.0 - exp_end) / BL;
    check(fabs(tr_first[ALK][on_blk] - (1.0 - step)) < 1e-12 && fabs(tr_end[ALK][on_blk] - exp_end) < 1e-12,
          "link on: ramp starts at 1.0 (first sample %.6f, end %.6f = partner %.6f), no jump", tr_first[ALK][on_blk], tr_end[ALK][on_blk], exp_end);
    bflogic_command("link list");
    printf("%s", bflogic_message());
    return finish();
}

/* T11: release ramp - no gain jumps at block boundaries */
static int cmd_t11(int argc, char **argv)
{
    if (argc < 4) return 2;
    g_rate = atoi(argv[3]); g_scen = SC_T11; g_fs = 1.0;
    if (setup(argv[2], g_rate, 8) != 0) return 3;
    run(0, (long)(4.0 * g_rate / BL), trace_pub_hook, NULL);
    double worst_ratio = 0, max_b = 0, minred = 0;
    for (long k = 1; k < tr_n; k++) {
        double inner = tr_maxstep[ALK][k];
        double lstep = fabs(tr_end[ALK][k] - tr_end[ALK][k - 1]) / BL;   /* expected ramp step */
        double b = tr_bstep[ALK][k];
        if (b > max_b) max_b = b;
        if (lstep > 0 && b > lstep * (1 + 1e-9) + 1e-15) { double r = b / lstep; if (r > worst_ratio) worst_ratio = r; }
        if (inner > lstep * (1 + 1e-9) + 1e-15) { double r = inner / lstep; if (r > worst_ratio) worst_ratio = r; }
        if (-db(tr_end[ALK][k]) > minred) minred = -db(tr_end[ALK][k]);
    }
    printf("T11 rate=%d: alkoven max reduction %.2f dB, max per-sample step at block boundary %.3e\n", g_rate, minred, max_b);
    check(minred > 6.0, "deep reduction reached (%.2f dB)", minred);
    check(worst_ratio == 0, "every per-sample step (incl. block boundaries) <= |dL|/block_length (linear ramp, no stairs)");
    return finish();
}

/* ------------------------------------------- T8: hot path + timing */
#ifdef HOTPATH_WRAP
/* linked with -Wl,--wrap=<sym> for each of these and _FORTIFY_SOURCE=0:
   every call from the module into these libc functions is counted while
   output_timed() runs */
static long hot_calls;
void *__real_malloc(size_t); void *__wrap_malloc(size_t n) { if (hot) __atomic_add_fetch(&hot_calls, 1, __ATOMIC_RELAXED); return __real_malloc(n); }
void *__real_calloc(size_t, size_t); void *__wrap_calloc(size_t a, size_t b) { if (hot) __atomic_add_fetch(&hot_calls, 1, __ATOMIC_RELAXED); return __real_calloc(a, b); }
void *__real_realloc(void *, size_t); void *__wrap_realloc(void *p, size_t n) { if (hot) __atomic_add_fetch(&hot_calls, 1, __ATOMIC_RELAXED); return __real_realloc(p, n); }
void __real_free(void *); void __wrap_free(void *p) { if (hot) __atomic_add_fetch(&hot_calls, 1, __ATOMIC_RELAXED); __real_free(p); }
int __real_vfprintf(FILE *, const char *, va_list); int __wrap_vfprintf(FILE *f, const char *fm, va_list ap) { if (hot) __atomic_add_fetch(&hot_calls, 1, __ATOMIC_RELAXED); return __real_vfprintf(f, fm, ap); }
size_t __real_fwrite(const void *, size_t, size_t, FILE *); size_t __wrap_fwrite(const void *p, size_t a, size_t b, FILE *f) { if (hot) __atomic_add_fetch(&hot_calls, 1, __ATOMIC_RELAXED); return __real_fwrite(p, a, b, f); }
int __real_fputs(const char *, FILE *); int __wrap_fputs(const char *s, FILE *f) { if (hot) __atomic_add_fetch(&hot_calls, 1, __ATOMIC_RELAXED); return __real_fputs(s, f); }
int __real_fputc(int, FILE *); int __wrap_fputc(int c, FILE *f) { if (hot) __atomic_add_fetch(&hot_calls, 1, __ATOMIC_RELAXED); return __real_fputc(c, f); }
ssize_t __real_write(int, const void *, size_t);
ssize_t __wrap_write(int fd, const void *p, size_t n) { if (hot) __atomic_add_fetch(&hot_calls, 1, __ATOMIC_RELAXED); return __real_write(fd, p, n); }
int __real_vsnprintf(char *, size_t, const char *, va_list); int __wrap_vsnprintf(char *s, size_t n, const char *fm, va_list ap) { if (hot) __atomic_add_fetch(&hot_calls, 1, __ATOMIC_RELAXED); return __real_vsnprintf(s, n, fm, ap); }
int __wrap_fprintf(FILE *f, const char *fm, ...) { if (hot) __atomic_add_fetch(&hot_calls, 1, __ATOMIC_RELAXED); va_list ap; va_start(ap, fm); int r = __real_vfprintf(f, fm, ap); va_end(ap); return r; }
int __wrap_snprintf(char *s, size_t n, const char *fm, ...) { if (hot) __atomic_add_fetch(&hot_calls, 1, __ATOMIC_RELAXED); va_list ap; va_start(ap, fm); int r = __real_vsnprintf(s, n, fm, ap); va_end(ap); return r; }

static double t_link_ns, t_plain_ns; static long t_link_n, t_plain_n;
static double t_ch_ns[NCH];
static int cmd_t8(int argc, char **argv)
{
    /* t8 CFG RATE NBLK : seq, timing per output_timed call, hot-path calls */
    if (argc < 5) return 2;
    g_rate = atoi(argv[3]); long nblk = atol(argv[4]);
    g_scen = SC_RANDOM; g_fs = 2147483647.0;
    if (setup(argv[2], g_rate, 8) != 0) return 3;
    for (long k = 0; k < nblk; k++) {
        for (int ch = 0; ch < NCH; ch++) fill(ch, k);
        for (int ch = 0; ch < NCH; ch++) {
            struct timespec a, b;
            memcpy(yout[ch], xin[ch], sizeof(yout[ch]));
            clock_gettime(CLOCK_MONOTONIC, &a);
            hot = 1;
            output_timed(yout[ch], ch);
            hot = 0;
            clock_gettime(CLOCK_MONOTONIC, &b);
            double ns = (b.tv_sec - a.tv_sec) * 1e9 + (b.tv_nsec - a.tv_nsec);
            t_ch_ns[ch] += ns;
        }
    }
    double blk_ms = 1000.0 * BL / g_rate;
    printf("T8 %s rate=%d blocks=%ld: output_timed mean per channel and block:", argv[2], g_rate, nblk);
    for (int ch = 0; ch < NCH; ch++) printf(" %s=%.2fus", out_names[ch], t_ch_ns[ch] / nblk / 1e3);
    printf(" (block time %.2f ms)\n", blk_ms);
    for (int ch = 0; ch < NCH; ch++) printf("T8TIME %s %.4f\n", out_names[ch], t_ch_ns[ch] / nblk / 1e3);
    check(hot_calls == 0, "no malloc/calloc/realloc/free/printf-family/fwrite/write inside output_timed (%ld calls)", hot_calls);
    (void)t_link_ns; (void)t_plain_ns; (void)t_link_n; (void)t_plain_n;
    return finish();
}
#endif /* HOTPATH_WRAP */
#endif /* !REFERENCE_BUILD */

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IOLBF, 0);
    if (argc < 2) { fprintf(stderr, "usage: harness <cmd> ...\n"); return 2; }
    if (!strcmp(argv[1], "preinit")) {
        if (argc < 3) return 2;
        int rc = setup(argv[2], 48000, 8);
        printf("preinit rc=%d\n", rc);
        return rc == 0 ? 0 : 3;
    }
    if (!strcmp(argv[1], "run")) return cmd_run(argc, argv);
    if (!strcmp(argv[1], "lmc")) return cmd_lmc(argc, argv);
#ifndef REFERENCE_BUILD
    if (!strcmp(argv[1], "t1a")) return cmd_t1a(argc, argv);
    if (!strcmp(argv[1], "follow")) return cmd_follow(argc, argv);
    if (!strcmp(argv[1], "t3")) return cmd_t3(argc, argv);
    if (!strcmp(argv[1], "t9")) return cmd_t9(argc, argv);
    if (!strcmp(argv[1], "t10")) return cmd_t10(argc, argv);
    if (!strcmp(argv[1], "t11")) return cmd_t11(argc, argv);
#ifdef HOTPATH_WRAP
    if (!strcmp(argv[1], "t8")) return cmd_t8(argc, argv);
#endif
#endif
    fprintf(stderr, "unknown command %s\n", argv[1]);
    return 2;
}
