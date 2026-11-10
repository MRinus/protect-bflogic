/*
 * bflogic_protect.c  (v3 -- generalized signal-chain model, + channel link)
 *
 * CHANNEL LINK (dev-2, 08.11.2026, design "protect Channel-Link Design
 * (08.11.2026)", architect 8a51a27, stage 1):
 *
 *   link: "<name>" { channels: "a", "b", ...;     2..MAX_CH_PER_LINK
 *                    [drive: "a", ...;]           subset, default: all
 *                    [ratio: r_a, r_b, ...;]      0..1 per member, default 1
 *                    [offset_db: o_a, ...;]       0..40 per member, default 0
 *                    [enabled: 1;] };
 *
 * Every member keeps its own limits exactly as before. Additionally it
 * follows the reduction of the driving partners of its link:
 *   follow_dB = ratio_me * max(0, A_partner_dB - offset_me),
 *   applied gain = min(own combined gain, 10^(-max follow_dB / 20)).
 * A link therefore never raises a gain (invariant |y_link| <= |y_nolink|
 * per sample, test T1a). The partner value is the minimum OWN gain of the
 * partner's PREVIOUS block, published lock-free in two parity slots per
 * channel; BruteFIR's per-block barrier between filter threads
 * (bfrun.c synch_filter_processes) orders the write before the read. The
 * follower ramps linearly from its previous link gain to the new one over
 * the block (no steps at block boundaries). Without a link block the
 * processing path is the unchanged v3 code (test T1b, bit-identical).
 * Config errors (unknown/duplicate names, member without group, channel in
 * two groups or two links, ratio/offset out of range, ...) make preinit
 * return -1 -> BruteFIR exits with BF_EXIT_INVALID_CONFIG (fail-safe).
 *
 * Ersetzt die fest vorgerechneten dBFS-Schwellen aus v1 durch eine
 * deklarative Beschreibung der realen Signalkette. Nichts wird mehr
 * von Hand in dBFS umgerechnet -- das Modul tut das selbst, aus:
 *
 *   chain  <name> { dac_max_output_vrms: ...; stage { gain_db: ...; }; ... }
 *   group  { channels: ...; chain: "<name>";
 *            limit { name: ...; max_vrms: ...; type: "peak"|"rms";
 *                    [ref_freq_hz: ...; slope_db_oct: ...;]
 *                    attack_ms/release_ms/window_ms/rms_release_ms: ...; };
 *            ... beliebig viele limit-Bloecke ...
 *          };
 *
 * KONZEPT
 * -------
 * Eine "chain" beschreibt den Signalweg vom DAC-Vollausschlag (0dBFS)
 * bis zu einem beliebigen Punkt in der Kette: DAC-Referenzspannung
 * mal Produkt aller stage-gains (in dB, koennen negativ sein fuer
 * Verluste/Adapter, positiv fuer Verstaerker-Gain).
 *
 * Eine "group" haengt sich an genau eine chain und definiert
 * beliebig viele "limit"-Checkpoints. Jeder Checkpoint hat eine
 * physikalische Grenzspannung (max_vrms) an SEINEM Punkt in der
 * Kette -- das Modul rechnet daraus selbst die noetige normierte
 * (0dBFS=1.0) Schwelle: threshold_lin = max_vrms / (dac_max_output_vrms
 * * chain_gain_lin). Kein Handrechnen mehr noetig, auch nicht bei
 * Aenderung von Kabeln/Adaptern/Amps -- nur die chain-Parameter
 * anpassen.
 *
 * Jeder Checkpoint kann optional frequenzgewichtet sein
 * (ref_freq_hz + slope_db_oct): fuer physikalisch frequenzabhaengige
 * Grenzen wie Trafo-Kernsaettigung (Grenzspannung steigt ~6dB/Oktave
 * mit der Frequenz, 1-polig) oder Woofer-Xmax in einer geschlossenen
 * Box (Grenzspannung steigt ~12dB/Oktave oberhalb der Boxreso, 2-polig).
 * Umgesetzt als Kaskade von Einpol-Tiefpaessen (gleiche Eckfrequenz
 * ref_freq_hz, N = slope_db_oct/6 Stufen hintereinander) auf den
 * Detektor-Eingang dieses einen Checkpoints -- andere Checkpoints in
 * derselben Gruppe bleiben davon unberuehrt. Unterstuetzt aktuell 1
 * oder 2 Stufen (6 oder 12 dB/Oktave); mehr waere per Kaskadierung
 * trivial erweiterbar, aber bisher nicht gebraucht.
 *
 * Ein Limit kann mit "stage: N;" (1-indiziert) sagen, dass es nur an
 * den ersten N Stufen der Chain gemessen ist, nicht an der vollen
 * Chain -- noetig, wenn mehrere Limits derselben Gruppe an
 * unterschiedlichen Punkten derselben Kaskade sitzen (z.B. Trafo-
 * Saettigung vor einer Endstufe, Amp-Rated-Power dahinter). Fehlt
 * "stage", gilt die komplette Chain -- das Feld ueberschreibt die
 * Referenz, es addiert sich NICHT zur Chain-Gesamtverstaerkung dazu.
 *
 * Alle Checkpoints (peak wie rms, ueber alle limits einer Gruppe)
 * laufen parallel; die tatsaechlich angewandte Verstaerkung ist das
 * Minimum aller Einzel-Gains -- der jeweils strengste Checkpoint
 * gewinnt automatisch, ganz ohne Prioritaeten von Hand festzulegen.
 *
 * Kompilieren:
 *   gcc -O2 -fPIC -shared -DIS_BFLOGIC_MODULE \
 *       -I<brutefir-source-dir> -o protect.bflogic bflogic_protect.c -lm
 */
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

/* glibc hides M_PI under strict -std=c99 -D_POSIX_C_SOURCE (no
   _DEFAULT_SOURCE/_GNU_SOURCE) -- exactly the flags this project's
   Makefile uses, so this isn't optional. */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define IS_BFLOGIC_MODULE
#include "bfmod.h"

#define MAX_CHAINS 8
#define MAX_STAGES_PER_CHAIN 8
#define MAX_GROUPS 8
#define MAX_CH_PER_GROUP 4
#define MAX_LIMITS_PER_GROUP 8

#define LIMIT_PEAK 0
#define LIMIT_RMS  1

/* ---- statische Konfiguration ------------------------------------------ */

struct chain {
    char   name[BF_MAXOBJECTNAME];
    double dac_max_output_vrms;
    double gain_lin;              /* Produkt ALLER stage-gains, linear */
    double stage_gain_db[MAX_STAGES_PER_CHAIN];  /* einzelne Stufen, der Reihe nach */
    int    n_stages;
};

#define MAX_SHELF_POLES 2   /* 6 oder 12 dB/Oktave; siehe Kommentar oben */

struct limit_cfg {
    char   name[BF_MAXOBJECTNAME];
    int    type;                  /* LIMIT_PEAK oder LIMIT_RMS */
    double threshold_lin;         /* aus max_vrms + chain berechnet */
    double attack_coeff;
    double release_coeff;
    double window_samples;        /* nur RMS */
    /* Frequenzgewichtung (optional, nur PEAK sinnvoll) */
    int    freq_weighted;
    double shelf_coeff;           /* Einpol-Tiefpass-Koeffizient, pro Kaskadenstufe gleich */
    int    shelf_poles;           /* 1 oder 2 kaskadierte Stufen -> 6 oder 12 dB/Oktave */
    /* 0 = volle Chain-Verstaerkung; sonst: nur die ersten `stage`
       Stufen der Chain summieren (1-indiziert), s.o. */
    int    stage;
    /* v3: Gewichtung VOR dem Betrag (1) statt dahinter (0, Altverhalten).
       Nur mit freq_weighted wirksam. Siehe Kommentar in process_channel(). */
    int    signed_weight;
    /* v3: zur Laufzeit per "lmc <idx> on|off <name>" schaltbar. 0 = der
       Checkpoint wird uebersprungen (Gain 1.0). always_on = 1 verbietet
       das Ausschalten (Verstaerker-Clip, Thermik). */
    volatile int enabled;
    int    always_on;
};

struct group {
    char   ch_names[MAX_CH_PER_GROUP][BF_MAXOBJECTNAME];
    int    ch_intname[MAX_CH_PER_GROUP];
    int    n_channels;
    int    chain_index;
    int    n_limits;
    struct limit_cfg limits[MAX_LIMITS_PER_GROUP];
};

/* ---- Laufzeit-Zustand pro Kanal und Checkpoint ------------------------- */

struct limit_state {
    double gain;        /* geglaettete aktuelle Verstaerkung dieses Checkpoints */
    double rms_sumsq;    /* nur RMS-Typ */
    double shelf_z1;      /* Filterzustand Kaskadenstufe 1, nur wenn freq_weighted */
    double shelf_z2;      /* Filterzustand Kaskadenstufe 2, nur wenn shelf_poles==2 */
    long   overs;
};

struct channel_runtime {
    struct limit_state st[MAX_LIMITS_PER_GROUP];
};

static struct chain chains[MAX_CHAINS];
static int n_chains = 0;
static struct group groups[MAX_GROUPS];
static int n_groups = 0;
static struct channel_runtime ch_runtime[MAX_GROUPS][MAX_CH_PER_GROUP];

/* CRITICAL, empirically verified (2026-08-22): for an integer output
   sample format (e.g. S32_LE, as used by every real sqnt440* config),
   the double/float buffer output_timed() receives is NOT normalised to
   -1.0..+1.0 -- it carries the raw integer-equivalent magnitude (e.g.
   +-2147483647 for S32_LE), despite brutefir.html's general claim that
   "all integer formats will be scaled to -1.0 to +1.0" (that apparently
   applies elsewhere in the pipeline, not to what this specific hook
   sees). Confirmed by forcing a known 0dBFS input through a real
   correction filter and reading the actual buffer values: a signal
   with Cor1's own ~-4.2dB gain at 20Hz settled at a peak of
   ~1.29e9 -- matching 2147483647*10^(-4.2/20) almost exactly, not
   anything near 1.0. Without this scale, EVERY threshold_lin (computed
   in Vrms-normalised 0..1 terms) is instantly and permanently exceeded
   by any real signal, collapsing gain to ~0 regardless of actual level.
   "output_scale" must match whatever raw full-scale magnitude the real
   output's sample format uses -- 2147483647.0 for S32_LE (the only
   format used anywhere in this project's live configs). Configurable
   via an optional top-level "output_scale: <real>;" field so this
   isn't silently wrong if a config ever uses a different sample
   format. */
static double output_scale = 2147483647.0;

static int sample_rate;
static int block_length;
static int realsize;
/* Sized for the theoretical worst case (MAX_GROUPS * MAX_CH_PER_GROUP *
   MAX_LIMITS_PER_GROUP = 8*4*8 = 256 limit-states), not just the current
   live config's 14 -- the "json"/"all" bflogic_command() output modes
   below are one line/object per limit-state, and bflogic_command() used
   to write into a bare 1024-byte buffer with unchecked sprintf(), which
   would have silently overflowed on a larger config even before these
   modes existed. All writes below go through snprintf() with remaining-
   space tracking, so this is a hard ceiling, not just headroom. */
#define MSG_BUF_SIZE 65536
static char msg[MSG_BUF_SIZE];

#define MAX_MAPPED_CHANNELS 64
static int chan_group[MAX_MAPPED_CHANNELS];
static int chan_idx[MAX_MAPPED_CHANNELS];

/* ---- Channel link (stage 1) -------------------------------------------- */

#define MAX_LINKS        4
#define MAX_CH_PER_LINK  8
#define LINK_RATIO_MAX   1.0
#define LINK_OFFSET_MAX  40.0

struct link_cfg {                          /* written in preinit only */
    char   name[BF_MAXOBJECTNAME];
    int    n;                              /* members */
    char   member_name[MAX_CH_PER_LINK][BF_MAXOBJECTNAME];
    int    member_ch[MAX_CH_PER_LINK];     /* BruteFIR intname */
    int    n_drive;                        /* 0 = field absent -> all drive */
    char   drive_name[MAX_CH_PER_LINK][BF_MAXOBJECTNAME];
    int    drives[MAX_CH_PER_LINK];        /* 0/1, resolved */
    int    n_ratio, n_offset;              /* 0 = field absent -> defaults */
    double ratio[MAX_CH_PER_LINK];         /* [0,1] */
    double offset_db[MAX_CH_PER_LINK];     /* [0,40] */
    int    enabled;                        /* atomic access only (lmc thread) */
};

struct pub_slot {                          /* 2 per channel (block parity) */
    double gmin;                           /* minimum OWN gain in the block */
    long   blk;                            /* block number of the writer */
} __attribute__((aligned(64)));            /* one cache line each, no false sharing */

static struct link_cfg links[MAX_LINKS];
static int    n_links = 0;
static int    chan_link[MAX_MAPPED_CHANNELS];     /* -1 = in no link */
static int    chan_member[MAX_MAPPED_CHANNELS];   /* index within the link */
static struct pub_slot pub[MAX_MAPPED_CHANNELS][2];
static long   blk_ctr[MAX_MAPPED_CHANNELS];       /* own filter thread only */
static double link_prev[MAX_MAPPED_CHANNELS];     /* link gain at block end */
static double link_now[MAX_MAPPED_CHANNELS];      /* display copy (atomic) */
static long   link_faults[MAX_MAPPED_CHANNELS];   /* atomic, own thread writes */

/* Relaxed atomics: the ordering comes from BruteFIR's per-block barrier
   (design 2.4); atomic access only keeps the compiler from tearing or
   caching the values and documents the cross-thread accesses (TSan). */
static inline double a_load_d(const double *p)
{ double v; __atomic_load(p, &v, __ATOMIC_RELAXED); return v; }
static inline void a_store_d(double *p, double v)
{ __atomic_store(p, &v, __ATOMIC_RELAXED); }
static inline long a_load_l(const long *p)
{ return __atomic_load_n(p, __ATOMIC_RELAXED); }
static inline void a_store_l(long *p, long v)
{ __atomic_store_n(p, v, __ATOMIC_RELAXED); }
static inline int a_load_i(const int *p)
{ return __atomic_load_n(p, __ATOMIC_RELAXED); }
static inline void a_store_i(int *p, int v)
{ __atomic_store_n(p, v, __ATOMIC_RELAXED); }

static inline double db2lin(double db) { return pow(10.0, db / 20.0); }

/* ---- Config-Parsing ----------------------------------------------------- */

#define GET_TOKEN(tok, errstr)                                         \
    if (get_config_token(&lexval) != (tok)) {                          \
        fprintf(stderr, "PROTECT: Parse error: " errstr);              \
        return -1;                                                     \
    }

static int
parse_chain(int (*get_config_token)(union bflexval *lexval),
            struct chain *c, const char *name)
{
    union bflexval lexval;
    int token;

    memset(c, 0, sizeof(*c));
    strncpy(c->name, name, BF_MAXOBJECTNAME - 1);
    c->gain_lin = 1.0;

    while ((token = get_config_token(&lexval)) > 0) {
        if (token == BF_LEX_RBRACE) break;
        if (token != BF_LEXVAL_FIELD) {
            if (token == BF_LEX_LBRACE) {
                /* stage { gain_db: X; } */
                double gain_db = 0.0;
                while ((token = get_config_token(&lexval)) > 0) {
                    if (token == BF_LEX_RBRACE) break;
                    if (token != BF_LEXVAL_FIELD ||
                        strcmp(lexval.field, "gain_db") != 0)
                    {
                        fprintf(stderr, "PROTECT: chain: expected "
                                "'gain_db' in stage.\n");
                        return -1;
                    }
                    GET_TOKEN(BF_LEXVAL_REAL, "expected real.\n");
                    gain_db = lexval.real;
                    GET_TOKEN(BF_LEX_EOS, "expected ';'.\n");
                }
                GET_TOKEN(BF_LEX_EOS, "expected ';' after stage block.\n");
                if (c->n_stages == MAX_STAGES_PER_CHAIN) {
                    fprintf(stderr, "PROTECT: chain: too many stages "
                            "(max %d).\n", MAX_STAGES_PER_CHAIN);
                    return -1;
                }
                c->stage_gain_db[c->n_stages++] = gain_db;
                c->gain_lin *= db2lin(gain_db);
                continue;
            }
            fprintf(stderr, "PROTECT: chain: expected field or stage.\n");
            return -1;
        }
        if (strcmp(lexval.field, "dac_max_output_vrms") == 0) {
            GET_TOKEN(BF_LEXVAL_REAL, "expected real.\n");
            c->dac_max_output_vrms = lexval.real;
            GET_TOKEN(BF_LEX_EOS, "expected ';'.\n");
        } else {
            fprintf(stderr, "PROTECT: chain: unknown field '%s'.\n",
                    lexval.field);
            return -1;
        }
    }
    if (c->dac_max_output_vrms <= 0.0) {
        fprintf(stderr, "PROTECT: chain '%s': dac_max_output_vrms not "
                "set.\n", name);
        return -1;
    }
    return 0;
}

static int
parse_limit(int (*get_config_token)(union bflexval *lexval),
            struct limit_cfg *l)
{
    union bflexval lexval;
    int token;
    double max_vrms = -1.0;
    double attack_ms = 1.0, release_ms = 200.0;
    double window_ms = 500.0, rms_release_ms = 2000.0;
    double ref_freq_hz = 0.0, slope_db_oct = 0.0;
    int stage = 0;
    char type_str[32] = "peak";
    char weighting_str[16] = "abs";
    int enabled = 1, always_on = 0;

    memset(l, 0, sizeof(*l));

    while ((token = get_config_token(&lexval)) > 0) {
        if (token == BF_LEX_RBRACE) break;
        if (token != BF_LEXVAL_FIELD) {
            fprintf(stderr, "PROTECT: limit: expected field.\n");
            return -1;
        }
        if (strcmp(lexval.field, "name") == 0) {
            GET_TOKEN(BF_LEXVAL_STRING, "expected string.\n");
            strncpy(l->name, lexval.string, BF_MAXOBJECTNAME - 1);
            GET_TOKEN(BF_LEX_EOS, "expected ';'.\n");
        } else if (strcmp(lexval.field, "type") == 0) {
            GET_TOKEN(BF_LEXVAL_STRING, "expected string.\n");
            strncpy(type_str, lexval.string, sizeof(type_str) - 1);
            GET_TOKEN(BF_LEX_EOS, "expected ';'.\n");
        } else if (strcmp(lexval.field, "max_vrms") == 0) {
            GET_TOKEN(BF_LEXVAL_REAL, "expected real.\n");
            max_vrms = lexval.real;
            GET_TOKEN(BF_LEX_EOS, "expected ';'.\n");
        } else if (strcmp(lexval.field, "ref_freq_hz") == 0) {
            GET_TOKEN(BF_LEXVAL_REAL, "expected real.\n");
            ref_freq_hz = lexval.real;
            GET_TOKEN(BF_LEX_EOS, "expected ';'.\n");
        } else if (strcmp(lexval.field, "slope_db_oct") == 0) {
            GET_TOKEN(BF_LEXVAL_REAL, "expected real.\n");
            slope_db_oct = lexval.real;
            GET_TOKEN(BF_LEX_EOS, "expected ';'.\n");
        } else if (strcmp(lexval.field, "attack_ms") == 0) {
            GET_TOKEN(BF_LEXVAL_REAL, "expected real.\n");
            attack_ms = lexval.real;
            GET_TOKEN(BF_LEX_EOS, "expected ';'.\n");
        } else if (strcmp(lexval.field, "release_ms") == 0) {
            GET_TOKEN(BF_LEXVAL_REAL, "expected real.\n");
            release_ms = lexval.real;
            GET_TOKEN(BF_LEX_EOS, "expected ';'.\n");
        } else if (strcmp(lexval.field, "window_ms") == 0) {
            GET_TOKEN(BF_LEXVAL_REAL, "expected real.\n");
            window_ms = lexval.real;
            GET_TOKEN(BF_LEX_EOS, "expected ';'.\n");
        } else if (strcmp(lexval.field, "rms_release_ms") == 0) {
            GET_TOKEN(BF_LEXVAL_REAL, "expected real.\n");
            rms_release_ms = lexval.real;
            GET_TOKEN(BF_LEX_EOS, "expected ';'.\n");
        } else if (strcmp(lexval.field, "stage") == 0) {
            GET_TOKEN(BF_LEXVAL_REAL, "expected real.\n");
            stage = (int)(lexval.real + 0.5);
            GET_TOKEN(BF_LEX_EOS, "expected ';'.\n");
        } else if (strcmp(lexval.field, "weighting") == 0) {
            /* "abs"    = Altverhalten: erst Betrag, dann Tiefpaesse
               "signed" = erst Tiefpaesse auf das vorzeichenbehaftete
                          Signal, dann Betrag (v3, physikalisch korrekt) */
            GET_TOKEN(BF_LEXVAL_STRING, "expected string.\n");
            strncpy(weighting_str, lexval.string, sizeof(weighting_str) - 1);
            GET_TOKEN(BF_LEX_EOS, "expected ';'.\n");
        } else if (strcmp(lexval.field, "enabled") == 0) {
            GET_TOKEN(BF_LEXVAL_REAL, "expected real.\n");
            enabled = (lexval.real != 0.0);
            GET_TOKEN(BF_LEX_EOS, "expected ';'.\n");
        } else if (strcmp(lexval.field, "always_on") == 0) {
            GET_TOKEN(BF_LEXVAL_REAL, "expected real.\n");
            always_on = (lexval.real != 0.0);
            GET_TOKEN(BF_LEX_EOS, "expected ';'.\n");
        } else {
            fprintf(stderr, "PROTECT: limit: unknown field '%s'.\n",
                    lexval.field);
            return -1;
        }
    }
    if (max_vrms <= 0.0) {
        fprintf(stderr, "PROTECT: limit '%s': max_vrms not set.\n",
                l->name);
        return -1;
    }
    if (stage < 0) {
        fprintf(stderr, "PROTECT: limit '%s': stage must be >= 1 (or "
                "omitted for the full chain), got %d.\n", l->name, stage);
        return -1;
    }
    if (strcmp(weighting_str, "abs") != 0 &&
        strcmp(weighting_str, "signed") != 0)
    {
        fprintf(stderr, "PROTECT: limit '%s': weighting must be \"abs\" or "
                "\"signed\", got '%s'.\n", l->name, weighting_str);
        return -1;
    }
    l->signed_weight = (strcmp(weighting_str, "signed") == 0);
    l->enabled = enabled;
    l->always_on = always_on;
    if (always_on && !enabled) {
        fprintf(stderr, "PROTECT: limit '%s': always_on with enabled: 0 "
                "makes no sense.\n", l->name);
        return -1;
    }
    l->stage = stage;
    l->type = (strcmp(type_str, "rms") == 0) ? LIMIT_RMS : LIMIT_PEAK;
    /* threshold_lin wird erst spaeter (nach Aufloesung der chain)
       final gesetzt -- hier zwischenspeichern in max_vrms-Feld via
       Missbrauch von threshold_lin als Rohwert: */
    l->threshold_lin = max_vrms; /* vorlaeufig: Rohspannung, s.u. */
    l->attack_coeff  = exp(-1.0 / (sample_rate * (attack_ms  / 1000.0)));
    l->release_coeff = exp(-1.0 / (sample_rate * (release_ms / 1000.0)));
    l->window_samples = sample_rate * (window_ms / 1000.0);
    /* rms_release_coeff wird im release_coeff-Feld gefuehrt, wenn RMS: */
    if (l->type == LIMIT_RMS) {
        l->release_coeff =
            exp(-1.0 / (sample_rate * (rms_release_ms / 1000.0)));
    }
    if (ref_freq_hz > 0.0 && slope_db_oct != 0.0) {
        int poles = (int)(slope_db_oct / 6.0 + 0.5);
        if (poles < 1 || poles > MAX_SHELF_POLES ||
            fabs(slope_db_oct - poles * 6.0) > 0.01)
        {
            fprintf(stderr, "PROTECT: limit '%s': slope_db_oct %.2f not "
                    "supported -- only exact multiples of 6 dB/octave up "
                    "to %d poles (%.0f dB/oct) are implemented, each pole "
                    "being one cascaded one-pole shelf stage.\n",
                    l->name, slope_db_oct, MAX_SHELF_POLES,
                    MAX_SHELF_POLES * 6.0);
            return -1;
        }
        l->freq_weighted = 1;
        l->shelf_poles = poles;
        /* Kaskade von `poles` Einpol-Tiefpaessen, jeweils gleiche
           Eckfrequenz = ref_freq_hz, um im Detektorpfad die physikalisch
           mit der Frequenz steigende Grenze nachzubilden. 1 Stufe = 6dB/
           Oktave (z.B. Trafo-Kernsaettigung), 2 Stufen = 12dB/Oktave
           (z.B. Xmax einer geschlossenen Box unterhalb/oberhalb Fc). */
        l->shelf_coeff = exp(-2.0 * M_PI * ref_freq_hz / sample_rate);
    }
    if (l->signed_weight && !l->freq_weighted) {
        fprintf(stderr, "PROTECT: limit '%s': weighting \"signed\" needs "
                "ref_freq_hz and slope_db_oct.\n", l->name);
        return -1;
    }
    return 0;
}

static int
parse_group(int (*get_config_token)(union bflexval *lexval),
            struct group *g)
{
    union bflexval lexval;
    int token;
    char chain_name[BF_MAXOBJECTNAME] = "";

    memset(g, 0, sizeof(*g));
    g->chain_index = -1;

    while ((token = get_config_token(&lexval)) > 0) {
        if (token == BF_LEX_RBRACE) break;
        if (token != BF_LEXVAL_FIELD) {
            fprintf(stderr, "PROTECT: group: expected field.\n");
            return -1;
        }
        if (strcmp(lexval.field, "channels") == 0) {
            token = BF_LEX_COMMA;
            for (g->n_channels = 0;
                 g->n_channels < MAX_CH_PER_GROUP && token == BF_LEX_COMMA;
                 g->n_channels++)
            {
                GET_TOKEN(BF_LEXVAL_STRING, "expected channel name.\n");
                strncpy(g->ch_names[g->n_channels], lexval.string,
                        BF_MAXOBJECTNAME - 1);
                token = get_config_token(&lexval);
            }
            if (token != BF_LEX_EOS) {
                fprintf(stderr, "PROTECT: group: expected ';'.\n");
                return -1;
            }
        } else if (strcmp(lexval.field, "chain") == 0) {
            GET_TOKEN(BF_LEXVAL_STRING, "expected chain name string.\n");
            strncpy(chain_name, lexval.string, BF_MAXOBJECTNAME - 1);
            GET_TOKEN(BF_LEX_EOS, "expected ';'.\n");
        } else if (strcmp(lexval.field, "limit") == 0) {
            GET_TOKEN(BF_LEX_LBRACE, "expected '{' after 'limit'.\n");
            if (g->n_limits == MAX_LIMITS_PER_GROUP) {
                fprintf(stderr, "PROTECT: too many limits in group.\n");
                return -1;
            }
            if (parse_limit(get_config_token, &g->limits[g->n_limits]) != 0)
            {
                return -1;
            }
            GET_TOKEN(BF_LEX_EOS, "expected ';' after limit block.\n");
            g->n_limits++;
        } else {
            fprintf(stderr, "PROTECT: group: unknown field '%s'.\n",
                    lexval.field);
            return -1;
        }
    }
    if (g->n_channels == 0) {
        fprintf(stderr, "PROTECT: group has no channels.\n");
        return -1;
    }
    if (chain_name[0] == '\0') {
        fprintf(stderr, "PROTECT: group has no chain reference.\n");
        return -1;
    }
    for (int i = 0; i < n_chains; i++) {
        if (strcmp(chains[i].name, chain_name) == 0) {
            g->chain_index = i;
            break;
        }
    }
    if (g->chain_index == -1) {
        fprintf(stderr, "PROTECT: unknown chain '%s'.\n", chain_name);
        return -1;
    }
    if (g->n_limits == 0) {
        fprintf(stderr, "PROTECT: group has no limit checkpoints -- "
                "waere wirkungslos.\n");
        return -1;
    }
    /* Jetzt die eigentliche normierte Schwelle aus max_vrms (bisher im
       threshold_lin-Feld zwischengespeichert) + chain-Gain berechnen.
       Pro Limit: "stage" (falls gesetzt) ueberschreibt, an wie vielen
       Stufen der Chain gemessen wird -- kein Aufaddieren zur vollen
       Chain-Verstaerkung, ein reiner Ersatzwert dafuer. */
    {
        struct chain *c = &chains[g->chain_index];
        for (int l = 0; l < g->n_limits; l++) {
            struct limit_cfg *lc = &g->limits[l];
            double gain_lin_here;
            if (lc->stage > 0) {
                if (lc->stage > c->n_stages) {
                    fprintf(stderr, "PROTECT: group chain='%s' limit='%s': "
                            "stage %d given, but chain only has %d "
                            "stage(s).\n", c->name, lc->name, lc->stage,
                            c->n_stages);
                    return -1;
                }
                double cum_db = 0.0;
                for (int s = 0; s < lc->stage; s++) {
                    cum_db += c->stage_gain_db[s];
                }
                gain_lin_here = db2lin(cum_db);
            } else {
                gain_lin_here = c->gain_lin;
            }
            double v_at_0dbfs = c->dac_max_output_vrms * gain_lin_here;
            double max_vrms_raw = lc->threshold_lin;
            lc->threshold_lin = max_vrms_raw / v_at_0dbfs;
            fprintf(stderr, "PROTECT: group chain='%s' limit='%s'%s: "
                    "%.3f Vrms -> %.2f dBFS%s\n",
                    c->name, lc->name,
                    lc->stage > 0 ? " (partielle Chain)" : "",
                    max_vrms_raw, 20.0 * log10(lc->threshold_lin),
                    lc->freq_weighted ? " (frequenzgewichtet)" : "");
        }
    }
    return 0;
}

/* Comma list of strings up to ';'. Returns count, or -1 (message printed)
   if longer than `max` or malformed. */
static int
parse_string_list(int (*get_config_token)(union bflexval *lexval),
                  char out[][BF_MAXOBJECTNAME], int max, const char *what)
{
    union bflexval lexval;
    int n = 0, token = BF_LEX_COMMA;
    while (token == BF_LEX_COMMA) {
        if (get_config_token(&lexval) != BF_LEXVAL_STRING) {
            fprintf(stderr, "PROTECT: link: %s: expected channel name "
                    "string.\n", what);
            return -1;
        }
        if (n == max) {
            fprintf(stderr, "PROTECT: link: %s: too many entries (max %d).\n",
                    what, max);
            return -1;
        }
        if (lexval.string[0] == '\0' ||
            strlen(lexval.string) >= BF_MAXOBJECTNAME)
        {
            fprintf(stderr, "PROTECT: link: %s: empty or too long name.\n",
                    what);
            return -1;
        }
        memcpy(out[n], lexval.string, strlen(lexval.string) + 1);  /* length checked above */
        out[n][BF_MAXOBJECTNAME - 1] = '\0';
        n++;
        token = get_config_token(&lexval);
    }
    if (token != BF_LEX_EOS) {
        fprintf(stderr, "PROTECT: link: %s: expected ',' or ';'.\n", what);
        return -1;
    }
    return n;
}

static int
parse_real_list(int (*get_config_token)(union bflexval *lexval),
                double out[], int max, const char *what)
{
    union bflexval lexval;
    int n = 0, token = BF_LEX_COMMA;
    while (token == BF_LEX_COMMA) {
        if (get_config_token(&lexval) != BF_LEXVAL_REAL) {
            fprintf(stderr, "PROTECT: link: %s: expected number.\n", what);
            return -1;
        }
        if (n == max) {
            fprintf(stderr, "PROTECT: link: %s: too many entries (max %d).\n",
                    what, max);
            return -1;
        }
        out[n++] = lexval.real;
        token = get_config_token(&lexval);
    }
    if (token != BF_LEX_EOS) {
        fprintf(stderr, "PROTECT: link: %s: expected ',' or ';'.\n", what);
        return -1;
    }
    return n;
}

/* link: "<name>" { channels/drive/ratio/offset_db/enabled }. Only syntax
   and per-field ranges here; names are resolved in resolve_links() after
   the whole config is read (so "link" may stand before or after the
   groups). */
static int
parse_link(int (*get_config_token)(union bflexval *lexval),
           struct link_cfg *lk, const char *name)
{
    union bflexval lexval;
    int token, seen_ch = 0, seen_en = 0;

    memset(lk, 0, sizeof(*lk));
    lk->enabled = 1;
    if (name[0] == '\0' || strlen(name) >= BF_MAXOBJECTNAME) {
        fprintf(stderr, "PROTECT: link: empty or too long link name.\n");
        return -1;
    }
    memcpy(lk->name, name, strlen(name) + 1);          /* length checked above */

    while ((token = get_config_token(&lexval)) > 0) {
        if (token == BF_LEX_RBRACE) break;
        if (token != BF_LEXVAL_FIELD) {
            fprintf(stderr, "PROTECT: link '%s': expected field.\n", name);
            return -1;
        }
        if (strcmp(lexval.field, "channels") == 0) {
            if (seen_ch++) goto dup;
            lk->n = parse_string_list(get_config_token, lk->member_name,
                                      MAX_CH_PER_LINK, "channels");
            if (lk->n < 0) return -1;
        } else if (strcmp(lexval.field, "drive") == 0) {
            if (lk->n_drive) goto dup;
            lk->n_drive = parse_string_list(get_config_token, lk->drive_name,
                                            MAX_CH_PER_LINK, "drive");
            if (lk->n_drive < 0) return -1;
        } else if (strcmp(lexval.field, "ratio") == 0) {
            if (lk->n_ratio) goto dup;
            lk->n_ratio = parse_real_list(get_config_token, lk->ratio,
                                          MAX_CH_PER_LINK, "ratio");
            if (lk->n_ratio < 0) return -1;
        } else if (strcmp(lexval.field, "offset_db") == 0) {
            if (lk->n_offset) goto dup;
            lk->n_offset = parse_real_list(get_config_token, lk->offset_db,
                                           MAX_CH_PER_LINK, "offset_db");
            if (lk->n_offset < 0) return -1;
        } else if (strcmp(lexval.field, "enabled") == 0) {
            if (seen_en++) goto dup;
            GET_TOKEN(BF_LEXVAL_REAL, "link: enabled: expected number.\n");
            lk->enabled = (lexval.real != 0.0);
            GET_TOKEN(BF_LEX_EOS, "link: expected ';'.\n");
        } else {
            fprintf(stderr, "PROTECT: link '%s': unknown field '%s'.\n",
                    name, lexval.field);
            return -1;
        }
        continue;
dup:
        fprintf(stderr, "PROTECT: link '%s': field '%s' given twice.\n",
                name, lexval.field);
        return -1;
    }
    if (lk->n < 2) {
        fprintf(stderr, "PROTECT: link '%s': needs 'channels' with at least "
                "2 members (got %d).\n", name, lk->n);
        return -1;
    }
    if (lk->n_ratio && lk->n_ratio != lk->n) {
        fprintf(stderr, "PROTECT: link '%s': ratio has %d entries, channels "
                "has %d (one per member, same order).\n", name, lk->n_ratio,
                lk->n);
        return -1;
    }
    if (lk->n_offset && lk->n_offset != lk->n) {
        fprintf(stderr, "PROTECT: link '%s': offset_db has %d entries, "
                "channels has %d (one per member, same order).\n", name,
                lk->n_offset, lk->n);
        return -1;
    }
    for (int m = 0; m < lk->n; m++) {
        if (!lk->n_ratio) lk->ratio[m] = 1.0;
        if (!lk->n_offset) lk->offset_db[m] = 0.0;
        /* written as !(a <= x <= b) so that NaN is rejected too */
        if (!(lk->ratio[m] >= 0.0 && lk->ratio[m] <= LINK_RATIO_MAX)) {
            fprintf(stderr, "PROTECT: link '%s': ratio %g for '%s' out of "
                    "range [0, %g].\n", name, lk->ratio[m],
                    lk->member_name[m], LINK_RATIO_MAX);
            return -1;
        }
        if (!(lk->offset_db[m] >= 0.0 && lk->offset_db[m] <= LINK_OFFSET_MAX)) {
            fprintf(stderr, "PROTECT: link '%s': offset_db %g for '%s' out "
                    "of range [0, %g].\n", name, lk->offset_db[m],
                    lk->member_name[m], LINK_OFFSET_MAX);
            return -1;
        }
    }
    return 0;
}

/* After the parse loop and the group mapping: names -> intnames, and all
   cross checks of design 3.3. */
static int
resolve_links(const int n_channels[2], const struct bfchannel *channels[2])
{
    for (int i = 0; i < MAX_MAPPED_CHANNELS; i++) {
        chan_link[i] = -1;
        chan_member[i] = -1;
    }
    for (int li = 0; li < n_links; li++) {
        struct link_cfg *lk = &links[li];
        for (int lj = 0; lj < li; lj++) {
            if (strcmp(links[lj].name, lk->name) == 0) {
                fprintf(stderr, "PROTECT: link name '%s' used twice.\n",
                        lk->name);
                return -1;
            }
        }
        for (int m = 0; m < lk->n; m++) {
            int idx = -1;
            for (int i = 0; i < n_channels[BF_OUT]; i++) {
                if (strcmp(lk->member_name[m], channels[BF_OUT][i].name) == 0) {
                    idx = channels[BF_OUT][i].intname;
                    break;
                }
            }
            if (idx < 0) {
                fprintf(stderr, "PROTECT: link '%s': '%s' is not a BruteFIR "
                        "output channel.\n", lk->name, lk->member_name[m]);
                return -1;
            }
            if (idx >= MAX_MAPPED_CHANNELS) {
                fprintf(stderr, "PROTECT: link '%s': intname %d out of "
                        "range.\n", lk->name, idx);
                return -1;
            }
            if (chan_group[idx] < 0) {
                fprintf(stderr, "PROTECT: link '%s': '%s' is in no protect "
                        "group (would not pass through this module).\n",
                        lk->name, lk->member_name[m]);
                return -1;
            }
            for (int q = 0; q < m; q++) {
                if (lk->member_ch[q] == idx) {
                    fprintf(stderr, "PROTECT: link '%s': '%s' listed twice.\n",
                            lk->name, lk->member_name[m]);
                    return -1;
                }
            }
            if (chan_link[idx] >= 0) {
                fprintf(stderr, "PROTECT: '%s' is in more than one link "
                        "('%s' and '%s') - write one link with the union "
                        "instead.\n", lk->member_name[m],
                        links[chan_link[idx]].name, lk->name);
                return -1;
            }
            lk->member_ch[m] = idx;
            chan_link[idx] = li;
            chan_member[idx] = m;
        }
        if (lk->n_drive == 0) {
            for (int m = 0; m < lk->n; m++) lk->drives[m] = 1;
        } else {
            for (int d = 0; d < lk->n_drive; d++) {
                int found = -1;
                for (int m = 0; m < lk->n; m++) {
                    if (strcmp(lk->drive_name[d], lk->member_name[m]) == 0) {
                        found = m;
                        break;
                    }
                }
                if (found < 0) {
                    fprintf(stderr, "PROTECT: link '%s': drive '%s' is not a "
                            "member of the link.\n", lk->name,
                            lk->drive_name[d]);
                    return -1;
                }
                if (lk->drives[found]) {
                    fprintf(stderr, "PROTECT: link '%s': drive '%s' listed "
                            "twice.\n", lk->name, lk->drive_name[d]);
                    return -1;
                }
                lk->drives[found] = 1;
            }
        }
        char line[1024];
        int pos = snprintf(line, sizeof(line), "PROTECT: link '%s'%s:",
                           lk->name, lk->enabled ? "" : " (off)");
        for (int m = 0; m < lk->n && pos > 0 && (size_t)pos < sizeof(line); m++) {
            pos += snprintf(line + pos, sizeof(line) - pos, " %s(%s,%.2f,%.1fdB)",
                            lk->member_name[m],
                            lk->drives[m] ? "drive" : "follow",
                            lk->ratio[m], lk->offset_db[m]);
        }
        fprintf(stderr, "%s\n", line);
    }
    return 0;
}

/* ---- Kern: Sample-Verarbeitung ----------------------------------------- */

/* Detector + checkpoint smoothing for ONE sample: unchanged v3 code, only
   moved out of process_channel() so that the plain path and the link path
   share exactly the same detector (design 3.2: the invariant rests on the
   detectors being identical with and without link; test T1a/T1b). */
static inline double
detect_sample(struct group *g, struct channel_runtime *rt, double s)
{
    int l;
    double mag, target, alpha, combined;

    {
        mag = fabs(s);
        combined = 1.0;

        for (l = 0; l < g->n_limits; l++) {
            struct limit_cfg *lc = &g->limits[l];
            struct limit_state *ls = &rt->st[l];
            double detect_mag = mag;

            if (!lc->enabled) {          /* v3: per Kommando abgeschaltet */
                ls->gain = 1.0;
                continue;
            }

            if (lc->freq_weighted && lc->signed_weight) {
                /* v3: Tiefpass-Kaskade auf das VORZEICHENBEHAFTETE Signal,
                   danach Betrag. Das ist die Uebertragungsfunktion
                   Spannung -> Membranauslenkung einer geschlossenen Box
                   (2 gleiche Einpol-Tiefpaesse = Q 0,5, Qtc der Box 0,49):
                   die zulaessige Spitzenspannung ist max_vrms*(1+(f/fc)^2).
                   Im Altpfad (erst fabs, dann Tiefpass) bleibt der
                   Gleichanteil des gleichgerichteten Signals ungedaempft
                   und die Gewichtung wirkt nur bei sehr tiefen
                   Frequenzen. */
                ls->shelf_z1 = (1.0 - lc->shelf_coeff) * s +
                                lc->shelf_coeff * ls->shelf_z1;
                detect_mag = ls->shelf_z1;
                if (lc->shelf_poles == 2) {
                    ls->shelf_z2 = (1.0 - lc->shelf_coeff) * detect_mag +
                                    lc->shelf_coeff * ls->shelf_z2;
                    detect_mag = ls->shelf_z2;
                }
                detect_mag = fabs(detect_mag);
            } else if (lc->freq_weighted) {
                /* Kaskade von Einpol-Tiefpaessen (shelf_poles Stufen,
                   gleiche Eckfrequenz) auf den Detektor-Eingang dieses
                   Checkpoints, bildet die mit der Frequenz steigende
                   physikalische Grenze nach -- 1 Stufe = 6dB/Oktave
                   (z.B. Trafo-Kernfluss), 2 Stufen = 12dB/Oktave (z.B.
                   Xmax einer geschlossenen Box). */
                ls->shelf_z1 = (1.0 - lc->shelf_coeff) * mag +
                                lc->shelf_coeff * ls->shelf_z1;
                detect_mag = ls->shelf_z1;
                if (lc->shelf_poles == 2) {
                    ls->shelf_z2 = (1.0 - lc->shelf_coeff) * detect_mag +
                                    lc->shelf_coeff * ls->shelf_z2;
                    detect_mag = ls->shelf_z2;
                }
            }

            if (lc->type == LIMIT_PEAK) {
                target = (detect_mag > lc->threshold_lin)
                             ? (lc->threshold_lin / detect_mag) : 1.0;
                alpha = (target < ls->gain) ? lc->attack_coeff
                                             : lc->release_coeff;
                ls->gain = target + alpha * (ls->gain - target);
                if (target < 0.999) ls->overs++;
            } else { /* LIMIT_RMS */
                ls->rms_sumsq += (s * s - ls->rms_sumsq) *
                                  (1.0 / lc->window_samples);
                if (ls->rms_sumsq >
                    lc->threshold_lin * lc->threshold_lin)
                {
                    target = sqrt((lc->threshold_lin * lc->threshold_lin) /
                                   ls->rms_sumsq);
                    ls->overs++;
                } else {
                    target = 1.0;
                }
                alpha = (target < ls->gain) ? lc->attack_coeff
                                             : lc->release_coeff;
                ls->gain = target + alpha * (ls->gain - target);
            }
            if (ls->gain < combined) combined = ls->gain;
        }
    }
    return combined;
}

static inline void
process_channel(struct group *g, int gi, int ci, double *samples, int n)
{
    struct channel_runtime *rt = &ch_runtime[gi][ci];
    int i;
    double s;

    for (i = 0; i < n; i++) {
        s = samples[i];
        samples[i] = s * detect_sample(g, rt, s);
    }
}

/* Link path (design 2.3): same detector, then min(own, link ramp). The
   ramp goes linearly from L_prev (link gain at the end of the previous
   block) to L_tgt over this block. Returns the minimum OWN gain of the
   block (published to the partners - never the linked gain, design 2.3
   point 2, otherwise symmetric links latch). */
static inline double
process_channel_link(struct group *g, int gi, int ci, double *samples,
                     int n, double L_prev, double L_tgt)
{
    struct channel_runtime *rt = &ch_runtime[gi][ci];
    double own_min = 1.0;
    double dL = L_tgt - L_prev;

    for (int i = 0; i < n; i++) {
        double s = samples[i];
        double combined = detect_sample(g, rt, s);
        if (combined < own_min) own_min = combined;
        double L_i = L_prev + dL * (double)(i + 1) / (double)n;
        /* NaN-safe: a NaN L_i compares false -> own gain is applied */
        double g_i = (L_i < combined) ? L_i : combined;
        samples[i] = s * g_i;
    }
    return own_min;
}

/* Link target for block k of `channel` from the partners' block k-1. */
static inline double
link_target(int channel, struct link_cfg *lk, long k)
{
    double L_tgt = 1.0;
    int me = chan_member[channel];

    for (int m = 0; m < lk->n; m++) {
        if (m == me || !lk->drives[m]) continue;
        int p = lk->member_ch[m];
        struct pub_slot *s = &pub[p][(k - 1) & 1];
        double gp = a_load_d(&s->gmin);
        long bk = a_load_l(&s->blk);
        if (bk != k - 1) {
            /* must not happen under BruteFIR's barrier (design 2.4);
               if the thread model ever changes: count, take the more
               conservative of both slots. The other slot may be written
               by the partner in this very block; the access is atomic, so
               it yields the old or the new value - either is a real gain
               of the partner, and the min stays <= the stale slot. */
            a_store_l(&link_faults[channel], a_load_l(&link_faults[channel]) + 1);
            double g0 = a_load_d(&pub[p][0].gmin);
            double g1 = a_load_d(&pub[p][1].gmin);
            gp = (g0 < g1) ? g0 : g1;      /* NaN -> rejected below */
        }
        if (!(gp > 0.0 && gp <= 1.0)) {   /* NaN/0/>1: ignore this partner */
            a_store_l(&link_faults[channel], a_load_l(&link_faults[channel]) + 1);
            continue;                      /* own limits stay active */
        }
        double a_db = -20.0 * log10(gp);                       /* >= 0 */
        double over = a_db - lk->offset_db[me];
        double f_db = lk->ratio[me] * (over > 0.0 ? over : 0.0);
        double Lp = pow(10.0, -f_db / 20.0);                   /* (0,1] */
        if (Lp < L_tgt) L_tgt = Lp;
    }
    return L_tgt;
}

static void
output_timed(void *buf, int channel)
{
    int gi, ci, li;
    double tmp[8192];
    double *samples;
    float *fbuf = NULL;
    int n = block_length;

    if (channel < 0 || channel >= MAX_MAPPED_CHANNELS ||
        chan_group[channel] < 0)
    {
        return;
    }
    gi = chan_group[channel];
    ci = chan_idx[channel];

    if (realsize == 4) {
        fbuf = (float *)buf;
        for (int i = 0; i < n; i++) tmp[i] = fbuf[i];
        samples = tmp;
    } else {
        samples = (double *)buf;
    }

    li = chan_link[channel];
    if (li < 0) {
        /* no link: unchanged v3 path */
        process_channel(&groups[gi], gi, ci, samples, n);
    } else {
        struct link_cfg *lk = &links[li];
        long k = ++blk_ctr[channel];
        double L_prev = link_prev[channel];
        double L_tgt = 1.0;
        if (a_load_i(&lk->enabled)) {
            L_tgt = link_target(channel, lk, k);
        } else {
            L_prev = 1.0;        /* link off: immediately own limits only */
        }
        double own_min = process_channel_link(&groups[gi], gi, ci, samples,
                                              n, L_prev, L_tgt);
        link_prev[channel] = L_tgt;
        a_store_d(&link_now[channel], L_tgt);
        struct pub_slot *w = &pub[channel][k & 1];      /* always publish */
        a_store_d(&w->gmin, own_min);
        a_store_l(&w->blk, k);
    }

    if (fbuf != NULL) {
        for (int i = 0; i < n; i++) fbuf[i] = (float)tmp[i];
    }
}

/* ---- Modul-Einstiegspunkte --------------------------------------------- */

int
bflogic_preinit(/* bfmod.h names these (version_minor, version_major, ...),
                   but the real caller (bfconf.c:3007, v1.1.2) passes
                   (&version_major, &version_minor, ...) -- verified by
                   an actual failed version check ("got 0, expected 3")
                   until swapped to match. Both are plain int*, so the
                   compiler never flags the header/caller mismatch. */
                int *version_major_out,
                int *version_minor_out,
                int (*get_config_token)(union bflexval *lexval),
                int _sample_rate,
                int _block_length,
                int n_maxblocks,
                int n_coeffs,
                const struct bfcoeff coeffs[],
                const int n_channels[2],
                const struct bfchannel *channels[2],
                int n_filters,
                const struct bffilter filters[],
                struct bfevents *bfevents,
                int *fork_mode,
                int debug)
{
    union bflexval lexval;
    int token, i, c;

    *version_major_out = BF_VERSION_MAJOR;
    *version_minor_out = BF_VERSION_MINOR;
    sample_rate = _sample_rate;
    block_length = _block_length;
    if (block_length > 8192) {
        fprintf(stderr, "PROTECT: block_length > 8192, tmp-Puffer im "
                "Code dynamisch allozieren.\n");
        return -1;
    }

    while ((token = get_config_token(&lexval)) > 0) {
        if (token != BF_LEXVAL_FIELD) {
            fprintf(stderr, "PROTECT: expected 'chain' or 'group'.\n");
            return -1;
        }
        if (strcmp(lexval.field, "chain") == 0) {
            union bflexval namelex;
            char chain_name[BF_MAXOBJECTNAME];
            GET_TOKEN(BF_LEXVAL_STRING, "expected chain name string.\n");
            strncpy(chain_name, lexval.string, BF_MAXOBJECTNAME - 1);
            GET_TOKEN(BF_LEX_LBRACE, "expected '{'.\n");
            if (n_chains == MAX_CHAINS) {
                fprintf(stderr, "PROTECT: too many chains.\n");
                return -1;
            }
            if (parse_chain(get_config_token, &chains[n_chains],
                             chain_name) != 0)
            {
                return -1;
            }
            GET_TOKEN(BF_LEX_EOS, "expected ';' after chain block.\n");
            n_chains++;
            (void)namelex;
        } else if (strcmp(lexval.field, "group") == 0) {
            GET_TOKEN(BF_LEX_LBRACE, "expected '{'.\n");
            if (n_groups == MAX_GROUPS) {
                fprintf(stderr, "PROTECT: too many groups.\n");
                return -1;
            }
            if (parse_group(get_config_token, &groups[n_groups]) != 0) {
                return -1;
            }
            GET_TOKEN(BF_LEX_EOS, "expected ';' after group block.\n");
            n_groups++;
        } else if (strcmp(lexval.field, "output_scale") == 0) {
            GET_TOKEN(BF_LEXVAL_REAL, "expected real.\n");
            output_scale = lexval.real;
            GET_TOKEN(BF_LEX_EOS, "expected ';'.\n");
        } else if (strcmp(lexval.field, "link") == 0) {
            char link_name[BF_MAXOBJECTNAME];
            GET_TOKEN(BF_LEXVAL_STRING, "expected link name string.\n");
            if (strlen(lexval.string) >= BF_MAXOBJECTNAME) {
                fprintf(stderr, "PROTECT: link name too long.\n");
                return -1;
            }
            memcpy(link_name, lexval.string, strlen(lexval.string) + 1);
            link_name[BF_MAXOBJECTNAME - 1] = '\0';
            GET_TOKEN(BF_LEX_LBRACE, "expected '{' after link name.\n");
            if (n_links == MAX_LINKS) {
                fprintf(stderr, "PROTECT: too many links (max %d).\n",
                        MAX_LINKS);
                return -1;
            }
            if (parse_link(get_config_token, &links[n_links], link_name) != 0)
            {
                return -1;
            }
            GET_TOKEN(BF_LEX_EOS, "expected ';' after link block.\n");
            n_links++;
        } else {
            fprintf(stderr, "PROTECT: expected 'chain', 'group', 'link' or "
                    "'output_scale', got '%s'.\n", lexval.field);
            return -1;
        }
    }

    /* threshold_lin (aus parse_group) ist bislang eine physikalisch
       normierte 0..1-Vollausschlag-Ratio -- erst hier auf die reale
       Roh-Puffer-Skala umrechnen, die output_timed() tatsaechlich sieht
       (s.o., output_scale). Erst nach der ganzen Parse-Schleife, damit
       "output_scale:" an beliebiger Stelle in der Config stehen darf. */
    for (int g = 0; g < n_groups; g++) {
        for (int l = 0; l < groups[g].n_limits; l++) {
            struct limit_cfg *lc = &groups[g].limits[l];
            double ratio = lc->threshold_lin;
            lc->threshold_lin = ratio * output_scale;
            fprintf(stderr, "PROTECT: group %d limit='%s': skaliert auf "
                    "Puffereinheiten: %.1f (output_scale=%.1f, "
                    "entspricht weiterhin %.2f dBFS)\n",
                    g, lc->name, lc->threshold_lin, output_scale,
                    20.0 * log10(ratio));
        }
    }

    for (i = 0; i < MAX_MAPPED_CHANNELS; i++) chan_group[i] = -1;
    for (int g = 0; g < n_groups; g++) {
        for (c = 0; c < groups[g].n_channels; c++) {
            int found = 0;
            for (i = 0; i < n_channels[BF_OUT]; i++) {
                if (strcmp(groups[g].ch_names[c],
                           channels[BF_OUT][i].name) == 0)
                {
                    int idx = channels[BF_OUT][i].intname;
                    if (idx < 0 || idx >= MAX_MAPPED_CHANNELS) {
                        fprintf(stderr, "PROTECT: intname %d out of "
                                "range.\n", idx);
                        return -1;
                    }
                    if (chan_group[idx] >= 0) {
                        /* used to overwrite the first mapping silently
                           (side finding, link design 1.4/3.3) */
                        fprintf(stderr, "PROTECT: output channel '%s' is in "
                                "more than one group (group %d and %d) - "
                                "config error.\n", groups[g].ch_names[c],
                                chan_group[idx], g);
                        return -1;
                    }
                    groups[g].ch_intname[c] = idx;
                    chan_group[idx] = g;
                    chan_idx[idx] = c;
                    found = 1;
                    break;
                }
            }
            if (!found) {
                fprintf(stderr, "PROTECT: output channel '%s' not "
                        "found.\n", groups[g].ch_names[c]);
                return -1;
            }
        }
    }

    if (resolve_links(n_channels, channels) != 0) {
        return -1;
    }

    bfevents->output_timed = output_timed;
    *fork_mode = BF_FORK_DONT_FORK;
    return 0;
}

static void
link_runtime_reset(void)
{
    for (int i = 0; i < MAX_MAPPED_CHANNELS; i++) {
        pub[i][0].gmin = pub[i][1].gmin = 1.0;   /* start without reduction */
        pub[i][0].blk = pub[i][1].blk = 0;
        blk_ctr[i] = 0;
        link_prev[i] = 1.0;
        link_now[i] = 1.0;
        link_faults[i] = 0;
    }
}

int
bflogic_init(struct bfaccess *bfaccess,
             int _sample_rate,
             int _block_length,
             int n_maxblocks,
             int n_coeffs,
             const struct bfcoeff coeffs[],
             const int n_channels[2],
             const struct bfchannel *channels[2],
             int n_filters,
             const struct bffilter filters[],
             int event_fd,
             int synch_fd)
{
    realsize = bfaccess->realsize;
    /* ch_runtime is static -> zero-initialised by the loader, but a
       fresh limit_state's `gain` must start at 1.0 ("no reduction"),
       not 0.0 ("fully engaged") -- otherwise every start/reload begins
       with a spurious full mute that only recovers after a full
       release time constant, regardless of actual signal level. */
    for (int g = 0; g < n_groups; g++) {
        for (int c = 0; c < groups[g].n_channels; c++) {
            for (int l = 0; l < groups[g].n_limits; l++) {
                ch_runtime[g][c].st[l].gain = 1.0;
            }
        }
    }
    link_runtime_reset();
    if (n_links > 0) {
        fprintf(stderr, "PROTECT: initialised, %d chain(s), %d group(s), "
                "%d link(s), realsize=%d\n", n_chains, n_groups, n_links,
                realsize);
    } else {
        fprintf(stderr, "PROTECT: initialised, %d chain(s), %d group(s), "
                "realsize=%d\n", n_chains, n_groups, realsize);
    }
    memset(msg, 0, sizeof(msg));
    return 0;
}

/* "active" for display-filtering purposes: has ever pulled gain down
   (recorded overs) or is pulling it down right now. Same 0.999
   threshold process_channel() itself uses to decide whether a sample
   counts as an "over" -- see the "if (target < 0.999) ls->overs++;"
   calls above. */
static inline int
limit_state_active(struct limit_state *ls)
{
    return ls->overs > 0 || ls->gain < 0.999;
}

static inline double
gain_to_db(double gain)
{
    /* gain is a smoothed target/gain ratio, always in (0, 1] by
       construction -- floor it before log10() only to guard against
       a theoretical exact 0.0 at startup before the first sample. */
    return 20.0 * log10(gain > 1e-12 ? gain : 1e-12);
}

static inline double
overs_to_ms(long overs)
{
    return (sample_rate > 0) ? (1000.0 * (double)overs / sample_rate) : 0.0;
}

/* Appends at most `remaining` bytes (incl. terminator) via snprintf,
   then advances the write pointer and remaining-space count by however
   much was actually written -- never past the buffer, unlike the old
   unchecked sprintf() this replaces. Silently truncates once the
   buffer fills, same as any snprintf-based accumulator. */
static void
msg_append(char **p, size_t *remaining, const char *fmt, ...)
{
    if (*remaining == 0) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(*p, *remaining, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    size_t written = ((size_t)n < *remaining) ? (size_t)n : *remaining - 1;
    *p += written;
    *remaining -= written;
}

/* lmc 1 <params> (BruteFIR CLI) - "1" is whatever `lm` currently shows
   as the protect module's index, not a stable/hardcoded number.
   v3 additions:
     "list"                 -- all limits: ON/off, type, weighting, threshold
     "on <name>|all"        -- enable a limit (state is cleared first)
     "off <name>|all"       -- disable a limit; refused for always_on limits
                                and for the last enabled non-always_on limit
                                of a group (so the Xmax/THD protection can
                                never be switched off completely)
     (no params) / "status" -- table, idle limit-states hidden
     "all"                  -- table, every limit-state shown
     "json"                 -- JSON array, every limit-state, for a
                                webUI/machine consumer
     "reset"                -- zero every overs counter (does NOT touch
                                gain/shelf filter state - a limit still
                                actively reducing gain when reset is
                                called keeps doing so; reset only clears
                                the historical counter) */
int
bflogic_command(const char params[])
{
    char *p = msg;
    size_t remaining = MSG_BUF_SIZE;
    memset(msg, 0, sizeof(msg));

    while (*params == ' ' || *params == '\t') params++;

    /* channel link: "link [list]" | "link on <name>|all" | "link off <name>|all".
       "off" only removes the ADDITIONAL reduction; every channel's own
       limits stay as they are, so it is allowed without restriction. */
    if (strncmp(params, "link", 4) == 0 &&
        (params[4] == '\0' || params[4] == ' ' || params[4] == '\t' ||
         params[4] == '\n' || params[4] == '\r'))
    {
        const char *q = params + 4;
        while (*q == ' ' || *q == '\t') q++;
        if (*q == '\0' || *q == '\n' || *q == '\r' || strncmp(q, "list", 4) == 0) {
            if (n_links == 0) {
                msg_append(&p, &remaining, "PROTECT: no links configured\n");
                return 0;
            }
            msg_append(&p, &remaining, "%-12s  %-5s  %-12s  %-6s  %5s  %6s  %9s  %7s\n",
                       "LINK", "STATE", "CHANNEL", "ROLE", "RATIO", "OFFSET",
                       "LINK_dB", "FAULTS");
            for (int li = 0; li < n_links; li++) {
                struct link_cfg *lk = &links[li];
                for (int m = 0; m < lk->n; m++) {
                    int ch = lk->member_ch[m];
                    msg_append(&p, &remaining,
                               "%-12s  %-5s  %-12s  %-6s  %5.2f  %6.1f  %9.2f  %7ld\n",
                               lk->name, a_load_i(&lk->enabled) ? "ON" : "off",
                               lk->member_name[m],
                               lk->drives[m] ? "drive" : "follow",
                               lk->ratio[m], lk->offset_db[m],
                               gain_to_db(a_load_d(&link_now[ch])),
                               a_load_l(&link_faults[ch]));
                }
            }
            return 0;
        }
        if (strncmp(q, "on ", 3) == 0 || strncmp(q, "off ", 4) == 0) {
            int turn_on = (q[1] == 'n');
            const char *name = q + (turn_on ? 3 : 4);
            while (*name == ' ') name++;
            char nm[BF_MAXOBJECTNAME];
            size_t k = 0;
            while (name[k] && name[k] != ' ' && name[k] != '\n' &&
                   name[k] != '\r' && k < sizeof(nm) - 1)
            {
                nm[k] = name[k]; k++;
            }
            nm[k] = 0;
            int all = (strcmp(nm, "all") == 0), hit = 0;
            for (int li = 0; li < n_links; li++) {
                if (!all && strcmp(links[li].name, nm) != 0) continue;
                a_store_i(&links[li].enabled, turn_on);
                hit++;
            }
            msg_append(&p, &remaining, "PROTECT: link %s '%s': %d link(s) "
                       "changed%s\n", turn_on ? "on" : "off", nm, hit,
                       hit ? "" : " - name not found");
            return 0;
        }
        msg_append(&p, &remaining, "PROTECT: usage: link [list] | link on "
                   "<name>|all | link off <name>|all\n");
        return 0;
    }

    /* v3: "list" | "on <name>|all" | "off <name>|all" */
    if (strncmp(params, "list", 4) == 0) {
        msg_append(&p, &remaining, "%-5s  %-22s  %-5s  %-6s  %-8s  %s\n",
                   "GROUP", "LIMIT", "STATE", "TYPE", "WEIGHT", "THRESHOLD");
        for (int g = 0; g < n_groups; g++)
            for (int l = 0; l < groups[g].n_limits; l++) {
                struct limit_cfg *lc = &groups[g].limits[l];
                msg_append(&p, &remaining,
                    "grp%-2d  %-22s  %-5s  %-6s  %-8s  %.2f dBFS%s\n", g,
                    lc->name, lc->enabled ? "ON" : "off",
                    lc->type == LIMIT_RMS ? "rms" : "peak",
                    !lc->freq_weighted ? "-" :
                        (lc->signed_weight ? "signed" : "abs"),
                    20.0 * log10(lc->threshold_lin / output_scale),
                    lc->always_on ? "  (always_on)" : "");
            }
        return 0;
    }
    if (strncmp(params, "on ", 3) == 0 || strncmp(params, "off ", 4) == 0) {
        int turn_on = (params[1] == 'n');
        const char *name = params + (turn_on ? 3 : 4);
        while (*name == ' ') name++;
        char nm[BF_MAXOBJECTNAME];
        size_t k = 0;
        while (name[k] && name[k] != ' ' && name[k] != '\n' &&
               name[k] != '\r' && k < sizeof(nm) - 1)
        {
            nm[k] = name[k]; k++;
        }
        nm[k] = 0;
        int all = (strcmp(nm, "all") == 0), hit = 0, refused = 0;
        for (int g = 0; g < n_groups; g++) {
            for (int l = 0; l < groups[g].n_limits; l++) {
                struct limit_cfg *lc = &groups[g].limits[l];
                if (!all && strcmp(lc->name, nm) != 0) continue;
                if (!turn_on) {
                    /* mindestens ein NICHT-always_on-Limit (Xmax/THD)
                       muss aktiv bleiben, sonst laeuft die Gruppe nur
                       noch mit Verstaerker-Clip und Thermik */
                    int others = 0;
                    for (int m = 0; m < groups[g].n_limits; m++)
                        if (m != l && groups[g].limits[m].enabled &&
                            !groups[g].limits[m].always_on) others++;
                    if (lc->always_on || others == 0) {
                        refused++;
                        continue;      /* nie die letzte Absicherung weg */
                    }
                    lc->enabled = 0;
                } else if (!lc->enabled) {
                    for (int c = 0; c < groups[g].n_channels; c++) {
                        struct limit_state *ls = &ch_runtime[g][c].st[l];
                        ls->shelf_z1 = ls->shelf_z2 = 0.0;
                        ls->rms_sumsq = 0.0;
                        ls->gain = 1.0;
                    }
                    lc->enabled = 1;
                }
                hit++;
            }
        }
        msg_append(&p, &remaining, "PROTECT: %s '%s': %d limit(s) changed, "
                   "%d refused (always_on or last Xmax/THD limit of its "
                   "group), %s\n", turn_on ? "on" : "off", nm, hit, refused,
                   (hit + refused) ? "see \"list\"" : "name not found");
        return 0;
    }

    if (strncmp(params, "reset", 5) == 0) {
        int count = 0;
        for (int g = 0; g < n_groups; g++)
            for (int c = 0; c < groups[g].n_channels; c++)
                for (int l = 0; l < groups[g].n_limits; l++) {
                    ch_runtime[g][c].st[l].overs = 0;
                    count++;
                }
        msg_append(&p, &remaining,
                   "PROTECT: overs counters reset (%d limit checkpoints)\n",
                   count);
        return 0;
    }

    int as_json = (strncmp(params, "json", 4) == 0);
    int show_all = as_json || (strncmp(params, "all", 3) == 0);

    if (as_json) {
        msg_append(&p, &remaining, "[\n");
        int first = 1;
        for (int g = 0; g < n_groups; g++) {
            struct group *gr = &groups[g];
            const char *chain_name =
                (gr->chain_index >= 0) ? chains[gr->chain_index].name : "";
            for (int c = 0; c < gr->n_channels; c++) {
                for (int l = 0; l < gr->n_limits; l++) {
                    struct limit_state *ls = &ch_runtime[g][c].st[l];
                    msg_append(&p, &remaining,
                        "%s{\"group\":%d,\"chain\":\"%s\",\"channel\":\"%s\","
                        "\"limit\":\"%s\",\"type\":\"%s\",\"overs\":%ld,"
                        "\"overs_ms\":%.1f,\"gain\":%.4f,\"gain_db\":%.2f}",
                        first ? "" : ",\n",
                        g, chain_name, gr->ch_names[c], gr->limits[l].name,
                        (gr->limits[l].type == LIMIT_RMS) ? "rms" : "peak",
                        ls->overs, overs_to_ms(ls->overs), ls->gain,
                        gain_to_db(ls->gain));
                    if (n_links > 0) {
                        /* the closing '}' above is undone and re-added
                           with the link fields (only when links exist,
                           so link-free output stays byte-identical) */
                        int ch = gr->ch_intname[c];
                        int li = chan_link[ch];
                        if (p > msg && p[-1] == '}') { p--; remaining++; }
                        msg_append(&p, &remaining,
                            ",\"link\":\"%s\",\"link_gain_db\":%.2f,"
                            "\"link_faults\":%ld}",
                            li >= 0 ? links[li].name : "",
                            gain_to_db(a_load_d(&link_now[ch])),
                            a_load_l(&link_faults[ch]));
                    }
                    first = 0;
                }
            }
        }
        msg_append(&p, &remaining, "\n]\n");
        return 0;
    }

    /* table (default "status", and "all") */
    int total = 0, with_overs = 0, reducing = 0;
    for (int g = 0; g < n_groups; g++)
        for (int c = 0; c < groups[g].n_channels; c++)
            for (int l = 0; l < groups[g].n_limits; l++) {
                struct limit_state *ls = &ch_runtime[g][c].st[l];
                total++;
                if (ls->overs > 0) with_overs++;
                if (ls->gain < 0.999) reducing++;
            }
    msg_append(&p, &remaining,
               "PROTECT: %d limits, %d with recorded overs, "
               "%d currently reducing gain\n\n",
               total, with_overs, reducing);
    msg_append(&p, &remaining, "%-5s  %-4s  %-22s  %6s  %8s  %6s  %7s\n",
               "GROUP", "CHAN", "LIMIT", "OVERS", "TIME", "GAIN", "dB");

    int hidden = 0;
    for (int g = 0; g < n_groups; g++) {
        struct group *gr = &groups[g];
        for (int c = 0; c < gr->n_channels; c++) {
            for (int l = 0; l < gr->n_limits; l++) {
                struct limit_state *ls = &ch_runtime[g][c].st[l];
                if (!show_all && !limit_state_active(ls)) {
                    hidden++;
                    continue;
                }
                char grpbuf[8], timebuf[16];
                snprintf(grpbuf, sizeof(grpbuf), "grp%d", g);
                snprintf(timebuf, sizeof(timebuf), "%.1fms",
                         overs_to_ms(ls->overs));
                msg_append(&p, &remaining,
                           "%-5s  %-4s  %-22s  %6ld  %8s  %6.3f  %7.2f%s\n",
                           grpbuf, gr->ch_names[c], gr->limits[l].name,
                           ls->overs, timebuf, ls->gain, gain_to_db(ls->gain),
                           gr->limits[l].enabled ? "" : "  OFF");
            }
        }
    }
    if (!show_all && hidden > 0) {
        msg_append(&p, &remaining,
                   "\n(%d idle limit%s hidden - \"lmc <idx> all\" for "
                   "everything, \"lmc <idx> json\" for machine-readable "
                   "output)\n",
                   hidden, hidden == 1 ? "" : "s");
    }
    if (n_links > 0) {
        msg_append(&p, &remaining, "\n%-12s  %-12s  %-6s  %9s  %7s\n",
                   "LINK", "CHANNEL", "ROLE", "LINK_dB", "FAULTS");
        for (int li = 0; li < n_links; li++) {
            struct link_cfg *lk = &links[li];
            for (int m = 0; m < lk->n; m++) {
                int ch = lk->member_ch[m];
                double ldb = gain_to_db(a_load_d(&link_now[ch]));
                long lf = a_load_l(&link_faults[ch]);
                if (!show_all && ldb > -0.01 && lf == 0) continue;
                msg_append(&p, &remaining, "%-12s  %-12s  %-6s  %9.2f  %7ld%s\n",
                           lk->name, lk->member_name[m],
                           lk->drives[m] ? "drive" : "follow", ldb, lf,
                           a_load_i(&lk->enabled) ? "" : "  OFF");
            }
        }
    }
    return 0;
}

const char *
bflogic_message(void)
{
    return msg;
}
