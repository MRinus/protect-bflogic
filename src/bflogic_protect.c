/*
 * bflogic_protect.c  (v2 -- generalized signal-chain model)
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
static char msg[1024];

#define MAX_MAPPED_CHANNELS 64
static int chan_group[MAX_MAPPED_CHANNELS];
static int chan_idx[MAX_MAPPED_CHANNELS];

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

/* ---- Kern: Sample-Verarbeitung ----------------------------------------- */

static inline void
process_channel(struct group *g, int gi, int ci, double *samples, int n)
{
    struct channel_runtime *rt = &ch_runtime[gi][ci];
    int i, l;
    double s, mag, target, alpha, combined;

    for (i = 0; i < n; i++) {
        s = samples[i];
        mag = fabs(s);
        combined = 1.0;

        for (l = 0; l < g->n_limits; l++) {
            struct limit_cfg *lc = &g->limits[l];
            struct limit_state *ls = &rt->st[l];
            double detect_mag = mag;

            if (lc->freq_weighted) {
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
        samples[i] = s * combined;
    }
}

static void
output_timed(void *buf, int channel)
{
    int gi, ci;

    if (channel < 0 || channel >= MAX_MAPPED_CHANNELS ||
        chan_group[channel] < 0)
    {
        return;
    }
    gi = chan_group[channel];
    ci = chan_idx[channel];

    if (realsize == 4) {
        float *fbuf = (float *)buf;
        double tmp[8192];
        int n = block_length;
        for (int i = 0; i < n; i++) tmp[i] = fbuf[i];
        process_channel(&groups[gi], gi, ci, tmp, n);
        for (int i = 0; i < n; i++) fbuf[i] = (float)tmp[i];
    } else {
        process_channel(&groups[gi], gi, ci, (double *)buf, block_length);
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
        } else {
            fprintf(stderr, "PROTECT: expected 'chain', 'group' or "
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

    bfevents->output_timed = output_timed;
    *fork_mode = BF_FORK_DONT_FORK;
    return 0;
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
    fprintf(stderr, "PROTECT: initialised, %d chain(s), %d group(s), "
            "realsize=%d\n", n_chains, n_groups, realsize);
    memset(msg, 0, sizeof(msg));
    return 0;
}

int
bflogic_command(const char params[])
{
    char *p = msg;
    memset(msg, 0, sizeof(msg));
    for (int g = 0; g < n_groups; g++) {
        for (int c = 0; c < groups[g].n_channels; c++) {
            for (int l = 0; l < groups[g].n_limits; l++) {
                p += sprintf(p, "grp%d/%s/%s: overs=%ld gain=%.3f\n",
                             g, groups[g].ch_names[c],
                             groups[g].limits[l].name,
                             ch_runtime[g][c].st[l].overs,
                             ch_runtime[g][c].st[l].gain);
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
