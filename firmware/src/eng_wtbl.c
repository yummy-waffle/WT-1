/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* WTBL: the dubstep wavetable engine (1.7): engine 1's index, where retired DIGITAL's slot was.
 *
 * The stores and the editor protocol name engines by index and ENGINES[] keeps its numbers, so a stored
 * DIGITAL sound still converts to FM6 before this engine is reached (ui.c apply_preset_to, upreset.c,
 * project.c: every path tests == ENGI_DIGITAL). A stored PHASE / TRIO / WHEEL sound of an older project or
 * user preset keeps its number and plays on; only the browsing order changes (engines.c ENGINE_ORDER:
 * PHASE, TRIO and WHEEL are out -- their ground is covered here, by ANALOG and by FM6).
 *
 * Eight single-cycle tables, walked as one row: TABLE picks the table, MORPH slides into the NEXT one (the
 * last folds back to the first, so the row is a loop). ENV adds its amount to the morph (the classic
 * keyed-bass sweep with no filter movement needed) and LFO adds its own -- that one is the wobble: the
 * track's LFO (RATE / WAVE / SYNC as everywhere) at depth LFO walks the tables in time.
 *   0 SINE    1 SAW     2 SQUARE   3 POLEZ   (the buzz ramp: saw x |sin|)
 *   4 PULSE3  5 TEETH   (half-wave stack)    6 STACK (odd harmonics)   7 BELLZ (stack x |cos|)
 * Per voice: ph[0] the oscillator phase, ph[1] a free-running LFO phase whose rate follows the track's
 * (per block, spread across the voices by UNISON detune -- v->fine); s[0] s[1] the resonant SVF states,
 * kept over a retrigger (keep 0x03). SUB E5 mixes in a sine an octave below the key (a sub for the club
 * PA); DRV E6 pre-gains the voice into the tanh soft clip before the filter (drive past ~96 costs
 * headroom, not clarity: the master limiter still has the last word). FLT moves the cutoff, PIT the
 * pitch. MONO presets slide with GLIDE; POLY chords stay clean. No part state (mem 0): it shares nothing. */

static const char *const N_WTBL_TABLE[] = {"SINE", "SAW", "SQR", "POLEZ", "PULSE3", "TEETH", "STACK", "BELLZ"};

#define WT_N 8u                          /* tables */
#define WT_PTS 256u                      /* points per cycle */
#define WT_LVL 32000                     /* peak normalisation (headroom under full scale) */
static int16_t wtbl_tab[WT_N][WT_PTS];   /* Q15, DC-free, filled once by wtbl_gen() (tests/wtbl_test.c checks the bounds) */

/* the tables from harmonic sums, integer only (no libm: this file builds into the bare-metal image too).
 * sinpi(k / d): k / d in eighths of a turn, the quarter table read by symmetry; sine_i() wants a Q32
 * phase, whose eighth of a turn is 2^29. */
static inline int32_t wtbl_sin_e(uint32_t k, uint32_t d)
{
    uint32_t m = k % d, q = m * d;                       /* (unused guard against odd ratios: none here) */
    (void)q;
    if (m <= d / 4u)
        return sine_i(m << 29);
    if (m <= d / 2u)
        return sine_i((d / 2u - m) << 29);
    return -sine_i((m - d / 2u) << 29);
}
static inline int32_t wtbl_cos_e(uint32_t k, uint32_t d) { return wtbl_sin_e(k + d / 4u, d); }

static void wtbl_gen(void)                       /* idempotent: fills wtbl_tab once */
{
    static uint8_t done;
    /* {harmonics, odd only?, envelope kind: 0 none, 1 |sin|, 2 |cos|, 3 half-wave rectify} */
    static const struct { uint8_t nh, odd, env; } H[WT_N] = {
        {1, 0, 0},        /* SINE */
        {24, 0, 0},       /* SAW: all harmonics, 1/k */
        {16, 1, 0},       /* SQUARE: odd, 1/k */
        {20, 0, 1},       /* POLEZ: saw x |sin| -- the buzz ramp */
        {12, 0, 0},       /* PULSE3: duty 1/3 weights, made below */
        {6, 0, 3},        /* TEETH: half-wave rectified stack */
        {9, 1, 0},        /* STACK: odd, 1/k (the fizzier square) */
        {10, 0, 2},       /* BELLZ: stack x |cos| */
    };
    int t, i, k;
    if (done)
        return;
    for (t = 0; t < (int)WT_N; t++) {
        int32_t v[WT_PTS], mx = 0, mean, sum = 0, a;
        for (i = 0; i < (int)WT_PTS; i++) {
            int32_t s = 0;
            for (k = 1; k <= H[t].nh; k++) {
                if (H[t].odd && !(k & 1))
                    continue;
                if (t == 4)                                   /* PULSE3: 2 (1 - cos(2 pi k / 3)) sin(k x) / (pi k) */
                    s += ((20480 - wtbl_cos_e(k, 3u)) * wtbl_sin_e((uint32_t)k, 8u)) >> 15;
                else if (H[t].env == 3)                       /* TEETH: (1 - cos(pi k)) sin(k x) / (pi k): even k drop */
                    s += (k & 1) ? ((2u * 32767u * wtbl_sin_e((uint32_t)k, 8u)) >> 15) : 0;
                else                                          /* all / odd stacks: sin(k x) / k, x in eighths */
                    s += wtbl_sin_e((uint32_t)k, 8u) / k;
            }
            v[i] = s;
        }
        {   /* the envelopes: |sin| and |cos| need the value at x, not only its harmonics */
            for (i = 0; i < (int)WT_PTS; i++) {
                if (H[t].env == 1)                            /* POLEZ: x |sin(x)| */
                    v[i] = (v[i] * abs(wtbl_sin_e((uint32_t)i, 8u))) >> 15;
                else if (H[t].env == 2)                       /* BELLZ: x |cos(x)| */
                    v[i] = (v[i] * abs(wtbl_cos_e((uint32_t)i, 8u))) >> 15;
            }
        }
        for (i = 0; i < (int)WT_PTS; i++)
            sum += v[i];
        mean = sum / (int)WT_PTS;
        for (i = 0; i < (int)WT_PTS; i++) {
            v[i] -= mean;                                     /* DC-free */
            a = v[i] < 0 ? -v[i] : v[i];
            if (a > mx)
                mx = a;
        }
        if (!mx)                                              /* a silent table falls back to a sine, never clicks */
            for (i = 0; i < (int)WT_PTS; i++)
                v[i] = wtbl_sin_e((uint32_t)i, 8u);
        for (i = 0; i < (int)WT_PTS; i++) {                   /* peak-normalise to WT_LVL */
            int32_t q = (int32_t)(((int64_t)v[i] * WT_LVL) / mx);
            wtbl_tab[t][i] = (int16_t)(q > WT_LVL ? WT_LVL : q < -WT_LVL ? -WT_LVL : q);
        }
    }
    done = 1;
}

/* the table point at phase p (Q32): linear between the 256 points */
static inline int32_t wtbl_at(const int16_t *tab, uint32_t p)
{
    uint32_t i = p >> 24;                          /* 0..255 */
    int32_t a = tab[i], b = tab[(i + 1u) & (WT_PTS - 1u)];
    return a + (((b - a) * (int32_t)((p >> 8) & 0xFFFFu)) >> 16);
}

static void wtbl_note_on(track_t *t, voice_t *v)
{
    (void)t;
    wtbl_gen();                                    /* the tables exist before the first render (boot-safe) */
    v->s[0] = v->s[1] = 0;                         /* the SVF starts from silence (as every engine's filters) */
}

static void wtbl_render(track_t *t, voice_t *v, int32_t *out, uint32_t n, const vmod_t *m)
{
    const int16_t *p = t->p;
    uint32_t sel = (uint32_t)clamp(p[P_E0], 0, (int32_t)WT_N - 1), i;
    int32_t ic1 = v->s[0], ic2 = v->s[1];          /* state in locals: out[] may alias v->s[] */
    uint32_t ph = v->ph[0], lph = v->ph[1];
    uint32_t inc = m->inc, lbase, linc;
    int32_t morph, cut, res = clamp(p[P_E4], 0, 127), drv = clamp(p[P_E6], 0, 127), sub = clamp(p[P_E5], 0, 127);
    int32_t pre, lvl, mm, t0, fr;
    const int16_t *ta, *tb;
    tsvf_t c;

    /* morph: base + envelope amount + LFO amount, in Q8-of-a-row steps (a row is eight tables, 0..7<<8) */
    morph = (p[P_E1] << 8) + mulq15(m->envq15, p[P_E2] << 8) + mulq15(m->shape - (64 << 8), p[P_E3] << 8);
    mm = clamp((int32_t)sel * 256 + (morph >> 8), 0, (int32_t)WT_N * 256 - 1);
    t0 = mm >> 8;                                  /* the table under the morph */
    fr = mm & 255;                                 /* .. and its fraction, Q8 */
    ta = wtbl_tab[t0];
    tb = wtbl_tab[(t0 + 1) % WT_N];                /* the row loops: BELLZ morphs back to SINE */

    /* the wobble runs on the track's LFO RATE but per voice: ph[1] free-runs, UNISON detune spreads it.
     * LFO_INC advances one step per control block (voice.c), so CTL samples carry one step here */
    lbase = LFO_INC[t->p[P_LRATE] & 127];
    linc = lbase + (uint32_t)(((int32_t)(lbase >> 12) * v->fine) >> 2);

    cut = clamp((64 << 8) + m->cutoff + ((p[P_E7] - 64) << 3), 0, 127 << 8);
    tsvf_coef(&c, cut, res);
    pre = 1024 + drv * drv / 4;                    /* DRV 0..127: x1 .. x4.1 into the clip */
    lvl = 1024 - drv * 3;                          /* ... and back down: the peaks saturate, the level holds */
    for (i = 0; i < n; i++) {
        int32_t s, y;
        ph += inc;
        lph += linc;
        s = wtbl_at(ta, ph) + mulq15(wtbl_at(tb, ph) - wtbl_at(ta, ph), fr << 7);
        if (sub)                                   /* SUB: a sine an octave below, mixed dry */
            s += mulq15(sine_i(ph >> 1) - s, sub << 8);
        y = softclip((s * pre) >> 10);             /* the folder before the filter: the growl's grit */
        y = (y * lvl) >> 10;
        s = tsvf_lp(&c, y, &ic1, &ic2);            /* the resonant low-pass after it: the vowel rides on it */
        out[i] += voice_amp(s, m, i);
    }
    v->s[0] = ic1;
    v->s[1] = ic2;
    v->ph[0] = ph;
    v->ph[1] = lph;
}

/* {TABLE, MORPH, ENV, LFO, RES, SUB, DRV, FREQ} */
static const preset_t WTBL_PRESETS[] = {
    {"WOBBLE", {4, 64, 96, 100, 70, 40, 80, 60}, {0, 60, 127, 40}, 0, 1, FX(20, 0, 24, 20), PAT(2)},
    {"VOWEL",  {6, 96, 110, 60, 90, 30, 90, 50}, {0, 50, 127, 30}, 0, 1, FX(30, 10, 30, 30), PAT(2)},
    {"SUBFOLD",{2, 32, 40, 20, 30, 100, 70, 40}, {0, 70, 127, 30}, 0, 1, FX(10, 0, 10, 10), PAT(8)},
    {"RAMP",   {1, 24, 60, 30, 50, 50, 60, 55}, {0, 60, 127, 35}, 0, 1, FX(25, 0, 20, 15), PAT(1)},
};

static const engine_t ENG_WTBL = {
    .name = "WTBL",
    .page_title = {"WAVE", "MOTION"},
    .edit = {
        {"TABLE", F_ENUM, 0, WT_N - 1, 1, N_WTBL_TABLE, 0},
        {"MORPH", F_PCT, 0, 127, 64, 0, 0},
        {"ENV", F_PCT, 0, 127, 64, 0, 0},          /* envelope -> morph */
        {"LFO", F_PCT, 0, 127, 0, 0, 0},           /* LFO shape -> morph (the wobble) */
        {"RES", F_PCT, 0, 127, 40, 0, 0},
        {"SUB", F_PCT, 0, 127, 0, 0, 0},           /* sine one octave below */
        {"DRV", F_PCT, 0, 127, 60, 0, 0},          /* tanh folder before the filter */
        {"FREQ", F_CUTOFF, 0, 127, 60, 0, 0},      /* FLT moves it, as everywhere */
    },
    .presets = WTBL_PRESETS,
    .npresets = NELEM(WTBL_PRESETS),
    .note_on = wtbl_note_on,
    .render = wtbl_render,
    .knob = {P_E0, P_E1, P_E7, P_E4},
    .keep = 0x03,                                  /* the filter states over a retrigger */
};
