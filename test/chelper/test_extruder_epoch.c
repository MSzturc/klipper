// Standalone unit tests for the extruder smoother epoch cache.
// Build & run (WSL, from repo root):
//   gcc -O2 -std=gnu99 -DUNIT_TEST -I klippy/chelper \
//       test/chelper/test_extruder_epoch.c klippy/chelper/integrate.c \
//       klippy/chelper/trapq.c -lm \
//       -o /tmp/test_ee && /tmp/test_ee
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "integrate.h"
#include "trapq.h"
#include "kin_extruder.c"

static int failures;
#define CHECK(cond) do { if (!(cond)) { \
    printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } } while (0)
#define CLOSE(a, b, tol) CHECK(fabs((a) - (b)) <= (tol) * (1.0 + fabs(b)))

// Evaluate a polynomial with coefficients c[0..deg] (ascending) via Horner.
static double
poly_eval(const double *c, int deg, double x)
{
    double v = 0.;
    for (int i = deg; i >= 0; --i)
        v = v * x + c[i];
    return v;
}

static void
test_bary_exactness_for_degree(int npts)
{
    int deg = npts - 1;
    double c[16];
    for (int i = 0; i <= deg; ++i)               // arbitrary, well-scaled coeffs
        c[i] = ((i & 1) ? -1.0 : 1.0) * (0.7 + 0.13 * i) / (1.0 + i);
    double a = 2.81, b = 3.07;                   // realistic absolute-time span
    double x[16], w[16], f[16];
    bary_nodes(a, b, npts, x);
    bary_weights(npts, w);
    for (int k = 0; k < npts; ++k) {
        CHECK(x[k] >= a - 1e-12 && x[k] <= b + 1e-12);
        f[k] = poly_eval(c, deg, x[k]);
    }
    for (int k = 0; k < npts; ++k)               // exact at the nodes
        CLOSE(bary_eval(npts, x, w, f, x[k]), f[k], 1e-12);
    for (int s = 0; s <= 40; ++s) {              // exact between the nodes
        double xq = a + (b - a) * s / 40.0;
        CLOSE(bary_eval(npts, x, w, f, xq), poly_eval(c, deg, xq), 1e-9);
    }
}

static void
test_bary_exactness(void)
{
    test_bary_exactness_for_degree(8);           // degree 7
    test_bary_exactness_for_degree(15);          // degree 14 (max cache degree)
    printf("bary-interp %s\n", failures ? "FAILED" : "OK");
}

// --- shared test helpers (trapq + extruder construction) -------------------

// Build a trapq of `count` constant-velocity cruise moves, each `dur` seconds
// long and `len` mm long, starting at print_time 0. Returns the trapq; the
// caller frees it. axes_r is (1,0,0): pure X motion.
static struct trapq *
build_cruise_trapq(int count, double dur, double len)
{
    struct trapq *tq = trapq_alloc();
    double pt = 0.;
    double v = len / dur;
    for (int i = 0; i < count; ++i) {
        trapq_append(tq, pt, 0., dur, 0.,
                     i * len, 0., 0.,
                     1., 0., 0.,
                     v, v, 0.);
        pt += dur;
    }
    trapq_check_sentinels(tq);
    return tq;
}

// Return the nth move after the head sentinel (0 = first). trapq_add_move may
// insert null-fill moves on time gaps (including before the first appended
// move), so an index is not necessarily "the nth appended move" -- the tests
// only use nth_move for relative index arithmetic, which stays consistent
// regardless of any null-fill moves.
static struct move *
nth_move(struct trapq *tq, int n)
{
    struct move *m = list_first_entry(&tq->moves, struct move, node);
    m = list_next_entry(m, node);                   // skip head sentinel
    for (int i = 0; i < n; ++i)
        m = list_next_entry(m, node);
    return m;
}

static void
test_pa_range_walk(void)
{
    struct trapq *tq = build_cruise_trapq(20, 0.01, 1.0);   // 20 moves x 10 ms
    struct smoother sm;
    double coeffs[5] = { 15./8., 0., -15., 0., 30. };
    init_smoother(5, coeffs, 0.02, &sm);                    // hst = 0.01
    struct move *m10 = nth_move(tq, 10);                    // 11th real move

    // pa_range_walk produces finite values and reports the window-edge moves.
    double pos_w, vel_w;
    const struct move *f, *l;
    pa_range_walk(m10, 'x', 0.005, &sm, &pos_w, &vel_w, &f, &l);
    CHECK(isfinite(pos_w) && isfinite(vel_w));

    // Symmetric smoother (all odd-index coeffs == 0) -> t_offs == 0, so the
    // effective window is centered on m10+0.005: [0.095, 0.115] in absolute time.
    // m10 covers [0.10, 0.11] -> window spans into m9 and m11 -> f = move 9, l = move 11.
    CHECK(f == nth_move(tq, 9));
    CHECK(l == nth_move(tq, 11));

    // fast branch: a wide-enough move so the window stays fully inside -> f==l==m
    struct trapq *tq2 = build_cruise_trapq(5, 1.0, 50.0);   // 1 s moves
    struct move *w = nth_move(tq2, 2);
    double p, v; const struct move *ff, *ll;
    pa_range_walk(w, 'x', 0.5, &sm, &p, &v, &ff, &ll);
    CHECK(ff == w && ll == w);

    trapq_free(tq);
    trapq_free(tq2);
    printf("pa-range-walk %s\n", failures ? "FAILED" : "OK");
}

static void
test_epoch_detection(void)
{
    // epoch_bounds against the documented formula
    struct move f, l;
    memset(&f, 0, sizeof(f)); memset(&l, 0, sizeof(l));
    f.print_time = 2.80; f.move_t = 0.20;        // f covers [2.80, 3.00)
    l.print_time = 3.00; l.move_t = 0.20;        // l covers [3.00, 3.20)
    double hst = 0.012, lo, hi;
    epoch_bounds(&f, &l, hst, &lo, &hi);
    CLOSE(lo, fmax(2.80 + hst, 3.00 - hst), 1e-15);   // 2.988
    CLOSE(hi, fmin(3.00 + hst, 3.20 - hst), 1e-15);   // 3.012

    // strict-open hit test
    struct epoch_cache c;
    memset(&c, 0, sizeof(c));
    c.valid = 1; c.epoch_lo = 2.90; c.epoch_hi = 3.10;
    CHECK( epoch_hit(&c, 3.00));                  // strictly inside
    CHECK(!epoch_hit(&c, 2.90));                  // exact boundary -> miss
    CHECK(!epoch_hit(&c, 3.10));                  // exact boundary -> miss
    CHECK(!epoch_hit(&c, 3.20));                  // outside
    c.valid = 0;
    CHECK(!epoch_hit(&c, 3.00));                  // invalid -> never a hit

    // sentinel detection
    struct move head, tail, real;
    memset(&head, 0, sizeof(head)); memset(&tail, 0, sizeof(tail));
    memset(&real, 0, sizeof(real));
    head.print_time = -1.0;
    tail.print_time = 9999999999999999.9; tail.move_t = 9999999999999999.9;
    real.print_time = 1.0; real.move_t = 0.01;
    CHECK( is_sentinel_move(&head));
    CHECK( is_sentinel_move(&tail));
    CHECK(!is_sentinel_move(&real));

    // alloc default: cache enabled, nothing valid yet
    struct stepper_kinematics *sk = extruder_stepper_alloc();
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    CHECK(es->epoch_cache_enabled == 1);
    for (int j = 0; j < 3; ++j)
        CHECK(es->cache[j].valid == 0);
    extruder_stepper_free(sk);

    printf("epoch-detection %s\n", failures ? "FAILED" : "OK");
}

// Build an extruder stepper whose X smoother has order n. The kernel is the
// order-5 pa-smoother zero-padded to length n: same smoother weight function,
// but sm->n == n exercises the n+3 cache-node count across the order range.
static struct stepper_kinematics *
make_extruder_n(int n)
{
    struct stepper_kinematics *sk = extruder_stepper_alloc();
    double coeffs[12] = { 15./8., 0., -15., 0., 30., 0,0,0,0,0,0,0 };
    int rc = extruder_set_smoothing_params(sk, 'x', n, coeffs, 0.024, 0.);
    CHECK(rc == 0);                               // hst = 0.012
    return sk;
}

// Evaluate axis position+velocity for an absolute window-center time, going
// through the public pa_range_integrate (cache wrapper). Locates a real move.
static void
eval_axis(struct extruder_stepper *es, struct trapq *tq, int axis,
          double tau_center, double *pos, double *vel)
{
    const struct smoother *sm = &es->sm[axis - 'x'];
    double want = tau_center - sm->t_offs;
    struct move *m = nth_move(tq, 0);
    while (want >= m->print_time + m->move_t
           && !is_sentinel_move(list_next_entry(m, node)))
        m = list_next_entry(m, node);
    pa_range_integrate(es, m, axis, want - m->print_time, pos, vel);
}

static void
test_epoch_cache_for_n(int n)
{
    struct trapq *tq = build_cruise_trapq(60, 0.008, 0.8);   // 60 moves x 8 ms
    struct stepper_kinematics *sk = make_extruder_n(n);
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);

    double times[800]; int nt = 0;
    for (double t = 0.05; t < 0.40; t += 0.001) {
        times[nt++] = t;
        if (nt % 7 == 0 && nt < 798) times[nt++] = t - 0.0006;
    }
    static double ref_pos[800], ref_vel[800];
    for (int pass = 0; pass < 2; ++pass) {
        extruder_set_epoch_cache_enabled(sk, pass);   // 0 = off, 1 = on
        for (int j = 0; j < 3; ++j) es->cache[j].valid = 0;
        diag_epoch_hits = diag_epoch_builds = 0;
        for (int i = 0; i < nt; ++i) {
            double pos, vel;
            eval_axis(es, tq, 'x', times[i], &pos, &vel);
            if (pass == 0) { ref_pos[i] = pos; ref_vel[i] = vel; }
            else { CLOSE(pos, ref_pos[i], 1e-9); CLOSE(vel, ref_vel[i], 1e-9); }
        }
        if (pass == 1) {
            CHECK(diag_epoch_builds > 0);
            CHECK(diag_epoch_hits > 100);
        }
    }
    extruder_stepper_free(sk);
    trapq_free(tq);
}

static void
test_epoch_cache(void)
{
    test_epoch_cache_for_n(5);
    test_epoch_cache_for_n(7);
    test_epoch_cache_for_n(9);
    test_epoch_cache_for_n(11);
    printf("epoch-cache %s\n", failures ? "FAILED" : "OK");
}

static void
test_cache_invalidation(void)
{
    // (a) synthetic move (unlinked, node.next == NULL) bypasses the cache
    struct stepper_kinematics *sk = make_extruder_n(5);
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    struct move syn;
    memset(&syn, 0, sizeof(syn));
    syn.move_t = 1000.; syn.start_pos.x = 5.0;
    CHECK(syn.node.next == NULL);
    double pos = extruder_calc_position(sk, &syn, 500.);
    (void)pos;
    CHECK(es->cache[0].valid == 0);                // nothing cached

    // (b) smoothing-param change invalidates
    es->cache[0].valid = 1;
    double coeffs[5] = { 15./8., 0., -15., 0., 30. };
    extruder_set_smoothing_params(sk, 'x', 5, coeffs, 0.024, 0.);
    CHECK(es->cache[0].valid == 0);

    // (c) batch change (last_flush_time) triggers the invalidation branch.
    //     extruder_calc_position rebuilds the cache right afterwards, so
    //     cache[0].valid is not a stable post-condition; the recorded flush
    //     time is -- and it proves the invalidation branch ran.
    struct trapq *tq = build_cruise_trapq(20, 0.01, 1.0);
    es->cache[0].valid = 1;
    es->cache_last_flush_time = 1.0;
    sk->last_flush_time = 2.0;
    struct move *rm = nth_move(tq, 10);
    extruder_calc_position(sk, rm, 0.005);
    CHECK(es->cache_last_flush_time == 2.0);

    // (d) time_offset change via the public extruder_set_pressure_advance
    //     must not produce stale cache hits.
    sk->last_flush_time = es->cache_last_flush_time;  // avoid batch invalidation
    double pa_params[1] = { 0.02 };
    struct move *dm = nth_move(tq, 10);
    extruder_set_epoch_cache_enabled(sk, 1);
    extruder_calc_position(sk, dm, 0.005);
    // Offset 0.001 s keeps the shifted query strictly inside the warmed epoch.
    extruder_set_pressure_advance(sk, 0.05, 1, pa_params,
                                  pressure_advance_linear_model_func, 0.001);
    diag_epoch_hits = 0;
    double on = extruder_calc_position(sk, dm, 0.005);
    CHECK(diag_epoch_hits >= 1);                      // a cache hit really occurred
    extruder_set_epoch_cache_enabled(sk, 0);
    for (int j = 0; j < 3; ++j) es->cache[j].valid = 0;
    double off = extruder_calc_position(sk, dm, 0.005);
    CLOSE(on, off, 1e-9);                             // and that hit was correct

    extruder_stepper_free(sk);
    trapq_free(tq);
    printf("cache-invalidation %s\n", failures ? "FAILED" : "OK");
}

// Build a cruise trapq whose first move starts at absolute X position x0.
// Used to reproduce the large-absolute-value scenario after a long print.
static struct trapq *
build_cruise_trapq_at(int count, double dur, double len, double x0)
{
    struct trapq *tq = trapq_alloc();
    double pt = 0.;
    double v = len / dur;
    for (int i = 0; i < count; ++i) {
        trapq_append(tq, pt, 0., dur, 0.,
                     x0 + i * len, 0., 0.,
                     1., 0., 0.,
                     v, v, 0.);
        pt += dur;
    }
    trapq_check_sentinels(tq);
    return tq;
}

// Test that the epoch cache reproduces pa_range_walk accurately even when
// the absolute extruder position is large (simulating the tail of a long
// print, where pos_integral values grow to ~1e7 mm).  The critical quantity
// is the *difference* between two adjacent evaluations, because that is what
// the secant solver consumes; catastrophic cancellation in bary_eval's
// alternating-sign sum corrupts this difference when fpos[] values are large.
//
// Design rationale for parameters and tolerance:
//   n=9 (npts=12 nodes): long enough alternating-weight sum to amplify
//   cancellation; n=11 happens to produce zero error at x0=1e7 due to
//   IEEE-754 alignment, so n=9 is more reliable across platforms.
//   x0=1e7 mm: fpos[k] ~ 1e7, step differences ~ 1e-3; ratio ~1e10 means
//   ~10 digits of cancellation loss -> errors of O(1e-8).
//   Tolerance 2e-9 mm: tighter than the observed pre-fix max error (~9.3e-9)
//   but well above the post-fix rounding floor (~1e-13) so the test passes
//   cleanly after centering and fails reliably before it.
static void
test_epoch_cache_large_magnitude(void)
{
    int n = 9;
    double x0 = 1e7;
    struct trapq *tq = build_cruise_trapq_at(60, 0.008, 0.8, x0);
    struct stepper_kinematics *sk = make_extruder_n(n);
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);

    // Ground truth: cache disabled, 350 evaluation points.
    extruder_set_epoch_cache_enabled(sk, 0);
    for (int j = 0; j < 3; ++j) es->cache[j].valid = 0;
    static double ref_pos[350];
    int nt = 0;
    for (double t = 0.05; t < 0.40; t += 0.001) {
        double pos, vel;
        eval_axis(es, tq, 'x', t, &pos, &vel);
        ref_pos[nt++] = pos;
    }

    // Cache ON: measure maximum absolute difference error over adjacent pairs.
    // Adjacent pairs (i, i+1) represent secant-solver step inputs.
    extruder_set_epoch_cache_enabled(sk, 1);
    for (int j = 0; j < 3; ++j) es->cache[j].valid = 0;
    diag_epoch_hits = diag_epoch_builds = 0;

    double times[350]; int nt2 = 0;
    for (double t = 0.05; t < 0.40; t += 0.001) times[nt2++] = t;
    assert(nt2 == nt);                  // both passes must sample the same grid

    double max_diff_err = 0.;
    for (int i = 0; i < nt2 - 1; ++i) {
        double pa, va, pb, vb;
        eval_axis(es, tq, 'x', times[i],   &pa, &va);
        eval_axis(es, tq, 'x', times[i+1], &pb, &vb);
        double cache_diff = pa - pb;
        double ref_diff   = ref_pos[i] - ref_pos[i+1];
        double e = fabs(cache_diff - ref_diff);
        if (e > max_diff_err) max_diff_err = e;
    }
    CHECK(diag_epoch_builds > 0);
    CHECK(diag_epoch_hits > 100);

    // 2e-9 mm absolute tolerance on the adjacent-pair difference.
    // Pre-fix: max error ~9.3e-9 (FAIL).  Post-fix: max error < 1e-13 (PASS).
    CHECK(max_diff_err <= 2e-9);

    extruder_stepper_free(sk);
    trapq_free(tq);
    printf("epoch-cache-large-magnitude %s\n", failures ? "FAILED" : "OK");
}

int
main(void)
{
    test_bary_exactness();
    test_pa_range_walk();
    test_epoch_detection();
    test_epoch_cache();
    test_cache_invalidation();
    test_epoch_cache_large_magnitude();
    return failures ? 1 : 0;
}
