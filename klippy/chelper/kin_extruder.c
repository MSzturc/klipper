// Extruder stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // tanh
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "integrate.h" // struct smoother
#include "list.h" // list_node
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_distance

struct pressure_advance_params;
typedef double (*pressure_advance_func)(
        double, double, struct pressure_advance_params *pa_params);

struct pressure_advance_params {
    union {
        struct {
            double pressure_advance;
        };
        struct {
            double linear_advance, nonlinear_offset, linearization_velocity;
        };
        double params[3];
    };
    double active_print_time;
    pressure_advance_func pa_func;
    struct list_node node;
};

// Smoother order n is at most 12, the cached position integral has degree
// n+2, so the barycentric interpolant uses npts = n+3 <= 15 nodes. The
// buffers are sized one slot beyond that.
#define EPOCH_CACHE_NPTS_MAX 16

struct epoch_cache {
    int valid;
    int npts;
    double epoch_lo, epoch_hi;
    double tau0;     // absolute time origin; nodes[] are stored relative to tau0
    // fpos0/fvel0 are the reference offsets subtracted from fpos[]/fvel[]
    // before the barycentric sum.  Mirroring the time-axis centering (tau0)
    // onto the value axis prevents catastrophic cancellation when absolute
    // integral values are large (e.g. extruder position after a long print).
    double fpos0, fvel0;
    double nodes[EPOCH_CACHE_NPTS_MAX], wbary[EPOCH_CACHE_NPTS_MAX];
    double fpos[EPOCH_CACHE_NPTS_MAX], fvel[EPOCH_CACHE_NPTS_MAX];
};

static const double pa_smoother_coeffs[] = {15./8., 0., -15., 0., 30.};

// Without pressure advance, the extruder stepper position is:
//     extruder_position(t) = nominal_position(t)
// When pressure advance is enabled, additional filament is pushed
// into the extruder during acceleration (and retracted during
// deceleration). The formula is:
//     pa_position(t) = (nominal_position(t)
//                       + pressure_advance * nominal_velocity(t))
// The nominal position and velocity are then smoothed using a weighted average:
//     smooth_position(t) = (
//         definitive_integral(nominal_position(x+t_offs) * smoother(t-x) * dx,
//                             from=t-smooth_time/2, to=t+smooth_time/2)
//     smooth_velocity(t) = (
//         definitive_integral(nominal_velocity(x+t_offs) * smoother(t-x) * dx,
//                             from=t-smooth_time/2, to=t+smooth_time/2)
// and the final pressure advance value calculated as
//     smooth_pa_position(t) = smooth_position(t) + pa_func(smooth_velocity(t))
// where pa_func(v) = pressure_advance * v for linear velocity model or a more
// complicated function for non-linear pressure advance models.

// Calculate the definitive integral of extruder for a given move
static inline void
pa_move_integrate(const struct move *m, int axis, double base
                  , double t0, const smoother_antiderivatives *ad
                  , double *pos_integral, double *pa_velocity_integral)
{
    // Calculate base position and velocity with pressure advance
    int can_pressure_advance = m->axes_r.x > 0. || m->axes_r.y > 0.;
    double smooth_velocity;
    // Calculate definitive integral
    *pos_integral += integrate_move(m, axis, base, t0, ad,
                                    can_pressure_advance ? &smooth_velocity
                                                         : NULL);
    if (can_pressure_advance) {
        *pa_velocity_integral += smooth_velocity;
    }
}

// Compute the open epoch interval (lo, hi) over which both the first move f
// and the last move l fully cover the smoother half-window hst.  Caller
// discards the epoch when lo >= hi (degenerate or sub-window gap).
static void
epoch_bounds(const struct move *f, const struct move *l, double hst
             , double *lo, double *hi)
{
    double f_lo = f->print_time + hst;
    double f_hi = f->print_time + f->move_t + hst;
    double l_lo = l->print_time - hst;
    double l_hi = l->print_time + l->move_t - hst;
    *lo = f_lo > l_lo ? f_lo : l_lo;
    *hi = f_hi < l_hi ? f_hi : l_hi;
}

// Returns true iff tau lies strictly inside the cached epoch interval.
// The interval is open on both ends: boundary hits are treated as misses
// so that the caller always recomputes at the exact transition point.
static inline int
epoch_hit(const struct epoch_cache *c, double tau)
{
    return c->valid && c->epoch_lo < tau && tau < c->epoch_hi;
}

// trapq sentinels (NEVER_TIME is defined in trapq.c, not exported in trapq.h):
//   head: print_time = -1.0;  tail: print_time = move_t = NEVER_TIME (~1e16).
// The 1e15 threshold sits well below NEVER_TIME to catch the tail robustly.
static inline int
is_sentinel_move(const struct move *m)
{
    return m->print_time < 0. || m->move_t > 1e15;
}

// Walk over the moves in the smoothing window and accumulate integrals.
// Optionally reports the first (earliest) and last (latest) move touched.
static void
pa_range_walk(const struct move *m, int axis, double move_time
              , const struct smoother *sm
              , double *pos_integral, double *pa_velocity_integral
              , const struct move **first_out, const struct move **last_out)
{
    move_time += sm->t_offs;
    while (unlikely(move_time < 0.)) {
        m = list_prev_entry(m, node);
        move_time += m->move_t;
    }
    while (unlikely(move_time > m->move_t)) {
        move_time -= m->move_t;
        m = list_next_entry(m, node);
    }
    // Calculate integral for the current move
    double start = move_time - sm->hst, end = move_time + sm->hst;
    double t0 = move_time;
    double start_base = m->start_pos.axis[axis - 'x'];
    *pos_integral = *pa_velocity_integral = 0.;
    if (unlikely(start >= 0. && end <= m->move_t)) {
        pa_move_integrate(m, axis, 0., t0, &sm->pm_diff,
                          pos_integral, pa_velocity_integral);
        *pos_integral += start_base;
        if (first_out) *first_out = m;
        if (last_out) *last_out = m;
        return;
    }
    smoother_antiderivatives left =
        likely(start < 0.) ? calc_antiderivatives(sm, t0) : sm->p_hst;
    smoother_antiderivatives right =
        likely(end > m->move_t) ? calc_antiderivatives(sm, t0 - m->move_t)
                                : sm->m_hst;
    smoother_antiderivatives diff = diff_antiderivatives(&right, &left);
    pa_move_integrate(m, axis, 0., t0, &diff,
                      pos_integral, pa_velocity_integral);
    // Integrate over previous moves
    const struct move *prev = m;
    while (likely(start < 0.)) {
        prev = list_prev_entry(prev, node);
        start += prev->move_t;
        t0 += prev->move_t;
        smoother_antiderivatives r = left;
        left = likely(start < 0.) ? calc_antiderivatives(sm, t0)
                                  : sm->p_hst;
        diff = diff_antiderivatives(&r, &left);
        double base = prev->start_pos.axis[axis - 'x'] - start_base;
        pa_move_integrate(prev, axis, base, t0, &diff,
                          pos_integral, pa_velocity_integral);
    }
    // Integrate over future moves
    t0 = move_time;
    while (likely(end > m->move_t)) {
        end -= m->move_t;
        t0 -= m->move_t;
        m = list_next_entry(m, node);
        smoother_antiderivatives l = right;
        right = likely(end > m->move_t) ? calc_antiderivatives(sm,
                                                               t0 - m->move_t)
                                        : sm->m_hst;
        diff = diff_antiderivatives(&right, &l);
        double base = m->start_pos.axis[axis - 'x'] - start_base;
        pa_move_integrate(m, axis, base, t0, &diff,
                          pos_integral, pa_velocity_integral);
    }
    *pos_integral += start_base;
    if (first_out) *first_out = prev;
    if (last_out) *last_out = m;
}

struct extruder_stepper {
    struct stepper_kinematics sk;
    struct smoother sm[3], pa_model_smoother;
    int smooth_extruding_moves, smooth_extrude_only_moves;
    struct list_head pa_list;
    double time_offset;
    struct epoch_cache cache[3];
    double cache_last_flush_time; // last seen last_flush_time; invalidates cache on batch change
    int cache_bypass;             // per-call flag: bypass cache entirely (e.g. for synthetic moves)
    int epoch_cache_enabled;
};

// An epoch must be revisited often enough to amortize its n+3 sample walks.
// Real epochs span ~0.6 ms (hundreds of solver evaluations); only degenerate
// epochs a few evaluation-spacings wide are skipped. Tunable heuristic --
// must stay far below the real epoch length, never near it.
#define SHORT_EPOCH_MIN 5e-5

#ifdef UNIT_TEST
long diag_epoch_hits, diag_epoch_builds;   // exercised-path counters for tests
#endif

void __visible
extruder_set_epoch_cache_enabled(struct stepper_kinematics *sk, int enabled)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    es->epoch_cache_enabled = enabled;
}

// Cache-wrapper around pa_range_walk. On a cache hit the position and velocity
// integrals are returned via barycentric interpolation of the stored epoch
// polynomial; on a miss the direct walk result is returned and, when conditions
// are met, a fresh cache entry is built from n+3 Chebyshev sample walks.
static void
pa_range_integrate(struct extruder_stepper *es, const struct move *m, int axis
                   , double move_time
                   , double *pos_integral, double *pa_velocity_integral)
{
    int ax = axis - 'x';
    const struct smoother *sm = &es->sm[ax];
    struct epoch_cache *c = &es->cache[ax];
    int use_cache = es->epoch_cache_enabled && !es->cache_bypass;

    // Absolute window-center time. Invariant under the walk's move_time
    // renormalization, so no list traversal is needed to compute it.
    double tau = m->print_time + move_time + sm->t_offs;

    if (use_cache && epoch_hit(c, tau)) {
        double d = tau - c->tau0;
        *pos_integral = c->fpos0 + bary_eval(c->npts, c->nodes, c->wbary, c->fpos, d);
        *pa_velocity_integral = c->fvel0 + bary_eval(c->npts, c->nodes, c->wbary, c->fvel, d);
#ifdef UNIT_TEST
        diag_epoch_hits++;
#endif
        return;
    }

    // Miss: the direct walk at tau is the returned value (ground truth).
    const struct move *f, *l;
    pa_range_walk(m, axis, move_time, sm, pos_integral, pa_velocity_integral,
                  &f, &l);
    if (!use_cache)
        return;

    // Build the epoch cache for future calls -- only strictly inside a
    // non-sentinel, long-enough epoch.
    if (is_sentinel_move(f) || is_sentinel_move(l)) {
        c->valid = 0;
        return;
    }
    double lo, hi;
    epoch_bounds(f, l, sm->hst, &lo, &hi);
    if (!(lo < tau && tau < hi) || hi - lo < SHORT_EPOCH_MIN) {
        c->valid = 0;
        return;
    }
    int npts = sm->n + 3;                          // pos degree n+2 -> n+3 nodes
    double tau0 = 0.5 * (lo + hi);
    double abs_nodes[EPOCH_CACHE_NPTS_MAX];
    bary_nodes(lo, hi, npts, abs_nodes);
    bary_weights(npts, c->wbary);
    for (int k = 0; k < npts; ++k) {
        double move_time_k = abs_nodes[k] - sm->t_offs - m->print_time;
        pa_range_walk(m, axis, move_time_k, sm, &c->fpos[k], &c->fvel[k],
                      NULL, NULL);
        c->nodes[k] = abs_nodes[k] - tau0;         // store shifted (local origin)
    }
    // Center the sampled values around the middle node so that bary_eval sums
    // small residuals rather than large absolute integrals.  This mirrors the
    // time-axis centering (tau0) onto the value axis and avoids catastrophic
    // cancellation in the alternating-sign barycentric sum when the extruder
    // has accumulated a large absolute position over a long print.
    c->fpos0 = c->fpos[npts / 2];
    c->fvel0 = c->fvel[npts / 2];
    for (int k = 0; k < npts; ++k) {
        c->fpos[k] -= c->fpos0;
        c->fvel[k] -= c->fvel0;
    }
    c->npts = npts;
    c->epoch_lo = lo;
    c->epoch_hi = hi;
    c->tau0 = tau0;
    c->valid = 1;
#ifdef UNIT_TEST
    diag_epoch_builds++;
#endif
}

double __visible
pressure_advance_linear_model_func(double position, double pa_velocity
                                   , struct pressure_advance_params *pa_params)
{
    return position + pa_velocity * pa_params->pressure_advance;
}

double __visible
pressure_advance_tanh_model_func(double position, double pa_velocity
                                 , struct pressure_advance_params *pa_params)
{
    position += pa_params->linear_advance * pa_velocity;
    if (pa_params->nonlinear_offset) {
        double rel_velocity = pa_velocity / pa_params->linearization_velocity;
        position += pa_params->nonlinear_offset * tanh(rel_velocity);
    }
    return position;
}

double __visible
pressure_advance_recipr_model_func(double position, double pa_velocity
                                   , struct pressure_advance_params *pa_params)
{
    position += pa_params->linear_advance * pa_velocity;
    if (pa_params->nonlinear_offset) {
        double rel_velocity = pa_velocity / pa_params->linearization_velocity;
        position +=
            pa_params->nonlinear_offset * rel_velocity / (1. + rel_velocity);
    }
    return position;
}

double __visible
pressure_advance_log_model_func(double position, double pa_velocity
                                , struct pressure_advance_params *pa_params)
{
    position += pa_params->linear_advance * pa_velocity;
    if (pa_params->nonlinear_offset) {
        double rel_velocity = pa_velocity / pa_params->linearization_velocity;
        position += pa_params->nonlinear_offset * log(1. + rel_velocity);
    }
    return position;
}

static double
pa_model_integrate(struct list_head *pa_list, double print_time
                   , const struct smoother *sm, double e_pos, double pa_vel)
{
    print_time += sm->t_offs;
    double start = print_time - sm->hst, end = print_time + sm->hst;
    // Calculate integral for the current move
    struct pressure_advance_params *pa = list_last_entry(
            pa_list, struct pressure_advance_params, node);
    struct pressure_advance_params *next_pa = NULL;
    while (unlikely(pa->active_print_time > print_time &&
                !list_is_first(&pa->node, pa_list))) {
        next_pa = pa;
        pa = list_prev_entry(pa, node);
    }
    if (likely(pa->active_print_time <= start &&
                (next_pa == NULL || end <= next_pa->active_print_time))) {
        return pa->pa_func(e_pos, pa_vel, pa);
    }
    smoother_antiderivatives left = likely(start < pa->active_print_time)
        ? calc_antiderivatives(sm, print_time - pa->active_print_time)
        : sm->p_hst;
    smoother_antiderivatives right = likely(
            next_pa != NULL && end > next_pa->active_print_time)
        ? calc_antiderivatives(sm, print_time - next_pa->active_print_time)
        : sm->m_hst;
    smoother_antiderivatives diff = diff_antiderivatives(&right, &left);
    double res = pa->pa_func(e_pos, pa_vel, pa) * diff.it0;

    // Integrate over previous PA configs
    while (likely(start < pa->active_print_time &&
                !list_is_first(&pa->node, pa_list))) {
        pa = list_prev_entry(pa, node);
        smoother_antiderivatives r = left;
        left = likely(start < pa->active_print_time)
            ? calc_antiderivatives(sm, print_time - pa->active_print_time)
            : sm->p_hst;
        diff = diff_antiderivatives(&r, &left);
        res += pa->pa_func(e_pos, pa_vel, pa) * diff.it0;
    }
    // Integrate over next PA configs
    while (likely(next_pa != NULL && end >= next_pa->active_print_time)) {
        pa = next_pa;
        next_pa = list_is_last(&next_pa->node, pa_list)
            ? NULL : list_next_entry(next_pa, node);
        smoother_antiderivatives l = right;
        right = likely(next_pa != NULL && end >= next_pa->active_print_time)
            ? calc_antiderivatives(sm, print_time - next_pa->active_print_time)
            : sm->m_hst;
        diff = diff_antiderivatives(&right, &l);
        res += pa->pa_func(e_pos, pa_vel, pa) * diff.it0;
    }
    return res;
}

static double
extruder_calc_position(struct stepper_kinematics *sk, struct move *m
                       , double move_time)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    // Synthetic moves (e.g. homing) are memset-zeroed and never linked into
    // a trapq, so node.next == NULL. They carry no valid epoch context and
    // must bypass the cache entirely -- never stored, never matched as a hit.
    es->cache_bypass = is_sentinel_move(m) || m->node.next == NULL;
    if (sk->last_flush_time != es->cache_last_flush_time) {
        for (int j = 0; j < 3; ++j)
            es->cache[j].valid = 0;
        es->cache_last_flush_time = sk->last_flush_time;
    }
    move_time += es->time_offset;
    while (unlikely(move_time < 0.)) {
        m = list_prev_entry(m, node);
        move_time += m->move_t;
    }
    while (unlikely(move_time >= m->move_t)) {
        move_time -= m->move_t;
        m = list_next_entry(m, node);
    }
    int i;
    struct coord e_pos, pa_vel;
    double move_dist = move_get_distance(m, move_time);
    for (i = 0; i < 3; ++i) {
        int axis = 'x' + i;
        const struct smoother* sm = &es->sm[i];
        if (!sm->hst) {
            pa_vel.axis[i] = 0.;
        } else {
            pa_range_integrate(es, m, axis, move_time,
                               &e_pos.axis[i], &pa_vel.axis[i]);
        }
        if (!sm->hst || !es->smooth_extruding_moves ||
                (!es->smooth_extrude_only_moves && axis == 'z')) {
            e_pos.axis[i] =
                m->start_pos.axis[i] + m->axes_r.axis[i] * move_dist;
        }
    }
    double position = e_pos.x + e_pos.y + e_pos.z;
    double pa_velocity = pa_vel.x + pa_vel.y + pa_vel.z;
    if (pa_velocity <= 0.)
        return position;
    return pa_model_integrate(
            &es->pa_list, m->print_time + move_time,
            &es->pa_model_smoother, position, pa_velocity);
}

static void
extruder_note_generation_time(struct extruder_stepper *es)
{
    double pre_active = 0., post_active = 0.;
    int i;
    for (i = 0; i < 3; ++i) {
        const struct smoother* sm = &es->sm[i];
        double pre_active_axis = sm->hst + sm->t_offs + es->time_offset;
        double post_active_axis = sm->hst - sm->t_offs - es->time_offset;
        if (pre_active_axis > pre_active)
            pre_active = pre_active_axis;
        if (post_active_axis > post_active)
            post_active = post_active_axis;
    }
    es->sk.gen_steps_pre_active = pre_active;
    es->sk.gen_steps_post_active = post_active;
    init_smoother(ARRAY_SIZE(pa_smoother_coeffs), pa_smoother_coeffs,
                  pre_active + post_active, &es->pa_model_smoother);
    es->pa_model_smoother.t_offs += 0.5 * (post_active - pre_active);
}

void __visible
extruder_set_pressure_advance(struct stepper_kinematics *sk, double print_time
                              , int n_params, double params[]
                              , pressure_advance_func func, double time_offset)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);

    // Cleanup old pressure advance parameters
    double cleanup_time = sk->last_flush_time - es->sk.gen_steps_pre_active;
    struct pressure_advance_params *first_pa = list_first_entry(
            &es->pa_list, struct pressure_advance_params, node);
    while (!list_is_last(&first_pa->node, &es->pa_list)) {
        struct pressure_advance_params *next_pa = list_next_entry(
                first_pa, node);
        if (next_pa->active_print_time >= cleanup_time) break;
        list_del(&first_pa->node);
        free(first_pa);
        first_pa = next_pa;
    }

    // No cache invalidation here: the epoch cache is keyed on absolute time
    // tau and its polynomial depends only on the smoother and trapq moves, not
    // on time_offset or PA parameters. A time_offset change shifts the
    // (m, move_time) -> tau mapping, which is recomputed fresh on every call.
    es->time_offset = time_offset;
    extruder_note_generation_time(es);

    struct pressure_advance_params *last_pa = list_last_entry(
            &es->pa_list, struct pressure_advance_params, node);
    if (n_params < 0 || n_params > ARRAY_SIZE(last_pa->params))
        return;
    size_t param_size = n_params * sizeof(params[0]);
    if (last_pa->pa_func == func &&
            memcmp(&last_pa->params, params, param_size) == 0) {
        // Retain old pa_params
        return;
    }
    // Add new pressure advance parameters
    struct pressure_advance_params *pa_params = malloc(sizeof(*pa_params));
    memset(pa_params, 0, sizeof(*pa_params));
    memcpy(&pa_params->params, params, param_size);
    pa_params->pa_func = func;
    pa_params->active_print_time = print_time;
    list_add_tail(&pa_params->node, &es->pa_list);
}

void __visible
extruder_set_smooth_moves_params(struct stepper_kinematics *sk
                                 , int smooth_extruding_moves
                                 , int smooth_extrude_only_moves)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    es->smooth_extruding_moves = smooth_extruding_moves;
    es->smooth_extrude_only_moves = smooth_extrude_only_moves;
}

int __visible
extruder_set_smoothing_params(struct stepper_kinematics *sk, char axis
                              , int n, double a[], double t_sm, double t_offs)
{
    if (axis != 'x' && axis != 'y' && axis != 'z')
        return -1;
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    struct smoother *sm = &es->sm[axis-'x'];
    int status = init_smoother(n, a, t_sm, sm);
    sm->t_offs = t_offs;
    extruder_note_generation_time(es);
    for (int j = 0; j < 3; ++j)
        es->cache[j].valid = 0;
    return status;
}

double __visible
extruder_get_step_gen_window(struct stepper_kinematics *sk)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    return es->sk.gen_steps_pre_active > es->sk.gen_steps_post_active
         ? es->sk.gen_steps_pre_active : es->sk.gen_steps_post_active;
}

struct stepper_kinematics * __visible
extruder_stepper_alloc(void)
{
    struct extruder_stepper *es = malloc(sizeof(*es));
    memset(es, 0, sizeof(*es));
    es->epoch_cache_enabled = 1;
    es->sk.calc_position_cb = extruder_calc_position;
    es->sk.active_flags = AF_X | AF_Y | AF_Z;
    // With half_accel==0 move_dist == start_v*t, e_pos is affine in t, and
    // pa_func evaluated at the constant cruise pa_velocity is constant in
    // position.  Smoother and pa_model_integrate boundary spans are guarded
    // by gen_steps_pre/post_active in the dispatcher.
    es->sk.is_linear = 1;
    list_init(&es->pa_list);
    struct pressure_advance_params *pa_params = malloc(sizeof(*pa_params));
    memset(pa_params, 0, sizeof(*pa_params));
    pa_params->pa_func = pressure_advance_linear_model_func;
    list_add_tail(&pa_params->node, &es->pa_list);
    return &es->sk;
}

void __visible
extruder_stepper_free(struct stepper_kinematics *sk)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    while (!list_empty(&es->pa_list)) {
        struct pressure_advance_params *pa = list_first_entry(
                &es->pa_list, struct pressure_advance_params, node);
        list_del(&pa->node);
        free(pa);
    }
    free(sk);
}
