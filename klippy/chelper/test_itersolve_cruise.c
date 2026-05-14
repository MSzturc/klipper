// Standalone TDD test for the cruise fast-path in itersolve.  Builds against
// the real itersolve.c, kin_*.c, trapq.c, and stepcompress.c.  Not part of
// c_helper.so.  Invoke via scripts/check_itersolve_cruise.sh.
//
// Covers:
//   T1-T6  -- is_linear flag plumbing across each kinematic
//   T7     -- cruise fast-path entry point itersolve_gen_steps_range_cruise
//             is exported (linker test)
//   T8     -- cartesian cruise emits uniform-interval steps through the full
//             itersolve_generate_steps path

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "itersolve.h"
#include "list.h"
#include "stepcompress.h"
#include "trapq.h"

// Allocators
extern struct stepper_kinematics *cartesian_stepper_alloc(char axis);
extern struct stepper_kinematics *corexy_stepper_alloc(char type);
extern struct stepper_kinematics *corexz_stepper_alloc(char type);
extern struct stepper_kinematics *generic_cartesian_stepper_alloc(
        double a_x, double a_y, double a_z);
extern struct stepper_kinematics *delta_stepper_alloc(
        double arm2, double tower_x, double tower_y);
extern struct stepper_kinematics *deltesian_stepper_alloc(
        double arm2, double arm_x);
extern struct stepper_kinematics *winch_stepper_alloc(
        double ax, double ay, double az);
extern struct stepper_kinematics *polar_stepper_alloc(char type);
extern struct stepper_kinematics *rotary_delta_stepper_alloc(
        double shoulder_radius, double shoulder_height, double angle,
        double upper_arm, double lower_arm);
extern struct stepper_kinematics *extruder_stepper_alloc(void);
extern void extruder_stepper_free(struct stepper_kinematics *sk);
extern struct stepper_kinematics *input_shaper_alloc(void);
extern int input_shaper_set_sk(struct stepper_kinematics *sk,
                               struct stepper_kinematics *orig_sk);
extern struct stepper_kinematics *dual_carriage_alloc(void);
extern void dual_carriage_set_sk(struct stepper_kinematics *sk,
                                 struct stepper_kinematics *orig_sk);

// Fast-path entry point exposed by M-Cruise.  Linker test in T7.
extern int32_t itersolve_gen_steps_range_cruise(
        struct stepper_kinematics *sk, struct stepcompress *sc,
        struct move *m, double abs_start, double abs_end);


/****************************************************************
 * T1 -- cartesian_stepper_alloc emits is_linear=1.
 *
 * cart_*_calc_position returns start_pos + axes_r * move_get_distance(m, t).
 * With half_accel==0 move_get_distance reduces to start_v*t, so the output
 * is affine in t.  Direct solve via one fdiv per step is mathematically
 * exact; the dispatcher gates on sk->is_linear.
 ****************************************************************/
static void
test_cartesian_is_linear(void)
{
    struct stepper_kinematics *sk = cartesian_stepper_alloc('x');
    assert(sk->is_linear == 1);
    free(sk);

    sk = cartesian_stepper_alloc('y');
    assert(sk->is_linear == 1);
    free(sk);

    sk = cartesian_stepper_alloc('z');
    assert(sk->is_linear == 1);
    free(sk);
}


/****************************************************************
 * T2 -- corexy_stepper_alloc('+' / '-') emits is_linear=1.
 *
 * pos = c.x + c.y (or c.x - c.y).  A linear combination of two linear
 * functions stays linear.  This is the T100/T250 hot path.
 ****************************************************************/
static void
test_corexy_is_linear(void)
{
    struct stepper_kinematics *sk_p = corexy_stepper_alloc('+');
    struct stepper_kinematics *sk_m = corexy_stepper_alloc('-');
    assert(sk_p->is_linear == 1);
    assert(sk_m->is_linear == 1);
    free(sk_p);
    free(sk_m);
}


/****************************************************************
 * T3 -- the remaining linear cartesian-family allocators set is_linear=1.
 *
 * corexz: c.x +/- c.z, linear.
 * generic_cartesian: a_x*c.x + a_y*c.y + a_z*c.z, linear in t.
 * dual_carriage: scale+offset wrap around an already-linear orig_sk, linear.
 * extruder (no smoother attached): start_pos + axes_r * dist, linear.
 ****************************************************************/
static void
test_other_linear_kinematics(void)
{
    struct stepper_kinematics *sk;

    sk = corexz_stepper_alloc('+');
    assert(sk->is_linear == 1);
    free(sk);

    sk = corexz_stepper_alloc('-');
    assert(sk->is_linear == 1);
    free(sk);

    sk = generic_cartesian_stepper_alloc(1.0, 0.5, 0.0);
    assert(sk->is_linear == 1);
    free(sk);

    struct stepper_kinematics *orig = cartesian_stepper_alloc('x');
    sk = dual_carriage_alloc();
    dual_carriage_set_sk(sk, orig);
    assert(sk->is_linear == 1);
    free(sk);
    free(orig);

    sk = extruder_stepper_alloc();
    assert(sk->is_linear == 1);
    extruder_stepper_free(sk);
}


/****************************************************************
 * T4 -- non-linear kinematics keep is_linear=0.
 *
 * delta: sqrt(arm^2 - planar^2) + z.
 * deltesian: sqrt(arm2 - dx^2) + z.
 * polar (radius): sqrt(x^2 + y^2).
 * rotary_delta: trig + sqrt.
 * winch: sqrt of squared anchor delta.
 ****************************************************************/
static void
test_nonlinear_kinematics(void)
{
    struct stepper_kinematics *sk;

    sk = delta_stepper_alloc(10000., 0., 0.);
    assert(sk->is_linear == 0);
    free(sk);

    sk = deltesian_stepper_alloc(10000., 100.);
    assert(sk->is_linear == 0);
    free(sk);

    sk = polar_stepper_alloc('r');
    assert(sk->is_linear == 0);
    free(sk);

    sk = polar_stepper_alloc('a');
    assert(sk->is_linear == 0);
    free(sk);

    sk = rotary_delta_stepper_alloc(100., 0., 0., 100., 100.);
    assert(sk->is_linear == 0);
    free(sk);

    sk = winch_stepper_alloc(0., 0., 100.);
    assert(sk->is_linear == 0);
    free(sk);
}


/****************************************************************
 * T5 -- input_shaper propagates is_linear from the wrapped kinematic.
 *
 * shift_pulses() centres the shaper centroid at 0.  Applied to a linear
 * input over a region untouched by the move boundary, the convolution
 * sum(a_i * (alpha + beta*(t+p_i))) collapses to alpha + beta*t (because
 * sum(a_i) == 1 and sum(a_i*p_i) == 0).  So a shaper-wrapped linear
 * stepper is linear inside the safe interior.
 ****************************************************************/
static void
test_shaper_propagates_linear(void)
{
    struct stepper_kinematics *cart = cartesian_stepper_alloc('x');
    struct stepper_kinematics *sh = input_shaper_alloc();
    int r = input_shaper_set_sk(sh, cart);
    assert(r == 0);
    assert(sh->is_linear == 1);
    free(sh);
    free(cart);
}


/****************************************************************
 * T6 -- input_shaper wrapping a non-linear stepper stays non-linear.
 *
 * Convolution of a non-linear function is not necessarily linear, so the
 * fast-path stays off when the inner kinematic is non-linear.
 ****************************************************************/
static void
test_shaper_propagates_nonlinear(void)
{
    struct stepper_kinematics *delta = delta_stepper_alloc(10000., 0., 0.);
    struct stepper_kinematics *sh = input_shaper_alloc();
    int r = input_shaper_set_sk(sh, delta);
    assert(r == 0);
    assert(sh->is_linear == 0);
    free(sh);
    free(delta);
}


/****************************************************************
 * T7 -- the fast-path helper itersolve_gen_steps_range_cruise is exported.
 *
 * Linker test only: take the symbol address.  Build fails before
 * implementation, which is the RED-phase failure mode.
 ****************************************************************/
static void
test_cruise_helper_linkable(void)
{
    void *fn = (void *)itersolve_gen_steps_range_cruise;
    assert(fn != NULL);
}


/****************************************************************
 * T8 -- cartesian cruise emits the correct number of steps over the correct
 *       wallclock span via the full itersolve_generate_steps dispatch.
 *
 * Setup: cart-X stepper, step_dist=0.005 mm, mcu_freq=1e6 (1 us ticks).
 * One pure-cruise sub-move: print_time=0, move_t=0.1 s, v=100 mm/s.
 *
 * Analytic step times: t_n = (step_dist * (n + 0.5)) / v
 *                          = 25e-6 + n*50e-6 s
 *                          = 25 + n*50 ticks for n in [0, 1999].
 *
 * Strong invariants (independent of stepcompress's encoding choices):
 *   - total step count is 2000 (+/- 1 for the trailing rollback policy)
 *   - the global wallclock span first-to-last clock is (steps - 1) * 50 ticks
 *   - the bulk of the run lands in a single (interval=50, add=0) batch
 *
 * Also verifies the commanded_pos lands on a whole-step boundary.
 ****************************************************************/
static void
test_cartesian_cruise_uniform_interval(void)
{
    struct trapq *tq = trapq_alloc();
    trapq_append(tq, 0.0,
                 0.0, 0.1, 0.0,    // accel_t, cruise_t, decel_t
                 0.0, 0.0, 0.0,    // start_pos
                 1.0, 0.0, 0.0,    // axes_r
                 100.0, 100.0, 0.0); // start_v, cruise_v, accel
    trapq_check_sentinels(tq);

    struct stepper_kinematics *sk = cartesian_stepper_alloc('x');
    itersolve_set_trapq(sk, tq, 0.005);
    itersolve_set_position(sk, 0.0, 0.0, 0.0);

    struct list_head msg_queue;
    list_init(&msg_queue);
    struct stepcompress *sc = stepcompress_alloc(&msg_queue);
    stepcompress_fill(sc, 1, 25000000, 1, 2);
    stepcompress_set_time(sc, 0.0, 1000000.0);

    int32_t ret = itersolve_generate_steps(sk, sc, 0.1);
    assert(ret == 0);

    int flush_ret = stepcompress_flush(sc, UINT64_MAX);
    assert(flush_ret == 0);

    struct pull_history_steps records[64];
    int n = stepcompress_extract_old(sc, records, 64, 0, UINT64_MAX);
    assert(n > 0);

    int total = 0;
    int max_batch_count = 0;
    uint64_t earliest = UINT64_MAX, latest = 0;
    for (int i = 0; i < n; i++) {
        if (records[i].step_count == 0)
            continue;  // position marker (none expected here)
        assert(records[i].step_count > 0);  // forward direction only
        total += records[i].step_count;
        if (records[i].step_count > max_batch_count) {
            max_batch_count = records[i].step_count;
            assert(records[i].interval == 50);
            assert(records[i].add == 0);
        }
        if (records[i].first_clock < earliest)
            earliest = records[i].first_clock;
        if (records[i].last_clock > latest)
            latest = records[i].last_clock;
    }
    assert(total >= 1999 && total <= 2000);

    // (steps - 1) * interval, allowing one tick of rounding slack and the
    // +/-1 step-count play.
    uint64_t expected_span = (uint64_t)(total - 1) * 50;
    uint64_t actual_span = latest - earliest;
    int64_t span_diff = (int64_t)actual_span - (int64_t)expected_span;
    assert(span_diff >= -2 && span_diff <= 2);

    // Bulk of the cruise must compress to a single uniform-interval run.
    assert(max_batch_count >= total - 10);

    // commanded_pos lands on a whole-step boundary: total * step_dist.
    double expected_commanded = total * 0.005;
    assert(fabs(sk->commanded_pos - expected_commanded) < 1e-9);

    stepcompress_free(sc);
    free(sk);
    trapq_free(tq);
}


/****************************************************************
 * Driver
 ****************************************************************/
int
main(void)
{
    test_cartesian_is_linear();              printf("T1 cart-linear OK\n");
    test_corexy_is_linear();                 printf("T2 corexy-linear OK\n");
    test_other_linear_kinematics();          printf("T3 other-linear OK\n");
    test_nonlinear_kinematics();             printf("T4 non-linear OK\n");
    test_shaper_propagates_linear();         printf("T5 shaper-linear OK\n");
    test_shaper_propagates_nonlinear();      printf("T6 shaper-nonlin OK\n");
    test_cruise_helper_linkable();           printf("T7 cruise-link OK\n");
    test_cartesian_cruise_uniform_interval();printf("T8 cruise-uniform OK\n");
    printf("ALL PASS\n");
    return 0;
}
