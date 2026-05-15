// Standalone TDD suite for AWD twin-stepper deduplication.  Builds against
// the real stepcompress.c / itersolve.c / steppersync.c.  Not part of
// c_helper.so -- invoke via scripts/check_twin_dedup.sh.

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "itersolve.h"
#include "list.h"
#include "serialqueue.h"
#include "stepcompress.h"
#include "steppersync.h"
#include "trapq.h"

extern struct stepper_kinematics *corexy_stepper_alloc(char type);

// The standalone test never attaches a serialqueue (steppersync ss->sq stays
// NULL, so steppersync_flush early-returns).  These stubs only satisfy the
// linker without pulling serialqueue.c into the build.
struct command_queue *serialqueue_alloc_commandqueue(void) { return NULL; }
void serialqueue_free_commandqueue(struct command_queue *cq) { (void)cq; }
void serialqueue_send_batch(struct serialqueue *sq, struct command_queue *cq
                            , struct list_head *msgs)
{ (void)sq; (void)cq; (void)msgs; }

/****************************************************************
 * T1 -- a fresh stepcompress dispatches append/commit to the plain
 *       implementation; the accessor returns a usable pointer.
 ****************************************************************/
static void
test_default_dispatch(void)
{
    struct list_head mq;
    list_init(&mq);
    struct stepcompress *sc = stepcompress_alloc(&mq);
    stepcompress_fill(sc, 1, 25000000, 1, 2);
    stepcompress_set_time(sc, 0.0, 1000000.0);

    stepcompress_append_fn append = stepcompress_get_append_fn(sc);
    stepcompress_commit_fn commit = stepcompress_get_commit_fn(sc);
    assert(append != NULL && commit != NULL);

    // Emit a few forward steps via the accessor and confirm they land.
    for (int i = 0; i < 8; i++) {
        int ret = append(sc, 1, 0.0, 0.001 * (i + 1));
        assert(ret == 0);
    }
    int ret = commit(sc);
    assert(ret == 0);
    ret = stepcompress_flush(sc, UINT64_MAX);
    assert(ret == 0);

    struct pull_history_steps rec[32];
    int n = stepcompress_extract_old(sc, rec, 32, 0, UINT64_MAX);
    assert(n > 0);
    int total = 0;
    for (int i = 0; i < n; i++)
        total += rec[i].step_count;
    assert(total == 8);

    stepcompress_free(sc);
}

// Append the same sequence into 'expect' (solo) and into 'primary' (mirroring
// into 'twin'); assert the twin's compressed output equals the solo output.
static void
append_sequence(struct stepcompress *sc, stepcompress_append_fn append
                 , stepcompress_commit_fn commit)
{
    // Forward cruise run.
    double t = 0.0;
    for (int i = 0; i < 40; i++) {
        t += 0.0005;
        assert(append(sc, 1, 0.0, t) == 0);
    }
    // Direction change (exercises the SDS step+dir+step rollback filter).
    for (int i = 0; i < 20; i++) {
        t += 0.0003;
        assert(append(sc, 0, 0.0, t) == 0);
    }
    // Far-future step (exercises queue_append_far / CLOCK_DIFF_MAX).
    assert(append(sc, 0, 0.0, t + 5.0) == 0);
    assert(commit(sc) == 0);
}

static void
compare_history(struct stepcompress *a, struct stepcompress *b)
{
    struct pull_history_steps ra[128], rb[128];
    int na = stepcompress_extract_old(a, ra, 128, 0, UINT64_MAX);
    int nb = stepcompress_extract_old(b, rb, 128, 0, UINT64_MAX);
    assert(na == nb);
    assert(na > 0);
    for (int i = 0; i < na; i++) {
        assert(ra[i].first_clock == rb[i].first_clock);
        assert(ra[i].last_clock == rb[i].last_clock);
        assert(ra[i].start_position == rb[i].start_position);
        assert(ra[i].step_count == rb[i].step_count);
        assert(ra[i].interval == rb[i].interval);
        assert(ra[i].add == rb[i].add);
    }
}

static struct stepcompress *
alloc_sc(struct list_head *mq, uint32_t oid)
{
    struct stepcompress *sc = stepcompress_alloc(mq);
    stepcompress_fill(sc, oid, 25000000, 1, 2);
    stepcompress_set_time(sc, 0.0, 1000000.0);
    return sc;
}

static void
test_mirror_identical(void)
{
    struct list_head mq_e, mq_p, mq_t;
    list_init(&mq_e); list_init(&mq_p); list_init(&mq_t);
    struct stepcompress *expect = alloc_sc(&mq_e, 1);
    struct stepcompress *primary = alloc_sc(&mq_p, 2);
    struct stepcompress *twin = alloc_sc(&mq_t, 3);

    // Solo reference.
    append_sequence(expect, stepcompress_get_append_fn(expect)
                    , stepcompress_get_commit_fn(expect));
    assert(stepcompress_flush(expect, UINT64_MAX) == 0);

    // Mirrored run: append into 'primary' replicates into 'twin'.
    stepcompress_set_twin(primary, twin);
    append_sequence(primary, stepcompress_get_append_fn(primary)
                    , stepcompress_get_commit_fn(primary));
    assert(stepcompress_flush(primary, UINT64_MAX) == 0);
    assert(stepcompress_flush(twin, UINT64_MAX) == 0);

    compare_history(expect, primary);   // primary == solo reference
    compare_history(expect, twin);      // twin    == solo reference

    // Detaching restores the plain dispatch.
    stepcompress_set_twin(primary, NULL);
    assert(stepcompress_get_append_fn(primary) == stepcompress_get_append_fn(expect));

    stepcompress_free(expect);
    stepcompress_free(primary);
    stepcompress_free(twin);
}

/****************************************************************
 * T3 -- itersolve on a corexy '+' kinematic; a paired primary stepcompress
 *       must replicate the full solved step stream into the twin queue.
 ****************************************************************/
static void
test_itersolve_mirror(void)
{
    struct trapq *tq = trapq_alloc();
    trapq_append(tq, 0.0, 0.005, 0.080, 0.005,
                 0.0, 0.0, 0.0,            // start_pos
                 0.707, 0.707, 0.0,        // axes_r (diagonal -> both belts)
                 20.0, 120.0, 1500.0);     // start_v, cruise_v, accel
    trapq_check_sentinels(tq);

    struct stepper_kinematics *sk = corexy_stepper_alloc('+');
    itersolve_set_trapq(sk, tq, 0.0125);
    itersolve_set_position(sk, 0.0, 0.0, 0.0);

    struct list_head mq_p, mq_t;
    list_init(&mq_p); list_init(&mq_t);
    struct stepcompress *primary = alloc_sc(&mq_p, 1);
    struct stepcompress *twin = alloc_sc(&mq_t, 2);
    stepcompress_set_twin(primary, twin);

    int32_t ret = itersolve_generate_steps(sk, primary, 0.090);
    assert(ret == 0);
    assert(stepcompress_flush(primary, UINT64_MAX) == 0);
    assert(stepcompress_flush(twin, UINT64_MAX) == 0);

    compare_history(primary, twin);

    stepcompress_free(primary);
    stepcompress_free(twin);
    free(sk);
    trapq_free(tq);
}

// Build a steppersyncmgr with two syncemitters on one steppersync, give each
// a corexy '+' kinematic on a shared trapq, pair them, and run a gen cycle.
// The twin's compressed history must equal the primary's.
static void
test_steppersync_pair(void)
{
    struct trapq *tq = trapq_alloc();
    trapq_append(tq, 0.0, 0.005, 0.080, 0.005,
                 0.0, 0.0, 0.0, 0.707, 0.707, 0.0,
                 20.0, 120.0, 1500.0);
    trapq_check_sentinels(tq);

    struct steppersyncmgr *ssm = steppersyncmgr_alloc();
    struct steppersync *ss = steppersyncmgr_alloc_steppersync(ssm);

    char n1[16] = "primary", n2[16] = "twin";
    struct syncemitter *se_p = steppersync_alloc_syncemitter(ss, n1, 1);
    struct syncemitter *se_t = steppersync_alloc_syncemitter(ss, n2, 1);

    struct stepper_kinematics *sk_p = corexy_stepper_alloc('+');
    struct stepper_kinematics *sk_t = corexy_stepper_alloc('+');
    itersolve_set_trapq(sk_p, tq, 0.0125);
    itersolve_set_trapq(sk_t, tq, 0.0125);
    itersolve_set_position(sk_p, 0.0, 0.0, 0.0);
    itersolve_set_position(sk_t, 0.0, 0.0, 0.0);
    syncemitter_set_stepper_kinematics(se_p, sk_p);
    syncemitter_set_stepper_kinematics(se_t, sk_t);
    stepcompress_fill(syncemitter_get_stepcompress(se_p), 1, 25000000, 1, 2);
    stepcompress_fill(syncemitter_get_stepcompress(se_t), 2, 25000000, 1, 2);
    // steppersync_set_time propagates the clock rate to every syncemitter's
    // stepcompress -- it MUST run after the syncemitters are allocated.
    steppersync_set_time(ss, 0.0, 1000000.0);

    int pair_ret = syncemitter_set_twin_pair(se_p, se_t);
    assert(pair_ret == 0);

    int32_t ret = steppersyncmgr_gen_steps(ssm, 0.090, 0.090, 0.0);
    assert(ret == 0);

    compare_history(syncemitter_get_stepcompress(se_p),
                    syncemitter_get_stepcompress(se_t));

    steppersyncmgr_free(ssm);
    free(sk_p);
    free(sk_t);
    trapq_free(tq);
}

// Suspend must decouple the pair: afterwards both run SE_MODE_SOLO, so the
// twin solves its OWN sk/trapq.  We re-point the (now solo) twin at an idle
// trapq and confirm it emits no steps while the primary still does.
static void
test_suspend_decouples(void)
{
    struct trapq *tq_move = trapq_alloc();
    trapq_append(tq_move, 0.0, 0.005, 0.080, 0.005,
                 0.0, 0.0, 0.0, 0.707, 0.707, 0.0,
                 20.0, 120.0, 1500.0);
    trapq_check_sentinels(tq_move);
    struct trapq *tq_idle = trapq_alloc();
    trapq_check_sentinels(tq_idle);

    struct steppersyncmgr *ssm = steppersyncmgr_alloc();
    struct steppersync *ss = steppersyncmgr_alloc_steppersync(ssm);
    char n1[16] = "primary", n2[16] = "twin";
    struct syncemitter *se_p = steppersync_alloc_syncemitter(ss, n1, 1);
    struct syncemitter *se_t = steppersync_alloc_syncemitter(ss, n2, 1);
    struct stepper_kinematics *sk_p = corexy_stepper_alloc('+');
    struct stepper_kinematics *sk_t = corexy_stepper_alloc('+');
    // Pairing requires an identical trapq -- pair on tq_move.
    itersolve_set_trapq(sk_p, tq_move, 0.0125);
    itersolve_set_trapq(sk_t, tq_move, 0.0125);
    itersolve_set_position(sk_p, 0.0, 0.0, 0.0);
    itersolve_set_position(sk_t, 0.0, 0.0, 0.0);
    syncemitter_set_stepper_kinematics(se_p, sk_p);
    syncemitter_set_stepper_kinematics(se_t, sk_t);
    stepcompress_fill(syncemitter_get_stepcompress(se_p), 1, 25000000, 1, 2);
    stepcompress_fill(syncemitter_get_stepcompress(se_t), 2, 25000000, 1, 2);
    steppersync_set_time(ss, 0.0, 1000000.0);
    assert(syncemitter_set_twin_pair(se_p, se_t) == 0);

    // Suspend, then re-point the now-solo twin at an idle trapq.
    syncemitter_suspend_twin(se_p);
    itersolve_set_trapq(sk_t, tq_idle, 0.0125);
    assert(steppersyncmgr_gen_steps(ssm, 0.090, 0.090, 0.0) == 0);

    struct pull_history_steps rec[128];
    int np = stepcompress_extract_old(syncemitter_get_stepcompress(se_p),
                                      rec, 128, 0, UINT64_MAX);
    int total_p = 0;
    for (int i = 0; i < np; i++) total_p += rec[i].step_count;
    int nt = stepcompress_extract_old(syncemitter_get_stepcompress(se_t),
                                      rec, 128, 0, UINT64_MAX);
    int total_t = 0;
    for (int i = 0; i < nt; i++) total_t += rec[i].step_count;
    assert(total_p > 0);    // primary solved tq_move independently
    // itersolve over an all-sentinel (idle) trapq emits no steps -- a move
    // with zero axes_r never satisfies check_active().  This assert both
    // verifies that suspend decoupled the pair and exercises that invariant.
    assert(total_t == 0);

    // No-op on an unpaired emitter.
    char n3[16] = "solo";
    struct syncemitter *se_solo = steppersync_alloc_syncemitter(ss, n3, 1);
    syncemitter_suspend_twin(se_solo);
    syncemitter_resume_twin(se_solo);

    steppersyncmgr_free(ssm);
    free(sk_p); free(sk_t);
    trapq_free(tq_move); trapq_free(tq_idle);
}

// Resume must restore mirroring: after suspend+resume a gen cycle mirrors
// the primary's solved stream into the twin queue again.
static void
test_resume_remirrors(void)
{
    struct trapq *tq = trapq_alloc();
    trapq_append(tq, 0.0, 0.005, 0.080, 0.005,
                 0.0, 0.0, 0.0, 0.707, 0.707, 0.0,
                 20.0, 120.0, 1500.0);
    trapq_check_sentinels(tq);

    struct steppersyncmgr *ssm = steppersyncmgr_alloc();
    struct steppersync *ss = steppersyncmgr_alloc_steppersync(ssm);
    char n1[16] = "primary", n2[16] = "twin";
    struct syncemitter *se_p = steppersync_alloc_syncemitter(ss, n1, 1);
    struct syncemitter *se_t = steppersync_alloc_syncemitter(ss, n2, 1);
    struct stepper_kinematics *sk_p = corexy_stepper_alloc('+');
    struct stepper_kinematics *sk_t = corexy_stepper_alloc('+');
    itersolve_set_trapq(sk_p, tq, 0.0125);
    itersolve_set_trapq(sk_t, tq, 0.0125);
    itersolve_set_position(sk_p, 0.0, 0.0, 0.0);
    itersolve_set_position(sk_t, 0.0, 0.0, 0.0);
    syncemitter_set_stepper_kinematics(se_p, sk_p);
    syncemitter_set_stepper_kinematics(se_t, sk_t);
    stepcompress_fill(syncemitter_get_stepcompress(se_p), 1, 25000000, 1, 2);
    stepcompress_fill(syncemitter_get_stepcompress(se_t), 2, 25000000, 1, 2);
    steppersync_set_time(ss, 0.0, 1000000.0);
    assert(syncemitter_set_twin_pair(se_p, se_t) == 0);

    syncemitter_suspend_twin(se_p);
    syncemitter_resume_twin(se_p);
    assert(steppersyncmgr_gen_steps(ssm, 0.090, 0.090, 0.0) == 0);

    // Mirroring restored: twin output equals primary output.
    compare_history(syncemitter_get_stepcompress(se_p),
                    syncemitter_get_stepcompress(se_t));

    steppersyncmgr_free(ssm);
    free(sk_p); free(sk_t);
    trapq_free(tq);
}

// Deadlock-safety: if the primary takes the !se->sk early-out, it must still
// signal the twin so the twin thread never blocks forever.  We force the
// early-out by nulling the primary's sk after pairing.
static void
test_primary_earlyout_signals(void)
{
    struct steppersyncmgr *ssm = steppersyncmgr_alloc();
    struct steppersync *ss = steppersyncmgr_alloc_steppersync(ssm);
    char n1[16] = "primary", n2[16] = "twin";
    struct syncemitter *se_p = steppersync_alloc_syncemitter(ss, n1, 1);
    struct syncemitter *se_t = steppersync_alloc_syncemitter(ss, n2, 1);
    struct stepper_kinematics *sk_p = corexy_stepper_alloc('+');
    struct stepper_kinematics *sk_t = corexy_stepper_alloc('+');
    syncemitter_set_stepper_kinematics(se_p, sk_p);
    syncemitter_set_stepper_kinematics(se_t, sk_t);
    stepcompress_fill(syncemitter_get_stepcompress(se_p), 1, 25000000, 1, 2);
    stepcompress_fill(syncemitter_get_stepcompress(se_t), 2, 25000000, 1, 2);
    steppersync_set_time(ss, 0.0, 1000000.0);
    assert(syncemitter_set_twin_pair(se_p, se_t) == 0);

    // Force the primary into the !se->sk early-out branch.
    syncemitter_set_stepper_kinematics(se_p, NULL);

    // Must return (not hang) -- the twin thread is released by the primary's
    // early-out signal.
    int32_t ret = steppersyncmgr_gen_steps(ssm, 0.090, 0.090, 0.0);
    assert(ret == 0);

    steppersyncmgr_free(ssm);
    free(sk_p);
    free(sk_t);
}

/****************************************************************
 * Driver
 ****************************************************************/
int
main(void)
{
    test_default_dispatch();   printf("T1 default-dispatch OK\n");
    test_mirror_identical();   printf("T2 mirror-identical OK\n");
    test_itersolve_mirror();   printf("T3 itersolve-mirror OK\n");
    test_steppersync_pair();        printf("T4 steppersync-pair OK\n");
    test_primary_earlyout_signals();printf("T5 primary-earlyout OK\n");
    test_suspend_decouples();  printf("T6 suspend-decouples OK\n");
    test_resume_remirrors();   printf("T7 resume-remirrors OK\n");
    printf("ALL PASS\n");
    return 0;
}
