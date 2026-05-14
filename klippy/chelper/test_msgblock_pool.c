// Standalone TDD test for the slab pool helpers backing queue_message and
// history_steps.  Builds against msgblock.c (real implementation) plus
// slab_pool.h (header-only).  Linked separately from c_helper.so; not part of
// the runtime build.  Invoke via scripts/check_msgblock_pool.sh.

#include <assert.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "list.h"
#include "msgblock.h"
#include "slab_pool.h"


/****************************************************************
 * T1 — queue_message: alloc -> free -> alloc reuses the same slot.
 *
 * The pool is a LIFO atomic stack.  Two consecutive alloc/free/alloc cycles
 * with no concurrent activity must return the same address.  This proves the
 * recycling path is wired (and that we are not silently falling back to
 * glibc malloc).
 ****************************************************************/
static void
test_qm_pool_lifo_roundtrip(void)
{
    struct queue_message *qm1 = message_alloc_pooled();
    assert(qm1 != NULL);
    assert(qm1->pool != NULL);
    message_free(qm1);

    struct queue_message *qm2 = message_alloc_pooled();
    assert(qm2 == qm1);
    message_free(qm2);
}


/****************************************************************
 * T2 — message_alloc fallback (no pool) still uses glibc malloc.
 *
 * Existing call sites that use the unpooled API must keep working
 * unchanged.  The pool back-pointer is NULL for those messages and
 * message_free routes them to free().
 ****************************************************************/
static void
test_qm_unpooled_fallback(void)
{
    struct queue_message *qm = message_alloc();
    assert(qm != NULL);
    assert(qm->pool == NULL);
    message_free(qm);  // must call free(), not pool push
}


/****************************************************************
 * T3 — message_queue_free drains a mixed pool/malloc list.
 *
 * The MCU-disconnect path uses message_queue_free to drop any pending
 * messages.  Some of those will have come from the pool, some from
 * malloc (depending on the call site).  The drain must dispatch each
 * to the right destructor without confusing the two.
 ****************************************************************/
static void
test_qm_mixed_queue_free(void)
{
    struct list_head root;
    list_init(&root);

    struct queue_message *a = message_alloc();           // malloc
    struct queue_message *b = message_alloc_pooled();    // pool
    struct queue_message *c = message_alloc();           // malloc
    struct queue_message *d = message_alloc_pooled();    // pool

    list_add_tail(&a->node, &root);
    list_add_tail(&b->node, &root);
    list_add_tail(&c->node, &root);
    list_add_tail(&d->node, &root);

    message_queue_free(&root);

    // After drain: pool should have recycled b and d.  A subsequent
    // pooled alloc must return one of them (LIFO -> the last freed).
    struct queue_message *e = message_alloc_pooled();
    assert(e == d || e == b);  // either order is acceptable
    message_free(e);
}


/****************************************************************
 * T4 — MPSC concurrent free safety.
 *
 * Two producer threads call message_free in parallel on independently
 * allocated messages.  The pool's atomic push must not lose any entry.
 * After both joiners return, popping N times must yield exactly the N
 * pushed objects (set equality, not order).
 ****************************************************************/
struct mpsc_args {
    struct queue_message **msgs;
    int start;
    int count;
};

static void *
mpsc_freer(void *arg)
{
    struct mpsc_args *a = arg;
    for (int i = 0; i < a->count; i++)
        message_free(a->msgs[a->start + i]);
    return NULL;
}

static void
test_qm_mpsc_free(void)
{
    const int total = 256;
    struct queue_message *msgs[total];

    // Pre-allocate from the pool so all of these have qm->pool set.
    for (int i = 0; i < total; i++)
        msgs[i] = message_alloc_pooled();

    pthread_t t1, t2;
    struct mpsc_args a1 = { msgs, 0,         total / 2 };
    struct mpsc_args a2 = { msgs, total / 2, total / 2 };
    int r1 = pthread_create(&t1, NULL, mpsc_freer, &a1);
    int r2 = pthread_create(&t2, NULL, mpsc_freer, &a2);
    assert(r1 == 0 && r2 == 0);
    pthread_join(t1, NULL);
    pthread_join(t2, NULL);

    // Pop them back, mark each one we see.  Every original pointer must
    // appear exactly once.
    int seen[total];
    for (int i = 0; i < total; i++) seen[i] = 0;

    for (int i = 0; i < total; i++) {
        struct queue_message *qm = message_alloc_pooled();
        assert(qm != NULL);
        int found = 0;
        for (int j = 0; j < total; j++) {
            if (msgs[j] == qm) {
                assert(seen[j] == 0);  // no duplicates
                seen[j] = 1;
                found = 1;
                break;
            }
        }
        assert(found);
    }

    for (int i = 0; i < total; i++) assert(seen[i] == 1);
}


/****************************************************************
 * T5 — Pool grows past initial chunk capacity.
 *
 * SLAB_POOL_CHUNK_SIZE is the per-chunk slot count.  Asking for more
 * than one chunk must trigger a second allocation and still hand out
 * usable messages.  This catches accidental fixed-size pool bugs.
 ****************************************************************/
static void
test_qm_pool_grow(void)
{
    const int n = SLAB_POOL_CHUNK_SIZE * 3 + 7;  // ~3 chunks + slack
    struct queue_message **bag = malloc(n * sizeof(*bag));
    assert(bag != NULL);

    for (int i = 0; i < n; i++) {
        bag[i] = message_alloc_pooled();
        assert(bag[i] != NULL);
        // Distinct pointers, all carry the pool back-link.
        assert(bag[i]->pool != NULL);
        for (int j = 0; j < i; j++)
            assert(bag[i] != bag[j]);
    }
    for (int i = 0; i < n; i++)
        message_free(bag[i]);
    free(bag);
}


/****************************************************************
 * T6 — Generic slab_pool: alloc/free LIFO roundtrip.
 *
 * Exercises the header with a synthetic object — same shape we'll
 * give history_steps (and the global queue_message pool).  The slot
 * returned by the first alloc after free must equal the freed slot.
 ****************************************************************/
struct test_obj {
    int payload[4];
    struct slab_pool *pool;
    struct test_obj *free_next;
};

static void
test_hs_pool_lifo_roundtrip(void)
{
    struct slab_pool pool;
    slab_pool_init(&pool, sizeof(struct test_obj),
                   offsetof(struct test_obj, free_next),
                   offsetof(struct test_obj, pool));

    struct test_obj *a = slab_pool_alloc(&pool);
    assert(a != NULL);
    assert(a->pool == &pool);
    slab_pool_free(&pool, a);

    struct test_obj *b = slab_pool_alloc(&pool);
    assert(b == a);
    slab_pool_free(&pool, b);

    slab_pool_destroy(&pool);
}


/****************************************************************
 * T7 — slab_pool_destroy releases backing memory.
 *
 * Destroying the pool must free every chunk that was carved.  Run
 * the binary under valgrind / ASan to confirm no leaks.  The plain
 * assertion here is: destroy completes, and re-init on the same
 * descriptor lands in a clean state.
 ****************************************************************/
static void
test_pool_destroy_clean(void)
{
    struct slab_pool pool;
    slab_pool_init(&pool, sizeof(struct test_obj),
                   offsetof(struct test_obj, free_next),
                   offsetof(struct test_obj, pool));

    const int n = SLAB_POOL_CHUNK_SIZE * 2 + 1;
    struct test_obj **bag = malloc(n * sizeof(*bag));
    for (int i = 0; i < n; i++) bag[i] = slab_pool_alloc(&pool);
    for (int i = 0; i < n; i++) slab_pool_free(&pool, bag[i]);
    free(bag);

    slab_pool_destroy(&pool);

    slab_pool_init(&pool, sizeof(struct test_obj),
                   offsetof(struct test_obj, free_next),
                   offsetof(struct test_obj, pool));
    struct test_obj *a = slab_pool_alloc(&pool);
    assert(a != NULL);
    slab_pool_free(&pool, a);
    slab_pool_destroy(&pool);
}


/****************************************************************
 * Driver
 ****************************************************************/
int
main(void)
{
    test_qm_pool_lifo_roundtrip();    printf("T1 qm-lifo OK\n");
    test_qm_unpooled_fallback();      printf("T2 qm-fallback OK\n");
    test_qm_mixed_queue_free();       printf("T3 qm-mixed-drain OK\n");
    test_qm_mpsc_free();              printf("T4 qm-mpsc OK\n");
    test_qm_pool_grow();              printf("T5 qm-grow OK\n");
    test_hs_pool_lifo_roundtrip();    printf("T6 hs-lifo OK\n");
    test_pool_destroy_clean();        printf("T7 pool-destroy OK\n");
    printf("ALL PASS\n");
    return 0;
}
