#ifndef SLAB_POOL_H
#define SLAB_POOL_H

// Header-only generic slab allocator used to recycle short-lived hot-path
// objects (queue_message in serialqueue, history_steps in stepcompress).
//
// Allocations come from a singly-linked free stack of pre-carved objects.
// When the stack is empty we malloc one fresh chunk of SLAB_POOL_CHUNK_SIZE
// objects and push the remainder onto the free stack.  Every chunk is kept
// in a side list so slab_pool_destroy can release the backing memory.
//
// The free stack is MPSC-safe: any thread may slab_pool_free(), exactly one
// thread should call slab_pool_alloc() concurrently with frees.  (Multiple
// concurrent allocators are technically safe because of the CAS loop, but
// the carve-fresh-chunk path writes to ->chunks without synchronization;
// allocations from a single owner are the supported model.)
//
// Each pooled object must reserve two trailing fields:
//   struct slab_pool *pool;         // back-pointer, set by carve
//   T *free_next;                   // intrusive freelist link, valid only
//                                   // while the object sits on the freelist
// The offsets are passed to slab_pool_init so the helpers stay strictly
// type-erased.

#include <stdatomic.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>


// Objects carved per chunk.  64 keeps each chunk at <= 16 KiB for the
// largest pooled object (queue_message ≈ 120 B → ~7.7 KiB / chunk).  Bump
// this if the alloc-side ever shows up as a hotspot in perf annotate of
// slab_pool_alloc's slow path.
#define SLAB_POOL_CHUNK_SIZE 64


struct slab_chunk {
    struct slab_chunk *next;
    void *base;
};

struct slab_pool {
    _Atomic(void *) free_head;
    struct slab_chunk *chunks;
    size_t object_size;
    size_t off_free_next;
    size_t off_pool_ptr;
};


static inline void
slab_pool_init(struct slab_pool *p, size_t object_size,
               size_t off_free_next, size_t off_pool_ptr)
{
    atomic_init(&p->free_head, NULL);
    p->chunks = NULL;
    p->object_size = object_size;
    p->off_free_next = off_free_next;
    p->off_pool_ptr = off_pool_ptr;
}


static inline void
slab_pool__push_free(struct slab_pool *p, void *obj)
{
    void *old = atomic_load_explicit(&p->free_head, memory_order_relaxed);
    for (;;) {
        *(void **)((char *)obj + p->off_free_next) = old;
        if (atomic_compare_exchange_weak_explicit(
                &p->free_head, &old, obj,
                memory_order_release, memory_order_relaxed))
            return;
    }
}


static inline void *
slab_pool_alloc(struct slab_pool *p)
{
    // Fast path — pop from the atomic stack.  Use load-acquire on success
    // so the object's contents publish to this thread (we never read
    // those contents, but it's the conservative pairing with push's
    // release.)
    void *head = atomic_load_explicit(&p->free_head, memory_order_acquire);
    while (head) {
        void *next = *(void **)((char *)head + p->off_free_next);
        if (atomic_compare_exchange_weak_explicit(
                &p->free_head, &head, next,
                memory_order_acquire, memory_order_acquire))
            return head;
    }

    // Slow path — out of free slots.  Carve a new chunk.  This path runs
    // single-threaded (the consumer side) so the chunks list write is safe.
    void *base = malloc(p->object_size * SLAB_POOL_CHUNK_SIZE);
    if (!base)
        return NULL;
    struct slab_chunk *chunk = malloc(sizeof(*chunk));
    if (!chunk) {
        free(base);
        return NULL;
    }
    chunk->base = base;
    chunk->next = p->chunks;
    p->chunks = chunk;

    // Stamp the pool back-pointer onto every carved object up front so
    // free()'rs reading it don't race a lazy first-use write.
    for (size_t i = 0; i < SLAB_POOL_CHUNK_SIZE; i++) {
        char *cur = (char *)base + i * p->object_size;
        *(struct slab_pool **)(cur + p->off_pool_ptr) = p;
    }

    // Push slots 1..N-1 onto the free stack; return slot 0 to the caller.
    for (size_t i = 1; i < SLAB_POOL_CHUNK_SIZE; i++) {
        char *cur = (char *)base + i * p->object_size;
        slab_pool__push_free(p, cur);
    }
    return base;
}


static inline void
slab_pool_free(struct slab_pool *p, void *obj)
{
    slab_pool__push_free(p, obj);
}


// Bulk release every chunk.  Caller is responsible for quiescence: no
// concurrent alloc or free may be in flight.  Used by stepcompress_free
// for the per-stepcompress history_steps pool; the global queue_message
// pool is never destroyed (process-lifetime).
static inline void
slab_pool_destroy(struct slab_pool *p)
{
    struct slab_chunk *c = p->chunks;
    while (c) {
        struct slab_chunk *next = c->next;
        free(c->base);
        free(c);
        c = next;
    }
    p->chunks = NULL;
    atomic_store_explicit(&p->free_head, NULL, memory_order_relaxed);
}


#endif // SLAB_POOL_H
