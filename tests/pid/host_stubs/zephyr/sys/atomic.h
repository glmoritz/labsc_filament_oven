#ifndef ATOMIC_STUB_H
#define ATOMIC_STUB_H
typedef long atomic_t;
typedef long atomic_val_t;
#define ATOMIC_INIT(v) (v)
static inline atomic_val_t atomic_get(const atomic_t *t){ return *t; }
#endif
