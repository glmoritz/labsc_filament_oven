#ifndef UTIL_STUB_H
#define UTIL_STUB_H
#define BUILD_ASSERT(EXPR, ...) _Static_assert(EXPR, "" __VA_ARGS__)
#define ARG_UNUSED(x) ((void)(x))
#define CLAMP(v, lo, hi) ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))
#endif
