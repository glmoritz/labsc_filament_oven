#ifndef ASSERT_STUB_H
#define ASSERT_STUB_H
/* Host stub: map Zephyr's __ASSERT(test, fmt, ...) onto standard assert(). */
#include <assert.h>
#define __ASSERT(test, ...)   assert(test)
#define __ASSERT_NO_MSG(test) assert(test)
#endif
