#ifndef __MISC_H__
#define __MISC_H__

#include "user.h"

extern char printfBuf[32];

////////////////////////////////////////////////////////////////////////////////

#define ASSERT(condition) do { if (condition) {} } while (0);

#define __PRINTF(...)  do { \
  snprintf(printfBuf, sizeof(printfBuf), __VA_ARGS__); \
  Serial.print(printfBuf); \
} while (0)

#define BITS_TO_BYTES(bits) \
    ((bits) / 8 + (((bits) % 8) ? 1 : 0))


#if defined(CRSTR_SERIAL_DEBUG)
#define DEBUG_PRINTF(...) __PRINTF(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...) do{ } while (0)
#endif /* #if defined(CRSTR_SERIAL_DEBUG) */

#endif /* #ifndef __MISC_H__ */
