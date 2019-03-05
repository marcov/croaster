#ifndef __DEBUG_H__
#define __DEBUG_H__

#include "user.h"

#if defined(CRSTR_SERIAL_DEBUG)
#define DEBUG_PRINTF(...) __PRINTF(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...) do{ } while (0)
#endif /* #if defined(CRSTR_SERIAL_DEBUG) */

#if defined(ADC_DEBUG)
#define ADC_PRINTF(...) __PRINTF(__VA_ARGS__)
#else
#define ADC_PRINTF(...) do{ } while (0)
#endif /* #if defined(CRSTR_SERIAL_DEBUG) */

#if defined(ANALOG_DEBUG)
#define ANALOG_PRINTF(...) __PRINTF(__VA_ARGS__)
#else
#define ANALOG_PRINTF(...) do{ } while (0)
#endif /* #if defined(CRSTR_SERIAL_DEBUG) */

////////////////////////////////////////////////////////////////////////////////

extern char printfBuf[32];

#if defined(ASSERT_ENABLED)
#define ASSERT(condition) do { \
    if (condition) { \
        __PRINTF("Assert failed %s:%d \n",__FILE__,__LINE__);\
    } \
} while (0)
#else
#define ASSERT(condition) do { if (condition) {} } while (0)
#endif /* #if defined(ASSERT_ENABLED) */

#define __PRINTF(...)  do { \
  snprintf(printfBuf, sizeof(printfBuf), __VA_ARGS__); \
  Serial.print(printfBuf); \
} while (0)

#endif /* #ifndef __DEBUG_H__ */
