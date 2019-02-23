#ifndef __MISC_H__
#define __MISC_H__


extern char printfBuf[32];

////////////////////////////////////////////////////////////////////////////////

#define ASSERT(condition) do { if (condition) {} } while (0);

#define PRINTF(...)  do { \
  snprintf(printfBuf, sizeof(printfBuf), __VA_ARGS__); \
  Serial.print(printfBuf); \
} while (0)

#define BITS_TO_BYTES(bits) \
    ((bits) / 8 + (((bits) % 8) ? 1 : 0))

#endif /* #ifndef __MISC_H__ */
