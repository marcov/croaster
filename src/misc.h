#ifndef __MISC_H__
#define __MISC_H__


#define ASSERT(condition) do { if (condition) {} } while (0);


extern char printfBuf[32];

#define PRINTF(...)  do { \
  snprintf(printfBuf, sizeof(printfBuf), __VA_ARGS__); \
  Serial.print(printfBuf); \
} while (0)



#endif /* #ifndef __MISC_H__ */
