#ifndef SHIM_STDIO_H
#define SHIM_STDIO_H
#include <stddef.h>
#include <stdarg.h>
typedef struct shim_file FILE;
#define stderr ((FILE *)0)
int snprintf(char *buf, size_t len, const char *fmt, ...);
int fprintf(FILE *f, const char *fmt, ...);
#endif
