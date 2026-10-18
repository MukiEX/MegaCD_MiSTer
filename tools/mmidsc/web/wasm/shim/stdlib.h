/* Minimal freestanding libc for the wasm build (see libc.c). */
#ifndef SHIM_STDLIB_H
#define SHIM_STDLIB_H
#include <stddef.h>
void *malloc(size_t n);
void *calloc(size_t n, size_t m);
void *realloc(void *p, size_t n);
void free(void *p);
_Noreturn void exit(int code);
_Noreturn void abort(void);
static inline int abs(int v) { return v < 0 ? -v : v; }
#endif
