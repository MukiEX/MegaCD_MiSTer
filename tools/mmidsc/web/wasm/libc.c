/* Freestanding pieces of libc for the wasm build.
 *
 * Memory: a bump allocator. JS calls heap_mark() after setting up the
 * long-lived buffers and heap_reset() after each frame, so everything the
 * encoder allocates for a frame is dropped at once. free() is a no-op;
 * realloc() copies (each block keeps its size just before it). */
#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>
#include "shim/stdlib.h"
#include "shim/string.h"
#include "shim/stdio.h"

extern unsigned char __heap_base;
static uintptr_t heap_top, heap_mark_at;

static void heap_init(void)
{
	if (!heap_top) heap_top = ((uintptr_t)&__heap_base + 15) & ~(uintptr_t)15;
}

void *malloc(size_t n)
{
	heap_init();
	uintptr_t p = heap_top + 16;               /* 16-byte header holds the size */
	uintptr_t end = (p + n + 15) & ~(uintptr_t)15;
	size_t have = __builtin_wasm_memory_size(0) * 65536;
	if (end > have) {
		size_t pages = (end - have + 65535) / 65536;
		if (__builtin_wasm_memory_grow(0, pages) == (size_t)-1) return NULL;
	}
	*(size_t *)(p - 16) = n;
	heap_top = end;
	return (void *)p;
}

void *calloc(size_t n, size_t m)
{
	void *p = malloc(n * m);
	if (p) memset(p, 0, n * m);   /* (memory past the old top may be dirty after a reset) */
	return p;
}

void *realloc(void *old, size_t n)
{
	void *p = malloc(n);
	if (p && old) {
		size_t o = *(size_t *)((uintptr_t)old - 16);
		memcpy(p, old, o < n ? o : n);
	}
	return p;
}

void free(void *p) { (void)p; }

__attribute__((export_name("heap_mark"))) void heap_mark(void) { heap_init(); heap_mark_at = heap_top; }
__attribute__((export_name("heap_reset"))) void heap_reset(void) { if (heap_mark_at) heap_top = heap_mark_at; }

_Noreturn void exit(int code) { (void)code; __builtin_trap(); }
_Noreturn void abort(void) { __builtin_trap(); }

void *memcpy(void *d, const void *s, size_t n) { return __builtin_memcpy(d, s, n); }
void *memmove(void *d, const void *s, size_t n) { return __builtin_memmove(d, s, n); }
void *memset(void *d, int c, size_t n) { return __builtin_memset(d, c, n); }
int memcmp(const void *a, const void *b, size_t n)
{
	const unsigned char *x = a, *y = b;
	for (size_t i = 0; i < n; i++) if (x[i] != y[i]) return x[i] - y[i];
	return 0;
}
size_t strlen(const char *s) { size_t n = 0; while (s[n]) n++; return n; }

/* Only what the encoder's error messages use: %s %d %u %%. */
int snprintf(char *buf, size_t len, const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	size_t o = 0;
#define PUT(c) do { if (o + 1 < len) buf[o] = (c); o++; } while (0)
	for (const char *f = fmt; *f; f++) {
		if (*f != '%') { PUT(*f); continue; }
		f++;
		if (*f == 's') { const char *s = va_arg(ap, const char *); while (*s) PUT(*s++); }
		else if (*f == 'd' || *f == 'u') {
			char t[12]; int k = 0;
			long v = *f == 'd' ? (long)va_arg(ap, int) : (long)va_arg(ap, unsigned);
			if (v < 0) { PUT('-'); v = -v; }
			do { t[k++] = (char)('0' + v % 10); v /= 10; } while (v);
			while (k) PUT(t[--k]);
		} else if (*f == '%') PUT('%');
		else if (!*f) break;
	}
	if (len) buf[o < len ? o : len - 1] = 0;
	va_end(ap);
	return (int)o;
#undef PUT
}

int fprintf(FILE *f, const char *fmt, ...) { (void)f; (void)fmt; return 0; }
