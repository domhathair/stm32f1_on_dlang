/**
 * D header file for C99.
 *
 * $(C_HEADER_DESCRIPTION pubs.opengroup.org/onlinepubs/009695399/basedefs/_stdlib.h.html, _stdlib.h)
 *
 * Copyright: Copyright Sean Kelly 2005 - 2014.
 * License: Distributed under the
 *      $(LINK2 http://www.boost.org/LICENSE_1_0.txt, Boost Software License 1.0).
 *    (See accompanying file LICENSE)
 * Authors:   Sean Kelly
 * Standards: ISO/IEC 9899:1999 (E)
 * Source: $(DRUNTIMESRC core/stdc/_stdlib.d)
 */

module cm3.core.stdc.stdlib;

import cm3.core.stdc.config;
public import cm3.core.stdc.stddef;

version (OSX)
    version = Darwin;
else version (iOS)
    version = Darwin;
else version (TVOS)
    version = Darwin;
else version (WatchOS)
    version = Darwin;

version (CRuntime_Glibc)
    version = AlignedAllocSupported;
else version (CRuntime_Newlib)
    version = AlignedAllocSupported;
else {
}

extern (C):

alias _compare_fp_t = int function(const void*, const void*);

nothrow:
@nogc:

inout(void)* bsearch(const void* key, inout(void)* base, size_t nmemb, size_t size, _compare_fp_t compar);

void qsort(void* base, size_t nmemb, size_t size, _compare_fp_t compar);

@system unittest {
    struct S {
        extern (C) static int cmp(const void*, const void*) {
            return 0;
        }
    }

    int[4] arr;
    qsort(arr.ptr, arr[0].sizeof, arr.length, &S.cmp);
    int key;
    bsearch(&key, arr.ptr, arr[0].sizeof, arr.length, &S.cmp);
}

struct div_t {
    int quot,
    rem;
}

struct ldiv_t {
    c_long quot,
    rem;
}

struct lldiv_t {
    long quot,
    rem;
}

enum EXIT_SUCCESS = 0;

enum EXIT_FAILURE = 1;

enum MB_CUR_MAX = 1;

version (cm3)
    enum RAND_MAX = 0x7fffffff;
else version (Windows)
    enum RAND_MAX = 0x7fff;
else version (CRuntime_Glibc)
    enum RAND_MAX = 0x7fffffff;
else version (Darwin)
    enum RAND_MAX = 0x7fffffff;
else version (FreeBSD)
    enum RAND_MAX = 0x7ffffffd;
else version (NetBSD)
    enum RAND_MAX = 0x7fffffff;
else version (OpenBSD)
    enum RAND_MAX = 0x7fffffff;
else version (DragonFlyBSD)
    enum RAND_MAX = 0x7fffffff;
else version (Solaris)
    enum RAND_MAX = 0x7fff;
else version (CRuntime_Bionic)
    enum RAND_MAX = 0x7fffffff;
else version (CRuntime_Musl)
    enum RAND_MAX = 0x7fffffff;
else version (CRuntime_Newlib)
    enum RAND_MAX = 0x7fffffff;
else version (CRuntime_UClibc)
    enum RAND_MAX = 0x7fffffff;
else version (WASI)
    enum RAND_MAX = 0x7fffffff;
else
    static assert(false, "Unsupported platform");

double atof(scope const char* nptr);

int atoi(scope const char* nptr);

c_long atol(scope const char* nptr);

long atoll(scope const char* nptr);

double strtod(scope inout(char)* nptr, scope inout(char)** endptr);

float strtof(scope inout(char)* nptr, scope inout(char)** endptr);

c_long strtol(scope inout(char)* nptr, scope inout(char)** endptr, int base);

long strtoll(scope inout(char)* nptr, scope inout(char)** endptr, int base);

c_ulong strtoul(scope inout(char)* nptr, scope inout(char)** endptr, int base);

ulong strtoull(scope inout(char)* nptr, scope inout(char)** endptr, int base);

version (CRuntime_Microsoft) {
    version (MinGW) {
        real __mingw_strtold(scope inout(char)* nptr, scope inout(char)** endptr);

        alias strtold = __mingw_strtold;
    } else {
        extern (D) real strtold(scope inout(char)* nptr, inout(char)** endptr) {
            return strtod(nptr, endptr);
        }
    }
} else {
    real strtold(scope inout(char)* nptr, scope inout(char)** endptr);
}

@trusted int rand();

@trusted void srand(uint seed);

void* malloc(size_t size);

void* calloc(size_t nmemb, size_t size);

void* realloc(void* ptr, size_t size);

void free(void* ptr);

version (AlignedAllocSupported) {
    void* aligned_alloc(size_t alignment, size_t size);
}

noreturn abort() @safe;

noreturn exit(int status);

int atexit(void function() func);

noreturn _Exit(int status);

char* getenv(scope const char* name);

int system(scope const char* string);

@trusted pure int abs(int j);

@trusted pure c_long labs(c_long j);

@trusted pure long llabs(long j);

@trusted div_t div(int numer, int denom);

@trusted ldiv_t ldiv(c_long numer, c_long denom);

@trusted lldiv_t lldiv(long numer, long denom);

int mblen(scope const char* s, size_t n);

int mbtowc(scope wchar_t* pwc, scope const char* s, size_t n);

int wctomb(scope char* s, wchar_t wc);

size_t mbstowcs(scope wchar_t* pwcs, scope const char* s, size_t n);

size_t wcstombs(scope char* s, scope const wchar_t* pwcs, size_t n);

version (DigitalMars) {
    void* alloca(size_t size) pure;
} else version (GNU) {
    void* alloca(size_t size) pure;
} else version (LDC) {
    pragma(LDC_alloca)
    void* alloca(size_t size) pure;
}

version (CRuntime_Microsoft) {
    ulong _strtoui64(scope inout(char)*, scope inout(char)**, int);

    ulong _wcstoui64(scope inout(wchar)*, scope inout(wchar)**, int);

    long _strtoi64(scope inout(char)*, scope inout(char)**, int);

    long _wcstoi64(scope inout(wchar)*, scope inout(wchar)**, int);
}
