/**
 * D header file for C99.
 *
 * $(C_HEADER_DESCRIPTION pubs.opengroup.org/onlinepubs/009695399/basedefs/_wchar.h.html, _wchar.h)
 *
 * Copyright: Copyright Sean Kelly 2005 - 2009.
 * License: Distributed under the
 *      $(LINK2 http://www.boost.org/LICENSE_1_0.txt, Boost Software License 1.0).
 *    (See accompanying file LICENSE)
 * Authors:   Sean Kelly
 * Source:    $(DRUNTIMESRC core/stdc/_wchar_.d)
 * Standards: ISO/IEC 9899:1999 (E)
 */

module cm3.core.stdc.wchar_;

import cm3.core.stdc.config;
import cm3.core.stdc.stdarg;
import cm3.core.stdc.stdio;
import core.atomic : atomicLoad;
public import cm3.core.stdc.stddef;
public import cm3.core.stdc.time;
public import cm3.core.stdc.stdint;

extern (C):
nothrow:
@nogc:

version (cm3) {
    struct mbstate_t {
        int __count;
        union ___value {
            wint_t __wch = 0;
            char[4] __wchb;
        }

        ___value __value;
    }
} else version (CRuntime_Glibc) {
    struct mbstate_t {
        int __count;
        union ___value {
            wint_t __wch = 0;
            char[4] __wchb;
        }

        ___value __value;
    }
} else version (FreeBSD) {
    union __mbstate_t {
        char[128] _mbstate8 = 0;
        long _mbstateL;
    }

    alias mbstate_t = __mbstate_t;
} else version (NetBSD) {
    union __mbstate_t {
        int64_t __mbstateL;
        char[128] __mbstate8;
    }

    alias mbstate_t = __mbstate_t;
} else version (OpenBSD) {
    union __mbstate_t {
        char[128] __mbstate8 = 0;
        int64_t __mbstateL;
    }

    alias mbstate_t = __mbstate_t;
} else version (DragonFlyBSD) {
    union __mbstate_t {
        char[128] _mbstate8 = 0;
        long _mbstateL;
    }

    alias mbstate_t = __mbstate_t;
} else version (Solaris) {
    struct __mbstate_t {
        version (D_LP64) {
            long[4] __filler;
        } else {
            int[6] __filler;
        }
    }

    alias mbstate_t = __mbstate_t;
} else version (CRuntime_Newlib) {
    struct mbstate_t {
        int __count;
        union ___value {
            wint_t __wch = 0;
            char[4] __wchb;
        }

        ___value __value;
    }
} else version (CRuntime_UClibc) {
    struct mbstate_t {
        wchar_t __mask = 0;
        wchar_t __wc = 0;
    }
} else {
    alias mbstate_t = int;
}

alias wint_t = wchar_t;

enum wchar_t WEOF = 0xFFFF;

version (CRuntime_Glibc) {
    int fwprintf(FILE* stream, const scope wchar_t* format, scope const...);

    int __isoc99_fwscanf(FILE* stream, const scope wchar_t* format, scope...);

    alias fwscanf = __isoc99_fwscanf;

    int swprintf(wchar_t* s, size_t n, const scope wchar_t* format, scope const...);

    int __isoc99_swscanf(const scope wchar_t* s, const scope wchar_t* format, scope...);

    alias swscanf = __isoc99_swscanf;

    int vfwprintf(FILE* stream, const scope wchar_t* format, va_list arg);

    int __isoc99_vfwscanf(FILE* stream, const scope wchar_t* format, va_list arg);

    alias vfwscanf = __isoc99_vfwscanf;

    int vswprintf(wchar_t* s, size_t n, const scope wchar_t* format, va_list arg);

    int __isoc99_vswscanf(const scope wchar_t* s, const scope wchar_t* format, va_list arg);

    alias vswscanf = __isoc99_vswscanf;

    int vwprintf(const scope wchar_t* format, va_list arg);

    int __isoc99_vwscanf(const scope wchar_t* format, va_list arg);

    alias vwscanf = __isoc99_vwscanf;

    int wprintf(const scope wchar_t* format, scope const...);

    int __isoc99_wscanf(const scope wchar_t* format, scope...);

    alias wscanf = __isoc99_wscanf;
} else {
    int fwprintf(FILE* stream, const scope wchar_t* format, scope const...);

    int fwscanf(FILE* stream, const scope wchar_t* format, scope...);

    int swprintf(wchar_t* s, size_t n, const scope wchar_t* format, scope const...);

    int swscanf(const scope wchar_t* s, const scope wchar_t* format, scope...);

    int vfwprintf(FILE* stream, const scope wchar_t* format, va_list arg);

    int vfwscanf(FILE* stream, const scope wchar_t* format, va_list arg);

    int vswprintf(wchar_t* s, size_t n, const scope wchar_t* format, va_list arg);

    int vswscanf(const scope wchar_t* s, const scope wchar_t* format, va_list arg);

    int vwprintf(const scope wchar_t* format, va_list arg);

    int vwscanf(const scope wchar_t* format, va_list arg);

    int wprintf(const scope wchar_t* format, scope const...);

    int wscanf(const scope wchar_t* format, scope...);
}

@trusted wint_t fgetwc(FILE* stream);

@trusted wint_t fputwc(wchar_t c, FILE* stream);

wchar_t* fgetws(wchar_t* s, int n, FILE* stream);

int fputws(const scope wchar_t* s, FILE* stream);

alias getwc = fgetwc;

alias putwc = fputwc;

@trusted wint_t ungetwc(wint_t c, FILE* stream);

version (CRuntime_Microsoft) {
    @trusted int fwide(FILE* stream, int mode) {
        return mode;
    }
} else {
    @trusted int fwide(FILE* stream, int mode);
}

double wcstod(const scope wchar_t* nptr, wchar_t** endptr);

float wcstof(const scope wchar_t* nptr, wchar_t** endptr);

real wcstold(const scope wchar_t* nptr, wchar_t** endptr);

c_long wcstol(const scope wchar_t* nptr, wchar_t** endptr, int base);

long wcstoll(const scope wchar_t* nptr, wchar_t** endptr, int base);

c_ulong wcstoul(const scope wchar_t* nptr, wchar_t** endptr, int base);

ulong wcstoull(const scope wchar_t* nptr, wchar_t** endptr, int base);

pure wchar_t* wcscpy(return scope wchar_t* s1, scope const wchar_t* s2);

pure wchar_t* wcsncpy(return scope wchar_t* s1, scope const wchar_t* s2, size_t n);

pure wchar_t* wcscat(return scope wchar_t* s1, scope const wchar_t* s2);

pure wchar_t* wcsncat(return scope wchar_t* s1, scope const wchar_t* s2, size_t n);

pure int wcscmp(scope const wchar_t* s1, scope const wchar_t* s2);

int wcscoll(scope const wchar_t* s1, scope const wchar_t* s2);

pure int wcsncmp(scope const wchar_t* s1, scope const wchar_t* s2, size_t n);

size_t wcsxfrm(scope wchar_t* s1, scope const wchar_t* s2, size_t n);

pure inout(wchar_t)* wcschr(return scope inout(wchar_t)* s, wchar_t c);

pure size_t wcscspn(scope const wchar_t* s1, scope const wchar_t* s2);

pure inout(wchar_t)* wcspbrk(return scope inout(wchar_t)* s1, scope const wchar_t* s2);

pure inout(wchar_t)* wcsrchr(return scope inout(wchar_t)* s, wchar_t c);

pure size_t wcsspn(scope const wchar_t* s1, scope const wchar_t* s2);

pure inout(wchar_t)* wcsstr(return scope inout(wchar_t)* s1, scope const wchar_t* s2);

wchar_t* wcstok(return scope wchar_t* s1, scope const wchar_t* s2, wchar_t** ptr);

pure size_t wcslen(scope const wchar_t* s);

pure inout(wchar_t)* wmemchr(return scope inout wchar_t* s, wchar_t c, size_t n);

pure int wmemcmp(scope const wchar_t* s1, scope const wchar_t* s2, size_t n);

pure wchar_t* wmemcpy(return scope wchar_t* s1, scope const wchar_t* s2, size_t n);

pure wchar_t* wmemmove(return scope wchar_t* s1, scope const wchar_t* s2, size_t n);

pure wchar_t* wmemset(return scope wchar_t* s, wchar_t c, size_t n);

size_t wcsftime(wchar_t* s, size_t maxsize, const scope wchar_t* format, const scope tm* timeptr);

version (Windows) {
    wchar_t* _wasctime(tm*);

    wchar_t* _wctime(time_t*);

    wchar_t* _wstrdate(wchar_t*);

    wchar_t* _wstrtime(wchar_t*);
}

@trusted wint_t btowc(int c);

@trusted int wctob(wint_t c);

int mbsinit(const scope mbstate_t* ps);

size_t mbrlen(const scope char* s, size_t n, mbstate_t* ps);

size_t mbrtowc(wchar_t* pwc, const scope char* s, size_t n, mbstate_t* ps);

size_t wcrtomb(char* s, wchar_t wc, mbstate_t* ps);

size_t mbsrtowcs(wchar_t* dst, const scope char** src, size_t len, mbstate_t* ps);

size_t wcsrtombs(char* dst, const scope wchar_t** src, size_t len, mbstate_t* ps);
