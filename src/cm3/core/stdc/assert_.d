/**
 * D header file for C99.
 *
 * $(C_HEADER_DESCRIPTION pubs.opengroup.org/onlinepubs/009695399/basedefs/_assert.h.html, _assert.h)
 *
 * License: Distributed under the
 *      $(LINK2 http://www.boost.org/LICENSE_1_0.txt, Boost Software License 1.0).
 *    (See accompanying file LICENSE)
 * Source:    $(DRUNTIMESRC core/stdc/_assert_.d)
 * Standards: ISO/IEC 9899:1999 (E)
 */

/****************************
 * These are the various functions called by the assert() macro.
 */

module cm3.core.stdc.assert_;

version (OSX)
    version = Darwin;
else version (iOS)
    version = Darwin;
else version (TVOS)
    version = Darwin;
else version (WatchOS)
    version = Darwin;

extern (C):
nothrow:
@nogc:

version (cm3) {
    noreturn __assert(const(char)* exp, const(char)* file, int line);
} else version (CRuntime_DigitalMars) {
    noreturn _assert(const(void)* exp, const(void)* file, uint line);
} else version (CRuntime_Microsoft) {
    noreturn _wassert(const(wchar)* exp, const(wchar)* file, uint line);
    noreturn _assert(const(char)* exp, const(char)* file, uint line);
} else version (Darwin) {
    noreturn __assert_rtn(const(char)* func, const(char)* file, uint line, const(char)* exp);
} else version (FreeBSD) {
    noreturn __assert(const(char)* func, const(char)* file, uint line, const(char)* exp);
} else version (NetBSD) {
    noreturn __assert(const(char)* file, int line, const(char)* exp);
    noreturn __assert13(const(char)* file, int line, const(char)* func, const(char)* exp);
} else version (OpenBSD) {
    noreturn __assert(const(char)* file, int line, const(char)* exp);
    noreturn __assert2(const(char)* file, int line, const(char)* func, const(char)* exp);
} else version (DragonFlyBSD) {
    noreturn __assert(const(char)* func, const(char)* file, uint line, const(char)* exp);
} else version (CRuntime_Glibc) {
    noreturn __assert(const(char)* exp, const(char)* file, uint line);
    noreturn __assert_fail(const(char)* exp, const(char)* file, uint line, const(char)* func);
    noreturn __assert_perror_fail(int errnum, const(char)* file, uint line, const(char)* func);
} else version (CRuntime_Bionic) {
    noreturn __assert(const(char)* __file, int __line, const(char)* __msg);
    noreturn __assert2(const(char)* __file, int __line, const(char)* __function, const(char)* __msg);
} else version (CRuntime_Musl) {
    noreturn __assert_fail(const(char)* exp, const(char)* file, uint line, const(char)* func);
} else version (CRuntime_Newlib) {
    noreturn __assert(const(char)* file, int line, const(char)* exp);
    noreturn __assert_func(const(char)* file, int line, const(char)* func, const(char)* exp);
} else version (CRuntime_UClibc) {
    noreturn __assert(const(char)* exp, const(char)* file, uint line, const(char)* func);
} else version (Solaris) {
    noreturn __assert_c99(const(char)* exp, const(char)* file, uint line, const(char)* func);
} else {
    static assert(0);
}
