/**
 * D header file for C99.
 *
 * $(C_HEADER_DESCRIPTION pubs.opengroup.org/onlinepubs/009695399/basedefs/_signal.h.html, _signal.h)
 *
 * Copyright: Copyright Sean Kelly 2005 - 2009.
 * License: Distributed under the
 *      $(LINK2 http://www.boost.org/LICENSE_1_0.txt, Boost Software License 1.0).
 *    (See accompanying file LICENSE)
 * Authors:   Sean Kelly
 * Source:    $(DRUNTIMESRC core/stdc/_signal.d)
 * Standards: ISO/IEC 9899:1999 (E)
 */

module cm3.core.stdc.signal;

extern (C):
nothrow:
@nogc:

alias sig_atomic_t = int;

private alias sigfn_t = void function(int);

version (cm3) {
    enum SIG_ERR = cast(sigfn_t)-1;

    enum SIG_DFL = cast(sigfn_t)0;

    enum SIG_IGN = cast(sigfn_t)1;

    enum SIGABRT = 6;

    enum SIGFPE = 8;

    enum SIGILL = 4;

    enum SIGINT = 2;

    enum SIGSEGV = 11;

    enum SIGTERM = 15;
} else version (Posix) {
    enum SIG_ERR = cast(sigfn_t)-1;

    enum SIG_DFL = cast(sigfn_t)0;

    enum SIG_IGN = cast(sigfn_t)1;

    enum SIGABRT = 6;

    enum SIGFPE = 8;

    enum SIGILL = 4;

    enum SIGINT = 2;

    enum SIGSEGV = 11;

    enum SIGTERM = 15;
} else version (Windows) {
    enum SIG_ERR = cast(sigfn_t)-1;

    enum SIG_DFL = cast(sigfn_t)0;

    enum SIG_IGN = cast(sigfn_t)1;

    enum SIGABRT = 22;

    enum SIGFPE = 8;

    enum SIGILL = 4;

    enum SIGINT = 2;

    enum SIGSEGV = 11;

    enum SIGTERM = 15;
}

sigfn_t signal(int sig, sigfn_t func);

int raise(int sig);
