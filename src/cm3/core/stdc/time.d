/**
 * D header file for C99.
 *
 * $(C_HEADER_DESCRIPTION pubs.opengroup.org/onlinepubs/009695399/basedefs/_time.h.html, _time.h)
 *
 * Copyright: Copyright Sean Kelly 2005 - 2009.
 * License: Distributed under the
 *      $(LINK2 http://www.boost.org/LICENSE_1_0.txt, Boost Software License 1.0).
 *    (See accompanying file LICENSE)
 * Authors:   Sean Kelly,
 *            Alex Rønne Petersen
 * Source:    $(DRUNTIMESRC core/stdc/_time.d)
 * Standards: ISO/IEC 9899:1999 (E)
 */

module cm3.core.stdc.time;

version (cm3) {
    struct tm {
        int tm_sec;
        int tm_min;
        int tm_hour;
        int tm_mday;
        int tm_mon;
        int tm_year;
        int tm_wday;
        int tm_yday;
        int tm_isdst;
        c_long tm_gmtoff;
        char* tm_zone;
    }

    public alias time_t = long, clock_t = long;

    enum clock_t CLOCKS_PER_SEC = 1_000_000;
    clock_t clock();

    void tzset();

    extern __gshared const(char)*[2] tzname;
} else version (Posix)
    public import core.sys.posix.stdc.time;
else version (Windows)
    public import core.sys.windows.stdc.time;
else
    static assert(0, "unsupported system");

import cm3.core.stdc.config;

extern (C):
nothrow:
@nogc:

pragma(mangle, muslRedirTime64Mangle!("difftime", "__difftime64"))
pure double difftime(time_t time1, time_t time0);

pragma(mangle, muslRedirTime64Mangle!("mktime", "__mktime64"))
@system time_t mktime(scope tm* timeptr);

pragma(mangle, muslRedirTime64Mangle!("time", "__time64"))
time_t time(scope time_t* timer);

@system char* asctime(const scope tm* timeptr);

pragma(mangle, muslRedirTime64Mangle!("ctime", "__ctime64"))
@system char* ctime(const scope time_t* timer);

pragma(mangle, muslRedirTime64Mangle!("gmtime", "__gmtime64"))
@system tm* gmtime(const scope time_t* timer);

pragma(mangle, muslRedirTime64Mangle!("localtime", "__localtime64"))
@system tm* localtime(const scope time_t* timer);

@system size_t strftime(scope char* s, size_t maxsize, const scope char* format, const scope tm* timeptr);
