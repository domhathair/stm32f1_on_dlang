/**
 * D header file for C99.
 *
 * $(C_HEADER_DESCRIPTION pubs.opengroup.org/onlinepubs/009695399/basedefs/_stddef.h.html, _stddef.h)
 *
 * Copyright: Copyright Sean Kelly 2005 - 2009.
 * License: Distributed under the
 *      $(LINK2 http://www.boost.org/LICENSE_1_0.txt, Boost Software License 1.0).
 *    (See accompanying file LICENSE)
 * Authors:   Sean Kelly
 * Source:    $(DRUNTIMESRC core/stdc/_stddef.d)
 * Standards: ISO/IEC 9899:1999 (E)
 */

module cm3.core.stdc.stddef;

extern (C):
nothrow:
@nogc:

alias nullptr_t = typeof(null);

version (cm3) {
    alias wchar_t = dchar;
} else version (Windows) {
    alias wchar_t = wchar;
} else version (Posix) {
    alias wchar_t = dchar;
} else version (WASI) {
    alias wchar_t = dchar;
}
