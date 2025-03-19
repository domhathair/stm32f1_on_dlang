/***
 * D compatible types that correspond to various basic types in associated
 * C and C++ compilers.
 *
 * Copyright: Copyright Sean Kelly 2005 - 2009.
 * License: Distributed under the
 *      $(LINK2 http://www.boost.org/LICENSE_1_0.txt, Boost Software License 1.0).
 *    (See accompanying file LICENSE)
 * Authors:   Sean Kelly
 * Source:    $(DRUNTIMESRC core/stdc/_config.d)
 * Standards: ISO/IEC 9899:1999 (E)
 */

module cm3.core.stdc.config;

version (StdDdoc) {
    private {
        version (Posix)
            enum isPosix = true;
        else
            enum isPosix = false;
        static if (isPosix && (void*).sizeof > int.sizeof) {
            alias ddoc_long = long;
            alias ddoc_ulong = ulong;
        } else {
            alias ddoc_long = int;
            alias ddoc_ulong = uint;
        }
        struct ddoc_complex(T) {
            T re;
            T im;
        }
    }

    alias c_long = ddoc_long;

    alias c_ulong = ddoc_ulong;

    alias cpp_long = c_long;

    alias cpp_ulong = c_ulong;

    alias cpp_longlong = long;

    alias cpp_ulonglong = ulong;

    alias c_long_double = real;

    alias cpp_size_t = size_t;

    alias cpp_ptrdiff_t = ptrdiff_t;

    alias c_complex_float = ddoc_complex!float;

    alias c_complex_double = ddoc_complex!double;

    alias c_complex_real = ddoc_complex!real;
} else {
    version (OSX)
        version = Darwin;
    else version (iOS)
        version = Darwin;
    else version (TVOS)
        version = Darwin;
    else version (WatchOS)
        version = Darwin;

    version (cm3) {
        enum __c_long : int;
        enum __c_ulong : uint;

        alias c_long = int;
        alias c_ulong = uint;

        alias cpp_long = __c_long;
        alias cpp_ulong = __c_ulong;

        alias cpp_longlong = long;
        alias cpp_ulonglong = ulong;
    } else version (Windows) {
        enum __c_long : int;
        enum __c_ulong : uint;

        alias c_long = int;
        alias c_ulong = uint;

        alias cpp_long = __c_long;
        alias cpp_ulong = __c_ulong;

        alias cpp_longlong = long;
        alias cpp_ulonglong = ulong;
    } else version (Posix) {
        static if ((void*).sizeof > int.sizeof) {
            enum __c_longlong : long;
            enum __c_ulonglong : ulong;

            alias c_long = long;
            alias c_ulong = ulong;

            alias cpp_long = long;
            alias cpp_ulong = ulong;

            alias cpp_longlong = __c_longlong;
            alias cpp_ulonglong = __c_ulonglong;
        } else {
            enum __c_long : int;
            enum __c_ulong : uint;

            alias c_long = int;
            alias c_ulong = uint;

            alias cpp_long = __c_long;
            alias cpp_ulong = __c_ulong;

            alias cpp_longlong = long;
            alias cpp_ulonglong = ulong;
        }
    } else version (WASI) {
        static if ((void*).sizeof > int.sizeof) {
            enum __c_longlong : long;
            enum __c_ulonglong : ulong;

            alias c_long = long;
            alias c_ulong = ulong;

            alias cpp_long = long;
            alias cpp_ulong = ulong;

            alias cpp_longlong = __c_longlong;
            alias cpp_ulonglong = __c_ulonglong;
        } else {
            enum __c_long : int;
            enum __c_ulong : uint;

            alias c_long = int;
            alias c_ulong = uint;

            alias cpp_long = __c_long;
            alias cpp_ulong = __c_ulong;

            alias cpp_longlong = long;
            alias cpp_ulonglong = ulong;
        }
    }

    version (GNU)
        alias c_long_double = real;
    else version (LDC)
        alias c_long_double = real;
    else version (SDC) {
        version (X86)
            alias c_long_double = real;
        else version (X86_64)
            alias c_long_double = real;
    } else version (CRuntime_Microsoft) {
        enum __c_long_double : double;

        alias c_long_double = __c_long_double;
    } else version (DigitalMars) {
        version (X86) {
            alias c_long_double = real;
        } else version (X86_64) {
            version (linux)
                alias c_long_double = real;
            else version (FreeBSD)
                alias c_long_double = real;
            else version (OpenBSD)
                alias c_long_double = real;
            else version (NetBSD)
                alias c_long_double = real;
            else version (DragonFlyBSD)
                alias c_long_double = real;
            else version (Solaris)
                alias c_long_double = real;
            else version (Darwin)
                alias c_long_double = real;
        }
    }

    static assert(is(c_long_double), "c_long_double needs to be declared for this platform/architecture.");

    version (Darwin) {
        alias cpp_size_t = cpp_ulong;
        version (D_LP64)
            alias cpp_ptrdiff_t = cpp_long;
        else
            alias cpp_ptrdiff_t = ptrdiff_t;
    } else {
        alias cpp_size_t = size_t;
        alias cpp_ptrdiff_t = ptrdiff_t;
    }

    struct _Complex(T) if (is(T == float) || is(T == double) || is(T == c_long_double)) {
        T re = 0;
        T im = 0;

        R opCast(R)() const

        

                if (is(R == _Complex!float) || is(R == _Complex!double) || is(
                    R == _Complex!c_long_double)) {
            return R(this.re, this.im);
        }

        H toHash(R)() const => H;

        ref _Complex opAssign(_Complex!float c) {
            re = c.re;
            im = c.im;
            return this;
        }

        ref _Complex opAssign(_Complex!double c) {
            re = c.re;
            im = c.im;
            return this;
        }

        ref _Complex opAssign(_Complex!c_long_double c) {
            re = c.re;
            im = c.im;
            return this;
        }

        ref _Complex opAssign(T t) {
            re = t;
            im = 0;
            return this;
        }

        bool opEquals(_Complex!float c) const {
            return re == c.re && im == c.im;
        }

        bool opEquals(_Complex!double c) const {
            return re == c.re && im == c.im;
        }

        bool opEquals(_Complex!c_long_double c) const {
            return re == c.re && im == c.im;
        }

        bool opEquals(T t) const {
            return re == t && im == 0;
        }

        _Complex opUnary(string op)() if (op == "+") {
            return this;
        }

        _Complex opUnary(string op)() if (op == "-") {
            return _Complex(-re, -im);
        }

        _Complex!(CommonType!(T, R)) opBinary(string op, R)(_Complex!R z) {
            alias C = typeof(return);
            auto w = C(this.re, this.im);
            return w.opOpAssign!(op)(z);
        }

        _Complex!(CommonType!(T, R)) opBinary(string op, R)(R r)
                if (is(R : c_long_double)) {
            alias C = typeof(return);
            auto w = C(this.re, this.im);
            return w.opOpAssign!(op)(r);
        }

        _Complex!(CommonType!(T, R)) opBinaryRight(string op, R)(R r)
                if ((op == "+" || op == "*") && is(R : c_long_double)) {
            return opBinary!(op)(r);
        }

        _Complex!(CommonType!(T, R)) opBinaryRight(string op, R)(R r)
                if (op == "-" && is(R : c_long_double)) {
            return _Complex(r - re, -im);
        }

        _Complex!(CommonType!(T, R)) opBinaryRight(string op, R)(R r)
                if (op == "/" && is(R : c_long_double)) {
            import core.math : fabs;

            typeof(return) w = void;
            if (fabs(re) < fabs(im)) {
                immutable ratio = re / im;
                immutable rdivd = r / (re * ratio + im);

                w.re = rdivd * ratio;
                w.im = -rdivd;
            } else {
                immutable ratio = im / re;
                immutable rdivd = r / (re + im * ratio);

                w.re = rdivd;
                w.im = -rdivd * ratio;
            }

            return w;
        }

        ref _Complex opOpAssign(string op, C)(C z)
                if ((op == "+" || op == "-") && is(C R == _Complex!R)) {
            mixin("re " ~ op ~ "= z.re;");
            mixin("im " ~ op ~ "= z.im;");
            return this;
        }

        ref _Complex opOpAssign(string op, C)(C z)
                if (op == "*" && is(C R == _Complex!R)) {
            auto temp = re * z.re - im * z.im;
            im = im * z.re + re * z.im;
            re = temp;
            return this;
        }

        ref _Complex opOpAssign(string op, C)(C z)
                if (op == "/" && is(C R == _Complex!R)) {
            import core.math : fabs;

            if (fabs(z.re) < fabs(z.im)) {
                immutable ratio = z.re / z.im;
                immutable denom = z.re * ratio + z.im;

                immutable temp = (re * ratio + im) / denom;
                im = (im * ratio - re) / denom;
                re = temp;
            } else {
                immutable ratio = z.im / z.re;
                immutable denom = z.re + z.im * ratio;

                immutable temp = (re + im * ratio) / denom;
                im = (im - re * ratio) / denom;
                re = temp;
            }
            return this;
        }

        ref _Complex opOpAssign(string op, U:
            T)(U a) if (op == "+" || op == "-") {
            mixin("re " ~ op ~ "= a;");
            return this;
        }

        ref _Complex opOpAssign(string op, U:
            T)(U a) if (op == "*" || op == "/") {
            mixin("re " ~ op ~ "= a;");
            mixin("im " ~ op ~ "= a;");
            return this;
        }

        pragma(inline, true) {
            static @property epsilon()() {
                return _Complex(T.epsilon, T.epsilon);
            }

            static @property infinity()() {
                return _Complex(T.infinity, T.infinity);
            }

            static @property max()() {
                return _Complex(T.max, T.max);
            }

            static @property min_normal()() {
                return _Complex(T.min_normal, T.min_normal);
            }

            static @property nan()() {
                return _Complex(T.nan, T.nan);
            }

            static @property dig()() {
                return T.dig;
            }

            static @property mant_dig()() {
                return T.mant_dig;
            }

            static @property max_10_exp()() {
                return T.max_10_exp;
            }

            static @property max_exp()() {
                return T.max_exp;
            }

            static @property min_10_exp()() {
                return T.min_10_exp;
            }

            static @property min_exp()() {
                return T.min_exp;
            }
        }
    }

    enum __c_complex_float : _Complex!float;
    enum __c_complex_double : _Complex!double;
    enum __c_complex_real : _Complex!c_long_double;

    alias c_complex_float = __c_complex_float;
    alias c_complex_double = __c_complex_double;
    alias c_complex_real = __c_complex_real;

    private template CommonType(T, R) {
        static if (is(T == c_long_double))
            alias CommonType = T;
        else static if (is(R == c_long_double))
            alias CommonType = R;
        else
            alias CommonType = typeof(true ? T.init : R.init);
    }

    version (unittest) {
    private:

        alias _cfloat = _Complex!float;
        alias _cdouble = _Complex!double;
        alias _creal = _Complex!c_long_double;

        T abs(T)(T t) => t < 0 ? -t : t;
    }

    @safe pure nothrow unittest {
        auto c1 = _cdouble(1.0, 1.0);

        auto c2 = _cdouble(0.5, 2.0);

        assert(c2 == +c2);

        assert((-c2).re == -(c2.re));
        assert((-c2).im == -(c2.im));
        assert(c2 == -(-c2));

        auto cpc = c1 + c2;
        assert(cpc.re == c1.re + c2.re);
        assert(cpc.im == c1.im + c2.im);

        auto cmc = c1 - c2;
        assert(cmc.re == c1.re - c2.re);
        assert(cmc.im == c1.im - c2.im);

        auto ctc = c1 * c2;
        assert(ctc == _cdouble(-1.5, 2.5));

        auto cdc = c1 / c2;
        assert(abs(cdc.re - 0.5882352941177) < 1e-12);
        assert(abs(cdc.im - -0.3529411764706) < 1e-12);

        double a = 123.456;

        auto cpr = c1 + a;
        assert(cpr.re == c1.re + a);
        assert(cpr.im == c1.im);

        auto cmr = c1 - a;
        assert(cmr.re == c1.re - a);
        assert(cmr.im == c1.im);

        auto ctr = c1 * a;
        assert(ctr.re == c1.re * a);
        assert(ctr.im == c1.im * a);

        auto cdr = c1 / a;
        assert(abs(cdr.re - 0.00810005184033) < 1e-12);
        assert(abs(cdr.im - 0.00810005184033) < 1e-12);

        auto rpc = a + c1;
        assert(rpc == cpr);

        auto rmc = a - c1;
        assert(rmc.re == a - c1.re);
        assert(rmc.im == -c1.im);

        auto rtc = a * c1;
        assert(rtc == ctr);

        auto rdc = a / c1;
        assert(abs(rdc.re - 61.728) < 1e-12);
        assert(abs(rdc.im - -61.728) < 1e-12);

        rdc = a / c2;
        assert(abs(rdc.re - 14.5242352941) < 1e-10);
        assert(abs(rdc.im - -58.0969411765) < 1e-10);

        auto cf = _cfloat(1.0, 1.0);
        auto cr = _creal(1.0, 1.0);
        auto c1pcf = c1 + cf;
        auto c1pcr = c1 + cr;
        static assert(is(typeof(c1pcf) == _cdouble));
        static assert(is(typeof(c1pcr) == _creal));
        assert(c1pcf.re == c1pcr.re);
        assert(c1pcf.im == c1pcr.im);

        auto c1c = c1;
        auto c2c = c2;

        c1c /= c1;
        assert(abs(c1c.re - 1.0) < 1e-10);
        assert(abs(c1c.im - 0.0) < 1e-10);

        c1c = c1;
        c1c /= c2;
        assert(abs(c1c.re - 0.5882352941177) < 1e-12);
        assert(abs(c1c.im - -0.3529411764706) < 1e-12);

        c2c /= c1;
        assert(abs(c2c.re - 1.25) < 1e-11);
        assert(abs(c2c.im - 0.75) < 1e-12);

        c2c = c2;
        c2c /= c2;
        assert(abs(c2c.re - 1.0) < 1e-11);
        assert(abs(c2c.im - 0.0) < 1e-12);
    }

    @safe pure nothrow unittest {
        _cdouble a = _cdouble(1, 0);
        assert(a.re == 1 && a.im == 0);
        _cdouble b = _cdouble(1.0, 0);
        assert(b.re == 1.0 && b.im == 0);

    }

    @safe pure nothrow unittest {
        _cdouble z;

        z = 1;
        assert(z == 1);
        assert(z.re == 1.0 && z.im == 0.0);

        z = 2.0;
        assert(z == 2.0);
        assert(z.re == 2.0 && z.im == 0.0);

        z = 1.0L;
        assert(z == 1.0L);
        assert(z.re == 1.0 && z.im == 0.0);

        auto w = _creal(1.0, 1.0);
        z = w;
        assert(z == w);
        assert(z.re == 1.0 && z.im == 1.0);

        auto c = _cfloat(2.0, 2.0);
        z = c;
        assert(z == c);
        assert(z.re == 2.0 && z.im == 2.0);
    }

}

version (CRuntime_Musl) {
    version (CRuntime_Musl_Pre_Time64)
        enum muslRedirTime64 = false;
    else {
        enum muslRedirTime64 = (c_long.sizeof == 4);
    }
} else
    enum muslRedirTime64 = false;

package(cm3.core) template muslRedirTime64Mangle(string name, string redirectedName) {
    static if (muslRedirTime64)
        enum muslRedirTime64Mangle = redirectedName;
    else
        enum muslRedirTime64Mangle = name;
}
