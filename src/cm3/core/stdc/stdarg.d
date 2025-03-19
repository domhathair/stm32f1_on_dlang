/**
 * D header file for C99.
 *
 * $(C_HEADER_DESCRIPTION pubs.opengroup.org/onlinepubs/009695399/basedefs/_stdarg.h.html, _stdarg.h)
 *
 * Copyright: Copyright Digital Mars 2000 - 2020.
 * License:   $(HTTP www.boost.org/LICENSE_1_0.txt, Boost License 1.0).
 * Authors:   Walter Bright, Hauke Duden
 * Standards: ISO/IEC 9899:1999 (E)
 * Source: $(DRUNTIMESRC core/stdc/_stdarg.d)
 */

module cm3.core.stdc.stdarg;

@nogc:
nothrow:

version (X86_64) {
    version (Windows) {
    } else version = SysV_x64;
}

version (GNU) {
    import gcc.builtins;
} else version (SysV_x64) {
    static import core.internal.vararg.sysv_x64;

    version (DigitalMars) {
        align(16) struct __va_argsave_t {
            size_t[6] regs;
            real[8] fpregs;
            __va_list va;
        }
    }
}

version (ARM) version = ARM_Any;
version (AArch64) version = ARM_Any;
version (MIPS32) version = MIPS_Any;
version (MIPS64) version = MIPS_Any;
version (PPC) version = PPC_Any;
version (PPC64) version = PPC_Any;
version (RISCV32) version = RISCV_Any;
version (RISCV64) version = RISCV_Any;

version (GNU) {
} else version (ARM_Any) {
    version (OSX) {
    } else version (iOS) {
    } else version (TVOS) {
    } else version (WatchOS) {
    } else:

        version (ARM) {
            version = AAPCS32;
        } else version (AArch64) {
            version = AAPCS64;
            static import core.internal.vararg.aarch64;
        }
}

T alignUp(size_t alignment = size_t.sizeof, T)(T base) pure {
    enum mask = alignment - 1;
    static assert(alignment > 0 && (alignment & mask) == 0, "alignment must be a power of 2");
    auto b = cast(size_t)base;
    b = (b + mask) & ~mask;
    return cast(T)b;
}

unittest {
    assert(1.alignUp == size_t.sizeof);
    assert(31.alignUp!16 == 32);
    assert(32.alignUp!16 == 32);
    assert(33.alignUp!16 == 48);
    assert((-9).alignUp!8 == -8);
}

version (BigEndian) {
    T* adjustForBigEndian(T)(T* p, size_t size) pure {
        return size >= size_t.sizeof ? p : cast(T*)((cast(void*)p) + (size_t.sizeof - size));
    }
}

version (GNU) {
    alias va_list = __gnuc_va_list;
    alias __gnuc_va_list = __builtin_va_list;
} else version (SysV_x64) {
    alias va_list = core.internal.vararg.sysv_x64.va_list;
    public import core.internal.vararg.sysv_x64 : __va_list, __va_list_tag;
} else version (AAPCS32) {
    alias va_list = __va_list;

} else version (AAPCS64) {
    alias va_list = core.internal.vararg.aarch64.va_list;
} else version (RISCV_Any) {
    alias va_list = void*;
} else {
    alias va_list = char*;
}

version (GNU) {
    void va_start(T)(out va_list ap, ref T parmn);
} else version (LDC) {
    pragma(LDC_va_start)
    void va_start(T)(out va_list ap, ref T parmn) @nogc;
} else version (DigitalMars) {
    version (X86) {
        void va_start(T)(out va_list ap, ref T parmn) {
            ap = cast(va_list)((cast(void*)&parmn) + T.sizeof.alignUp);
        }
    } else {
        void va_start(T)(out va_list ap, ref T parmn);
    }
}

version (GNU)
    T va_arg(T)(ref va_list ap);
else
    T va_arg(T)(ref va_list ap) {
    version (X86) {
        auto p = cast(T*)ap;
        ap += T.sizeof.alignUp;
        return *p;
    } else version (Win64) {
        version (LDC)
            enum isLDC = true;
        else
            enum isLDC = false;
        static if (isLDC && is(T == E[], E)) {
            auto p = cast(T*)ap;
            ap += T.sizeof;
            return *p;
        } else {
            static if (T.sizeof > size_t.sizeof || (T.sizeof & (T.sizeof - 1)) != 0)
                auto p = *cast(T**)ap;
            else
                auto p = cast(T*)ap;
            ap += size_t.sizeof;
            return *p;
        }
    } else version (SysV_x64) {
        return core.internal.vararg.sysv_x64.va_arg!T(ap);
    } else version (AAPCS32) {
        if (T.alignof >= 8)
            ap.__ap = ap.__ap.alignUp!8;
        auto p = cast(T*)ap.__ap;
        version (BigEndian)
            static if (T.sizeof < size_t.sizeof)
                p = adjustForBigEndian(p, T.sizeof);
        ap.__ap += T.sizeof.alignUp;
        return *p;
    } else version (AAPCS64) {
        return core.internal.vararg.aarch64.va_arg!T(ap);
    } else version (ARM_Any) {
        auto p = cast(T*)ap;
        version (BigEndian)
            static if (T.sizeof < size_t.sizeof)
                p = adjustForBigEndian(p, T.sizeof);
        ap += T.sizeof.alignUp;
        return *p;
    } else version (PPC_Any) {
        if (T.alignof >= 8)
            ap = ap.alignUp!8;
        auto p = cast(T*)ap;
        version (BigEndian)
            static if (T.sizeof < size_t.sizeof)
                p = adjustForBigEndian(p, T.sizeof);
        ap += T.sizeof.alignUp;
        return *p;
    } else version (LoongArch64) {
        auto p = cast(T*)ap;
        ap += T.sizeof.alignUp;
        return *p;
    } else version (MIPS_Any) {
        auto p = cast(T*)ap;
        version (BigEndian)
            static if (T.sizeof < size_t.sizeof)
                p = adjustForBigEndian(p, T.sizeof);
        ap += T.sizeof.alignUp;
        return *p;
    } else version (RISCV_Any) {
        static if (T.sizeof > (size_t.sizeof << 1))
            auto p = *cast(T**)ap;
        else {
            static if (T.alignof == (size_t.sizeof << 1))
                ap = ap.alignUp!(size_t.sizeof << 1);
            auto p = cast(T*)ap;
        }
        ap += T.sizeof.alignUp;
        return *p;
    } else
        static assert(0, "Unsupported platform");
}

version (GNU)
    void va_arg(T)(ref va_list ap, ref T parmn);
else
    void va_arg(T)(ref va_list ap, ref T parmn) {
    parmn = va_arg!T(ap);
}

version (GNU) {
    alias va_end = __builtin_va_end;
} else version (LDC) {
    pragma(LDC_va_end)
    void va_end(va_list ap);
} else version (DigitalMars) {
    void va_end(va_list ap) {
    }
}

version (GNU) {
    alias va_copy = __builtin_va_copy;
} else version (LDC) {
    pragma(LDC_va_copy)
    void va_copy(out va_list dest, va_list src);
} else version (DigitalMars) {
    version (SysV_x64) {
        void va_copy(out va_list dest, va_list src, void* storage = alloca(__va_list_tag.sizeof)) {
            dest = cast(va_list)storage;
            *dest = *src;
        }

        import cm3.core.stdc.stdlib : alloca;
    } else {
        void va_copy(out va_list dest, va_list src) {
            dest = src;
        }
    }
}
