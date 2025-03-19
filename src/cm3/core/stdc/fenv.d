/**
 * D header file for C99.
 *
 * $(C_HEADER_DESCRIPTION pubs.opengroup.org/onlinepubs/009695399/basedefs/_fenv.h.html, _fenv.h)
 *
 * Copyright: Copyright Sean Kelly 2005 - 2009.
 * License: Distributed under the
 *      $(LINK2 http://www.boost.org/LICENSE_1_0.txt, Boost Software License 1.0).
 *    (See accompanying file LICENSE)
 * Authors:   Sean Kelly
 * Source:    $(DRUNTIMESRC core/stdc/_fenv.d)
 * Standards: ISO/IEC 9899:1999 (E)
 */

module cm3.core.stdc.fenv;

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

    version (ARM) version = ARM_Any;
version (AArch64) version = ARM_Any;
version (HPPA) version = HPPA_Any;
version (MIPS32) version = MIPS_Any;
version (MIPS64) version = MIPS_Any;
version (PPC) version = PPC_Any;
version (PPC64) version = PPC_Any;
version (RISCV32) version = RISCV_Any;
version (RISCV64) version = RISCV_Any;
version (S390) version = IBMZ_Any;
version (SPARC) version = SPARC_Any;
version (SPARC64) version = SPARC_Any;
version (SystemZ) version = IBMZ_Any;
version (X86) version = X86_Any;
version (X86_64) version = X86_Any;

version (MinGW) version = GNUFP;
version (CRuntime_Glibc) version = GNUFP;

version (cm3) {
    version (AArch64) {
        alias fenv_t = ulong;
        alias fexcept_t = ulong;
    } else version (RISCV_Any) {
        alias fenv_t = size_t;
        alias fexcept_t = size_t;
    } else version (X86_Any) {
        struct fenv_t {
            uint _fpu_cw;
            uint _fpu_sw;
            uint _fpu_tagw;
            uint _fpu_ipoff;
            uint _fpu_ipsel;
            uint _fpu_opoff;
            uint _fpu_opsel;
            uint _sse_mxcsr;
        }

        alias fexcept_t = uint;
    } else version (SPARC64) {
        alias fenv_t = ulong;
        alias fexcept_t = ulong;
    } else version (SPARC) {
        alias fenv_t = uint;
        alias fexcept_t = uint;
    } else {
        alias fenv_t = int;
        alias fexcept_t = int;
    }
} else version (GNUFP) {
    version (X86) {
        struct fenv_t {
            ushort __control_word;
            ushort __unused1;
            ushort __status_word;
            ushort __unused2;
            ushort __tags;
            ushort __unused3;
            uint __eip;
            ushort __cs_selector;
            ushort __opcode;
            uint __data_offset;
            ushort __data_selector;
            ushort __unused5;
        }

        alias fexcept_t = ushort;
    } else version (X86_64) {
        struct fenv_t {
            ushort __control_word;
            ushort __unused1;
            ushort __status_word;
            ushort __unused2;
            ushort __tags;
            ushort __unused3;
            uint __eip;
            ushort __cs_selector;
            ushort __opcode;
            uint __data_offset;
            ushort __data_selector;
            ushort __unused5;
            uint __mxcsr;
        }

        alias fexcept_t = ushort;
    } else version (HPPA_Any) {
        struct fenv_t {
            uint __status_word;
            uint[7] __exception;
        }

        alias fexcept_t = uint;
    } else version (MIPS_Any) {
        struct fenv_t {
            uint __fp_control_register;
        }

        alias fexcept_t = ushort;
    } else version (AArch64) {
        struct fenv_t {
            uint __fpcr;
            uint __fpsr;
        }

        alias fexcept_t = uint;
    } else version (ARM) {
        struct fenv_t {
            uint __cw;
        }

        alias fexcept_t = uint;
    } else version (PPC_Any) {
        alias fenv_t = double;
        alias fexcept_t = uint;
    } else version (RISCV_Any) {
        alias fenv_t = uint;
        alias fexcept_t = uint;
    } else version (SPARC_Any) {
        import cm3.core.stdc.config : c_ulong;

        alias fenv_t = c_ulong;
        alias fexcept_t = c_ulong;
    } else version (IBMZ_Any) {
        struct fenv_t {
            fexcept_t __fpc;
            void* __unused;
        }

        alias fexcept_t = uint;
    } else version (LoongArch64) {
        struct fenv_t {
            uint __fp_control_register;
        }

        alias fexcept_t = uint;
    } else {
        static assert(0, "Unimplemented architecture");
    }
} else version (CRuntime_DigitalMars) {
    struct fenv_t {
        ushort status;
        ushort control;
        ushort round;
        ushort[2] reserved;
    }

    alias fexcept_t = int;
} else version (CRuntime_Microsoft) {
    struct fenv_t {
        uint ctl;
        uint stat;
    }

    alias fexcept_t = uint;
} else version (Darwin) {
    version (BigEndian) {
        alias fenv_t = uint;
        alias fexcept_t = uint;
    }
    version (LittleEndian) {
        struct fenv_t {
            ushort __control;
            ushort __status;
            uint __mxcsr;
            byte[8] __reserved;
        }

        alias fexcept_t = ushort;
    }
} else version (FreeBSD) {
    struct fenv_t {
        ushort __control;
        ushort __mxcsr_hi;
        ushort __status;
        ushort __mxcsr_lo;
        uint __tag;
        byte[16] __other;
    }

    alias fexcept_t = ushort;
} else version (NetBSD) {
    version (X86_64) {
        struct fenv_t {
            struct _x87 {
                uint control;
                uint status;
                uint tag;
                uint[4] others;
            }

            _x87 x87;

            uint mxcsr;
        }
    }
    version (X86) {
        struct fenv_t {
            struct _x87 {
                ushort control;
                ushort unused1;
                ushort status;
                ushort unused2;
                ushort tag;
                ushort unused3;
                uint[4] others;
            }

            _x87 x87;
            uint mxcsr;
        }

    }

    alias fexcept_t = uint;
} else version (OpenBSD) {
    struct fenv_t {
        struct __x87 {
            uint __control;
            uint __status;
            uint __tag;
            uint[4] __others;
        }
    }

    uint __mxcsr;

    alias fexcept_t = uint;
} else version (DragonFlyBSD) {
    struct fenv_t {
        struct _x87 {
            uint control;
            uint status;
            uint tag;
            uint[4] others;
        }

        _x87 x87;

        uint mxcsr;
    }

    alias fexcept_t = uint;
} else version (CRuntime_Bionic) {
    version (X86) {
        struct fenv_t {
            ushort __control;
            ushort __mxcsr_hi;
            ushort __status;
            ushort __mxcsr_lo;
            uint __tag;
            byte[16] __other;
        }

        alias fexcept_t = ushort;
    } else version (ARM) {
        alias fenv_t = uint;
        alias fexcept_t = uint;
    } else version (AArch64) {
        struct fenv_t {
            uint __control;
            uint __status;
        }

        alias fexcept_t = uint;
    } else version (X86_64) {
        struct fenv_t {
            struct _x87 {
                uint __control;
                uint __status;
                uint __tag;
                uint[4] __others;
            }

            _x87 __x87;

            uint __mxcsr;
        }

        alias fexcept_t = uint;
    } else {
        static assert(false, "Architecture not supported.");
    }
} else version (Solaris) {
    import cm3.core.stdc.config : c_ulong;

    enum FEX_NUM_EXC = 12;

    struct fex_handler_t {
        int __mode;
        void function() __handler;
    }

    struct fenv_t {
        fex_handler_t[FEX_NUM_EXC] __handler;
        c_ulong __fsr;
    }

    alias fexcept_t = int;
} else version (CRuntime_Musl) {
    version (AArch64) {
        struct fenv_t {
            uint __fpcr;
            uint __fpsr;
        }

        alias fexcept_t = uint;
    } else version (ARM) {
        import cm3.core.stdc.config : c_ulong;

        struct fenv_t {
            c_ulong __cw;
        }

        alias fexcept_t = c_ulong;
    } else version (IBMZ_Any) {
        alias fenv_t = uint;
        alias fexcept_t = uint;
    } else version (MIPS_Any) {
        struct fenv_t {
            uint __cw;
        }

        alias fexcept_t = ushort;
    } else version (PPC_Any) {
        alias fenv_t = double;
        alias fexcept_t = uint;
    } else version (X86_Any) {
        struct fenv_t {
            ushort __control_word;
            ushort __unused1;
            ushort __status_word;
            ushort __unused2;
            ushort __tags;
            ushort __unused3;
            uint __eip;
            ushort __cs_selector;
            ushort __opcode;
            uint __data_offset;
            ushort __data_selector;
            ushort __unused5;
            version (X86_64) uint __mxcsr;
        }

        alias fexcept_t = ushort;
    } else {
        static assert(false, "Architecture not supported.");
    }
} else version (CRuntime_Newlib) {
    version (AArch64) {
        alias fenv_t = ulong;
        alias fexcept_t = ulong;
    } else version (RISCV_Any) {
        alias fenv_t = size_t;
        alias fexcept_t = size_t;
    } else version (X86_Any) {
        struct fenv_t {
            uint _fpu_cw;
            uint _fpu_sw;
            uint _fpu_tagw;
            uint _fpu_ipoff;
            uint _fpu_ipsel;
            uint _fpu_opoff;
            uint _fpu_opsel;
            uint _sse_mxcsr;
        }

        alias fexcept_t = uint;
    } else version (SPARC64) {
        alias fenv_t = ulong;
        alias fexcept_t = ulong;
    } else version (SPARC) {
        alias fenv_t = uint;
        alias fexcept_t = uint;
    } else {
        alias fenv_t = int;
        alias fexcept_t = int;
    }
} else version (CRuntime_UClibc) {
    version (X86) {
        struct fenv_t {
            ushort __control_word;
            ushort __unused1;
            ushort __status_word;
            ushort __unused2;
            ushort __tags;
            ushort __unused3;
            uint __eip;
            ushort __cs_selector;
            ushort __opcode;
            uint __data_offset;
            ushort __data_selector;
            ushort __unused5;
        }

        alias fexcept_t = ushort;
    } else version (X86_64) {
        struct fenv_t {
            ushort __control_word;
            ushort __unused1;
            ushort __status_word;
            ushort __unused2;
            ushort __tags;
            ushort __unused3;
            uint __eip;
            ushort __cs_selector;
            ushort __opcode;
            uint __data_offset;
            ushort __data_selector;
            ushort __unused5;
            uint __mxcsr;
        }

        alias fexcept_t = ushort;
    } else version (MIPS_Any) {
        struct fenv_t {
            uint __fp_control_register;
        }

        alias fexcept_t = ushort;
    } else version (ARM) {
        struct fenv_t {
            uint __cw;
        }

        alias fexcept_t = uint;
    } else {
        static assert(false, "Architecture not supported.");
    }
} else {
    static assert(false, "Unsupported platform");
}

version (CRuntime_Microsoft) {
    enum {
        FE_INEXACT = 1,
        FE_UNDERFLOW = 2,
        FE_OVERFLOW = 4,
        FE_DIVBYZERO = 8,
        FE_INVALID = 0x10,
        FE_ALL_EXCEPT = 0x1F,
        FE_TONEAREST = 0,
        FE_UPWARD = 0x100,
        FE_DOWNWARD = 0x200,
        FE_TOWARDZERO = 0x300,
    }
} else version (Solaris) {
    version (SPARC_Any) {
        enum {
            FE_TONEAREST = 0,
            FE_TOWARDZERO = 1,
            FE_UPWARD = 2,
            FE_DOWNWARD = 3,
        }

        enum {
            FE_INEXACT = 0x01,
            FE_DIVBYZERO = 0x02,
            FE_UNDERFLOW = 0x04,
            FE_OVERFLOW = 0x08,
            FE_INVALID = 0x10,
            FE_ALL_EXCEPT = 0x1f,
        }

    } else version (X86_Any) {
        enum {
            FE_TONEAREST = 0,
            FE_DOWNWARD = 1,
            FE_UPWARD = 2,
            FE_TOWARDZERO = 3,
        }

        enum {
            FE_INVALID = 0x01,
            FE_DIVBYZERO = 0x04,
            FE_OVERFLOW = 0x08,
            FE_UNDERFLOW = 0x10,
            FE_INEXACT = 0x20,
            FE_ALL_EXCEPT = 0x3d,
        }
    } else {
        static assert(0, "Unimplemented architecture");
    }
} else {
    version (X86) {

        enum {
            FE_INVALID = 0x01,
            FE_DENORMAL = 0x02,
            FE_DIVBYZERO = 0x04,
            FE_OVERFLOW = 0x08,
            FE_UNDERFLOW = 0x10,
            FE_INEXACT = 0x20,
            FE_ALL_EXCEPT = 0x3F,
        }

        enum {
            FE_TONEAREST = 0,
            FE_DOWNWARD = 0x400,
            FE_UPWARD = 0x800,
            FE_TOWARDZERO = 0xC00,
        }
    } else version (X86_64) {

        enum {
            FE_INVALID = 0x01,
            FE_DENORMAL = 0x02,
            FE_DIVBYZERO = 0x04,
            FE_OVERFLOW = 0x08,
            FE_UNDERFLOW = 0x10,
            FE_INEXACT = 0x20,
            FE_ALL_EXCEPT = 0x3F,
        }

        enum {
            FE_TONEAREST = 0,
            FE_DOWNWARD = 0x400,
            FE_UPWARD = 0x800,
            FE_TOWARDZERO = 0xC00,
        }
    } else version (ARM_Any) {

        enum {
            FE_INVALID = 1,
            FE_DIVBYZERO = 2,
            FE_OVERFLOW = 4,
            FE_UNDERFLOW = 8,
            FE_INEXACT = 16,
            FE_ALL_EXCEPT = 31,
        }

        enum {
            FE_TONEAREST = 0,
            FE_UPWARD = 0x400000,
            FE_DOWNWARD = 0x800000,
            FE_TOWARDZERO = 0xC00000,
        }
    } else version (HPPA_Any) {

        enum {
            FE_INEXACT = 0x01,
            FE_UNDERFLOW = 0x02,
            FE_OVERFLOW = 0x04,
            FE_DIVBYZERO = 0x08,
            FE_INVALID = 0x10,
            FE_ALL_EXCEPT = 0x1F,
        }

        enum {
            FE_TONEAREST = 0x0,
            FE_TOWARDZERO = 0x200,
            FE_UPWARD = 0x400,
            FE_DOWNWARD = 0x600,
        }
    } else version (MIPS_Any) {

        enum {
            FE_INEXACT = 0x04,
            FE_UNDERFLOW = 0x08,
            FE_OVERFLOW = 0x10,
            FE_DIVBYZERO = 0x20,
            FE_INVALID = 0x40,
            FE_ALL_EXCEPT = 0x7C,
        }

        enum {
            FE_TONEAREST = 0x0,
            FE_TOWARDZERO = 0x1,
            FE_UPWARD = 0x2,
            FE_DOWNWARD = 0x3,
        }
    } else version (PPC_Any) {

        enum {
            FE_INEXACT = 0x2000000,
            FE_DIVBYZERO = 0x4000000,
            FE_UNDERFLOW = 0x8000000,
            FE_OVERFLOW = 0x10000000,
            FE_INVALID = 0x20000000,
            FE_INVALID_SNAN = 0x1000000,
            FE_INVALID_ISI = 0x800000,
            FE_INVALID_IDI = 0x400000,
            FE_INVALID_ZDZ = 0x200000,
            FE_INVALID_IMZ = 0x100000,
            FE_INVALID_COMPARE = 0x80000,
            FE_INVALID_SOFTWARE = 0x400,
            FE_INVALID_SQRT = 0x200,
            FE_INVALID_INTEGER_CONVERSION = 0x100,
            FE_ALL_INVALID = 0x1F80700,
            FE_ALL_EXCEPT = 0x3E000000,
        }

        enum {
            FE_TONEAREST = 0,
            FE_TOWARDZERO = 1,
            FE_UPWARD = 2,
            FE_DOWNWARD = 3,
        }
    } else version (RISCV_Any) {

        enum {
            FE_INEXACT = 0x01,
            FE_UNDERFLOW = 0x02,
            FE_OVERFLOW = 0x04,
            FE_DIVBYZERO = 0x08,
            FE_INVALID = 0x10,
            FE_ALL_EXCEPT = 0x1f,
        }

        enum {
            FE_TONEAREST = 0x0,
            FE_TOWARDZERO = 0x1,
            FE_DOWNWARD = 0x2,
            FE_UPWARD = 0x3,
        }
    } else version (SPARC_Any) {

        enum {
            FE_INVALID = 0x200,
            FE_OVERFLOW = 0x100,
            FE_UNDERFLOW = 0x80,
            FE_DIVBYZERO = 0x40,
            FE_INEXACT = 0x20,
            FE_ALL_EXCEPT = 0x3E0,
        }

        enum {
            FE_TONEAREST = 0x0,
            FE_TOWARDZERO = 0x40000000,
            FE_UPWARD = 0x80000000,
            FE_DOWNWARD = 0xc0000000,
        }
    } else version (IBMZ_Any) {

        enum {
            FE_INVALID = 0x80,
            FE_DIVBYZERO = 0x40,
            FE_OVERFLOW = 0x20,
            FE_UNDERFLOW = 0x10,
            FE_INEXACT = 0x08,
            FE_ALL_EXCEPT = 0xF8,
        }

        enum {
            FE_TONEAREST = 0x0,
            FE_DOWNWARD = 0x3,
            FE_UPWARD = 0x2,
            FE_TOWARDZERO = 0x1,
        }
    } else version (LoongArch64) {

        enum {
            FE_INEXACT = 0x010000,
            FE_UNDERFLOW = 0x020000,
            FE_OVERFLOW = 0x040000,
            FE_DIVBYZERO = 0x080000,
            FE_INVALID = 0x100000,
            FE_ALL_EXCEPT = 0x1f0000,
        }

        enum {
            FE_TONEAREST = 0x000,
            FE_TOWARDZERO = 0x100,
            FE_UPWARD = 0x200,
            FE_DOWNWARD = 0x300,
        }
    } else {
        static assert(0, "Unimplemented architecture");
    }

}

version (cm3) {
    enum FE_DFL_ENV = cast(fenv_t*)(-1);
} else version (GNUFP) {
    enum FE_DFL_ENV = cast(fenv_t*)(-1);
} else version (CRuntime_DigitalMars) {
    private extern __gshared fenv_t _FE_DFL_ENV;

    enum fenv_t* FE_DFL_ENV = &_FE_DFL_ENV;
} else version (CRuntime_Microsoft) {
    private extern __gshared fenv_t _Fenv0;

    enum FE_DFL_ENV = &_Fenv0;
} else version (Darwin) {
    private extern __gshared fenv_t _FE_DFL_ENV;

    enum FE_DFL_ENV = &_FE_DFL_ENV;
} else version (FreeBSD) {
    private extern const fenv_t __fe_dfl_env;

    enum FE_DFL_ENV = &__fe_dfl_env;
} else version (NetBSD) {
    private extern const fenv_t __fe_dfl_env;

    enum FE_DFL_ENV = &__fe_dfl_env;
} else version (OpenBSD) {
    private extern const fenv_t __fe_dfl_env;

    enum FE_DFL_ENV = &__fe_dfl_env;
} else version (DragonFlyBSD) {
    private extern const fenv_t __fe_dfl_env;

    enum FE_DFL_ENV = &__fe_dfl_env;
} else version (CRuntime_Bionic) {
    private extern const fenv_t __fe_dfl_env;

    enum FE_DFL_ENV = &__fe_dfl_env;
} else version (Solaris) {
    private extern const fenv_t __fenv_def_env;

    enum FE_DFL_ENV = &__fenv_def_env;
} else version (CRuntime_Musl) {
    enum FE_DFL_ENV = cast(fenv_t*)(-1);
} else version (CRuntime_UClibc) {
    enum FE_DFL_ENV = cast(fenv_t*)(-1);
} else {
    static assert(false, "Unsupported platform");
}

int feclearexcept(int excepts);

int fetestexcept(int excepts);

int feholdexcept(fenv_t* envp);

int fegetexceptflag(fexcept_t* flagp, int excepts);

int fesetexceptflag(const scope fexcept_t* flagp, int excepts);

int fegetround();

int fesetround(int round);

int fegetenv(fenv_t* envp);

int fesetenv(const scope fenv_t* envp);

version (CRuntime_Microsoft) {
    int feraiseexcept()(int excepts) {
        struct Entry {
            int exceptVal;
            double num;
            double denom;
        }

        __gshared Entry[5] table =
            [
                {FE_INVALID, 0.0, 0.0},
                {FE_DIVBYZERO, 1.0, 0.0},
                {FE_OVERFLOW, 1e+300, 1e-300},
                {FE_UNDERFLOW, 1e-300, 1e+300},
                {FE_INEXACT, 2.0, 3.0}
            ];

        if ((excepts &= FE_ALL_EXCEPT) == 0)
            return 0;

        double ans = void;
        foreach (i; 0 .. table.length) {
            if ((excepts & table[i].exceptVal) != 0)
                ans = table[i].num / table[i].denom;
        }

        return 0;
    }

    int feupdateenv()(const scope fenv_t* envp) {
        int excepts = fetestexcept(FE_ALL_EXCEPT);
        return (fesetenv(envp) != 0 || feraiseexcept(excepts) != 0 ? 1 : 0);
    }
} else {
    int feraiseexcept(int excepts);

    int feupdateenv(const scope fenv_t* envp);
}
