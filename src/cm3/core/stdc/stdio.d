/**
 * D header file for C99 <stdio.h>
 *
 * $(C_HEADER_DESCRIPTION pubs.opengroup.org/onlinepubs/009695399/basedefs/_stdio.h.html, _stdio.h)
 *
 * Copyright: Copyright Sean Kelly 2005 - 2009.
 * License: Distributed under the
 *      $(LINK2 http://www.boost.org/LICENSE_1_0.txt, Boost Software License 1.0).
 *    (See accompanying file LICENSE)
 * Authors:   Sean Kelly,
 *            Alex Rønne Petersen
 * Source:    https://github.com/dlang/dmd/blob/master/druntime/src/core/stdc/stdio.d
 * Standards: ISO/IEC 9899:1999 (E)
 */

module cm3.core.stdc.stdio;

version (OSX)
    version = Darwin;
else version (iOS)
    version = Darwin;
else version (TVOS)
    version = Darwin;
else version (WatchOS)
    version = Darwin;

private {
    import cm3.core.stdc.config;
    import cm3.core.stdc.stdarg;
    import cm3.core.stdc.stdint : intptr_t;

    version (FreeBSD) {
        import core.sys.posix.sys.types;
    } else version (OpenBSD) {
        import core.sys.posix.sys.types;
    } else version (NetBSD) {
        import core.sys.posix.sys.types;
    } else version (DragonFlyBSD) {
        import core.sys.posix.sys.types;
    }
}

extern (C):
nothrow:
@nogc:

version (cm3) {
    enum {
        BUFSIZ = 1024,

        EOF = -1,

        FOPEN_MAX = 20,

        FILENAME_MAX = 1024,

        TMP_MAX = 26,

        L_tmpnam = 1024
    }

    struct __sbuf {
        ubyte* _base;
        int _size;
    }
} else version (CRuntime_DigitalMars) {
    enum {
        BUFSIZ = 0x4000,

        EOF = -1,

        FOPEN_MAX = 20,

        FILENAME_MAX = 256,

        TMP_MAX = 32_767,

        SYS_OPEN = 20,
    }

    enum int _NFILE = 60;

    enum string _P_tmpdir = "\\";

    enum wstring _wP_tmpdir = "\\";

    enum int L_tmpnam = _P_tmpdir.length + 12;
} else version (CRuntime_Microsoft) {
    enum {
        BUFSIZ = 512,

        EOF = -1,

        FOPEN_MAX = 20,

        FILENAME_MAX = 260,

        TMP_MAX = 32_767,

        _SYS_OPEN = 20,
    }

    enum int _NFILE = 512;

    enum string _P_tmpdir = "\\";

    enum wstring _wP_tmpdir = "\\";

    enum int L_tmpnam = _P_tmpdir.length + 12;
} else version (CRuntime_Glibc) {
    enum {
        BUFSIZ = 8192,

        EOF = -1,

        FOPEN_MAX = 16,

        FILENAME_MAX = 4095,

        TMP_MAX = 238_328,

        L_tmpnam = 20
    }
} else version (CRuntime_Musl) {
    enum {
        BUFSIZ = 1024,

        EOF = -1,

        FOPEN_MAX = 1000,

        FILENAME_MAX = 4096,

        TMP_MAX = 10_000,

        L_tmpnam = 20
    }
} else version (Darwin) {
    enum {
        BUFSIZ = 1024,

        EOF = -1,

        FOPEN_MAX = 20,

        FILENAME_MAX = 1024,

        TMP_MAX = 308_915_776,

        L_tmpnam = 1024,
    }

    private {
        struct __sbuf {
            ubyte* _base;
            int _size;
        }

        struct __sFILEX {
        }
    }
} else version (FreeBSD) {
    enum {
        BUFSIZ = 1024,

        EOF = -1,

        FOPEN_MAX = 20,

        FILENAME_MAX = 1024,

        TMP_MAX = 308_915_776,

        L_tmpnam = 1024
    }

    struct __sbuf {
        ubyte* _base;
        int _size;
    }
} else version (NetBSD) {
    enum {
        BUFSIZ = 1024,

        EOF = -1,

        FOPEN_MAX = 20,

        FILENAME_MAX = 1024,

        TMP_MAX = 308_915_776,

        L_tmpnam = 1024
    }

    struct __sbuf {
        ubyte* _base;
        int _size;
    }
} else version (OpenBSD) {
    enum {
        BUFSIZ = 1024,

        EOF = -1,

        FOPEN_MAX = 20,

        FILENAME_MAX = 1024,

        TMP_MAX = 0x7fffffff,

        L_tmpnam = 1024
    }

    struct __sbuf {
        ubyte* _base;
        int _size;
    }
} else version (DragonFlyBSD) {
    enum {
        BUFSIZ = 1024,
        EOF = -1,
        FOPEN_MAX = 20,
        FILENAME_MAX = 1024,
        TMP_MAX = 308_915_776,
        L_tmpnam = 1024
    }

    struct __sbuf {
        byte* s_buf;
        int function(void*, const char*, int) sbuf_drain_func;
        void* s_drain_arg;
        int s_error;
        ssize_t s_size;
        ssize_t s_len;
        int s_flags;
        ssize_t s_sect_len;
    }

    enum {
        SBUF_FIXEDLEN = 0x00000000,
        SBUF_AUTOEXTEND = 0x00000001,
        SBUF_USRFLAGMSK = 0x0000ffff,
        SBUF_DYNAMIC = 0x00010000,
        SBUF_FINISHED = 0x00020000,
        SBUF_DYNSTRUCT = 0x00080000,
        SBUF_INSECTION = 0x00100000,
    }
} else version (Solaris) {
    enum {
        BUFSIZ = 1024,

        EOF = -1,

        FOPEN_MAX = _NFILE,

        FILENAME_MAX = 1024,

        TMP_MAX = 17_576,

        L_tmpnam = 25,
    }

    version (X86)
        enum int _NFILE = 60;
    else
        enum int _NFILE = 20;
} else version (CRuntime_Bionic) {
    enum {
        BUFSIZ = 1024,

        EOF = -1,

        FOPEN_MAX = 20,

        FILENAME_MAX = 1024,

        TMP_MAX = 308_915_776,

        L_tmpnam = 1024
    }

    struct __sbuf {
        ubyte* _base;
        int _size;
    }
} else version (CRuntime_Newlib) {
    enum {
        BUFSIZ = 1024,

        EOF = -1,

        FOPEN_MAX = 20,

        FILENAME_MAX = 1024,

        TMP_MAX = 26,

        L_tmpnam = 1024
    }

    struct __sbuf {
        ubyte* _base;
        int _size;
    }
} else version (CRuntime_UClibc) {
    enum {
        BUFSIZ = 4096,

        EOF = -1,

        FOPEN_MAX = 16,

        FILENAME_MAX = 4095,

        TMP_MAX = 238_328,

        L_tmpnam = 20
    }
} else version (WASI) {
    enum {
        BUFSIZ = 1024,

        EOF = -1,

        FOPEN_MAX = 1000,

        FILENAME_MAX = 4096,

        TMP_MAX = 10_000,

        L_tmpnam = 20
    }
} else {
    static assert(false, "Unsupported platform");
}

enum {
    SEEK_SET,

    SEEK_CUR,

    SEEK_END
}

version (cm3) {
    import cm3.core.stdc.wchar_ : mbstate_t;

    alias fpos_t = c_long;

    struct __sFILE {
        ubyte* _p;
        int _r;
        int _w;
        short _flags;
        short _file;
        __sbuf _bf;
        int _lbfsize;

        void* _data;
        void* _cookie;

        c_long function(void*, void*, scope char*, size_t) _read;
        c_long function(void*, void*, scope const char*, size_t) _write;
        fpos_t function(void*, void*, fpos_t, int) _seek;
        int function(void*, void*) _close;

        __sbuf _ub;
        ubyte* _up;
        int _ur;

        ubyte[3] _ubuf;
        ubyte[1] _nbuf;

        __sbuf _lb;

        int _blksize;
        int _flags2;

        long _offset;
        void* _unused;

        void* _lock;
        mbstate_t _mbstate;
    }

    alias _iobuf = __sFILE;

    alias FILE = shared(__sFILE);
} else version (CRuntime_DigitalMars) {
    alias fpos_t = c_long;

    struct _iobuf {
        char* _ptr;
        int _cnt;
        char* _base;
        int _flag;
        int _file;
        int _charbuf;
        int _bufsiz;
        char* __tmpnum;
    }

    alias FILE = shared(_iobuf);
} else version (CRuntime_Microsoft) {
    alias fpos_t = long;

    struct _iobuf {
        void* undefined;
    }

    alias FILE = shared(_iobuf);
} else version (CRuntime_Glibc) {
    import cm3.core.stdc.wchar_ : mbstate_t;

    struct fpos_t {
        long __pos;
        mbstate_t __state;
    }

    struct _IO_FILE {
        int _flags;
        char* _read_ptr;
        char* _read_end;
        char* _read_base;
        char* _write_base;
        char* _write_ptr;
        char* _write_end;
        char* _buf_base;
        char* _buf_end;
        char* _save_base;
        char* _backup_base;
        char* _save_end;
        void* _markers;
        _IO_FILE* _chain;
        int _fileno;
        int _flags2;
        ptrdiff_t _old_offset;
        ushort _cur_column;
        byte _vtable_offset;
        char[1] _shortbuf = 0;
        void* _lock;

        ptrdiff_t _offset;

        void* _codecvt;

        void* _wide_data;
        _IO_FILE* _freeres_list;
        void* _freeres_buf;
        size_t __pad5;
        int _mode;

        char[15 * int.sizeof - 4 * (void*).sizeof - size_t.sizeof] _unused2;
    }

    alias _iobuf = _IO_FILE;

    alias FILE = shared(_IO_FILE);
} else version (WASI) {
    union fpos_t {
        char[16] __opaque = 0;
        double __align;
    }

    struct _IO_FILE;

    alias _iobuf = _IO_FILE;

    alias FILE = shared(_IO_FILE);
} else version (CRuntime_Musl) {
    union fpos_t {
        char[16] __opaque = 0;
        double __align;
    }

    struct _IO_FILE;

    alias _iobuf = _IO_FILE;

    alias FILE = shared(_IO_FILE);
} else version (Darwin) {
    alias fpos_t = long;

    struct __sFILE {
        ubyte* _p;
        int _r;
        int _w;
        short _flags;
        short _file;
        __sbuf _bf;
        int _lbfsize;

        void* _cookie;
        int function(void*) _close;
        int function(void*, char*, int) _read;
        fpos_t function(void*, fpos_t, int) _seek;
        int function(void*, char*, int) _write;

        __sbuf _ub;
        __sFILEX* _extra;
        int _ur;

        ubyte[3] _ubuf;
        ubyte[1] _nbuf;

        __sbuf _lb;

        int _blksize;
        fpos_t _offset;
    }

    alias _iobuf = __sFILE;

    alias FILE = shared(__sFILE);
} else version (FreeBSD) {
    import cm3.core.stdc.wchar_ : mbstate_t;

    alias fpos_t = off_t;

    struct __sFILE {
        ubyte* _p;
        int _r;
        int _w;
        short _flags;
        short _file;
        __sbuf _bf;
        int _lbfsize;

        void* _cookie;
        int function(void*) _close;
        int function(void*, char*, int) _read;
        fpos_t function(void*, fpos_t, int) _seek;
        int function(void*, const scope char*, int) _write;

        __sbuf _ub;
        ubyte* _up;
        int _ur;

        ubyte[3] _ubuf;
        ubyte[1] _nbuf;

        __sbuf _lb;

        int _blksize;
        fpos_t _offset;

        pthread_mutex_t _fl_mutex;
        pthread_t _fl_owner;
        int _fl_count;
        int _orientation;
        mbstate_t _mbstate;
    }

    alias _iobuf = __sFILE;

    alias FILE = shared(__sFILE);
} else version (NetBSD) {
    alias fpos_t = off_t;

    struct __sFILE {
        ubyte* _p;
        int _r;
        int _w;
        ushort _flags;
        short _file;
        __sbuf _bf;
        int _lbfsize;

        void* _cookie;
        int function(void*) _close;
        ssize_t function(void*, char*, size_t) _read;
        fpos_t function(void*, fpos_t, int) _seek;
        ssize_t function(void*, const scope char*, size_t) _write;

        __sbuf _ub;
        ubyte* _up;
        int _ur;

        ubyte[3] _ubuf;
        ubyte[1] _nbuf;

        int function(void*) _flush;

        char[__sbuf.sizeof - _flush.sizeof] _lb_unused = void;

        int _blksize;
        off_t _offset;
        static assert(off_t.sizeof == 8);
    }

    alias _iobuf = __sFILE;

    alias FILE = shared(__sFILE);
} else version (OpenBSD) {
    alias fpos_t = off_t;

    struct __sFILE {
        ubyte* _p;
        int _r;
        int _w;
        short _flags;
        short _file;
        __sbuf _bf;
        int _lbfsize;

        void* _cookie;
        int function(void*) _close;
        int function(void*, scope char*, int) _read;
        fpos_t function(void*, fpos_t, int) _seek;
        int function(void*, scope const char*, int) _write;

        __sbuf _ext;
        ubyte* _up;
        int _ur;

        ubyte[3] _ubuf;
        ubyte[1] _nbuf;

        __sbuf _lb;

        int _blksize;
        fpos_t _offset;
    }

    alias _iobuf = __sFILE;

    alias FILE = shared(__sFILE);
} else version (DragonFlyBSD) {
    alias fpos_t = off_t;

    struct __FILE_public {
        ubyte** _p;
        int _flags;
        int _fileno;
        ssize_t _r;
        ssize_t _w;
        ssize_t _lbfsize;
    }

    alias _iobuf = __FILE_public;
    alias FILE = shared(__FILE_public);
} else version (Solaris) {
    import cm3.core.stdc.wchar_ : mbstate_t;

    alias fpos_t = c_long;

    version (D_LP64) {
        struct _iobuf {
            char* _ptr;
            char* _base;
            char* _end;
            size_t _cnt;
            int _file;
            int _flag;
            ubyte[24] _lock;
            mbstate_t _state;
            ubyte[32] __fill;
        }
    } else {
        struct _iobuf {
            char* _ptr;
            int _cnt;
            char* _base;
            char _flag = 0;
            char _magic = 0;
            ushort __flags;

        }
    }

    alias FILE = shared(_iobuf);
} else version (CRuntime_Bionic) {
    alias fpos_t = c_long;

    struct __sFILE {
        ubyte* _p;
        int _r;
        int _w;
        short _flags;
        short _file;
        __sbuf _bf;
        int _lbfsize;

        void* _cookie;
        int function(void*) _close;
        int function(void*, scope char*, int) _read;
        fpos_t function(void*, fpos_t, int) _seek;
        int function(void*, scope const char*, int) _write;

        __sbuf _ext;
        ubyte* _up;
        int _ur;

        ubyte[3] _ubuf;
        ubyte[1] _nbuf;

        __sbuf _lb;

        int _blksize;
        fpos_t _offset;
    }

    alias _iobuf = __sFILE;

    alias FILE = shared(__sFILE);
} else version (CRuntime_Newlib) {
    import core.sys.posix.sys.types : ssize_t;
    import cm3.core.stdc.wchar_ : mbstate_t;

    alias fpos_t = c_long;

    struct __sFILE {
        ubyte* _p;
        int _r;
        int _w;
        short _flags;
        short _file;
        __sbuf _bf;
        int _lbfsize;

        void* _data;
        void* _cookie;

        ssize_t function(void*, void*, scope char*, size_t) _read;
        ssize_t function(void*, void*, scope const char*, size_t) _write;
        fpos_t function(void*, void*, fpos_t, int) _seek;
        int function(void*, void*) _close;

        __sbuf _ub;
        ubyte* _up;
        int _ur;

        ubyte[3] _ubuf;
        ubyte[1] _nbuf;

        __sbuf _lb;

        int _blksize;
        int _flags2;

        long _offset;
        void* _unused;

        void* _lock;
        mbstate_t _mbstate;
    }

    alias _iobuf = __sFILE;

    alias FILE = shared(__sFILE);
} else version (CRuntime_UClibc) {
    import cm3.core.stdc.wchar_ : mbstate_t;
    import cm3.core.stdc.stddef : wchar_t;
    import core.sys.posix.sys.types : ssize_t, pthread_mutex_t;

    struct fpos_t {
        long __pos;
        mbstate_t __state;
        int __mblen_pending;
    }

    struct _IO_cookie_io_functions_t {
        ssize_t function(void* __cookie, char* __buf, size_t __bufsize) read;
        ssize_t function(void* __cookie, const char* __buf, size_t __bufsize) write;
        int function(void* __cookie, long* __pos, int __whence) seek;
        int function(void* __cookie) close;
    }

    alias cookie_io_functions_t = _IO_cookie_io_functions_t;

    struct __STDIO_FILE_STRUCT {
        ushort __modeflags;
        char[2] __ungot_width = 0;
        int __filedes;
        char* __bufstart;
        char* __bufend;
        char* __bufpos;
        char* __bufread;
        char* __bufgetc_u;
        char* __bufputc_u;
        __STDIO_FILE_STRUCT* __nextopen;
        void* __cookie;
        _IO_cookie_io_functions_t __gcs;
        wchar_t[2] __ungot = 0;
        mbstate_t __state;
        void* __unused;
        int __user_locking;
        pthread_mutex_t __lock;
    }

    alias _iobuf = __STDIO_FILE_STRUCT;

    alias FILE = shared(__STDIO_FILE_STRUCT);
} else {
    static assert(false, "Unsupported platform");
}

enum {
    _F_RDWR = 0x0003,

    _F_READ = 0x0001,

    _F_WRIT = 0x0002,

    _F_BUF = 0x0004,

    _F_LBUF = 0x0008,

    _F_ERR = 0x0010,

    _F_EOF = 0x0020,

    _F_BIN = 0x0040,

    _F_IN = 0x0080,

    _F_OUT = 0x0100,

    _F_TERM = 0x0200,
}

version (cm3) {
    enum {
        _IOFBF = 0,

        _IOLBF = 1,

        _IONBF = 2,
    }

    private {
        shared struct _reent {
            int _errno;
            __sFILE* _stdin;
            __sFILE* _stdout;
            __sFILE* _stderr;
        }

        _reent* __getreent();
    }

    pragma(inline, true) {
        @property auto stdin()() {
            return __getreent()._stdin;
        }

        @property auto stdout()() {
            return __getreent()._stdout;
        }

        @property auto stderr()() {
            return __getreent()._stderr;
        }
    }
} else version (CRuntime_DigitalMars) {
    enum {
        _IOFBF = 0,

        _IOLBF = 0x40,

        _IONBF = 4,

        _IOREAD = 1,

        _IOWRT = 2,

        _IOMYBUF = 8,

        _IOEOF = 0x10,

        _IOERR = 0x20,

        _IOSTRG = 0x40,

        _IORW = 0x80,

        _IOTRAN = 0x100,

        _IOAPP = 0x200,
    }

    extern shared void function() _fcloseallp;

    private extern shared FILE[_NFILE] _iob;

    enum stdin = &_iob[0];

    enum stdout = &_iob[1];

    enum stderr = &_iob[2];

    enum stdaux = &_iob[3];

    enum stdprn = &_iob[4];
} else version (CRuntime_Microsoft) {
    enum {
        _IOFBF = 0,

        _IOLBF = 0x40,

        _IONBF = 4,

        _IOREAD = 1,

        _IOWRT = 2,

        _IOMYBUF = 8,

        _IOEOF = 0x10,

        _IOERR = 0x20,

        _IOSTRG = 0x40,

        _IORW = 0x80,

        _IOAPP = 0x200,

        _IOAPPEND = 0x200,
    }

    extern shared void function() _fcloseallp;

    FILE* __acrt_iob_func(int hnd);

    FILE* stdin()() {
        return __acrt_iob_func(0);
    }

    FILE* stdout()() {
        return __acrt_iob_func(1);
    }

    FILE* stderr()() {
        return __acrt_iob_func(2);
    }
} else version (CRuntime_Glibc) {
    enum {
        _IOFBF = 0,

        _IOLBF = 1,

        _IONBF = 2,
    }

    extern shared FILE* stdin;

    extern shared FILE* stdout;

    extern shared FILE* stderr;
} else version (Darwin) {
    enum {
        _IOFBF = 0,

        _IOLBF = 1,

        _IONBF = 2,
    }

    private extern shared FILE* __stdinp;
    private extern shared FILE* __stdoutp;
    private extern shared FILE* __stderrp;

    alias stdin = __stdinp;

    alias stdout = __stdoutp;

    alias stderr = __stderrp;
} else version (FreeBSD) {
    enum {
        _IOFBF = 0,

        _IOLBF = 1,

        _IONBF = 2,
    }

    private extern shared FILE* __stdinp;
    private extern shared FILE* __stdoutp;
    private extern shared FILE* __stderrp;

    alias stdin = __stdinp;

    alias stdout = __stdoutp;

    alias stderr = __stderrp;
} else version (NetBSD) {
    enum {
        _IOFBF = 0,

        _IOLBF = 1,

        _IONBF = 2,
    }

    private extern shared FILE[3] __sF;
    @property auto __stdin()() {
        return &__sF[0];
    }

    @property auto __stdout()() {
        return &__sF[1];
    }

    @property auto __stderr()() {
        return &__sF[2];
    }

    alias stdin = __stdin;

    alias stdout = __stdout;

    alias stderr = __stderr;
} else version (OpenBSD) {
    enum {
        _IOFBF = 0,

        _IOLBF = 1,

        _IONBF = 2,
    }

    private extern shared FILE[3] __sF;
    @property auto __stdin()() {
        return &__sF[0];
    }

    @property auto __stdout()() {
        return &__sF[1];
    }

    @property auto __stderr()() {
        return &__sF[2];
    }

    alias stdin = __stdin;

    alias stdout = __stdout;

    alias stderr = __stderr;
} else version (DragonFlyBSD) {
    enum {
        _IOFBF = 0,
        _IOLBF = 1,
        _IONBF = 2,
    }

    private extern shared FILE* __stdinp;
    private extern shared FILE* __stdoutp;
    private extern shared FILE* __stderrp;

    alias stdin = __stdinp;
    alias stdout = __stdoutp;
    alias stderr = __stderrp;
} else version (Solaris) {
    enum {
        _IOFBF = 0x00,

        _IOLBF = 0x40,

        _IONBF = 0x04,

        _IOEOF = 0x20,

        _IOERR = 0x40,

        _IOREAD = 0x01,

        _IOWRT = 0x02,

        _IORW = 0x80,

        _IOMYBUF = 0x08,
    }

    private extern shared FILE[_NFILE] __iob;

    @property auto stdin()() {
        return &__iob[0];
    }

    @property auto stdout()() {
        return &__iob[1];
    }

    @property auto stderr()() {
        return &__iob[2];
    }
} else version (CRuntime_Bionic) {
    enum {
        _IOFBF = 0,

        _IOLBF = 1,

        _IONBF = 2,
    }

    private extern shared FILE[3] __sF;

    @property auto stdin()() {
        return &__sF[0];
    }

    @property auto stdout()() {
        return &__sF[1];
    }

    @property auto stderr()() {
        return &__sF[2];
    }
} else version (CRuntime_Musl) {
    extern shared FILE* stdin;

    extern shared FILE* stdout;

    extern shared FILE* stderr;
    enum {
        _IOFBF = 0,

        _IOLBF = 1,

        _IONBF = 2,
    }
} else version (CRuntime_Newlib) {
    enum {
        _IOFBF = 0,

        _IOLBF = 1,

        _IONBF = 2,
    }

    private {
        shared struct _reent {
            int _errno;
            __sFILE* _stdin;
            __sFILE* _stdout;
            __sFILE* _stderr;
        }

        _reent* __getreent();
    }

    pragma(inline, true) {
        @property auto stdin()() {
            return __getreent()._stdin;
        }

        @property auto stdout()() {
            return __getreent()._stdout;
        }

        @property auto stderr()() {
            return __getreent()._stderr;
        }
    }
} else version (CRuntime_UClibc) {
    enum {
        _IOFBF = 0,

        _IOLBF = 1,

        _IONBF = 2,
    }

    extern shared FILE* stdin;

    extern shared FILE* stdout;

    extern shared FILE* stderr;
} else version (WASI) {
    extern shared FILE* stdin;

    extern shared FILE* stdout;

    extern shared FILE* stderr;
    enum {
        _IOFBF = 0,

        _IOLBF = 1,

        _IONBF = 2,
    }
} else {
    static assert(false, "Unsupported platform");
}

int remove(scope const char* filename);

int rename(scope const char* from, scope const char* to);

@trusted FILE* tmpfile();

char* tmpnam(char* s);

int fclose(FILE* stream);

@trusted int fflush(FILE* stream);

FILE* fopen(scope const char* filename, scope const char* mode);

FILE* freopen(scope const char* filename, scope const char* mode, FILE* stream);

void setbuf(FILE* stream, char* buf);

int setvbuf(FILE* stream, char* buf, int mode, size_t size);

version (MinGW) {
    pragma(printf)
    int __mingw_fprintf(FILE* stream, scope const char* format, scope const...);

    alias fprintf = __mingw_fprintf;

    pragma(scanf)
    int __mingw_fscanf(FILE* stream, scope const char* format, scope...);

    alias fscanf = __mingw_fscanf;

    pragma(printf)
    int __mingw_sprintf(scope char* s, scope const char* format, scope const...);

    alias sprintf = __mingw_sprintf;

    pragma(scanf)
    int __mingw_sscanf(scope const char* s, scope const char* format, scope...);

    alias sscanf = __mingw_sscanf;

    pragma(printf)
    int __mingw_vfprintf(FILE* stream, scope const char* format, va_list arg);

    alias vfprintf = __mingw_vfprintf;

    pragma(scanf)
    int __mingw_vfscanf(FILE* stream, scope const char* format, va_list arg);

    alias vfscanf = __mingw_vfscanf;

    pragma(printf)
    int __mingw_vsprintf(scope char* s, scope const char* format, va_list arg);

    alias vsprintf = __mingw_vsprintf;

    pragma(scanf)
    int __mingw_vsscanf(scope const char* s, scope const char* format, va_list arg);

    alias vsscanf = __mingw_vsscanf;

    pragma(printf)
    int __mingw_vprintf(scope const char* format, va_list arg);

    alias vprintf = __mingw_vprintf;

    pragma(scanf)
    int __mingw_vscanf(scope const char* format, va_list arg);

    alias vscanf = __mingw_vscanf;

    pragma(printf)
    int __mingw_printf(scope const char* format, scope const...);

    alias printf = __mingw_printf;

    pragma(scanf)
    int __mingw_scanf(scope const char* format, scope...);

    alias scanf = __mingw_scanf;
} else version (CRuntime_Glibc) {
    pragma(printf)
    int fprintf(FILE* stream, scope const char* format, scope const...);

    pragma(scanf)
    int __isoc99_fscanf(FILE* stream, scope const char* format, scope...);

    alias fscanf = __isoc99_fscanf;

    pragma(printf)
    int sprintf(scope char* s, scope const char* format, scope const...);

    pragma(scanf)
    int __isoc99_sscanf(scope const char* s, scope const char* format, scope...);

    alias sscanf = __isoc99_sscanf;

    pragma(printf)
    int vfprintf(FILE* stream, scope const char* format, va_list arg);

    pragma(scanf)
    int __isoc99_vfscanf(FILE* stream, scope const char* format, va_list arg);

    alias vfscanf = __isoc99_vfscanf;

    pragma(printf)
    int vsprintf(scope char* s, scope const char* format, va_list arg);

    pragma(scanf)
    int __isoc99_vsscanf(scope const char* s, scope const char* format, va_list arg);

    alias vsscanf = __isoc99_vsscanf;

    pragma(printf)
    int vprintf(scope const char* format, va_list arg);

    pragma(scanf)
    int __isoc99_vscanf(scope const char* format, va_list arg);

    alias vscanf = __isoc99_vscanf;

    pragma(printf)
    int printf(scope const char* format, scope const...);

    pragma(scanf)
    int __isoc99_scanf(scope const char* format, scope...);

    alias scanf = __isoc99_scanf;
} else {
    pragma(printf)
    int fprintf(FILE* stream, scope const char* format, scope const...);

    pragma(scanf)
    int fscanf(FILE* stream, scope const char* format, scope...);

    pragma(printf)
    int sprintf(scope char* s, scope const char* format, scope const...);

    pragma(scanf)
    int sscanf(scope const char* s, scope const char* format, scope...);

    pragma(printf)
    int vfprintf(FILE* stream, scope const char* format, va_list arg);

    pragma(scanf)
    int vfscanf(FILE* stream, scope const char* format, va_list arg);

    pragma(printf)
    int vsprintf(scope char* s, scope const char* format, va_list arg);

    pragma(scanf)
    int vsscanf(scope const char* s, scope const char* format, va_list arg);

    pragma(printf)
    int vprintf(scope const char* format, va_list arg);

    pragma(scanf)
    int vscanf(scope const char* format, va_list arg);

    pragma(printf)
    int printf(scope const char* format, scope const...);

    pragma(scanf)
    int scanf(scope const char* format, scope...);
}

@trusted int fgetc(FILE* stream);

@trusted int fputc(int c, FILE* stream);

char* fgets(char* s, int n, FILE* stream);

int fputs(scope const char* s, FILE* stream);

char* gets(char* s);

int puts(scope const char* s);

extern (D) {
    @trusted int getchar()() {
        return getc(stdin);
    }

    @trusted int putchar()(int c) {
        return putc(c, stdout);
    }
}

alias getc = fgetc;

alias putc = fputc;

@trusted int ungetc(int c, FILE* stream);

size_t fread(scope void* ptr, size_t size, size_t nmemb, FILE* stream);

size_t fwrite(scope const void* ptr, size_t size, size_t nmemb, FILE* stream);

@trusted int fgetpos(FILE* stream, scope fpos_t* pos);

@trusted int fsetpos(FILE* stream, scope const fpos_t* pos);

@trusted int fseek(FILE* stream, c_long offset, int whence);

@trusted c_long ftell(FILE* stream);

version (cm3) {
    @trusted void rewind(FILE* stream);

    @trusted pure void clearerr(FILE* stream);

    @trusted pure int feof(FILE* stream);

    @trusted pure int ferror(FILE* stream);

    @trusted int fileno(FILE*);

    pragma(printf)
    int snprintf(scope char* s, size_t n, scope const char* format, scope const...);

    pragma(printf)
    int vsnprintf(scope char* s, size_t n, scope const char* format, va_list arg);
} else version (CRuntime_DigitalMars) {
    extern (D) {
        @trusted void rewind()(FILE* stream) {
            int __u;
            __u = fseek(stream, 0L, SEEK_SET);
            stream._flag = stream._flag & ~_IOERR;
        }

        @trusted pure void clearerr()(FILE* stream) {
            stream._flag = stream._flag & ~(_IOERR | _IOEOF);
        }

        @trusted pure int feof()(FILE* stream) {
            return stream._flag & _IOEOF;
        }

        @trusted pure int ferror()(FILE* stream) {
            return stream._flag & _IOERR;
        }

        @trusted pure int fileno()(FILE* stream) {
            return stream._file;
        }
    }

    pragma(printf)
    int _snprintf(scope char* s, size_t n, scope const char* fmt, scope const...);

    alias snprintf = _snprintf;

    pragma(printf)
    int _vsnprintf(scope char* s, size_t n, scope const char* format, va_list arg);

    alias vsnprintf = _vsnprintf;

    int _fputc_nlock(int c, _iobuf* fp);

    int _fputwc_nlock(int c, _iobuf* fp);

    int _fgetc_nlock(_iobuf* fp);

    int _fgetwc_nlock(_iobuf* fp);

    int __fp_lock(FILE* fp);

    void __fp_unlock(FILE* fp);

    int setmode(int fd, int mode);
} else version (CRuntime_Microsoft) {
    @trusted void rewind(FILE* stream);

    @trusted pure void clearerr(FILE* stream);

    @trusted pure int feof(FILE* stream);

    @trusted pure int ferror(FILE* stream);

    @trusted pure int fileno(FILE* stream);

    version (MinGW) {
        pragma(printf)
        int __mingw_snprintf(scope char* s, size_t n, scope const char* fmt, scope const...);

        alias _snprintf = __mingw_snprintf;

        alias snprintf = __mingw_snprintf;

        pragma(printf)
        int __mingw_vsnprintf(scope char* s, size_t n, scope const char* format, va_list arg);

        alias _vsnprintf = __mingw_vsnprintf;

        alias vsnprintf = __mingw_vsnprintf;
    } else {
        pragma(printf)
        int _snprintf(scope char* s, size_t n, scope const char* format, scope const...);

        pragma(printf)
        int snprintf(scope char* s, size_t n, scope const char* format, scope const...);

        pragma(printf)
        int _vsnprintf(scope char* s, size_t n, scope const char* format, va_list arg);

        pragma(printf)
        int vsnprintf(scope char* s, size_t n, scope const char* format, va_list arg);
    }

    import cm3.core.stdc.stddef : wchar_t;
    import cm3.core.stdc.wchar_ : wint_t;

    int _fputc_nolock(int c, _iobuf* fp);

    int _fgetc_nolock(_iobuf* fp);

    wint_t _fputwc_nolock(wchar_t c, _iobuf* fp);

    wint_t _fgetwc_nolock(_iobuf* fp);

    void _lock_file(FILE* fp);

    void _unlock_file(FILE* fp);

    int _setmode(int fd, int mode);

    int _fseeki64(FILE* stream, long offset, int origin);

    long _ftelli64(FILE* stream);

    intptr_t _get_osfhandle(int fd);

    int _open_osfhandle(intptr_t osfhandle, int flags);
} else version (CRuntime_Glibc) {
    @trusted void rewind(FILE* stream);

    @trusted pure void clearerr(FILE* stream);

    @trusted pure int feof(FILE* stream);

    @trusted pure int ferror(FILE* stream);

    @trusted int fileno(FILE*);

    pragma(printf)
    int snprintf(scope char* s, size_t n, scope const char* format, scope const...);

    pragma(printf)
    int vsnprintf(scope char* s, size_t n, scope const char* format, va_list arg);

    import cm3.core.stdc.wchar_ : wint_t;
    import cm3.core.stdc.stddef : wchar_t;

    int fputc_unlocked(int c, _iobuf* stream);

    int fgetc_unlocked(_iobuf* stream);

    wint_t fputwc_unlocked(wchar_t wc, _iobuf* stream);

    wint_t fgetwc_unlocked(_iobuf* stream);
} else version (Darwin) {
    @trusted void rewind(FILE*);

    @trusted pure void clearerr(FILE*);

    @trusted pure int feof(FILE*);

    @trusted pure int ferror(FILE*);

    @trusted int fileno(FILE*);

    pragma(printf)
    int snprintf(scope char* s, size_t n, scope const char* format, scope const...);

    pragma(printf)
    int vsnprintf(scope char* s, size_t n, scope const char* format, va_list arg);
} else version (FreeBSD) {
    @trusted void rewind(FILE*);

    @trusted pure void clearerr(FILE*);

    @trusted pure int feof(FILE*);

    @trusted pure int ferror(FILE*);

    @trusted int fileno(FILE*);

    pragma(printf)
    int snprintf(scope char* s, size_t n, scope const char* format, scope const...);

    pragma(printf)
    int vsnprintf(scope char* s, size_t n, scope const char* format, va_list arg);
} else version (NetBSD) {
    @trusted void rewind(FILE*);

    @trusted pure void clearerr(FILE*);

    @trusted pure int feof(FILE*);

    @trusted pure int ferror(FILE*);

    @trusted int fileno(FILE*);

    pragma(printf)
    int snprintf(char* s, size_t n, const scope char* format, scope const...);

    pragma(printf)
    int vsnprintf(char* s, size_t n, const scope char* format, va_list arg);
} else version (OpenBSD) {
    @trusted void rewind(FILE*);

    private {
        pragma(mangle, "clearerr")
        @trusted pure void __clearerr(FILE*);

        pragma(mangle, "feof")
        @trusted pure int __feof(FILE*);

        pragma(mangle, "ferror")
        @trusted pure int __ferror(FILE*);

        pragma(mangle, "fileno")
        @trusted int __fileno(FILE*);
    }

    enum __SLBF = 0x0001;
    enum __SNBF = 0x0002;
    enum __SRD = 0x0004;
    enum __SWR = 0x0008;
    enum __SRW = 0x0010;
    enum __SEOF = 0x0020;
    enum __SERR = 0x0040;
    enum __SMBF = 0x0080;
    enum __SAPP = 0x0100;
    enum __SSTR = 0x0200;
    enum __SOPT = 0x0400;
    enum __SNPT = 0x0800;
    enum __SOFF = 0x1000;
    enum __SMOD = 0x2000;
    enum __SALC = 0x4000;
    enum __SIGN = 0x8000;

    extern __gshared int __isthreaded;

    extern (D) {
        @trusted void __sclearerr()(FILE* p) {
            p._flags = p._flags & ~(__SERR | __SEOF);
        }

        @trusted int __sfeof()(FILE* p) {
            return (p._flags & __SEOF) != 0;
        }

        @trusted int __sferror()(FILE* p) {
            return (p._flags & __SERR) != 0;
        }

        @trusted int __sfileno()(FILE* p) {
            return p._file;
        }

        @trusted pure void clearerr()(FILE* file) {
            !__isthreaded ? __sclearerr(file) : __clearerr(file);
        }

        @trusted pure int feof()(FILE* file) {
            return !__isthreaded ? __sfeof(file) : __feof(file);
        }

        @trusted pure int ferror()(FILE* file) {
            return !__isthreaded ? __sferror(file) : __ferror(file);
        }

        @trusted int fileno()(FILE* file) {
            return !__isthreaded ? __sfileno(file) : __fileno(file);
        }
    }

    pragma(printf)
    int snprintf(scope char* s, size_t n, scope const char* format, scope const...);

    pragma(printf)
    int vsnprintf(scope char* s, size_t n, scope const char* format, va_list arg);
} else version (DragonFlyBSD) {
    @trusted void rewind(FILE*);
    @trusted pure void clearerr(FILE*);
    @trusted pure int feof(FILE*);
    @trusted pure int ferror(FILE*);
    @trusted int fileno(FILE*);

    enum __SLBF = 0x0001;
    enum __SNBF = 0x0002;
    enum __SRD = 0x0004;
    enum __SWR = 0x0008;
    enum __SRW = 0x0010;
    enum __SEOF = 0x0020;
    enum __SERR = 0x0040;
    enum __SMBF = 0x0080;
    enum __SAPP = 0x0100;
    enum __SSTR = 0x0200;
    enum __SOPT = 0x0400;
    enum __SNPT = 0x0800;
    enum __SOFF = 0x1000;
    enum __SMOD = 0x2000;
    enum __SALC = 0x4000;
    enum __SIGN = 0x8000;

    pragma(printf)
    int snprintf(scope char* s, size_t n, scope const char* format, scope const...);
    pragma(printf)
    int vsnprintf(scope char* s, size_t n, scope const char* format, va_list arg);
} else version (Solaris) {
    @trusted void rewind(FILE*);

    @trusted pure void clearerr(FILE*);

    @trusted pure int feof(FILE*);

    @trusted pure int ferror(FILE*);

    @trusted int fileno(FILE*);

    pragma(printf)
    int snprintf(scope char* s, size_t n, scope const char* format, scope const...);

    pragma(printf)
    int vsnprintf(scope char* s, size_t n, scope const char* format, va_list arg);
} else version (CRuntime_Bionic) {
    @trusted void rewind(FILE*);

    @trusted pure void clearerr(FILE*);

    @trusted pure int feof(FILE*);

    @trusted pure int ferror(FILE*);

    @trusted int fileno(FILE*);

    pragma(printf)
    int snprintf(scope char* s, size_t n, scope const char* format, scope const...);

    pragma(printf)
    int vsnprintf(scope char* s, size_t n, scope const char* format, va_list arg);
} else version (CRuntime_Musl) {
    @trusted void rewind(FILE* stream);

    @trusted pure void clearerr(FILE* stream);

    @trusted pure int feof(FILE* stream);

    @trusted pure int ferror(FILE* stream);

    @trusted int fileno(FILE*);

    pragma(printf)
    int snprintf(scope char* s, size_t n, scope const char* format, scope const...);

    pragma(printf)
    int vsnprintf(scope char* s, size_t n, scope const char* format, va_list arg);
} else version (CRuntime_Newlib) {
    @trusted void rewind(FILE* stream);

    @trusted pure void clearerr(FILE* stream);

    @trusted pure int feof(FILE* stream);

    @trusted pure int ferror(FILE* stream);

    @trusted int fileno(FILE*);

    pragma(printf)
    int snprintf(scope char* s, size_t n, scope const char* format, scope const...);

    pragma(printf)
    int vsnprintf(scope char* s, size_t n, scope const char* format, va_list arg);

    import cm3.core.stdc.wchar_ : wint_t;
    import cm3.core.stdc.stddef : wchar_t;

    int fputc_unlocked(int c, _iobuf* stream);

    int fgetc_unlocked(_iobuf* stream);

    wint_t fputwc_unlocked(wchar_t wc, _iobuf* stream);

    wint_t fgetwc_unlocked(_iobuf* stream);
} else version (CRuntime_UClibc) {
    @trusted void rewind(FILE* stream);

    @trusted pure void clearerr(FILE* stream);

    @trusted pure int feof(FILE* stream);

    @trusted pure int ferror(FILE* stream);

    @trusted int fileno(FILE*);

    pragma(printf)
    int snprintf(scope char* s, size_t n, scope const char* format, scope const...);

    pragma(printf)
    int vsnprintf(scope char* s, size_t n, scope const char* format, va_list arg);
} else version (WASI) {
    @trusted void rewind(FILE* stream);

    @trusted pure void clearerr(FILE* stream);

    @trusted pure int feof(FILE* stream);

    @trusted pure int ferror(FILE* stream);

    @trusted int fileno(FILE*);

    int snprintf(scope char* s, size_t n, scope const char* format, ...);

    int vsnprintf(scope char* s, size_t n, scope const char* format, va_list arg);
} else {
    static assert(false, "Unsupported platform");
}

void perror(scope const char* s);

version (CRuntime_DigitalMars) {
    version (none)
        import core.sys.windows.windows : HANDLE, _WaitSemaphore, _ReleaseSemaphore;
    else {
        private alias HANDLE = void*;
        private void _WaitSemaphore(int iSemaphore);
        private void _ReleaseSemaphore(int iSemaphore);
    }

    enum {
        FHND_APPEND = 0x04,

        FHND_DEVICE = 0x08,

        FHND_TEXT = 0x10,

        FHND_BYTE = 0x20,

        FHND_WCHAR = 0x40,
    }

    private enum _MAX_SEMAPHORES = 10 + _NFILE;
    private enum _semIO = 3;

    private extern __gshared short[_MAX_SEMAPHORES] _iSemLockCtrs;
    private extern __gshared int[_MAX_SEMAPHORES] _iSemThreadIds;
    private extern __gshared int[_MAX_SEMAPHORES] _iSemNestCount;
    private extern __gshared HANDLE[_NFILE] _osfhnd;
    extern shared ubyte[_NFILE] __fhnd_info;

    private void LockSemaphore()(uint num) {
        ref auto __iSemLockCtrs = _iSemLockCtrs[EDX * 2];
        asm nothrow @nogc {
            mov EDX, num;
            lock;
            inc __iSemLockCtrs;
            jz lsDone;
            push EDX;
            call _WaitSemaphore;
            add ESP, 4;
        }

    lsDone: {
        }
    }

    private void UnlockSemaphore()(uint num) {
        ref auto __iSemLockCtrs = _iSemLockCtrs[EDX * 2];
        asm nothrow @nogc {
            mov EDX, num;
            lock;
            dec __iSemLockCtrs;
            js usDone;
            push EDX;
            call _ReleaseSemaphore;
            add ESP, 4;
        }

    usDone: {
        }
    }

    int _handleToFD()(HANDLE h, int flags) {
        LockSemaphore(_semIO);
        scope (exit)
            UnlockSemaphore(_semIO);

        foreach (fd; 0 .. _NFILE) {
            if (!_osfhnd[fd]) {
                _osfhnd[fd] = h;
                __fhnd_info[fd] = cast(ubyte)flags;
                return fd;
            }
        }

        return -1;
    }

    HANDLE _fdToHandle()(int fd) {
        if (fd < 0 || fd >= _NFILE)
            return null;

        return _osfhnd[fd];
    }

    enum {
        STDIN_FILENO = 0,

        STDOUT_FILENO = 1,

        STDERR_FILENO = 2,
    }

    int open(scope const(char)* filename, int flags, ...);
    alias _open = open;
    int _wopen(scope const wchar* filename, int oflag, ...);
    int sopen(scope const char* filename, int oflag, int shflag, ...);
    alias _sopen = sopen;
    int _wsopen(scope const wchar* filename, int oflag, int shflag, ...);
    int close(int fd);
    alias _close = close;
    FILE* fdopen(int fd, scope const(char)* flags);
    alias _fdopen = fdopen;
    FILE* _wfdopen(int fd, scope const(wchar)* flags);

} else version (CRuntime_Microsoft) {
    int _open(scope const char* filename, int oflag, ...);
    int _wopen(scope const wchar* filename, int oflag, ...);
    int _sopen(scope const char* filename, int oflag, int shflag, ...);
    int _wsopen(scope const wchar* filename, int oflag, int shflag, ...);
    int _close(int fd);
    FILE* _fdopen(int fd, scope const(char)* flags);
    FILE* _wfdopen(int fd, scope const(wchar)* flags);
}

version (Windows) {
    enum {
        _O_RDONLY = 0x0000,
        O_RDONLY = _O_RDONLY,
        _O_WRONLY = 0x0001,
        O_WRONLY = _O_WRONLY,
        _O_RDWR = 0x0002,
        O_RDWR = _O_RDWR,
        _O_APPEND = 0x0008,
        O_APPEND = _O_APPEND,
        _O_CREAT = 0x0100,
        O_CREAT = _O_CREAT,
        _O_TRUNC = 0x0200,
        O_TRUNC = _O_TRUNC,
        _O_EXCL = 0x0400,
        O_EXCL = _O_EXCL,
        _O_TEXT = 0x4000,
        O_TEXT = _O_TEXT,
        _O_BINARY = 0x8000,
        O_BINARY = _O_BINARY,
        _O_WTEXT = 0x10000,
        _O_U16TEXT = 0x20000,
        _O_U8TEXT = 0x40000,
        _O_ACCMODE = (_O_RDONLY | _O_WRONLY | _O_RDWR),
        O_ACCMODE = _O_ACCMODE,
        _O_RAW = _O_BINARY,
        O_RAW = _O_BINARY,
        _O_NOINHERIT = 0x0080,
        O_NOINHERIT = _O_NOINHERIT,
        _O_TEMPORARY = 0x0040,
        O_TEMPORARY = _O_TEMPORARY,
        _O_SHORT_LIVED = 0x1000,
        _O_SEQUENTIAL = 0x0020,
        O_SEQUENTIAL = _O_SEQUENTIAL,
        _O_RANDOM = 0x0010,
        O_RANDOM = _O_RANDOM,
    }

    enum {
        _S_IREAD = 0x0100,
        S_IREAD = _S_IREAD,
        _S_IWRITE = 0x0080,
        S_IWRITE = _S_IWRITE,
    }

    enum {
        _SH_DENYRW = 0x10,
        SH_DENYRW = _SH_DENYRW,
        _SH_DENYWR = 0x20,
        SH_DENYWR = _SH_DENYWR,
        _SH_DENYRD = 0x30,
        SH_DENYRD = _SH_DENYRD,
        _SH_DENYNO = 0x40,
        SH_DENYNO = _SH_DENYNO,
    }
}
