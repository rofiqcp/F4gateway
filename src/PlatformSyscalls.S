#if defined(BOARD_F103C8)
.syntax unified
.thumb

.section .text.Platform_UnsupportedSyscall,"ax",%progbits
.global Platform_UnsupportedSyscall
.type Platform_UnsupportedSyscall, %function
.thumb_func
Platform_UnsupportedSyscall:
    movs r0, #1
    rsbs r0, r0, #0
    bx lr

.global _close
.thumb_set _close, Platform_UnsupportedSyscall
.global _isatty
.thumb_set _isatty, Platform_UnsupportedSyscall
.global _lseek
.thumb_set _lseek, Platform_UnsupportedSyscall
.global _read
.thumb_set _read, Platform_UnsupportedSyscall
.global _write
.thumb_set _write, Platform_UnsupportedSyscall
.global _fstat
.thumb_set _fstat, Platform_UnsupportedSyscall
#endif
