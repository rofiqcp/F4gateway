#if defined(BOARD_F103_BOOT)
#include <stddef.h>
#include <sys/stat.h>
#include <sys/types.h>

/*
 * Resident bootloader has no POSIX file-descriptor backend.
 * Every hook fails explicitly so accidental libc I/O can never look successful.
 */
int _close(int fd) {
  (void)fd;
  return -1;
}

int _isatty(int fd) {
  (void)fd;
  return 0;
}

off_t _lseek(int fd, off_t offset, int whence) {
  (void)fd;
  (void)offset;
  (void)whence;
  return (off_t)-1;
}

ssize_t _read(int fd, void *buffer, size_t length) {
  (void)fd;
  (void)buffer;
  (void)length;
  return (ssize_t)-1;
}

ssize_t _write(int fd, const void *buffer, size_t length) {
  (void)fd;
  (void)buffer;
  (void)length;
  return (ssize_t)-1;
}

int _fstat(int fd, struct stat *status) {
  (void)fd;
  (void)status;
  return -1;
}
#endif
