/* Evidence-only newlib syscall stubs for the M5-06 stack-image link probe.
 *
 * This translation unit is NEVER part of the product firmware. It exists only
 * so that tests/guidance/stack_image_check.ps1 can link the project objects
 * with the real toolchain libm/libc and analyse the resulting static image,
 * which is how imported libm/libc call edges stop being "unknown".
 */
#include <stddef.h>

void _exit(int status) { (void)status; for (;;) { } }
int _close(int fd) { (void)fd; return -1; }
int _lseek(int fd, int offset, int whence) { (void)fd; (void)offset; (void)whence; return 0; }
int _read(int fd, void *buf, unsigned int len) { (void)fd; (void)buf; (void)len; return 0; }
int _write(int fd, const void *buf, unsigned int len) { (void)fd; (void)buf; return (int)len; }
void *_sbrk(int inc) { (void)inc; return (void *)-1; }
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { (void)pid; (void)sig; return -1; }
int _fstat(int fd, void *st) { (void)fd; (void)st; return 0; }
int _isatty(int fd) { (void)fd; return 1; }
