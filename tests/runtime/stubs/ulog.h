#ifndef HOST_ULOG_H
#define HOST_ULOG_H

#include <stdio.h>

#define LOG_E(...) do { printf(__VA_ARGS__); putchar('\n'); } while (0)
#define LOG_I(...) do { printf(__VA_ARGS__); putchar('\n'); } while (0)

#endif
