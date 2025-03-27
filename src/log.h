#ifndef LOG_H
#define LOG_H

#include <stdio.h>

typedef enum {
    LOG_DEBUG,
    LOG_INFO,
    LOG_WARN,
    LOG_ERROR
} log_level_t;

void log_init(void);
void log_printf(log_level_t level, const char *format, ...);

#endif