#include "log.h"
#include <stdarg.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

static const char *level_strings[] = {
    "DEBUG", "INFO", "WARN", "ERROR"
};

void log_init(void) {
    // Already done in stdio_init_all, but you can re-init if needed
}

void log_printf(log_level_t level, const char *format, ...) {
    static char buffer[256];

    uint32_t timestamp = to_ms_since_boot(get_absolute_time());

    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    printf("[%lu ms] [%s] %s\n", timestamp, level_strings[level], buffer);
}