#include "log.h"
#include "pico/stdio.h" // Or any output function you're using

void logInfo(const char *format, ...) {
    va_list args;
    va_start(args, format);

    printf("[INFO] ");
    vprintf(format, args);

    va_end(args);
}

void logWarn(const char *format, ...) {
    va_list args;
    va_start(args, format);

    printf("[WARN] ");
    vprintf(format, args);

    va_end(args);
}

void logError(const char *format, ...) {
    va_list args;
    va_start(args, format);

    printf("[ERROR] ");
    vprintf(format, args);

    va_end(args);
}