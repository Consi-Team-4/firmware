#ifndef LOG_H
#define LOG_H

#include <stdio.h>
#include <stdarg.h>

// Basic log function
void logInfo(const char *format, ...);

// Optionally, for future log levels
void logError(const char *format, ...);
void logWarn(const char *format, ...);

#endif // LOG_H