// gg_format.h: C++ formatting utilities (replaces printf/fprintf/sprintf)
//
// All string operations use std::string exclusively.
// Public API is pure C++17. Internal implementation wraps vsnprintf.
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GG_FORMAT_H
#define GG_FORMAT_H

#include <string>
#include <cstdio>
#include <cstdarg>
#include <iostream>
#include <mutex>

namespace gg {

// Format a string (replaces sprintf/snprintf)
__attribute__((format(printf, 1, 2)))
inline std::string format(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    va_list args_copy;
    va_copy(args_copy, args);
    int32_t size = vsnprintf(nullptr, 0, fmt, args);
    va_end(args);
    if (size <= 0) {
        va_end(args_copy);
        return {};
    }
    std::string result(static_cast<size_t>(size), '\0');
    vsnprintf(result.data(), static_cast<size_t>(size) + 1, fmt, args_copy);
    va_end(args_copy);
    return result;
}

// Print to stdout (replaces printf)
__attribute__((format(printf, 1, 2)))
inline void print(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    va_list args_copy;
    va_copy(args_copy, args);
    int32_t size = vsnprintf(nullptr, 0, fmt, args);
    va_end(args);
    if (size > 0) {
        std::string s(static_cast<size_t>(size), '\0');
        vsnprintf(s.data(), static_cast<size_t>(size) + 1, fmt, args_copy);
        std::cout << s;
    }
    va_end(args_copy);
}

// Print to stderr (replaces fprintf(stderr, ...))
__attribute__((format(printf, 1, 2)))
inline void eprint(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    va_list args_copy;
    va_copy(args_copy, args);
    int32_t size = vsnprintf(nullptr, 0, fmt, args);
    va_end(args);
    if (size > 0) {
        std::string s(static_cast<size_t>(size), '\0');
        vsnprintf(s.data(), static_cast<size_t>(size) + 1, fmt, args_copy);
        std::cerr << s;
    }
    va_end(args_copy);
}

// Print to a FILE* (replaces fprintf to log files, etc.)
__attribute__((format(printf, 2, 3)))
inline void fprint(FILE* fp, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vfprintf(fp, fmt, args);
    va_end(args);
}

} // namespace gg

#endif // GG_FORMAT_H
