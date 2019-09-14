/*
 * debug_printf.h
 *
 *  Created on: Feb 3, 2018
 *      Author: dev
 */

#ifndef AGBOTS_INCLUDE_DEBUG_PRINTF_H_
#define AGBOTS_INCLUDE_DEBUG_PRINTF_H_

#define DEBUG 0
#define LOG_LEVEL_THRES 0
//#define DEBUG_PRINT(level, fmt, ...) \
//            do { if (level <= DEBUG) fprintf(stderr, fmt, __VA_ARGS__); } while (0)
#define DEBUG_PRINT(level, fmt, ...) \
        do { if (level <= DEBUG) fprintf(stdout, fmt, __VA_ARGS__); } while (0)
#define DEBUG_PS(level, str) \
        do { if (level <= DEBUG) fprintf(stdout, "%s", str); } while (0)

/*#if defined(DEBUG) && DEBUG >= 0
 #define DEBUG_PRINT(level, fmt, args...) if () fprintf(stderr, "DEBUG:%s:%d:%s(): " fmt, __FILE__, __LINE__, __func__, ##args)
#else
 #define DEBUG_PRINT(fmt, args...)
#endif*/

void LOG(bool addTimeInfo, uint8_t Level, const char* format, ...);
#endif /* AGBOTS_INCLUDE_DEBUG_PRINTF_H_ */
