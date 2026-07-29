#ifndef GH_GSP_CONSTS_H
#define GH_GSP_CONSTS_H

/**
 * Copyright © 2025 TruePoint Technology Inc. All rights reserved.
 *
 * All the code, data, software, text, documentation, API and other TruePoint Technology Inc.'s
 * material cannot be used, or spread, or reproduced, or modified under any condition in any form
 * or means without TruePoint Technology Inc.'s prior official authorization.
 */

#ifdef WIN32
#pragma warning(disable : 4005)
#else
#define MSC_VER 0
#endif // _WIN_32_

// #pragma once

#include <stddef.h>

typedef signed char INT8;
typedef unsigned char UINT8;

typedef short INT16;
typedef unsigned short UINT16;

typedef int INT32;
typedef unsigned int UINT32;

typedef long long INT64;
typedef unsigned long long UINT64;

typedef float FLOAT32;
typedef double DOUBLE64;

typedef unsigned char BYTE;

#ifndef GSP_WINDOWS_BUILD
typedef unsigned short WORD;
typedef unsigned int DWORD;
#endif

typedef int INT;
typedef unsigned int UINT;

#ifndef CHAR_DEFINED
#define CHAR_DEFINED

typedef char CHAR;
#endif

typedef unsigned char UCHAR;
typedef short SHORT;
typedef unsigned short USHORT;
typedef long LONG;
typedef unsigned long ULONG;

typedef long long LONGLONG;
typedef unsigned long long ULONGLONG;

typedef float FLOAT;
typedef double DOUBLE;
typedef void *HANDLE;
#ifndef BOOL_DEFINED
#define BOOL_DEFINED
typedef int BOOL;
#endif

typedef unsigned char HEX;
typedef unsigned char uint8;

#ifndef US_IN_SECOND
#define US_IN_SECOND (1000000UL)
#define LL_US_PER_WEEK (1000000ULL * 3600ULL * 24ULL * 7ULL)
#endif
#ifndef MS_OF_WEEK
#define MS_OF_WEEK (604800000)
#endif

#define BIT_VALUE_ONE (1)
#define BIT_VALUE_ZERO (0)

#ifndef OPTIONAL
#define OPTIONAL
#endif

#ifndef FALSE
#define FALSE (0)
#define TRUE (1)
#endif // !FALSE

#define ISNULL(p) (INT32)(((p) == NULL) ? 1 : 0)

#if defined(__GCC__)
#define TP_SDK_PACKED __attribute__((packed))
#else
#define TP_SDK_PACKED
#endif

#if defined(__GCC__)
#define TP_SDK_PACKED8 __attribute__((packed, aligned(8)))
#else

#endif

#ifdef GSP_WINDOWS_BUILD
#define access _access
#endif

/* C11 用 _Thread_local；C23 的 thread_local 在旧标准下非法。优先 GNU __thread 以兼容
 * -std=c99/gnu99。 */
#if defined(__GNUC__) || defined(__clang__) || defined(__INTEL_COMPILER)
#define TP_SDK_THREAD_LOCAL __thread
#elif defined(_MSC_VER)
#define TP_SDK_THREAD_LOCAL __declspec(thread)
#elif defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
#define TP_SDK_THREAD_LOCAL _Thread_local
#else
#define TP_SDK_THREAD_LOCAL
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#if 0
    extern void _assert(const void *exp, const void *file, UINT32 line);

#define ASSERT(exp) (void)(!!(exp) || (_assert(#exp, __FILE__, __LINE__), 0))
#endif

#ifdef __cplusplus
}
#endif

#endif /* GH_GSP_CONSTS_H */
