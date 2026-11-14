/*
 * Types.h -- Basic types, trimmed from the LZMA SDK 4.65 original
 * 2008-11-23 : Igor Pavlov : Public domain
 */

#ifndef LZMA_TYPES_H
#define LZMA_TYPES_H

#define SZ_OK 0

#define SZ_ERROR_DATA 1
#define SZ_ERROR_MEM 2
#define SZ_ERROR_UNSUPPORTED 4
#define SZ_ERROR_INPUT_EOF 6

typedef int SRes;
typedef int Bool;
#define True 1
#define False 0
#define MY_FAST_CALL

#ifndef RINOK
#define RINOK(x) { int __result__ = (x); if (__result__ != 0) return __result__; }
#endif

typedef unsigned char Byte;
typedef unsigned short UInt16;
typedef int Int32;
typedef unsigned int UInt32;
typedef size_t SizeT;

typedef struct
{
  void *(*Alloc)(void *p, size_t size);
  void (*Free)(void *p, void *address); /* address can be 0 */
} ISzAlloc;

#define IAlloc_Alloc(p, size) (p)->Alloc((p), size)
#define IAlloc_Free(p, a) (p)->Free((p), a)

#endif
