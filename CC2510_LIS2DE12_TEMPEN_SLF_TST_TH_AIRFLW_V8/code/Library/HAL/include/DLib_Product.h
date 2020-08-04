/* ICC8051 DLib configuration.                                */
/* Copyright (C) 2003-2011 IAR Systems.  All rights reserved. */

#ifndef _DLIB_PRODUCT_H
#define _DLIB_PRODUCT_H

#ifdef __cplusplus
  #if ( __DATA_MODEL__ <= __DM_SMALL__)
    #error "Data models tiny and small does not support C++"
  #endif
#endif

#ifndef _DLIB_NO_DEFAULT_HEAP
  #if ( __DATA_MODEL__ <= __DM_SMALL__)
    #define _DLIB_NO_DEFAULT_HEAP 1
    #define _DO_NOT_INLINE_MALLOC 1
  #endif
  #if ( __DATA_MODEL__ == __DM_GENERIC__)
    #define _DO_NOT_INLINE_MALLOC 1
  #endif
  #if ( __DATA_MODEL__ == __DM_FAR_GENERIC__)
    #define _DO_NOT_INLINE_MALLOC 1
  #endif
#endif

#ifndef _DLIB_QSORT_BUF_SIZE
  #if ( __CALLING_CONVENTION__ <= __CC_PR__) /* overlay and idata/pdata reentrant */
    #define _DLIB_QSORT_BUF_SIZE 10
  #else
    #define _DLIB_QSORT_BUF_SIZE 64
  #endif
#endif

/* 8051 is to be considered as a small target */
#ifndef _DLIB_SMALL_TARGET
 #define _DLIB_SMALL_TARGET 1
#endif

#endif /* _DLIB_PRODUCT_H */
