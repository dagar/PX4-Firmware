/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_defines.h
 *
 * Generally used magic defines
 */

#pragma once

/****************************************************************************
 * Defines for all platforms.
 ****************************************************************************/

#define PX4_ERROR (-1)
#define PX4_OK 0


#if defined(__PX4_NUTTX)
/****************************************************************************
 * NuttX specific defines.
 ****************************************************************************/
#define PX4_ROOTFSDIR ""
#define _PX4_IOC(x,y) _IOC(x,y)

// mode for open with O_CREAT
#define PX4_O_MODE_777 0777
#define PX4_O_MODE_666 0666
#define PX4_O_MODE_600 0600

#else

#define PX4_ROOTFSDIR ""

/****************************************************************************
 * POSIX Specific defines
 ****************************************************************************/

// All POSIX except QURT.

__BEGIN_DECLS
extern long PX4_TICKS_PER_SEC;
__END_DECLS

// Flag is meaningless on Linux
#define O_BINARY 0

// mode for open with O_CREAT
#define PX4_O_MODE_777 (S_IRWXU | S_IRWXG | S_IRWXO)
#define PX4_O_MODE_666 (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH )
#define PX4_O_MODE_600 (S_IRUSR | S_IWUSR)

// NuttX _IOC is equivalent to Linux _IO
#define _PX4_IOC(x,y) _IO(x,y)

/* FIXME - Used to satisfy build */
#define getreg32(a)    (*(volatile uint32_t *)(a))

#define USEC_PER_TICK (1000000UL/PX4_TICKS_PER_SEC)
#define USEC2TICK(x) (((x)+(USEC_PER_TICK/2))/USEC_PER_TICK)

#endif // __PX4_NUTTX


#ifdef __PX4_QURT
/****************************************************************************
 * QURT Specific defines
 ****************************************************************************/

#  include "dspal_math.h"
#  define PX4_ROOTFSDIR ""
#  define PX4_TICKS_PER_SEC 1000L
#  define SIOCDEVPRIVATE 999999

#  if defined(__PX4_POSIX_EAGLE) || defined(__PX4_POSIX_EXCELSIOR)
#    define PX4_ROOTFSDIR "/home/linaro"
#  elif defined(__PX4_POSIX_BEBOP)
#    define PX4_ROOTFSDIR "/data/ftp/internal_000"
#  else
#    define PX4_ROOTFSDIR "rootfs"
#  endif

#endif // __PX4_QURT
