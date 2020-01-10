/*
 *  IBM eServer eHCA Infiniband device driver for Linux on POWER
 *
 *  Userspace functions
 *
 *  Authors: Khadija Souissi <souissik@de.ibm.com>
 *           Christoph Raisch <raisch@de.ibm.com>
 *
 *  Copyright (c) 2005 IBM Corporation
 *
 *  All rights reserved.
 *
 *  This source code is distributed under a dual license of GPL v2.0 and OpenIB
 *  BSD.
 *
 * OpenIB BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials
 * provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __EHCA_UTOOL_H__
#define __EHCA_UTOOL_H__

#include <stdio.h>
#include <unistd.h>

#include <linux/types.h>
#include <linux/errno.h>
#define u64 __u64
#define u32 __u32
#define u16 __u16
#define u8 __u8

#define EHCA_PAGESIZE 4096

#define unlikely(x) __builtin_expect(!!(x), 0)

/* defines for EHCA traces */
extern int libehca_trace_on;

extern unsigned long kpage_size;

extern unsigned long kpage_shift;

extern int use_mmap;

/* checks if debug is on for the given level
 * caller's module must have this decl: extern int libehca_trace_on;
 */
#define ehca_dbg(ibv_dev, format, arg...) \
	do { \
		if (unlikely(libehca_trace_on)) \
			fprintf(stderr, "PID%04x %s EHCA_DBG:%s " format "\n", \
				getpid(), (ibv_dev)->name, __func__, ##arg); \
	} while (0)

#define ehca_err(ibv_dev, format, arg...) \
	fprintf(stderr, "PID%04x %s EHCA_ERR:%s " format "\n", \
		getpid(), (ibv_dev)->name, __func__, ##arg)

/* use this one only if no ibv_dev available */
#define ehca_gen_dbg(format, arg...) \
	do { \
		if (unlikely(libehca_trace_on)) \
			fprintf(stderr, "PID%04x EHCA_DBG:%s " format "\n", \
				getpid(), __func__, ##arg); \
	} while (0)

#define ehca_gen_err(format, arg...) \
	fprintf(stderr, "PID%04x EHCA_ERR:%s " format "\n", \
		getpid(), __func__, ##arg)


/**
 * EHCA macro to dump a memory block, whose length is n*8 bytes.
 * Each line has the following layout:
 * <format string> adr=X ofs=Y <8 bytes hex> <8 bytes hex>
 */
#ifndef __PPC64__
#define FORMAT_2U64 "%016llx %016llx"
#else
#define FORMAT_2U64 "%016lx %016lx"
#endif
#define ehca_dmp_dbg(adr, len, format, args...) \
do { \
	if (unlikely(libehca_trace_on)) { \
                unsigned int x; \
		unsigned int l = (unsigned int)(len); \
                unsigned char *deb = (unsigned char*)(adr); \
		for (x = 0; x < l; x += 16) { \
		        ehca_gen_dbg(format " adr=%p ofs=%04x " FORMAT_2U64, \
			     ##args, deb, x, \
			     *((u64 *)&deb[0]), *((u64 *)&deb[8])); \
			deb += 16; \
		} \
	} \
} while (0)

#define ehca_dmp_err(adr, len, format, args...) \
	do { \
                unsigned int x; \
		unsigned int l = (unsigned int)(len); \
                unsigned char *deb = (unsigned char*)(adr); \
		for (x = 0; x < l; x += 16) { \
		        ehca_gen_err(format " adr=%p ofs=%04x " FORMAT_2U64, \
			     ##args, deb, x, \
			     *((u64 *)&deb[0]), *((u64 *)&deb[8])); \
			deb += 16; \
		} \
	} while (0)

/* define a bitmask, the ibm way... */
#define EHCA_BMASK_IBM(from, to) (((63-to)<<16)+((to)-(from)+1))
/* internal function, don't use */
#define EHCA_BMASK_SHIFTPOS(mask) (((mask)>>16)&0xffff)
/* internal function, don't use */
#define EHCA_BMASK_MASK(mask) (0xffffffffffffffffULL >> ((64-(mask))&0xffff))
/** return value shifted and masked by mask
    variable|=HCA_BMASK_SET(MY_MASK,0x4711) ORs the bits in variable
    variable&=~HCA_BMASK_SET(MY_MASK,-1) clears the bits from the mask
    in variable
 */
#define EHCA_BMASK_SET(mask, value) \
        ((EHCA_BMASK_MASK(mask) & ((u64)(value)))<<EHCA_BMASK_SHIFTPOS(mask))
#ifndef offsetof
#define offsetof(TYPE, MEMBER) ((size_t) &((TYPE *)0)->MEMBER)
#endif

#define container_of(ptr, type, member) ({			\
        const typeof(((type *)0)->member) *__mptr = (ptr);	\
        (type *)((char *)__mptr - offsetof(type, member));})


#define be64_to_cpu(x) (x)

#endif /* __EHCA_UTOOL_H__ */
