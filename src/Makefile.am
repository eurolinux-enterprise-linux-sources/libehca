#  IBM eServer eHCA Infiniband device driver for Linux on POWER
#   
#  makefile for openib stack
#
#  Authors: Heiko J Schick <schickhj@de.ibm.com>
#           Christoph Raisch <raisch@de.ibm.com> 
#
#  Copyright (c) 2005 IBM Corporation
#    
#  All rights reserved.
# 
#  This source code is distributed under a dual license of GPL v2.0 and OpenIB 
#  BSD. 
#
#  OpenIB BSD License
#
#  Redistribution and use in source and binary forms, with or without 
#  modification, are permitted provided that the following conditions are met: 
#
#  Redistributions of source code must retain the above copyright notice, this 
#  list of conditions and the following disclaimer. 
#
#  Redistributions in binary form must reproduce the above copyright notice, 
#  this list of conditions and the following disclaimer in the documentation 
#  and/or other materials
#  provided with the distribution. 
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
#  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
#  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
#  POSSIBILITY OF SUCH DAMAGE.
    
AM_CFLAGS = -O2 -g -Wall -D_GNU_SOURCE -DP_SERIES -Isrc

EHCA_SOURCES = src/ehca_umain.c src/ehca_u_mrmw.c src/ehca_uinit.c \
	src/ehca_ureqs.c src/hcp_phyp.c src/ehca_uabiver5.c src/ehca_uabiver7.c \
	src/ehca_uabiver6.c

if HAVE_IBV_DEVICE_LIBRARY_EXTENSION
    lib_LTLIBRARIES = src/libehca.la
    src_libehca_la_SOURCES = $(EHCA_SOURCES)
    src_libehca_la_LDFLAGS = -avoid-version -release @IBV_DEVICE_LIBRARY_EXTENSION@ \
				-Wl,--version-script=$(srcdir)/src/libehca.map \
				-lpthread -libverbs -nostdlib
    ehcaconfdir = $(sysconfdir)/libibverbs.d
    ehcaconf_DATA = ehca.driver

else
    ehcalibdir = $(libdir)/infiniband
    ehcalib_LTLIBRARIES = src/libehca.la
    src_libehca_la_SOURCES = $(EHCA_SOURCES)
    src_libehca_la_LDFLAGS = -version-info 1 -export-dynamic \
				-Wl,--version-script=$(srcdir)/src/libehca.map \
				-lpthread -libverbs -nostdlib
endif

EXTRA_DIST = src/ehca_asm.h \
	src/ehca_galpa.h \
	src/ehca_uclasses.h \
	src/hipz_fns_core.h \
	src/ipzu_pt_fn.h \
	src/ehca_everbs.h \
	src/ehca_qes.h \
	src/ehca_utools.h \
	src/hipz_hw.h \
	src/libehca.map \
	ehca.driver

# dist-hook: libehca.spec
#     cp libehca.spec $(distdir)
