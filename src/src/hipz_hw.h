/*
 *  IBM eServer eHCA Infiniband device driver for Linux on POWER
 *
 *  eHCA register definitions
 *
 *  Authors: Christoph Raisch <raisch@de.ibm.com>
 *           Reinhard Ernst <rernst@de.ibm.com>
 *           Waleri Fomin <fomin@de.ibm.com>
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
 */

#ifndef __HIPZ_HW_H__
#define __HIPZ_HW_H__

#ifdef __KERNEL__
#include "ehca_tools.h"
#include "ehca_kernel.h"
#else				/*  !__KERNEL__ */
#include "ehca_utools.h"
#endif

/* Queue Pair Table Memory */
struct hipz_QPTEMM {
	u64 QPx_HCR;
#define QPx_HCR_PKEY_Mode EHCA_BMASK_IBM(1, 2)
#define QPx_HCR_Special_QP_Mode EHCA_BMASK_IBM(6, 7)
	u64 QPx_C;
#define QPx_C_Enabled EHCA_BMASK_IBM(0, 0)
#define QPx_C_Disabled EHCA_BMASK_IBM(1, 1)
#define QPx_C_Req_State EHCA_BMASK_IBM(16, 23)
#define QPx_C_Res_State EHCA_BMASK_IBM(25, 31)
#define QPx_C_disable_ETE_check EHCA_BMASK_IBM(7, 7)
	u64 QPx_HERR;
	u64 QPx_AER;
	/* 0x20*/
	u64 QPx_SQA;
#define QPx_SQAdder EHCA_BMASK_IBM(48, 63)
	u64 QPx_SQC;
	u64 QPx_RQA;
#define QPx_RQAdder EHCA_BMASK_IBM(48, 63)
	u64 QPx_RQC;
	/* 0x40*/
	u64 QPx_ST;
	u64 QPx_PMSTATE;
#define  QPx_PMSTATE_BITS  EHCA_BMASK_IBM(30, 31)
	u64 QPx_PMFA;
	u64 QPx_PKEY;
#define QPx_PKEY_value EHCA_BMASK_IBM(48, 63)
	/* 0x60*/
	u64 QPx_PKEYA;
#define QPx_PKEYA_index0 EHCA_BMASK_IBM(0, 15)
#define QPx_PKEYA_index1 EHCA_BMASK_IBM(16, 31)
#define QPx_PKEYA_index2 EHCA_BMASK_IBM(32, 47)
#define QPx_PKEYA_index3 EHCA_BMASK_IBM(48, 63)
	u64 QPx_PKEYB;
#define QPx_PKEYB_index4 EHCA_BMASK_IBM(0, 15)
#define QPx_PKEYB_index5 EHCA_BMASK_IBM(16, 31)
#define QPx_PKEYB_index6 EHCA_BMASK_IBM(32, 47)
#define QPx_PKEYB_index7 EHCA_BMASK_IBM(48, 63)
	u64 QPx_PKEYC;
#define QPx_PKEYC_index8 EHCA_BMASK_IBM(0, 15)
#define QPx_PKEYC_index9 EHCA_BMASK_IBM(16, 31)
#define QPx_PKEYC_index10 EHCA_BMASK_IBM(32, 47)
#define QPx_PKEYC_index11 EHCA_BMASK_IBM(48, 63)
	u64 QPx_PKEYD;
#define QPx_PKEYD_index12 EHCA_BMASK_IBM(0, 15)
#define QPx_PKEYD_index13 EHCA_BMASK_IBM(16, 31)
#define QPx_PKEYD_index14 EHCA_BMASK_IBM(32, 47)
#define QPx_PKEYD_index15 EHCA_BMASK_IBM(48, 63)
	/* 0x80*/
	u64 QPx_QKEY;
#define QPx_QKEY_value EHCA_BMASK_IBM(32, 63)
	u64 QPx_DQP;
#define QPx_DQP_number EHCA_BMASK_IBM(40, 63)
	u64 QPx_DLIDP;
#define QPx_DLID_PRIMARY EHCA_BMASK_IBM(48, 63)
#define QPx_DLIDP_GRH    EHCA_BMASK_IBM(31, 31)
	u64 QPx_PORTP;
#define QPx_PORT_Primary EHCA_BMASK_IBM(57, 63)
	/* 0xa0*/
	u64 QPx_SLIDP;
#define QPx_SLIDP_p_path EHCA_BMASK_IBM(48, 63)
#define QPx_SLIDP_lmc    EHCA_BMASK_IBM(37, 39)
	u64 QPx_SLIDPP;
#define QPx_SLID_PRIM_PATH EHCA_BMASK_IBM(57, 63)
	u64 QPx_DLIDA;
#define QPx_DLIDA_GRH    EHCA_BMASK_IBM(31, 31)
	u64 QPx_PORTA;
#define QPx_PORT_Alternate EHCA_BMASK_IBM(57, 63)
	/* 0xc0*/
	u64 QPx_SLIDA;
	u64 QPx_SLIDPA;
	u64 QPx_SLVL;
#define QPx_SLVL_BITS  EHCA_BMASK_IBM(56, 59)
#define QPx_SLVL_VL    EHCA_BMASK_IBM(60, 63)
	u64 QPx_IPD;
#define QPx_IPD_max_static_rate EHCA_BMASK_IBM(56, 63)
	/* 0xe0*/
	u64 QPx_MTU;
#define QPx_MTU_size EHCA_BMASK_IBM(56, 63)
	u64 QPx_LATO;
#define QPx_LATO_BITS EHCA_BMASK_IBM(59, 63)
	u64 QPx_RLIMIT;
#define QPx_RETRY_COUNT EHCA_BMASK_IBM(61, 63)
	u64 QPx_RNRLIMIT;
#define QPx_RNR_RETRY_COUNT EHCA_BMASK_IBM(61, 63)
	/* 0x100*/
	u64 QPx_T;
	u64 QPx_SQHP;
	u64 QPx_SQPTP;
	u64 QPx_NSPSN;
#define QPx_NSPSN_value EHCA_BMASK_IBM(40, 63)
	/* 0x120*/
	u64 QPx_NSPSNHWM;
#define QPx_NSPSNHWM_value EHCA_BMASK_IBM(40, 63)
	u64 reserved1;
	u64 QPx_SDSI;
	u64 QPx_SDSBC;
	/* 0x140*/
	u64 QPx_SQWSIZE;
#define QPx_SQWSIZE_value EHCA_BMASK_IBM(61, 63)
	u64 QPx_SQWTS;
	u64 QPx_LSN;
	u64 QPx_NSSN;
	/* 0x160 */
	u64 QPx_MOR;
#define QPx_MOR_value EHCA_BMASK_IBM(48, 63)
	u64 QPx_COR;
	u64 QPx_SQSIZE;
#define QPx_SQSIZE_value EHCA_BMASK_IBM(60, 63)
	u64 QPx_ERC;
	/* 0x180*/
	u64 QPx_RNRRC;
#define QPx_RNRRESP_value EHCA_BMASK_IBM(59, 63)
	u64 QPx_ERNRWT;
	u64 QPx_RNRRESP;
#define QPx_RNRRESP_WTR EHCA_BMASK_IBM(59, 63)
	u64 QPx_LMSNA;
	/* 0x1a0 */
	u64 QPx_SQHPC;
	u64 QPx_SQCPTP;
	u64 QPx_SIGT;
	u64 QPx_WQECNT;
	/* 0x1c0*/
	u64 QPx_RQHP;
	u64 QPx_RQPTP;
	u64 QPx_RQSIZE;
#define QPx_RQSIZE_value EHCA_BMASK_IBM(60, 63)
	u64 QPx_NRR;
#define QPx_NRR_value EHCA_BMASK_IBM(61, 63)
	/* 0x1e0*/
	u64 QPx_RDMAC;
#define QPx_RDMAC_value EHCA_BMASK_IBM(61, 63)
	u64 QPx_NRPSN;
#define QPx_NRPSN_value EHCA_BMASK_IBM(40, 63)
	u64 QPx_LAPSN;
#define QPx_LAPSN_value EHCA_BMASK_IBM(40, 63)
	u64 QPx_LCR;
	/* 0x200*/
	u64 QPx_RWC;
	u64 QPx_RWVA;
	u64 QPx_RDSI;
	u64 QPx_RDSBC;
	/* 0x220*/
	u64 QPx_RQWSIZE;
#define QPx_RQWSIZE_value EHCA_BMASK_IBM(61, 63)
	u64 QPx_CRMSN;
	u64 QPx_RDD;
#define QPx_RDD_VALUE  EHCA_BMASK_IBM(32, 63)
	u64 QPx_LARPSN;
#define QPx_LARPSN_value EHCA_BMASK_IBM(40, 63)
	/* 0x240*/
	u64 QPx_PD;
	u64 QPx_SCQN;
	u64 QPx_RCQN;
	u64 QPx_AEQN;
	/* 0x260*/
	u64 QPx_AAELOG;
	u64 QPx_RAM;
	u64 QPx_RDMAQE0;
	u64 QPx_RDMAQE1;
	/* 0x280*/
	u64 QPx_RDMAQE2;
	u64 QPx_RDMAQE3;
	u64 QPx_NRPSNHWM;
#define QPx_NRPSNHWM_value EHCA_BMASK_IBM(40, 63)
	/* 0x298*/
	u64 reserved[(0x400 - 0x298) / 8];
	/* 0x400 extended data */
	u64 reserved_ext[(0x500 - 0x400) / 8];
	/* 0x500 */
	u64 reserved2[(0x1000 - 0x500) / 8];
	/* 0x1000      */
};

#define QPTEMM_OFFSET(x) offsetof(struct hipz_QPTEMM, x)

/* MRMWPT Entry Memory Map */
struct hipz_MRMWMM {
	/* 0x00 */
	u64 MRx_HCR;
#define MRx_HCR_LPARID_VALID EHCA_BMASK_IBM(0, 0)
	u64 MRx_C;
	u64 MRx_HERR;
	u64 MRx_AER;
	/* 0x20 */
	u64 MRx_PP;
	u64 reserved1;
	u64 reserved2;
	u64 reserved3;
	/* 0x40 */
	u64 reserved4[(0x200 - 0x40) / 8];
	/* 0x200 */
	u64 MRx_CTL[64];
};

#define MRMWMM_OFFSET(x) offsetof(struct hipz_MRMWMM, x)

/* QPEDMM */
struct hipz_QPEDMM {
	/* 0x00 */
	u64 reserved0[(0x400) / 8];
	/* 0x400 */
	u64 QPEDx_PHH;
#define QPEDx_PHH_TClass EHCA_BMASK_IBM(4, 11)
#define QPEDx_PHH_HopLimit EHCA_BMASK_IBM(56, 63)
#define QPEDx_PHH_FlowLevel EHCA_BMASK_IBM(12, 31)
	u64 QPEDx_PPSGP;
#define QPEDx_PPSGP_PPPidx EHCA_BMASK_IBM(0, 63)
	/* 0x410 */
	u64 QPEDx_PPSGU;
#define QPEDx_PPSGU_PPPSGID EHCA_BMASK_IBM(0, 63)
	u64 QPEDx_PPDGP;
	/* 0x420 */
	u64 QPEDx_PPDGU;
	u64 QPEDx_APH;
	/* 0x430 */
	u64 QPEDx_APSGP;
	u64 QPEDx_APSGU;
	/* 0x440 */
	u64 QPEDx_APDGP;
	u64 QPEDx_APDGU;
	/* 0x450 */
	u64 QPEDx_APAV;
	u64 QPEDx_APSAV;
	/* 0x460  */
	u64 QPEDx_HCR;
	u64 reserved1[4];
	/* 0x488 */
	u64 QPEDx_RRL0;
	/* 0x490 */
	u64 QPEDx_RRRKEY0;
	u64 QPEDx_RRVA0;
	/* 0x4A0 */
	u64 reserved2;
	u64 QPEDx_RRL1;
	/* 0x4B0 */
	u64 QPEDx_RRRKEY1;
	u64 QPEDx_RRVA1;
	/* 0x4C0 */
	u64 reserved3;
	u64 QPEDx_RRL2;
	/* 0x4D0 */
	u64 QPEDx_RRRKEY2;
	u64 QPEDx_RRVA2;
	/* 0x4E0 */
	u64 reserved4;
	u64 QPEDx_RRL3;
	/* 0x4F0 */
	u64 QPEDx_RRRKEY3;
	u64 QPEDx_RRVA3;
};

#define QPEDMM_OFFSET(x) offsetof(struct hipz_QPEDMM, x)

/* CQ Table Entry Memory Map */
struct hipz_CQTEMM {
	u64 CQx_HCR;
#define CQx_HCR_LPARID_valid EHCA_BMASK_IBM(0, 0)
	u64 CQx_C;
#define CQx_C_Enable EHCA_BMASK_IBM(0, 0)
#define CQx_C_Disable_Complete EHCA_BMASK_IBM(1, 1)
#define CQx_C_Error_Reset EHCA_BMASK_IBM(23, 23)
	u64 CQx_HERR;
	u64 CQx_AER;
	/* 0x20  */
	u64 CQx_PTP;
	u64 CQx_TP;
#define CQx_FEC_CQE_cnt EHCA_BMASK_IBM(32, 63)
	u64 CQx_FEC;
	u64 CQx_FECA;
#define CQx_FECAdder EHCA_BMASK_IBM(32, 63)
	/* 0x40  */
	u64 CQx_EP;
#define CQx_EP_Event_Pending EHCA_BMASK_IBM(0, 0)
#define CQx_EQ_number EHCA_BMASK_IBM(0, 15)
#define CQx_EQ_CQtoken EHCA_BMASK_IBM(32, 63)
	u64 CQx_EQ;
	/* 0x50  */
	u64 reserved1;
	u64 CQx_N0;
#define CQx_N0_generate_solicited_comp_event EHCA_BMASK_IBM(0, 0)
	/* 0x60  */
	u64 CQx_N1;
#define CQx_N1_generate_comp_event EHCA_BMASK_IBM(0, 0)
	u64 reserved2[(0x1000 - 0x60) / 8];
	/* 0x1000 */
};

#define CQTEMM_OFFSET(x) offsetof(struct hipz_CQTEMM, x)

/* EQ Table Entry Memory Map */
struct hipz_EQTEMM {
	u64 EQx_HCR;
#define EQx_HCR_LPARID_valid EHCA_BMASK_IBM(0, 0)
#define EQx_HCR_ENABLE_PSB EHCA_BMASK_IBM(8, 8)
	u64 EQx_C;
#define EQx_C_Enable EHCA_BMASK_IBM(0, 0)
#define EQx_C_Error_Reset EHCA_BMASK_IBM(23, 23)
#define EQx_C_Comp_Event EHCA_BMASK_IBM(17, 17)

	u64 EQx_HERR;
	u64 EQx_AER;
	/* 0x20 */
	u64 EQx_PTP;
	u64 EQx_TP;
	u64 EQx_SSBA;
	u64 EQx_PSBA;

	/* 0x40 */
	u64 EQx_CEC;
	u64 EQx_MEQL;
	u64 EQx_XISBI;
	u64 EQx_XISC;
	/* 0x60 */
	u64 EQx_IT;

};
#define EQTEMM_OFFSET(x) offsetof(struct hipz_EQTEMM, x)

#endif
