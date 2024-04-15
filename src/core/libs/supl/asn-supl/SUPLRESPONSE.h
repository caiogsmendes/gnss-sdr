/*
 * SPDX-FileCopyrightText: (c) 2003, 2004 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * SPDX-License-Identifier: BSD-1-Clause
 * Generated by asn1c-0.9.22 (http://lionet.info/asn1c)
 * From ASN.1 module "SUPL-RESPONSE"
 *     found in "../supl-response.asn"
 */

#ifndef _SUPLRESPONSE_H
#define _SUPLRESPONSE_H

#include <asn_application.h>

/* Including external dependencies */
#include "KeyIdentity4.h"
#include "PosMethod.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* Forward declarations */
    struct SLPAddress;
    struct SETAuthKey;

    /* SUPLRESPONSE */
    typedef struct SUPLRESPONSE
    {
        PosMethod_t posMethod;
        struct SLPAddress *sLPAddress /* OPTIONAL */;
        struct SETAuthKey *sETAuthKey /* OPTIONAL */;
        KeyIdentity4_t *keyIdentity4 /* OPTIONAL */;
        /*
         * This type is extensible,
         * possible extensions are below.
         */

        /* Context for parsing across buffer boundaries */
        asn_struct_ctx_t _asn_ctx;
    } SUPLRESPONSE_t;

    /* Implementation */
    extern asn_TYPE_descriptor_t asn_DEF_SUPLRESPONSE;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "SETAuthKey.h"
#include "SLPAddress.h"

#endif /* _SUPLRESPONSE_H_ */
#include <asn_internal.h>
