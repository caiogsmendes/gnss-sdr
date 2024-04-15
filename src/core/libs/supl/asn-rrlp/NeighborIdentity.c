/*
 * SPDX-FileCopyrightText: (c) 2003, 2004 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * SPDX-License-Identifier: BSD-1-Clause
 * Generated by asn1c-0.9.22 (http://lionet.info/asn1c)
 * From ASN.1 module "RRLP-Components"
 *     found in "../rrlp-components.asn"
 */

#include "NeighborIdentity.h"

static asn_per_constraints_t ASN_PER_TYPE_NEIGHBOR_IDENTITY_CONSTR_1 = {
    {APC_CONSTRAINED, 3, 3, 0, 5} /* (0..5) */,
    {APC_UNCONSTRAINED, -1, -1, 0, 0},
    0,
    0 /* No PER value map */
};
static asn_TYPE_member_t asn_MBR_NeighborIdentity_1[] = {
    {ATF_NOFLAGS, 0, offsetof(struct NeighborIdentity, choice.bsicAndCarrier),
        (ASN_TAG_CLASS_CONTEXT | (0 << 2)), -1, /* IMPLICIT tag at current level */
        &asn_DEF_BSICAndCarrier,
        0, /* Defer constraints checking to the member type */
        0, /* No PER visible constraints */
        0, "bsicAndCarrier"},
    {ATF_NOFLAGS, 0, offsetof(struct NeighborIdentity, choice.ci),
        (ASN_TAG_CLASS_CONTEXT | (1 << 2)), -1, /* IMPLICIT tag at current level */
        &asn_DEF_CellID, 0,                     /* Defer constraints checking to the member type */
        0,                                      /* No PER visible constraints */
        0, "ci"},
    {ATF_NOFLAGS, 0,
        offsetof(struct NeighborIdentity, choice.multiFrameCarrier),
        (ASN_TAG_CLASS_CONTEXT | (2 << 2)), -1, /* IMPLICIT tag at current level */
        &asn_DEF_MultiFrameCarrier,
        0, /* Defer constraints checking to the member type */
        0, /* No PER visible constraints */
        0, "multiFrameCarrier"},
    {ATF_NOFLAGS, 0, offsetof(struct NeighborIdentity, choice.requestIndex),
        (ASN_TAG_CLASS_CONTEXT | (3 << 2)), -1, /* IMPLICIT tag at current level */
        &asn_DEF_RequestIndex,
        0, /* Defer constraints checking to the member type */
        0, /* No PER visible constraints */
        0, "requestIndex"},
    {ATF_NOFLAGS, 0, offsetof(struct NeighborIdentity, choice.systemInfoIndex),
        (ASN_TAG_CLASS_CONTEXT | (4 << 2)), -1, /* IMPLICIT tag at current level */
        &asn_DEF_SystemInfoIndex,
        0, /* Defer constraints checking to the member type */
        0, /* No PER visible constraints */
        0, "systemInfoIndex"},
    {ATF_NOFLAGS, 0, offsetof(struct NeighborIdentity, choice.ciAndLAC),
        (ASN_TAG_CLASS_CONTEXT | (5 << 2)), -1, /* IMPLICIT tag at current level */
        &asn_DEF_CellIDAndLAC,
        0, /* Defer constraints checking to the member type */
        0, /* No PER visible constraints */
        0, "ciAndLAC"},
};
static asn_TYPE_tag2member_t asn_MAP_NeighborIdentity_tag2el_1[] = {
    {(ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0}, /* bsicAndCarrier at 398 */
    {(ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0}, /* ci at 399 */
    {(ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0,
        0},                                        /* multiFrameCarrier at 400 */
    {(ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0}, /* requestIndex at 401 */
    {(ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0}, /* systemInfoIndex at 402 */
    {(ASN_TAG_CLASS_CONTEXT | (5 << 2)), 5, 0, 0}  /* ciAndLAC at 407 */
};
static asn_CHOICE_specifics_t asn_SPC_NeighborIdentity_specs_1 = {
    sizeof(struct NeighborIdentity),
    offsetof(struct NeighborIdentity, _asn_ctx),
    offsetof(struct NeighborIdentity, present),
    sizeof(((struct NeighborIdentity *)0)->present),
    asn_MAP_NeighborIdentity_tag2el_1,
    6, /* Count of tags in the map */
    0,
    -1 /* Extensions start */
};
asn_TYPE_descriptor_t asn_DEF_NeighborIdentity = {
    "NeighborIdentity",
    "NeighborIdentity",
    CHOICE_free,
    CHOICE_print,
    CHOICE_constraint,
    CHOICE_decode_ber,
    CHOICE_encode_der,
    CHOICE_decode_xer,
    CHOICE_encode_xer,
    CHOICE_decode_uper,
    CHOICE_encode_uper,
    CHOICE_outmost_tag,
    0, /* No effective tags (pointer) */
    0, /* No effective tags (count) */
    0, /* No tags (pointer) */
    0, /* No tags (count) */
    &ASN_PER_TYPE_NEIGHBOR_IDENTITY_CONSTR_1,
    asn_MBR_NeighborIdentity_1,
    6,                                /* Elements count */
    &asn_SPC_NeighborIdentity_specs_1 /* Additional specs */
};
