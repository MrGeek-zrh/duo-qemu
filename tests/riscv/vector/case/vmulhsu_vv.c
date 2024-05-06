/*
 * Copyright (c) 2021 T-Head Semiconductor Co., Ltd. All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License  *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "testsuite.h"
#include "rvv_insn.h"

struct rvv_reg src0[] = {
    {
        .fixs64 = {
            {0x3970b5993ab1f212, 0xc6a630b347e7377b, },
            {0x3970b5993ab1f212, 0xc6a630b347e7377b, },
        },
        .fixs32 = {
            {0xcfe06686, 0x8262f661, 0x15fc5221, 0xd6b9745a, },
            {0xcfe06686, 0x8262f661, 0x15fc5221, 0xd6b9745a, },
        },
        .fixs16 = {
            {0xd6d6, 0x51f2, 0x10ef, 0x0ea1, 0xa349, 0x4d3f, 0x475d, 0xa164, },
            {0xd6d6, 0x51f2, 0x10ef, 0x0ea1, 0xa349, 0x4d3f, 0x475d, 0xa164, },
        },
        .fixs8 = {
            {0xa5, 0x34, 0x8c, 0x74, 0xcd, 0x75, 0x92, 0x7a, 0x60, 0x19, 0x3c, 0x91, 0xfd, 0xab, 0x23, 0x21, },
            {0xa5, 0x34, 0x8c, 0x74, 0xcd, 0x75, 0x92, 0x7a, 0x60, 0x19, 0x3c, 0x91, 0xfd, 0xab, 0x23, 0x21, },
        },
    },
};

struct rvv_reg src1[] = {
    {
        .fixu64 = {
            {0x07d215928aa0d7b6, 0x07186161e5f9e80f, },
            {0x07d215928aa0d7b6, 0x07186161e5f9e80f, },
        },
        .fixu32 = {
            {0x7794d541, 0xf1bfac15, 0x67e3b37f, 0x12df3e7c, },
            {0x7794d541, 0xf1bfac15, 0x67e3b37f, 0x12df3e7c, },
        },
        .fixu16 = {
            {0x4fb8, 0x39b3, 0x6744, 0xfa98, 0xce81, 0x997d, 0x301c, 0xcfb5, },
            {0x4fb8, 0x39b3, 0x6744, 0xfa98, 0xce81, 0x997d, 0x301c, 0xcfb5, },
        },
        .fixu8 = {
            {0x21, 0xb6, 0x8d, 0x3a, 0xe2, 0x09, 0x90, 0x15, 0x2c, 0x13, 0xac, 0x86, 0x28, 0xce, 0x4f, 0xbb, },
            {0x21, 0xb6, 0x8d, 0x3a, 0xe2, 0x09, 0x90, 0x15, 0x2c, 0x13, 0xac, 0x86, 0x28, 0xce, 0x4f, 0xbb, },
        },
    },
};
struct rvv_reg dst_vl[] = {
    {
        .fixs8 = {
            {0xf4, 0x24, 0xc0, 0x1a, 0xd2, 0x04, 0xc2, 0x0a, 0x10, 0x01, 0x28, 0xc5, 0xff, 0xbb, 0x0a, 0x18, },
            {0xf4, 0x24, 0xc0, 0x1a, 0xd2, 0x04, 0xc2, 0x0a, 0x10, 0x01, 0x28, 0xc5, 0xff, 0xbb, 0x0a, 0x00, },
        },
        .fixs16 = {
            {0xf32e, 0x1278, 0x06d4, 0x0e51, 0xb536, 0x2e50, 0x0d69, 0xb33c, },
            {0xf32e, 0x1278, 0x06d4, 0x0e51, 0xb536, 0x2e50, 0x0d69, 0x0000, },
        },
        .fixs32 = {
            {0xe985554d, 0x89611d57, 0x08ec1332, 0xfcf50ba7, },
            {0xe985554d, 0x89611d57, 0x08ec1332, 0x00000000, },
        },
        .fixs64 = {
            {0x01c138434894ff87, 0xfe69151881ad1f97, },
            {0x01c138434894ff87, 0x0000000000000000, },
        },
    },
};

struct rvv_reg dst_even[] = {
    {
        .fixs8 = {
            {0xf4, 0x11, 0xc0, 0x11, 0xd2, 0x11, 0xc2, 0x11, 0x10, 0x11, 0x28, 0x11, 0xff, 0x11, 0x0a, 0x11, },
            {0xf4, 0x11, 0xc0, 0x11, 0xd2, 0x11, 0xc2, 0x11, 0x10, 0x11, 0x28, 0x11, 0xff, 0x11, 0x0a, 0x11, },
        },
        .fixs16 = {
            {0xf32e, 0x1111, 0x06d4, 0x1111, 0xb536, 0x1111, 0x0d69, 0x1111, },
            {0xf32e, 0x1111, 0x06d4, 0x1111, 0xb536, 0x1111, 0x0d69, 0x1111, },
        },
        .fixs32 = {
            {0xe985554d, 0x11111111, 0x08ec1332, 0x11111111, },
            {0xe985554d, 0x11111111, 0x08ec1332, 0x11111111, },
        },
        .fixs64 = {
            {0x01c138434894ff87, 0x1111111111111111, },
            {0x01c138434894ff87, 0x1111111111111111, },
        },
    },
};

struct rvv_reg dst_odd[] = {
    {
        .fixs8 = {
            {0x11, 0x24, 0x11, 0x1a, 0x11, 0x04, 0x11, 0x0a, 0x11, 0x01, 0x11, 0xc5, 0x11, 0xbb, 0x11, 0x18, },
            {0x11, 0x24, 0x11, 0x1a, 0x11, 0x04, 0x11, 0x0a, 0x11, 0x01, 0x11, 0xc5, 0x11, 0xbb, 0x11, 0x18, },
        },
        .fixs16 = {
            {0x1111, 0x1278, 0x1111, 0x0e51, 0x1111, 0x2e50, 0x1111, 0xb33c, },
            {0x1111, 0x1278, 0x1111, 0x0e51, 0x1111, 0x2e50, 0x1111, 0xb33c, },
        },
        .fixs32 = {
            {0x11111111, 0x89611d57, 0x11111111, 0xfcf50ba7, },
            {0x11111111, 0x89611d57, 0x11111111, 0xfcf50ba7, },
        },
        .fixs64 = {
            {0x1111111111111111, 0xfe69151881ad1f97, },
            {0x1111111111111111, 0xfe69151881ad1f97, },
        },
    },
};

struct rvv_reg res;

int main(void)
{
    int i = 0;
    init_testsuite("Testing insn vmulhsu.vv\n");

    /* uint8_t vmulhsu */
    test_vmulhsu_vv_8(vlmax_8 - 1, src0[i].fixs8[0], src1[i].fixu8[0], res.fixs8[0], vma.fixs8, pad.fixs8[0]);
    result_compare_s8_lmul(res.fixs8[0], dst_vl[i].fixs8[0]);

    test_vmulhsu_vv_8_vm(src0[i].fixs8[0], src1[i].fixu8[0], res.fixs8[0], vme.fixs8, pad.fixs8[0]);
    result_compare_s8_lmul(res.fixs8[0], dst_even[i].fixs8[0]);

    test_vmulhsu_vv_8_vm(src0[i].fixs8[0], src1[i].fixu8[0], res.fixs8[0], vmo.fixs8, pad.fixs8[0]);
    result_compare_s8_lmul(res.fixs8[0], dst_odd[i].fixs8[0]);

    /* uint16_t vmulhsu */
    test_vmulhsu_vv_16(vlmax_16 - 1, src0[i].fixs16[0], src1[i].fixu16[0], res.fixs16[0], vma.fixs16, pad.fixs16[0]);
    result_compare_s16_lmul(res.fixs16[0], dst_vl[i].fixs16[0]);

    test_vmulhsu_vv_16_vm(src0[i].fixs16[0], src1[i].fixu16[0], res.fixs16[0], vme.fixs16, pad.fixs16[0]);
    result_compare_s16_lmul(res.fixs16[0], dst_even[i].fixs16[0]);

    test_vmulhsu_vv_16_vm(src0[i].fixs16[0], src1[i].fixu16[0], res.fixs16[0], vmo.fixs16, pad.fixs16[0]);
    result_compare_s16_lmul(res.fixs16[0], dst_odd[i].fixs16[0]);

    /* uint32_t vmulhsu */
    test_vmulhsu_vv_32(vlmax_32 - 1, src0[i].fixs32[0], src1[i].fixu32[0], res.fixs32[0], vma.fixs32, pad.fixs32[0]);
    result_compare_s32_lmul(res.fixs32[0], dst_vl[i].fixs32[0]);

    test_vmulhsu_vv_32_vm(src0[i].fixs32[0], src1[i].fixu32[0], res.fixs32[0], vme.fixs32, pad.fixs32[0]);
    result_compare_s32_lmul(res.fixs32[0], dst_even[i].fixs32[0]);

    test_vmulhsu_vv_32_vm(src0[i].fixs32[0], src1[i].fixu32[0], res.fixs32[0], vmo.fixs32, pad.fixs32[0]);
    result_compare_s32_lmul(res.fixs32[0], dst_odd[i].fixs32[0]);

    /* uint64_t vmulhsu */
    test_vmulhsu_vv_64(vlmax_64 - 1, src0[i].fixs64[0], src1[i].fixu64[0], res.fixs64[0], vma.fixs64, pad.fixs64[0]);
    result_compare_s64_lmul(res.fixs64[0], dst_vl[i].fixs64[0]);

    test_vmulhsu_vv_64_vm(src0[i].fixs64[0], src1[i].fixu64[0], res.fixs64[0], vme.fixs64, pad.fixs64[0]);
    result_compare_s64_lmul(res.fixs64[0], dst_even[i].fixs64[0]);

    test_vmulhsu_vv_64_vm(src0[i].fixs64[0], src1[i].fixu64[0], res.fixs64[0], vmo.fixs64, pad.fixs64[0]);
    result_compare_s64_lmul(res.fixs64[0], dst_odd[i].fixs64[0]);

    return done_testing();
}