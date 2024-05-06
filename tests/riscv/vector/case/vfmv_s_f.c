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
            {0x7ff0e92e56fe196a, 0x1c626bcd2715a031, },
            {0x7ff0e92e56fe196a, 0x1c626bcd2715a031, },
        },
        .fixs32 = {
            {0xc5cf5eb9, 0x75bc7c30, 0xf0bec11d, 0x683e0475, },
            {0xc5cf5eb9, 0x75bc7c30, 0xf0bec11d, 0x683e0475, },
        },
        .fixs16 = {
            {0xfe56, 0x4b63, 0xf166, 0x08bc, 0x6f16, 0x0ddb, 0xdeb9, 0xafbe, },
            {0xfe56, 0x4b63, 0xf166, 0x08bc, 0x6f16, 0x0ddb, 0xdeb9, 0xafbe, },
        },
    },
};

uint64_t src[6] = {0x4b63, 0x0, 0x683e0475, 0x0, 0x1c626bcd2715a031, 0x0};

struct rvv_reg dst[] = {
    {
        .fixs64 = {
            {0x1c626bcd2715a031, },
        },
        .fixs32 = {
            {0x683e0475, },
        },
        .fixs16 = {
            {0x4b63,},
        },
    },
};

struct rvv_reg res;

int main(void)
{
    init_testsuite("Testing insn vfmv.s.f\n");

    test_vfmv_s_f_16(src[0], res.fixs16[0], src0[0].fixs16[0]);
    result_compare_s16(res.fixs16[0], dst[0].fixs16[0]);

    test_vfmv_s_f_32(src[2], res.fixs32[0], src0[0].fixs32[0]);
    result_compare_s32(res.fixs32[0], dst[0].fixs32[0]);

    test_vfmv_s_f_64(src[4], res.fixs64[0], src0[0].fixs64[0]);
    result_compare_s64(res.fixs64[0], dst[0].fixs64[0]);

    return done_testing();
}
