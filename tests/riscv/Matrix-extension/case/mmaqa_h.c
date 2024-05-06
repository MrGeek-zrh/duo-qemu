/*
 * Copyright (c) 2011-2019 C-SKY Limited. All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */


#include "testsuite.h"
#include "matrix_insn.h"


struct matrix_mmaqa_h src0[] = {
    {
        .matrix_int16_s4x8 = {
                {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF},
                {0x0002, 0x0002, 0x0002, 0x0002, 0x0002, 0x0002, 0x0002, 0x0002},
                {0x0010, 0x0010, 0x0010, 0x0010, 0x0010, 0x0010, 0x0010, 0x0010},
                {0x0FFF, 0x0FFF, 0x0FFF, 0x0FFF, 0x0FFF, 0x0FFF, 0x0FFF, 0x0FFF},
        },
    },
};

struct matrix_mmaqa_h src1[] = {
    {
        .matrix_int16_s4x8 = {
                {0x0002, 0x0002, 0x0002, 0x0002, 0x0002, 0x0002, 0x0002, 0x0002},
                {0x0010, 0x0010, 0x0010, 0x0010, 0x0010, 0x0010, 0x0010, 0x0010},
                {0x0008, 0x0008, 0x0008, 0x0008, 0x0008, 0x0008, 0x0008, 0x0008},
                {0x0001, 0x0001, 0x0001, 0x0001, 0x0001, 0x0001, 0x0001, 0x0001},
        },
    },
};

struct matrix_mmaqa_h src2[] = {
    {
        .mat_int64_s4x2_pair = {
                {{0x0000000000000000, 0x0000000000000000},
                 {0x0000000000000001, 0x0000000000000001},
                 {0x0000000000000002, 0x0000000000000002},
                 {0x0000000000000003, 0x0000000000000003}},

                {{0x0000000000000000, 0x0000000000000000},
                 {0x0000000000000001, 0x0000000000000001},
                 {0x0000000000000002, 0x0000000000000002},
                 {0x0000000000000003, 0x0000000000000003}}
        },
    },
};

struct matrix_mmaqa_h dst0[] = {
    {
        .mat_int64_s4x2_pair = {
                {{0xFFFFFFFFFFFFFFF0, 0xFFFFFFFFFFFFFF80},
                 {0x0000000000000021, 0x0000000000000101},
                 {0x0000000000000102, 0x0000000000000802},
                 {0x000000000000FFF3, 0x000000000007FF83}},

                {{0xFFFFFFFFFFFFFFC0, 0xFFFFFFFFFFFFFFF8},
                 {0x0000000000000081, 0x0000000000000011},
                 {0x0000000000000402, 0x0000000000000082},
                 {0x000000000003FFC3, 0x0000000000007FFB}}
        },
    },
};

struct matrix_mmaqa_h res;

#include <stdio.h>
int main(void)
{
    int i = 0;
    init_testsuite("Testing insn mmaqa.h\n");

    test_mmaqa_full_h128(src0[i].matrix_int16_s4x8,
                         src1[i].matrix_int16_s4x8,
                         src2[i].mat_int64_s4x2_pair,
                         res.mat_int64_s4x2_pair);
    result_compare_mmaqa_h_pair_128(dst0[0].mat_int64_s4x2_pair, res.mat_int64_s4x2_pair);

    return done_testing();
}
