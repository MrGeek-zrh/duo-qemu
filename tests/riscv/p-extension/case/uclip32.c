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
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "testsuite.h"
#include "test_device.h"
#include "rvp_insn.h"
#include "sample_array.h"
int main(void)
{
    int i = 0;
    init_testsuite("Testing insn uclip32\n");

    for (i = 0;
         i < sizeof(samples_uclip32)/sizeof(struct binary_calculation);
         i++) {
        if (samples_uclip32[i].op2 == 0) {
            TEST(test_uclip32_0(samples_uclip32[i].op1)
                 == samples_uclip32[i].result);
        } else if (samples_uclip32[i].op2 == 1) {
            TEST(test_uclip32_1(samples_uclip32[i].op1)
                 == samples_uclip32[i].result);
        } else if (samples_uclip32[i].op2 == 2) {
            TEST(test_uclip32_2(samples_uclip32[i].op1)
                 == samples_uclip32[i].result);
        } else if (samples_uclip32[i].op2 == 6) {
            TEST(test_uclip32_6(samples_uclip32[i].op1)
                 == samples_uclip32[i].result);
        } else if (samples_uclip32[i].op2 == 31) {
            TEST(test_uclip32_31(samples_uclip32[i].op1)
                 == samples_uclip32[i].result);
        }
    }
    return done_testing();
}
