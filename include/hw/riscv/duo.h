/*
 * QEMU RISC-V Duo Board
 *
 * Copyright (c) 2024 zhangrenhao
 * reference: https://github.com/QQxiaoming/quard_star_tutorial
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HW_RISCV_DUO__H
#define HW_RISCV_DUO__H

#include "hw/block/flash.h"
#include "hw/sysbus.h"
#include "qom/object.h"
#include "target/riscv/cpu-qom.h"

// 总cpu core的最大个数
#define DUO_CPUS_MAX 2
// 插槽个数
#define DUO_SOCKETS_MAX 2

#define TYPE_RISCV_DUO_MACHINE MACHINE_TYPE_NAME("duo")
typedef struct DUOSocState DUOSocState;
DECLARE_INSTANCE_CHECKER(DUOSocState, RISCV_DUO_MACHINE, TYPE_RISCV_DUO_MACHINE)

struct DUOSocState {
    /*< private >*/
    MachineState parent;

    /*< public >*/
    // 一个插槽放一个soc
    RISCVHartArrayState soc[DUO_SOCKETS_MAX];
    DeviceState *plic[DUO_SOCKETS_MAX];
    // 板上的flash
    PFlashCFI01 *flash;
};

enum {
    DUO_MROM,
    DUO_SRAM,
    DUO_CLINT,
    DUO_PLIC,
    DUO_UART0,
    DUO_FLASH,
    DUO_DRAM,
};

enum {
    DUO_UART0_IRQ = 10,
};

#define DUO_PLIC_HART_CONFIG "MS"
// 中断源个数
#define DUO_PLIC_NUM_SOURCES 1023
// 优先级个数
#define DUO_PLIC_NUM_PRIORITIES 32
// 优先级寄存器起始地址
#define DUO_PLIC_PRIORITY_BASE 0x04
// 等待寄存器起始地址
#define DUO_PLIC_PENDING_BASE 0x1000
// 使能寄存器起始地址
#define DUO_PLIC_ENABLE_BASE 0x2000
// 使能寄存器步长:需要1024bit
#define DUO_PLIC_ENABLE_STRIDE 0x400
// 中断上下文起始地址
#define DUO_PLIC_CONTEXT_BASE 0x200000
#define DUO_PLIC_CONTEXT_STRIDE 0x1000

#define DUO_PLIC_SIZE(__num_context)                                           \
    (DUO_PLIC_CONTEXT_BASE + (__num_context) * DUO_PLIC_CONTEXT_STRIDE)

#endif
