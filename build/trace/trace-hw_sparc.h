/* This file is autogenerated by tracetool, do not edit. */

#ifndef TRACE_HW_SPARC_GENERATED_TRACERS_H
#define TRACE_HW_SPARC_GENERATED_TRACERS_H

#include "trace/control.h"

extern TraceEvent _TRACE_SUN4M_CPU_SET_IRQ_RAISE_EVENT;
extern TraceEvent _TRACE_SUN4M_CPU_SET_IRQ_LOWER_EVENT;
extern TraceEvent _TRACE_SUN4M_IOMMU_MEM_READL_EVENT;
extern TraceEvent _TRACE_SUN4M_IOMMU_MEM_WRITEL_EVENT;
extern TraceEvent _TRACE_SUN4M_IOMMU_MEM_WRITEL_CTRL_EVENT;
extern TraceEvent _TRACE_SUN4M_IOMMU_MEM_WRITEL_TLBFLUSH_EVENT;
extern TraceEvent _TRACE_SUN4M_IOMMU_MEM_WRITEL_PGFLUSH_EVENT;
extern TraceEvent _TRACE_SUN4M_IOMMU_PAGE_GET_FLAGS_EVENT;
extern TraceEvent _TRACE_SUN4M_IOMMU_TRANSLATE_PA_EVENT;
extern TraceEvent _TRACE_SUN4M_IOMMU_BAD_ADDR_EVENT;
extern TraceEvent _TRACE_LEON3_SET_IRQ_EVENT;
extern TraceEvent _TRACE_LEON3_RESET_IRQ_EVENT;
extern TraceEvent _TRACE_INT_HELPER_ICACHE_FREEZE_EVENT;
extern TraceEvent _TRACE_INT_HELPER_DCACHE_FREEZE_EVENT;
extern uint16_t _TRACE_SUN4M_CPU_SET_IRQ_RAISE_DSTATE;
extern uint16_t _TRACE_SUN4M_CPU_SET_IRQ_LOWER_DSTATE;
extern uint16_t _TRACE_SUN4M_IOMMU_MEM_READL_DSTATE;
extern uint16_t _TRACE_SUN4M_IOMMU_MEM_WRITEL_DSTATE;
extern uint16_t _TRACE_SUN4M_IOMMU_MEM_WRITEL_CTRL_DSTATE;
extern uint16_t _TRACE_SUN4M_IOMMU_MEM_WRITEL_TLBFLUSH_DSTATE;
extern uint16_t _TRACE_SUN4M_IOMMU_MEM_WRITEL_PGFLUSH_DSTATE;
extern uint16_t _TRACE_SUN4M_IOMMU_PAGE_GET_FLAGS_DSTATE;
extern uint16_t _TRACE_SUN4M_IOMMU_TRANSLATE_PA_DSTATE;
extern uint16_t _TRACE_SUN4M_IOMMU_BAD_ADDR_DSTATE;
extern uint16_t _TRACE_LEON3_SET_IRQ_DSTATE;
extern uint16_t _TRACE_LEON3_RESET_IRQ_DSTATE;
extern uint16_t _TRACE_INT_HELPER_ICACHE_FREEZE_DSTATE;
extern uint16_t _TRACE_INT_HELPER_DCACHE_FREEZE_DSTATE;
#define TRACE_SUN4M_CPU_SET_IRQ_RAISE_ENABLED 1
#define TRACE_SUN4M_CPU_SET_IRQ_LOWER_ENABLED 1
#define TRACE_SUN4M_IOMMU_MEM_READL_ENABLED 1
#define TRACE_SUN4M_IOMMU_MEM_WRITEL_ENABLED 1
#define TRACE_SUN4M_IOMMU_MEM_WRITEL_CTRL_ENABLED 1
#define TRACE_SUN4M_IOMMU_MEM_WRITEL_TLBFLUSH_ENABLED 1
#define TRACE_SUN4M_IOMMU_MEM_WRITEL_PGFLUSH_ENABLED 1
#define TRACE_SUN4M_IOMMU_PAGE_GET_FLAGS_ENABLED 1
#define TRACE_SUN4M_IOMMU_TRANSLATE_PA_ENABLED 1
#define TRACE_SUN4M_IOMMU_BAD_ADDR_ENABLED 1
#define TRACE_LEON3_SET_IRQ_ENABLED 1
#define TRACE_LEON3_RESET_IRQ_ENABLED 1
#define TRACE_INT_HELPER_ICACHE_FREEZE_ENABLED 1
#define TRACE_INT_HELPER_DCACHE_FREEZE_ENABLED 1
#include "qemu/log-for-trace.h"
#include "qemu/error-report.h"


#define TRACE_SUN4M_CPU_SET_IRQ_RAISE_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_SUN4M_CPU_SET_IRQ_RAISE) || \
    false)

static inline void _nocheck__trace_sun4m_cpu_set_irq_raise(int level)
{
    if (trace_event_get_state(TRACE_SUN4M_CPU_SET_IRQ_RAISE) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 4 "../hw/sparc/trace-events"
            qemu_log("%d@%zu.%06zu:sun4m_cpu_set_irq_raise " "Raise CPU IRQ %d" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , level);
#line 70 "trace/trace-hw_sparc.h"
        } else {
#line 4 "../hw/sparc/trace-events"
            qemu_log("sun4m_cpu_set_irq_raise " "Raise CPU IRQ %d" "\n", level);
#line 74 "trace/trace-hw_sparc.h"
        }
    }
}

static inline void trace_sun4m_cpu_set_irq_raise(int level)
{
    if (true) {
        _nocheck__trace_sun4m_cpu_set_irq_raise(level);
    }
}

#define TRACE_SUN4M_CPU_SET_IRQ_LOWER_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_SUN4M_CPU_SET_IRQ_LOWER) || \
    false)

static inline void _nocheck__trace_sun4m_cpu_set_irq_lower(int level)
{
    if (trace_event_get_state(TRACE_SUN4M_CPU_SET_IRQ_LOWER) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 5 "../hw/sparc/trace-events"
            qemu_log("%d@%zu.%06zu:sun4m_cpu_set_irq_lower " "Lower CPU IRQ %d" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , level);
#line 101 "trace/trace-hw_sparc.h"
        } else {
#line 5 "../hw/sparc/trace-events"
            qemu_log("sun4m_cpu_set_irq_lower " "Lower CPU IRQ %d" "\n", level);
#line 105 "trace/trace-hw_sparc.h"
        }
    }
}

static inline void trace_sun4m_cpu_set_irq_lower(int level)
{
    if (true) {
        _nocheck__trace_sun4m_cpu_set_irq_lower(level);
    }
}

#define TRACE_SUN4M_IOMMU_MEM_READL_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_SUN4M_IOMMU_MEM_READL) || \
    false)

static inline void _nocheck__trace_sun4m_iommu_mem_readl(uint64_t addr, uint32_t ret)
{
    if (trace_event_get_state(TRACE_SUN4M_IOMMU_MEM_READL) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 8 "../hw/sparc/trace-events"
            qemu_log("%d@%zu.%06zu:sun4m_iommu_mem_readl " "read reg[0x%"PRIx64"] = 0x%x" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , addr, ret);
#line 132 "trace/trace-hw_sparc.h"
        } else {
#line 8 "../hw/sparc/trace-events"
            qemu_log("sun4m_iommu_mem_readl " "read reg[0x%"PRIx64"] = 0x%x" "\n", addr, ret);
#line 136 "trace/trace-hw_sparc.h"
        }
    }
}

static inline void trace_sun4m_iommu_mem_readl(uint64_t addr, uint32_t ret)
{
    if (true) {
        _nocheck__trace_sun4m_iommu_mem_readl(addr, ret);
    }
}

#define TRACE_SUN4M_IOMMU_MEM_WRITEL_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_SUN4M_IOMMU_MEM_WRITEL) || \
    false)

static inline void _nocheck__trace_sun4m_iommu_mem_writel(uint64_t addr, uint32_t val)
{
    if (trace_event_get_state(TRACE_SUN4M_IOMMU_MEM_WRITEL) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 9 "../hw/sparc/trace-events"
            qemu_log("%d@%zu.%06zu:sun4m_iommu_mem_writel " "write reg[0x%"PRIx64"] = 0x%x" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , addr, val);
#line 163 "trace/trace-hw_sparc.h"
        } else {
#line 9 "../hw/sparc/trace-events"
            qemu_log("sun4m_iommu_mem_writel " "write reg[0x%"PRIx64"] = 0x%x" "\n", addr, val);
#line 167 "trace/trace-hw_sparc.h"
        }
    }
}

static inline void trace_sun4m_iommu_mem_writel(uint64_t addr, uint32_t val)
{
    if (true) {
        _nocheck__trace_sun4m_iommu_mem_writel(addr, val);
    }
}

#define TRACE_SUN4M_IOMMU_MEM_WRITEL_CTRL_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_SUN4M_IOMMU_MEM_WRITEL_CTRL) || \
    false)

static inline void _nocheck__trace_sun4m_iommu_mem_writel_ctrl(uint64_t iostart)
{
    if (trace_event_get_state(TRACE_SUN4M_IOMMU_MEM_WRITEL_CTRL) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 10 "../hw/sparc/trace-events"
            qemu_log("%d@%zu.%06zu:sun4m_iommu_mem_writel_ctrl " "iostart = 0x%"PRIx64 "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , iostart);
#line 194 "trace/trace-hw_sparc.h"
        } else {
#line 10 "../hw/sparc/trace-events"
            qemu_log("sun4m_iommu_mem_writel_ctrl " "iostart = 0x%"PRIx64 "\n", iostart);
#line 198 "trace/trace-hw_sparc.h"
        }
    }
}

static inline void trace_sun4m_iommu_mem_writel_ctrl(uint64_t iostart)
{
    if (true) {
        _nocheck__trace_sun4m_iommu_mem_writel_ctrl(iostart);
    }
}

#define TRACE_SUN4M_IOMMU_MEM_WRITEL_TLBFLUSH_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_SUN4M_IOMMU_MEM_WRITEL_TLBFLUSH) || \
    false)

static inline void _nocheck__trace_sun4m_iommu_mem_writel_tlbflush(uint32_t val)
{
    if (trace_event_get_state(TRACE_SUN4M_IOMMU_MEM_WRITEL_TLBFLUSH) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 11 "../hw/sparc/trace-events"
            qemu_log("%d@%zu.%06zu:sun4m_iommu_mem_writel_tlbflush " "tlb flush 0x%x" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , val);
#line 225 "trace/trace-hw_sparc.h"
        } else {
#line 11 "../hw/sparc/trace-events"
            qemu_log("sun4m_iommu_mem_writel_tlbflush " "tlb flush 0x%x" "\n", val);
#line 229 "trace/trace-hw_sparc.h"
        }
    }
}

static inline void trace_sun4m_iommu_mem_writel_tlbflush(uint32_t val)
{
    if (true) {
        _nocheck__trace_sun4m_iommu_mem_writel_tlbflush(val);
    }
}

#define TRACE_SUN4M_IOMMU_MEM_WRITEL_PGFLUSH_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_SUN4M_IOMMU_MEM_WRITEL_PGFLUSH) || \
    false)

static inline void _nocheck__trace_sun4m_iommu_mem_writel_pgflush(uint32_t val)
{
    if (trace_event_get_state(TRACE_SUN4M_IOMMU_MEM_WRITEL_PGFLUSH) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 12 "../hw/sparc/trace-events"
            qemu_log("%d@%zu.%06zu:sun4m_iommu_mem_writel_pgflush " "page flush 0x%x" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , val);
#line 256 "trace/trace-hw_sparc.h"
        } else {
#line 12 "../hw/sparc/trace-events"
            qemu_log("sun4m_iommu_mem_writel_pgflush " "page flush 0x%x" "\n", val);
#line 260 "trace/trace-hw_sparc.h"
        }
    }
}

static inline void trace_sun4m_iommu_mem_writel_pgflush(uint32_t val)
{
    if (true) {
        _nocheck__trace_sun4m_iommu_mem_writel_pgflush(val);
    }
}

#define TRACE_SUN4M_IOMMU_PAGE_GET_FLAGS_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_SUN4M_IOMMU_PAGE_GET_FLAGS) || \
    false)

static inline void _nocheck__trace_sun4m_iommu_page_get_flags(uint64_t pa, uint64_t iopte, uint32_t ret)
{
    if (trace_event_get_state(TRACE_SUN4M_IOMMU_PAGE_GET_FLAGS) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 13 "../hw/sparc/trace-events"
            qemu_log("%d@%zu.%06zu:sun4m_iommu_page_get_flags " "get flags addr 0x%"PRIx64" => pte 0x%"PRIx64", *pte = 0x%x" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , pa, iopte, ret);
#line 287 "trace/trace-hw_sparc.h"
        } else {
#line 13 "../hw/sparc/trace-events"
            qemu_log("sun4m_iommu_page_get_flags " "get flags addr 0x%"PRIx64" => pte 0x%"PRIx64", *pte = 0x%x" "\n", pa, iopte, ret);
#line 291 "trace/trace-hw_sparc.h"
        }
    }
}

static inline void trace_sun4m_iommu_page_get_flags(uint64_t pa, uint64_t iopte, uint32_t ret)
{
    if (true) {
        _nocheck__trace_sun4m_iommu_page_get_flags(pa, iopte, ret);
    }
}

#define TRACE_SUN4M_IOMMU_TRANSLATE_PA_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_SUN4M_IOMMU_TRANSLATE_PA) || \
    false)

static inline void _nocheck__trace_sun4m_iommu_translate_pa(uint64_t addr, uint64_t pa, uint32_t iopte)
{
    if (trace_event_get_state(TRACE_SUN4M_IOMMU_TRANSLATE_PA) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 14 "../hw/sparc/trace-events"
            qemu_log("%d@%zu.%06zu:sun4m_iommu_translate_pa " "xlate dva 0x%"PRIx64" => pa 0x%"PRIx64" iopte = 0x%x" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , addr, pa, iopte);
#line 318 "trace/trace-hw_sparc.h"
        } else {
#line 14 "../hw/sparc/trace-events"
            qemu_log("sun4m_iommu_translate_pa " "xlate dva 0x%"PRIx64" => pa 0x%"PRIx64" iopte = 0x%x" "\n", addr, pa, iopte);
#line 322 "trace/trace-hw_sparc.h"
        }
    }
}

static inline void trace_sun4m_iommu_translate_pa(uint64_t addr, uint64_t pa, uint32_t iopte)
{
    if (true) {
        _nocheck__trace_sun4m_iommu_translate_pa(addr, pa, iopte);
    }
}

#define TRACE_SUN4M_IOMMU_BAD_ADDR_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_SUN4M_IOMMU_BAD_ADDR) || \
    false)

static inline void _nocheck__trace_sun4m_iommu_bad_addr(uint64_t addr)
{
    if (trace_event_get_state(TRACE_SUN4M_IOMMU_BAD_ADDR) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 15 "../hw/sparc/trace-events"
            qemu_log("%d@%zu.%06zu:sun4m_iommu_bad_addr " "bad addr 0x%"PRIx64 "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , addr);
#line 349 "trace/trace-hw_sparc.h"
        } else {
#line 15 "../hw/sparc/trace-events"
            qemu_log("sun4m_iommu_bad_addr " "bad addr 0x%"PRIx64 "\n", addr);
#line 353 "trace/trace-hw_sparc.h"
        }
    }
}

static inline void trace_sun4m_iommu_bad_addr(uint64_t addr)
{
    if (true) {
        _nocheck__trace_sun4m_iommu_bad_addr(addr);
    }
}

#define TRACE_LEON3_SET_IRQ_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_LEON3_SET_IRQ) || \
    false)

static inline void _nocheck__trace_leon3_set_irq(int intno)
{
    if (trace_event_get_state(TRACE_LEON3_SET_IRQ) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 18 "../hw/sparc/trace-events"
            qemu_log("%d@%zu.%06zu:leon3_set_irq " "Set CPU IRQ %d" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , intno);
#line 380 "trace/trace-hw_sparc.h"
        } else {
#line 18 "../hw/sparc/trace-events"
            qemu_log("leon3_set_irq " "Set CPU IRQ %d" "\n", intno);
#line 384 "trace/trace-hw_sparc.h"
        }
    }
}

static inline void trace_leon3_set_irq(int intno)
{
    if (true) {
        _nocheck__trace_leon3_set_irq(intno);
    }
}

#define TRACE_LEON3_RESET_IRQ_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_LEON3_RESET_IRQ) || \
    false)

static inline void _nocheck__trace_leon3_reset_irq(int intno)
{
    if (trace_event_get_state(TRACE_LEON3_RESET_IRQ) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 19 "../hw/sparc/trace-events"
            qemu_log("%d@%zu.%06zu:leon3_reset_irq " "Reset CPU IRQ %d" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , intno);
#line 411 "trace/trace-hw_sparc.h"
        } else {
#line 19 "../hw/sparc/trace-events"
            qemu_log("leon3_reset_irq " "Reset CPU IRQ %d" "\n", intno);
#line 415 "trace/trace-hw_sparc.h"
        }
    }
}

static inline void trace_leon3_reset_irq(int intno)
{
    if (true) {
        _nocheck__trace_leon3_reset_irq(intno);
    }
}

#define TRACE_INT_HELPER_ICACHE_FREEZE_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_INT_HELPER_ICACHE_FREEZE) || \
    false)

static inline void _nocheck__trace_int_helper_icache_freeze(void)
{
    if (trace_event_get_state(TRACE_INT_HELPER_ICACHE_FREEZE) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 20 "../hw/sparc/trace-events"
            qemu_log("%d@%zu.%06zu:int_helper_icache_freeze " "Instruction cache: freeze" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     );
#line 442 "trace/trace-hw_sparc.h"
        } else {
#line 20 "../hw/sparc/trace-events"
            qemu_log("int_helper_icache_freeze " "Instruction cache: freeze" "\n");
#line 446 "trace/trace-hw_sparc.h"
        }
    }
}

static inline void trace_int_helper_icache_freeze(void)
{
    if (true) {
        _nocheck__trace_int_helper_icache_freeze();
    }
}

#define TRACE_INT_HELPER_DCACHE_FREEZE_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_INT_HELPER_DCACHE_FREEZE) || \
    false)

static inline void _nocheck__trace_int_helper_dcache_freeze(void)
{
    if (trace_event_get_state(TRACE_INT_HELPER_DCACHE_FREEZE) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 21 "../hw/sparc/trace-events"
            qemu_log("%d@%zu.%06zu:int_helper_dcache_freeze " "Data cache: freeze" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     );
#line 473 "trace/trace-hw_sparc.h"
        } else {
#line 21 "../hw/sparc/trace-events"
            qemu_log("int_helper_dcache_freeze " "Data cache: freeze" "\n");
#line 477 "trace/trace-hw_sparc.h"
        }
    }
}

static inline void trace_int_helper_dcache_freeze(void)
{
    if (true) {
        _nocheck__trace_int_helper_dcache_freeze();
    }
}
#endif /* TRACE_HW_SPARC_GENERATED_TRACERS_H */
