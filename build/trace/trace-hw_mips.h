/* This file is autogenerated by tracetool, do not edit. */

#ifndef TRACE_HW_MIPS_GENERATED_TRACERS_H
#define TRACE_HW_MIPS_GENERATED_TRACERS_H

#include "trace/control.h"

extern TraceEvent _TRACE_MALTA_FPGA_LEDS_EVENT;
extern TraceEvent _TRACE_MALTA_FPGA_DISPLAY_EVENT;
extern uint16_t _TRACE_MALTA_FPGA_LEDS_DSTATE;
extern uint16_t _TRACE_MALTA_FPGA_DISPLAY_DSTATE;
#define TRACE_MALTA_FPGA_LEDS_ENABLED 1
#define TRACE_MALTA_FPGA_DISPLAY_ENABLED 1
#include "qemu/log-for-trace.h"
#include "qemu/error-report.h"


#define TRACE_MALTA_FPGA_LEDS_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_MALTA_FPGA_LEDS) || \
    false)

static inline void _nocheck__trace_malta_fpga_leds(const char * text)
{
    if (trace_event_get_state(TRACE_MALTA_FPGA_LEDS) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 2 "../hw/mips/trace-events"
            qemu_log("%d@%zu.%06zu:malta_fpga_leds " "LEDs %s" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , text);
#line 34 "trace/trace-hw_mips.h"
        } else {
#line 2 "../hw/mips/trace-events"
            qemu_log("malta_fpga_leds " "LEDs %s" "\n", text);
#line 38 "trace/trace-hw_mips.h"
        }
    }
}

static inline void trace_malta_fpga_leds(const char * text)
{
    if (true) {
        _nocheck__trace_malta_fpga_leds(text);
    }
}

#define TRACE_MALTA_FPGA_DISPLAY_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_MALTA_FPGA_DISPLAY) || \
    false)

static inline void _nocheck__trace_malta_fpga_display(const char * text)
{
    if (trace_event_get_state(TRACE_MALTA_FPGA_DISPLAY) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 3 "../hw/mips/trace-events"
            qemu_log("%d@%zu.%06zu:malta_fpga_display " "ASCII '%s'" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , text);
#line 65 "trace/trace-hw_mips.h"
        } else {
#line 3 "../hw/mips/trace-events"
            qemu_log("malta_fpga_display " "ASCII '%s'" "\n", text);
#line 69 "trace/trace-hw_mips.h"
        }
    }
}

static inline void trace_malta_fpga_display(const char * text)
{
    if (true) {
        _nocheck__trace_malta_fpga_display(text);
    }
}
#endif /* TRACE_HW_MIPS_GENERATED_TRACERS_H */