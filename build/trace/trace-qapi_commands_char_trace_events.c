/* This file is autogenerated by tracetool, do not edit. */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "trace-qapi_commands_char_trace_events.h"

uint16_t _TRACE_QMP_ENTER_QUERY_CHARDEV_DSTATE;
uint16_t _TRACE_QMP_EXIT_QUERY_CHARDEV_DSTATE;
uint16_t _TRACE_QMP_ENTER_QUERY_CHARDEV_BACKENDS_DSTATE;
uint16_t _TRACE_QMP_EXIT_QUERY_CHARDEV_BACKENDS_DSTATE;
uint16_t _TRACE_QMP_ENTER_RINGBUF_WRITE_DSTATE;
uint16_t _TRACE_QMP_EXIT_RINGBUF_WRITE_DSTATE;
uint16_t _TRACE_QMP_ENTER_RINGBUF_READ_DSTATE;
uint16_t _TRACE_QMP_EXIT_RINGBUF_READ_DSTATE;
uint16_t _TRACE_QMP_ENTER_CHARDEV_ADD_DSTATE;
uint16_t _TRACE_QMP_EXIT_CHARDEV_ADD_DSTATE;
uint16_t _TRACE_QMP_ENTER_CHARDEV_CHANGE_DSTATE;
uint16_t _TRACE_QMP_EXIT_CHARDEV_CHANGE_DSTATE;
uint16_t _TRACE_QMP_ENTER_CHARDEV_REMOVE_DSTATE;
uint16_t _TRACE_QMP_EXIT_CHARDEV_REMOVE_DSTATE;
uint16_t _TRACE_QMP_ENTER_CHARDEV_SEND_BREAK_DSTATE;
uint16_t _TRACE_QMP_EXIT_CHARDEV_SEND_BREAK_DSTATE;
TraceEvent _TRACE_QMP_ENTER_QUERY_CHARDEV_EVENT = {
    .id = 0,
    .name = "qmp_enter_query_chardev",
    .sstate = TRACE_QMP_ENTER_QUERY_CHARDEV_ENABLED,
    .dstate = &_TRACE_QMP_ENTER_QUERY_CHARDEV_DSTATE 
};
TraceEvent _TRACE_QMP_EXIT_QUERY_CHARDEV_EVENT = {
    .id = 0,
    .name = "qmp_exit_query_chardev",
    .sstate = TRACE_QMP_EXIT_QUERY_CHARDEV_ENABLED,
    .dstate = &_TRACE_QMP_EXIT_QUERY_CHARDEV_DSTATE 
};
TraceEvent _TRACE_QMP_ENTER_QUERY_CHARDEV_BACKENDS_EVENT = {
    .id = 0,
    .name = "qmp_enter_query_chardev_backends",
    .sstate = TRACE_QMP_ENTER_QUERY_CHARDEV_BACKENDS_ENABLED,
    .dstate = &_TRACE_QMP_ENTER_QUERY_CHARDEV_BACKENDS_DSTATE 
};
TraceEvent _TRACE_QMP_EXIT_QUERY_CHARDEV_BACKENDS_EVENT = {
    .id = 0,
    .name = "qmp_exit_query_chardev_backends",
    .sstate = TRACE_QMP_EXIT_QUERY_CHARDEV_BACKENDS_ENABLED,
    .dstate = &_TRACE_QMP_EXIT_QUERY_CHARDEV_BACKENDS_DSTATE 
};
TraceEvent _TRACE_QMP_ENTER_RINGBUF_WRITE_EVENT = {
    .id = 0,
    .name = "qmp_enter_ringbuf_write",
    .sstate = TRACE_QMP_ENTER_RINGBUF_WRITE_ENABLED,
    .dstate = &_TRACE_QMP_ENTER_RINGBUF_WRITE_DSTATE 
};
TraceEvent _TRACE_QMP_EXIT_RINGBUF_WRITE_EVENT = {
    .id = 0,
    .name = "qmp_exit_ringbuf_write",
    .sstate = TRACE_QMP_EXIT_RINGBUF_WRITE_ENABLED,
    .dstate = &_TRACE_QMP_EXIT_RINGBUF_WRITE_DSTATE 
};
TraceEvent _TRACE_QMP_ENTER_RINGBUF_READ_EVENT = {
    .id = 0,
    .name = "qmp_enter_ringbuf_read",
    .sstate = TRACE_QMP_ENTER_RINGBUF_READ_ENABLED,
    .dstate = &_TRACE_QMP_ENTER_RINGBUF_READ_DSTATE 
};
TraceEvent _TRACE_QMP_EXIT_RINGBUF_READ_EVENT = {
    .id = 0,
    .name = "qmp_exit_ringbuf_read",
    .sstate = TRACE_QMP_EXIT_RINGBUF_READ_ENABLED,
    .dstate = &_TRACE_QMP_EXIT_RINGBUF_READ_DSTATE 
};
TraceEvent _TRACE_QMP_ENTER_CHARDEV_ADD_EVENT = {
    .id = 0,
    .name = "qmp_enter_chardev_add",
    .sstate = TRACE_QMP_ENTER_CHARDEV_ADD_ENABLED,
    .dstate = &_TRACE_QMP_ENTER_CHARDEV_ADD_DSTATE 
};
TraceEvent _TRACE_QMP_EXIT_CHARDEV_ADD_EVENT = {
    .id = 0,
    .name = "qmp_exit_chardev_add",
    .sstate = TRACE_QMP_EXIT_CHARDEV_ADD_ENABLED,
    .dstate = &_TRACE_QMP_EXIT_CHARDEV_ADD_DSTATE 
};
TraceEvent _TRACE_QMP_ENTER_CHARDEV_CHANGE_EVENT = {
    .id = 0,
    .name = "qmp_enter_chardev_change",
    .sstate = TRACE_QMP_ENTER_CHARDEV_CHANGE_ENABLED,
    .dstate = &_TRACE_QMP_ENTER_CHARDEV_CHANGE_DSTATE 
};
TraceEvent _TRACE_QMP_EXIT_CHARDEV_CHANGE_EVENT = {
    .id = 0,
    .name = "qmp_exit_chardev_change",
    .sstate = TRACE_QMP_EXIT_CHARDEV_CHANGE_ENABLED,
    .dstate = &_TRACE_QMP_EXIT_CHARDEV_CHANGE_DSTATE 
};
TraceEvent _TRACE_QMP_ENTER_CHARDEV_REMOVE_EVENT = {
    .id = 0,
    .name = "qmp_enter_chardev_remove",
    .sstate = TRACE_QMP_ENTER_CHARDEV_REMOVE_ENABLED,
    .dstate = &_TRACE_QMP_ENTER_CHARDEV_REMOVE_DSTATE 
};
TraceEvent _TRACE_QMP_EXIT_CHARDEV_REMOVE_EVENT = {
    .id = 0,
    .name = "qmp_exit_chardev_remove",
    .sstate = TRACE_QMP_EXIT_CHARDEV_REMOVE_ENABLED,
    .dstate = &_TRACE_QMP_EXIT_CHARDEV_REMOVE_DSTATE 
};
TraceEvent _TRACE_QMP_ENTER_CHARDEV_SEND_BREAK_EVENT = {
    .id = 0,
    .name = "qmp_enter_chardev_send_break",
    .sstate = TRACE_QMP_ENTER_CHARDEV_SEND_BREAK_ENABLED,
    .dstate = &_TRACE_QMP_ENTER_CHARDEV_SEND_BREAK_DSTATE 
};
TraceEvent _TRACE_QMP_EXIT_CHARDEV_SEND_BREAK_EVENT = {
    .id = 0,
    .name = "qmp_exit_chardev_send_break",
    .sstate = TRACE_QMP_EXIT_CHARDEV_SEND_BREAK_ENABLED,
    .dstate = &_TRACE_QMP_EXIT_CHARDEV_SEND_BREAK_DSTATE 
};
TraceEvent *qapi_commands_char_trace_events_trace_events[] = {
    &_TRACE_QMP_ENTER_QUERY_CHARDEV_EVENT,
    &_TRACE_QMP_EXIT_QUERY_CHARDEV_EVENT,
    &_TRACE_QMP_ENTER_QUERY_CHARDEV_BACKENDS_EVENT,
    &_TRACE_QMP_EXIT_QUERY_CHARDEV_BACKENDS_EVENT,
    &_TRACE_QMP_ENTER_RINGBUF_WRITE_EVENT,
    &_TRACE_QMP_EXIT_RINGBUF_WRITE_EVENT,
    &_TRACE_QMP_ENTER_RINGBUF_READ_EVENT,
    &_TRACE_QMP_EXIT_RINGBUF_READ_EVENT,
    &_TRACE_QMP_ENTER_CHARDEV_ADD_EVENT,
    &_TRACE_QMP_EXIT_CHARDEV_ADD_EVENT,
    &_TRACE_QMP_ENTER_CHARDEV_CHANGE_EVENT,
    &_TRACE_QMP_EXIT_CHARDEV_CHANGE_EVENT,
    &_TRACE_QMP_ENTER_CHARDEV_REMOVE_EVENT,
    &_TRACE_QMP_EXIT_CHARDEV_REMOVE_EVENT,
    &_TRACE_QMP_ENTER_CHARDEV_SEND_BREAK_EVENT,
    &_TRACE_QMP_EXIT_CHARDEV_SEND_BREAK_EVENT,
  NULL,
};

static void trace_qapi_commands_char_trace_events_register_events(void)
{
    trace_event_register_group(qapi_commands_char_trace_events_trace_events);
}
trace_init(trace_qapi_commands_char_trace_events_register_events)
