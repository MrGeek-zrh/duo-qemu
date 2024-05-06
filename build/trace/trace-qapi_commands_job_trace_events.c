/* This file is autogenerated by tracetool, do not edit. */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "trace-qapi_commands_job_trace_events.h"

uint16_t _TRACE_QMP_ENTER_JOB_PAUSE_DSTATE;
uint16_t _TRACE_QMP_EXIT_JOB_PAUSE_DSTATE;
uint16_t _TRACE_QMP_ENTER_JOB_RESUME_DSTATE;
uint16_t _TRACE_QMP_EXIT_JOB_RESUME_DSTATE;
uint16_t _TRACE_QMP_ENTER_JOB_CANCEL_DSTATE;
uint16_t _TRACE_QMP_EXIT_JOB_CANCEL_DSTATE;
uint16_t _TRACE_QMP_ENTER_JOB_COMPLETE_DSTATE;
uint16_t _TRACE_QMP_EXIT_JOB_COMPLETE_DSTATE;
uint16_t _TRACE_QMP_ENTER_JOB_DISMISS_DSTATE;
uint16_t _TRACE_QMP_EXIT_JOB_DISMISS_DSTATE;
uint16_t _TRACE_QMP_ENTER_JOB_FINALIZE_DSTATE;
uint16_t _TRACE_QMP_EXIT_JOB_FINALIZE_DSTATE;
uint16_t _TRACE_QMP_ENTER_QUERY_JOBS_DSTATE;
uint16_t _TRACE_QMP_EXIT_QUERY_JOBS_DSTATE;
TraceEvent _TRACE_QMP_ENTER_JOB_PAUSE_EVENT = {
    .id = 0,
    .name = "qmp_enter_job_pause",
    .sstate = TRACE_QMP_ENTER_JOB_PAUSE_ENABLED,
    .dstate = &_TRACE_QMP_ENTER_JOB_PAUSE_DSTATE 
};
TraceEvent _TRACE_QMP_EXIT_JOB_PAUSE_EVENT = {
    .id = 0,
    .name = "qmp_exit_job_pause",
    .sstate = TRACE_QMP_EXIT_JOB_PAUSE_ENABLED,
    .dstate = &_TRACE_QMP_EXIT_JOB_PAUSE_DSTATE 
};
TraceEvent _TRACE_QMP_ENTER_JOB_RESUME_EVENT = {
    .id = 0,
    .name = "qmp_enter_job_resume",
    .sstate = TRACE_QMP_ENTER_JOB_RESUME_ENABLED,
    .dstate = &_TRACE_QMP_ENTER_JOB_RESUME_DSTATE 
};
TraceEvent _TRACE_QMP_EXIT_JOB_RESUME_EVENT = {
    .id = 0,
    .name = "qmp_exit_job_resume",
    .sstate = TRACE_QMP_EXIT_JOB_RESUME_ENABLED,
    .dstate = &_TRACE_QMP_EXIT_JOB_RESUME_DSTATE 
};
TraceEvent _TRACE_QMP_ENTER_JOB_CANCEL_EVENT = {
    .id = 0,
    .name = "qmp_enter_job_cancel",
    .sstate = TRACE_QMP_ENTER_JOB_CANCEL_ENABLED,
    .dstate = &_TRACE_QMP_ENTER_JOB_CANCEL_DSTATE 
};
TraceEvent _TRACE_QMP_EXIT_JOB_CANCEL_EVENT = {
    .id = 0,
    .name = "qmp_exit_job_cancel",
    .sstate = TRACE_QMP_EXIT_JOB_CANCEL_ENABLED,
    .dstate = &_TRACE_QMP_EXIT_JOB_CANCEL_DSTATE 
};
TraceEvent _TRACE_QMP_ENTER_JOB_COMPLETE_EVENT = {
    .id = 0,
    .name = "qmp_enter_job_complete",
    .sstate = TRACE_QMP_ENTER_JOB_COMPLETE_ENABLED,
    .dstate = &_TRACE_QMP_ENTER_JOB_COMPLETE_DSTATE 
};
TraceEvent _TRACE_QMP_EXIT_JOB_COMPLETE_EVENT = {
    .id = 0,
    .name = "qmp_exit_job_complete",
    .sstate = TRACE_QMP_EXIT_JOB_COMPLETE_ENABLED,
    .dstate = &_TRACE_QMP_EXIT_JOB_COMPLETE_DSTATE 
};
TraceEvent _TRACE_QMP_ENTER_JOB_DISMISS_EVENT = {
    .id = 0,
    .name = "qmp_enter_job_dismiss",
    .sstate = TRACE_QMP_ENTER_JOB_DISMISS_ENABLED,
    .dstate = &_TRACE_QMP_ENTER_JOB_DISMISS_DSTATE 
};
TraceEvent _TRACE_QMP_EXIT_JOB_DISMISS_EVENT = {
    .id = 0,
    .name = "qmp_exit_job_dismiss",
    .sstate = TRACE_QMP_EXIT_JOB_DISMISS_ENABLED,
    .dstate = &_TRACE_QMP_EXIT_JOB_DISMISS_DSTATE 
};
TraceEvent _TRACE_QMP_ENTER_JOB_FINALIZE_EVENT = {
    .id = 0,
    .name = "qmp_enter_job_finalize",
    .sstate = TRACE_QMP_ENTER_JOB_FINALIZE_ENABLED,
    .dstate = &_TRACE_QMP_ENTER_JOB_FINALIZE_DSTATE 
};
TraceEvent _TRACE_QMP_EXIT_JOB_FINALIZE_EVENT = {
    .id = 0,
    .name = "qmp_exit_job_finalize",
    .sstate = TRACE_QMP_EXIT_JOB_FINALIZE_ENABLED,
    .dstate = &_TRACE_QMP_EXIT_JOB_FINALIZE_DSTATE 
};
TraceEvent _TRACE_QMP_ENTER_QUERY_JOBS_EVENT = {
    .id = 0,
    .name = "qmp_enter_query_jobs",
    .sstate = TRACE_QMP_ENTER_QUERY_JOBS_ENABLED,
    .dstate = &_TRACE_QMP_ENTER_QUERY_JOBS_DSTATE 
};
TraceEvent _TRACE_QMP_EXIT_QUERY_JOBS_EVENT = {
    .id = 0,
    .name = "qmp_exit_query_jobs",
    .sstate = TRACE_QMP_EXIT_QUERY_JOBS_ENABLED,
    .dstate = &_TRACE_QMP_EXIT_QUERY_JOBS_DSTATE 
};
TraceEvent *qapi_commands_job_trace_events_trace_events[] = {
    &_TRACE_QMP_ENTER_JOB_PAUSE_EVENT,
    &_TRACE_QMP_EXIT_JOB_PAUSE_EVENT,
    &_TRACE_QMP_ENTER_JOB_RESUME_EVENT,
    &_TRACE_QMP_EXIT_JOB_RESUME_EVENT,
    &_TRACE_QMP_ENTER_JOB_CANCEL_EVENT,
    &_TRACE_QMP_EXIT_JOB_CANCEL_EVENT,
    &_TRACE_QMP_ENTER_JOB_COMPLETE_EVENT,
    &_TRACE_QMP_EXIT_JOB_COMPLETE_EVENT,
    &_TRACE_QMP_ENTER_JOB_DISMISS_EVENT,
    &_TRACE_QMP_EXIT_JOB_DISMISS_EVENT,
    &_TRACE_QMP_ENTER_JOB_FINALIZE_EVENT,
    &_TRACE_QMP_EXIT_JOB_FINALIZE_EVENT,
    &_TRACE_QMP_ENTER_QUERY_JOBS_EVENT,
    &_TRACE_QMP_EXIT_QUERY_JOBS_EVENT,
  NULL,
};

static void trace_qapi_commands_job_trace_events_register_events(void)
{
    trace_event_register_group(qapi_commands_job_trace_events_trace_events);
}
trace_init(trace_qapi_commands_job_trace_events_register_events)