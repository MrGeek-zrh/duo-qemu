/* This file is autogenerated by tracetool, do not edit. */

#ifndef TRACE_QAPI_COMMANDS_JOB_TRACE_EVENTS_GENERATED_TRACERS_H
#define TRACE_QAPI_COMMANDS_JOB_TRACE_EVENTS_GENERATED_TRACERS_H

#include "trace/control.h"

extern TraceEvent _TRACE_QMP_ENTER_JOB_PAUSE_EVENT;
extern TraceEvent _TRACE_QMP_EXIT_JOB_PAUSE_EVENT;
extern TraceEvent _TRACE_QMP_ENTER_JOB_RESUME_EVENT;
extern TraceEvent _TRACE_QMP_EXIT_JOB_RESUME_EVENT;
extern TraceEvent _TRACE_QMP_ENTER_JOB_CANCEL_EVENT;
extern TraceEvent _TRACE_QMP_EXIT_JOB_CANCEL_EVENT;
extern TraceEvent _TRACE_QMP_ENTER_JOB_COMPLETE_EVENT;
extern TraceEvent _TRACE_QMP_EXIT_JOB_COMPLETE_EVENT;
extern TraceEvent _TRACE_QMP_ENTER_JOB_DISMISS_EVENT;
extern TraceEvent _TRACE_QMP_EXIT_JOB_DISMISS_EVENT;
extern TraceEvent _TRACE_QMP_ENTER_JOB_FINALIZE_EVENT;
extern TraceEvent _TRACE_QMP_EXIT_JOB_FINALIZE_EVENT;
extern TraceEvent _TRACE_QMP_ENTER_QUERY_JOBS_EVENT;
extern TraceEvent _TRACE_QMP_EXIT_QUERY_JOBS_EVENT;
extern uint16_t _TRACE_QMP_ENTER_JOB_PAUSE_DSTATE;
extern uint16_t _TRACE_QMP_EXIT_JOB_PAUSE_DSTATE;
extern uint16_t _TRACE_QMP_ENTER_JOB_RESUME_DSTATE;
extern uint16_t _TRACE_QMP_EXIT_JOB_RESUME_DSTATE;
extern uint16_t _TRACE_QMP_ENTER_JOB_CANCEL_DSTATE;
extern uint16_t _TRACE_QMP_EXIT_JOB_CANCEL_DSTATE;
extern uint16_t _TRACE_QMP_ENTER_JOB_COMPLETE_DSTATE;
extern uint16_t _TRACE_QMP_EXIT_JOB_COMPLETE_DSTATE;
extern uint16_t _TRACE_QMP_ENTER_JOB_DISMISS_DSTATE;
extern uint16_t _TRACE_QMP_EXIT_JOB_DISMISS_DSTATE;
extern uint16_t _TRACE_QMP_ENTER_JOB_FINALIZE_DSTATE;
extern uint16_t _TRACE_QMP_EXIT_JOB_FINALIZE_DSTATE;
extern uint16_t _TRACE_QMP_ENTER_QUERY_JOBS_DSTATE;
extern uint16_t _TRACE_QMP_EXIT_QUERY_JOBS_DSTATE;
#define TRACE_QMP_ENTER_JOB_PAUSE_ENABLED 1
#define TRACE_QMP_EXIT_JOB_PAUSE_ENABLED 1
#define TRACE_QMP_ENTER_JOB_RESUME_ENABLED 1
#define TRACE_QMP_EXIT_JOB_RESUME_ENABLED 1
#define TRACE_QMP_ENTER_JOB_CANCEL_ENABLED 1
#define TRACE_QMP_EXIT_JOB_CANCEL_ENABLED 1
#define TRACE_QMP_ENTER_JOB_COMPLETE_ENABLED 1
#define TRACE_QMP_EXIT_JOB_COMPLETE_ENABLED 1
#define TRACE_QMP_ENTER_JOB_DISMISS_ENABLED 1
#define TRACE_QMP_EXIT_JOB_DISMISS_ENABLED 1
#define TRACE_QMP_ENTER_JOB_FINALIZE_ENABLED 1
#define TRACE_QMP_EXIT_JOB_FINALIZE_ENABLED 1
#define TRACE_QMP_ENTER_QUERY_JOBS_ENABLED 1
#define TRACE_QMP_EXIT_QUERY_JOBS_ENABLED 1
#include "qemu/log-for-trace.h"
#include "qemu/error-report.h"


#define TRACE_QMP_ENTER_JOB_PAUSE_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_QMP_ENTER_JOB_PAUSE) || \
    false)

static inline void _nocheck__trace_qmp_enter_job_pause(const char * json)
{
    if (trace_event_get_state(TRACE_QMP_ENTER_JOB_PAUSE) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 3 "qapi/qapi-commands-job.trace-events"
            qemu_log("%d@%zu.%06zu:qmp_enter_job_pause " "%s" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , json);
#line 70 "trace/trace-qapi_commands_job_trace_events.h"
        } else {
#line 3 "qapi/qapi-commands-job.trace-events"
            qemu_log("qmp_enter_job_pause " "%s" "\n", json);
#line 74 "trace/trace-qapi_commands_job_trace_events.h"
        }
    }
}

static inline void trace_qmp_enter_job_pause(const char * json)
{
    if (true) {
        _nocheck__trace_qmp_enter_job_pause(json);
    }
}

#define TRACE_QMP_EXIT_JOB_PAUSE_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_QMP_EXIT_JOB_PAUSE) || \
    false)

static inline void _nocheck__trace_qmp_exit_job_pause(const char * result, bool succeeded)
{
    if (trace_event_get_state(TRACE_QMP_EXIT_JOB_PAUSE) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 4 "qapi/qapi-commands-job.trace-events"
            qemu_log("%d@%zu.%06zu:qmp_exit_job_pause " "%s %d" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , result, succeeded);
#line 101 "trace/trace-qapi_commands_job_trace_events.h"
        } else {
#line 4 "qapi/qapi-commands-job.trace-events"
            qemu_log("qmp_exit_job_pause " "%s %d" "\n", result, succeeded);
#line 105 "trace/trace-qapi_commands_job_trace_events.h"
        }
    }
}

static inline void trace_qmp_exit_job_pause(const char * result, bool succeeded)
{
    if (true) {
        _nocheck__trace_qmp_exit_job_pause(result, succeeded);
    }
}

#define TRACE_QMP_ENTER_JOB_RESUME_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_QMP_ENTER_JOB_RESUME) || \
    false)

static inline void _nocheck__trace_qmp_enter_job_resume(const char * json)
{
    if (trace_event_get_state(TRACE_QMP_ENTER_JOB_RESUME) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 5 "qapi/qapi-commands-job.trace-events"
            qemu_log("%d@%zu.%06zu:qmp_enter_job_resume " "%s" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , json);
#line 132 "trace/trace-qapi_commands_job_trace_events.h"
        } else {
#line 5 "qapi/qapi-commands-job.trace-events"
            qemu_log("qmp_enter_job_resume " "%s" "\n", json);
#line 136 "trace/trace-qapi_commands_job_trace_events.h"
        }
    }
}

static inline void trace_qmp_enter_job_resume(const char * json)
{
    if (true) {
        _nocheck__trace_qmp_enter_job_resume(json);
    }
}

#define TRACE_QMP_EXIT_JOB_RESUME_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_QMP_EXIT_JOB_RESUME) || \
    false)

static inline void _nocheck__trace_qmp_exit_job_resume(const char * result, bool succeeded)
{
    if (trace_event_get_state(TRACE_QMP_EXIT_JOB_RESUME) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 6 "qapi/qapi-commands-job.trace-events"
            qemu_log("%d@%zu.%06zu:qmp_exit_job_resume " "%s %d" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , result, succeeded);
#line 163 "trace/trace-qapi_commands_job_trace_events.h"
        } else {
#line 6 "qapi/qapi-commands-job.trace-events"
            qemu_log("qmp_exit_job_resume " "%s %d" "\n", result, succeeded);
#line 167 "trace/trace-qapi_commands_job_trace_events.h"
        }
    }
}

static inline void trace_qmp_exit_job_resume(const char * result, bool succeeded)
{
    if (true) {
        _nocheck__trace_qmp_exit_job_resume(result, succeeded);
    }
}

#define TRACE_QMP_ENTER_JOB_CANCEL_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_QMP_ENTER_JOB_CANCEL) || \
    false)

static inline void _nocheck__trace_qmp_enter_job_cancel(const char * json)
{
    if (trace_event_get_state(TRACE_QMP_ENTER_JOB_CANCEL) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 7 "qapi/qapi-commands-job.trace-events"
            qemu_log("%d@%zu.%06zu:qmp_enter_job_cancel " "%s" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , json);
#line 194 "trace/trace-qapi_commands_job_trace_events.h"
        } else {
#line 7 "qapi/qapi-commands-job.trace-events"
            qemu_log("qmp_enter_job_cancel " "%s" "\n", json);
#line 198 "trace/trace-qapi_commands_job_trace_events.h"
        }
    }
}

static inline void trace_qmp_enter_job_cancel(const char * json)
{
    if (true) {
        _nocheck__trace_qmp_enter_job_cancel(json);
    }
}

#define TRACE_QMP_EXIT_JOB_CANCEL_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_QMP_EXIT_JOB_CANCEL) || \
    false)

static inline void _nocheck__trace_qmp_exit_job_cancel(const char * result, bool succeeded)
{
    if (trace_event_get_state(TRACE_QMP_EXIT_JOB_CANCEL) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 8 "qapi/qapi-commands-job.trace-events"
            qemu_log("%d@%zu.%06zu:qmp_exit_job_cancel " "%s %d" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , result, succeeded);
#line 225 "trace/trace-qapi_commands_job_trace_events.h"
        } else {
#line 8 "qapi/qapi-commands-job.trace-events"
            qemu_log("qmp_exit_job_cancel " "%s %d" "\n", result, succeeded);
#line 229 "trace/trace-qapi_commands_job_trace_events.h"
        }
    }
}

static inline void trace_qmp_exit_job_cancel(const char * result, bool succeeded)
{
    if (true) {
        _nocheck__trace_qmp_exit_job_cancel(result, succeeded);
    }
}

#define TRACE_QMP_ENTER_JOB_COMPLETE_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_QMP_ENTER_JOB_COMPLETE) || \
    false)

static inline void _nocheck__trace_qmp_enter_job_complete(const char * json)
{
    if (trace_event_get_state(TRACE_QMP_ENTER_JOB_COMPLETE) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 9 "qapi/qapi-commands-job.trace-events"
            qemu_log("%d@%zu.%06zu:qmp_enter_job_complete " "%s" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , json);
#line 256 "trace/trace-qapi_commands_job_trace_events.h"
        } else {
#line 9 "qapi/qapi-commands-job.trace-events"
            qemu_log("qmp_enter_job_complete " "%s" "\n", json);
#line 260 "trace/trace-qapi_commands_job_trace_events.h"
        }
    }
}

static inline void trace_qmp_enter_job_complete(const char * json)
{
    if (true) {
        _nocheck__trace_qmp_enter_job_complete(json);
    }
}

#define TRACE_QMP_EXIT_JOB_COMPLETE_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_QMP_EXIT_JOB_COMPLETE) || \
    false)

static inline void _nocheck__trace_qmp_exit_job_complete(const char * result, bool succeeded)
{
    if (trace_event_get_state(TRACE_QMP_EXIT_JOB_COMPLETE) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 10 "qapi/qapi-commands-job.trace-events"
            qemu_log("%d@%zu.%06zu:qmp_exit_job_complete " "%s %d" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , result, succeeded);
#line 287 "trace/trace-qapi_commands_job_trace_events.h"
        } else {
#line 10 "qapi/qapi-commands-job.trace-events"
            qemu_log("qmp_exit_job_complete " "%s %d" "\n", result, succeeded);
#line 291 "trace/trace-qapi_commands_job_trace_events.h"
        }
    }
}

static inline void trace_qmp_exit_job_complete(const char * result, bool succeeded)
{
    if (true) {
        _nocheck__trace_qmp_exit_job_complete(result, succeeded);
    }
}

#define TRACE_QMP_ENTER_JOB_DISMISS_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_QMP_ENTER_JOB_DISMISS) || \
    false)

static inline void _nocheck__trace_qmp_enter_job_dismiss(const char * json)
{
    if (trace_event_get_state(TRACE_QMP_ENTER_JOB_DISMISS) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 11 "qapi/qapi-commands-job.trace-events"
            qemu_log("%d@%zu.%06zu:qmp_enter_job_dismiss " "%s" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , json);
#line 318 "trace/trace-qapi_commands_job_trace_events.h"
        } else {
#line 11 "qapi/qapi-commands-job.trace-events"
            qemu_log("qmp_enter_job_dismiss " "%s" "\n", json);
#line 322 "trace/trace-qapi_commands_job_trace_events.h"
        }
    }
}

static inline void trace_qmp_enter_job_dismiss(const char * json)
{
    if (true) {
        _nocheck__trace_qmp_enter_job_dismiss(json);
    }
}

#define TRACE_QMP_EXIT_JOB_DISMISS_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_QMP_EXIT_JOB_DISMISS) || \
    false)

static inline void _nocheck__trace_qmp_exit_job_dismiss(const char * result, bool succeeded)
{
    if (trace_event_get_state(TRACE_QMP_EXIT_JOB_DISMISS) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 12 "qapi/qapi-commands-job.trace-events"
            qemu_log("%d@%zu.%06zu:qmp_exit_job_dismiss " "%s %d" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , result, succeeded);
#line 349 "trace/trace-qapi_commands_job_trace_events.h"
        } else {
#line 12 "qapi/qapi-commands-job.trace-events"
            qemu_log("qmp_exit_job_dismiss " "%s %d" "\n", result, succeeded);
#line 353 "trace/trace-qapi_commands_job_trace_events.h"
        }
    }
}

static inline void trace_qmp_exit_job_dismiss(const char * result, bool succeeded)
{
    if (true) {
        _nocheck__trace_qmp_exit_job_dismiss(result, succeeded);
    }
}

#define TRACE_QMP_ENTER_JOB_FINALIZE_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_QMP_ENTER_JOB_FINALIZE) || \
    false)

static inline void _nocheck__trace_qmp_enter_job_finalize(const char * json)
{
    if (trace_event_get_state(TRACE_QMP_ENTER_JOB_FINALIZE) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 13 "qapi/qapi-commands-job.trace-events"
            qemu_log("%d@%zu.%06zu:qmp_enter_job_finalize " "%s" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , json);
#line 380 "trace/trace-qapi_commands_job_trace_events.h"
        } else {
#line 13 "qapi/qapi-commands-job.trace-events"
            qemu_log("qmp_enter_job_finalize " "%s" "\n", json);
#line 384 "trace/trace-qapi_commands_job_trace_events.h"
        }
    }
}

static inline void trace_qmp_enter_job_finalize(const char * json)
{
    if (true) {
        _nocheck__trace_qmp_enter_job_finalize(json);
    }
}

#define TRACE_QMP_EXIT_JOB_FINALIZE_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_QMP_EXIT_JOB_FINALIZE) || \
    false)

static inline void _nocheck__trace_qmp_exit_job_finalize(const char * result, bool succeeded)
{
    if (trace_event_get_state(TRACE_QMP_EXIT_JOB_FINALIZE) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 14 "qapi/qapi-commands-job.trace-events"
            qemu_log("%d@%zu.%06zu:qmp_exit_job_finalize " "%s %d" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , result, succeeded);
#line 411 "trace/trace-qapi_commands_job_trace_events.h"
        } else {
#line 14 "qapi/qapi-commands-job.trace-events"
            qemu_log("qmp_exit_job_finalize " "%s %d" "\n", result, succeeded);
#line 415 "trace/trace-qapi_commands_job_trace_events.h"
        }
    }
}

static inline void trace_qmp_exit_job_finalize(const char * result, bool succeeded)
{
    if (true) {
        _nocheck__trace_qmp_exit_job_finalize(result, succeeded);
    }
}

#define TRACE_QMP_ENTER_QUERY_JOBS_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_QMP_ENTER_QUERY_JOBS) || \
    false)

static inline void _nocheck__trace_qmp_enter_query_jobs(const char * json)
{
    if (trace_event_get_state(TRACE_QMP_ENTER_QUERY_JOBS) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 15 "qapi/qapi-commands-job.trace-events"
            qemu_log("%d@%zu.%06zu:qmp_enter_query_jobs " "%s" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , json);
#line 442 "trace/trace-qapi_commands_job_trace_events.h"
        } else {
#line 15 "qapi/qapi-commands-job.trace-events"
            qemu_log("qmp_enter_query_jobs " "%s" "\n", json);
#line 446 "trace/trace-qapi_commands_job_trace_events.h"
        }
    }
}

static inline void trace_qmp_enter_query_jobs(const char * json)
{
    if (true) {
        _nocheck__trace_qmp_enter_query_jobs(json);
    }
}

#define TRACE_QMP_EXIT_QUERY_JOBS_BACKEND_DSTATE() ( \
    trace_event_get_state_dynamic_by_id(TRACE_QMP_EXIT_QUERY_JOBS) || \
    false)

static inline void _nocheck__trace_qmp_exit_query_jobs(const char * result, bool succeeded)
{
    if (trace_event_get_state(TRACE_QMP_EXIT_QUERY_JOBS) && qemu_loglevel_mask(LOG_TRACE)) {
        if (message_with_timestamp) {
            struct timeval _now;
            gettimeofday(&_now, NULL);
#line 16 "qapi/qapi-commands-job.trace-events"
            qemu_log("%d@%zu.%06zu:qmp_exit_query_jobs " "%s %d" "\n",
                     qemu_get_thread_id(),
                     (size_t)_now.tv_sec, (size_t)_now.tv_usec
                     , result, succeeded);
#line 473 "trace/trace-qapi_commands_job_trace_events.h"
        } else {
#line 16 "qapi/qapi-commands-job.trace-events"
            qemu_log("qmp_exit_query_jobs " "%s %d" "\n", result, succeeded);
#line 477 "trace/trace-qapi_commands_job_trace_events.h"
        }
    }
}

static inline void trace_qmp_exit_query_jobs(const char * result, bool succeeded)
{
    if (true) {
        _nocheck__trace_qmp_exit_query_jobs(result, succeeded);
    }
}
#endif /* TRACE_QAPI_COMMANDS_JOB_TRACE_EVENTS_GENERATED_TRACERS_H */