/****************************************************************************
 *
 *   Copyright (c) 2012 - 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file drv_hrt.cpp
 *
 * High-resolution timer with callouts and timekeeping.
 */

#include <px4_platform_common/time.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/workqueue.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/atomic.h>
#include <drivers/drv_hrt.h>

#include <semaphore.h>
#include <time.h>
#include <string.h>
#include <errno.h>

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
#include <lockstep_scheduler/lockstep_scheduler.h>
#endif

// Intervals in usec
static constexpr unsigned HRT_INTERVAL_MIN = 1;
static constexpr unsigned HRT_INTERVAL_MAX = 50000000;

/*
 * Queue of callout entries.
 */
static struct sq_queue_s	callout_queue;

/* latency baseline (last compare value applied) */
static uint64_t			latency_baseline;

/* timer count at interrupt (for latency purposes) */
static uint64_t			latency_actual;

/* latency histogram */
const uint16_t latency_bucket_count = LATENCY_BUCKET_COUNT;
const uint16_t latency_buckets[LATENCY_BUCKET_COUNT] = { 1, 2, 5, 10, 20, 50, 100, 1000 };
__EXPORT uint32_t latency_counters[LATENCY_BUCKET_COUNT + 1];

static px4_sem_t g_hrt_lock;

static hrt_abstime px4_timestart_monotonic = 0;

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
static LockstepScheduler *lockstep_scheduler = new LockstepScheduler();
#endif

#ifdef CONFIG_DEBUG_HRT
#  define hrtinfo PX4_INFO
#else
#  define hrtinfo(x...)
#endif

static void hrt_latency_update();

hrt_abstime hrt_absolute_time_offset()
{
	return px4_timestart_monotonic;
}

static px4_sem_t g_hrt_work_lock;
static volatile pid_t g_hrt_work_pid = -1;

px4::atomic<hrt_abstime> g_hrt_deadline{0}; // Delay until work performed

/* callout list manipulation */
static void		hrt_call_internal(struct hrt_call *entry,
		hrt_abstime deadline,
		hrt_abstime interval,
		hrt_callout callout,
		void *arg);
static void		hrt_call_enter(struct hrt_call *entry);
static void		hrt_call_reschedule(void);
static void		hrt_call_invoke(void);


static void hrt_lock()
{
	// loop as the wait may be interrupted by a signal
	do {} while (px4_sem_wait(&g_hrt_lock) != 0);
}

static void hrt_unlock()
{
	px4_sem_post(&g_hrt_lock);
}

static int work_hrtthread(int argc, char *argv[])
{
	// set the threads name
#ifdef __PX4_DARWIN
	pthread_setname_np("HRT");
#else
	// The Linux headers do not actually contain this
	//rv = pthread_setname_np(pthread_self(), "HRT");
#endif

	// Loop forever
	for (;;) {
		hrt_lock();

		uint64_t deadline = g_hrt_deadline.load(); // update

		if (deadline != 0) {
			const hrt_abstime time_now_us = hrt_absolute_time();

			if (time_now_us > deadline) {
				PX4_WARN("work_hrtthread running at %lu (next deadline %lu) %lu late", time_now_us, deadline, time_now_us - deadline);
			}

			if (time_now_us >= deadline) {

				/* grab the timer for latency tracking purposes */
				latency_actual = time_now_us;

				/* do latency calculations */
				hrt_latency_update();

				/* run any callouts that have met their deadline */
				hrt_unlock();
				hrt_call_invoke();
				hrt_lock(); // re-lock

				hrt_call_reschedule();
			}
		}

		// Default to sleeping for 100 milliseconds
		int64_t next = 100000;

		if (g_hrt_deadline.load() != 0) {

			int64_t delay = g_hrt_deadline.load() - hrt_absolute_time();

			if (delay < next) {
				next = delay;
			}
		}

		hrt_unlock();

		if (next > 0) {
			// Get the current time
			struct timespec ts;
			// Note, we can't actually use CLOCK_MONOTONIC on macOS
			// but that's hidden and implemented in px4_clock_gettime.
			px4_clock_gettime(CLOCK_MONOTONIC, &ts);

			// Calculate an absolute time in the future
			const unsigned billion = (1000 * 1000 * 1000);
			uint64_t nsecs = ts.tv_nsec + (next * 1000);
			ts.tv_sec += nsecs / billion;
			nsecs -= (nsecs / billion) * billion;
			ts.tv_nsec = nsecs;

			//PX4_INFO("sem timed wait for %lu", next);

			px4_sem_timedwait(&g_hrt_work_lock, &ts);
		}
	}

	return PX4_OK; /* To keep some compilers happy */
}

#if defined(__PX4_APPLE_LEGACY)
#include <sys/time.h>

int px4_clock_gettime(clockid_t clk_id, struct timespec *tp)
{
	struct timeval now;
	int rv = gettimeofday(&now, nullptr);

	if (rv) {
		return rv;
	}

	tp->tv_sec = now.tv_sec;
	tp->tv_nsec = now.tv_usec * 1000;

	return 0;
}

int px4_clock_settime(clockid_t clk_id, struct timespec *tp)
{
	/* do nothing right now */
	return 0;
}
#endif

/**
 * Fetch a never-wrapping absolute time value in microseconds from
 * some arbitrary epoch shortly after system start.
 */
hrt_abstime
hrt_absolute_time(void)
{
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	// optimized case (avoid ts_to_abstime) if lockstep scheduler is used
	const uint64_t abstime = lockstep_scheduler->get_absolute_time();
	return abstime - px4_timestart_monotonic;
#else // defined(ENABLE_LOCKSTEP_SCHEDULER)
	struct timespec ts;
	px4_clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts_to_abstime(&ts);
#endif // defined(ENABLE_LOCKSTEP_SCHEDULER)
}

/**
 * Store the absolute time in an interrupt-safe fashion
 */
void
hrt_store_absolute_time(volatile hrt_abstime *t)
{
	*t = hrt_absolute_time();
}

/**
 * Initialise the high-resolution timing module.
 */
void
hrt_init(void)
{
	if (g_hrt_work_pid == -1) {
		sq_init(&callout_queue);

		int sem_ret = px4_sem_init(&g_hrt_lock, 0, 1);

		if (sem_ret != 0) {
			PX4_ERR("SEM INIT FAIL: %s", strerror(errno));
		}

		px4_sem_init(&g_hrt_work_lock, 0, 0);
		px4_sem_setprotocol(&g_hrt_work_lock, SEM_PRIO_NONE);

		// Create high priority worker thread
		g_hrt_work_pid = px4_task_spawn_cmd("wkr_hrt",
						    SCHED_DEFAULT,
						    SCHED_PRIORITY_MAX,
						    2000,
						    work_hrtthread,
						    (char *const *)NULL);

	} else {
		PX4_ERR("HRT already initialized");
	}
}

/**
 * Call callout(arg) after interval has elapsed.
 */
void
hrt_call_after(struct hrt_call *entry, hrt_abstime delay, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry,
			  hrt_absolute_time() + delay,
			  0,
			  callout,
			  arg);
}

/**
 * Call callout(arg) at calltime.
 */
void
hrt_call_at(struct hrt_call *entry, hrt_abstime calltime, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry, calltime, 0, callout, arg);
}

/**
 * Call callout(arg) every period.
 */
void
hrt_call_every(struct hrt_call *entry, hrt_abstime delay, hrt_abstime interval, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry,
			  hrt_absolute_time() + delay,
			  interval,
			  callout,
			  arg);
}

static void
hrt_call_internal(struct hrt_call *entry, hrt_abstime deadline, hrt_abstime interval, hrt_callout callout, void *arg)
{
	hrt_lock();

	/* if the entry is currently queued, remove it */
	/* note that we are using a potentially uninitialised
	   entry->link here, but it is safe as sq_rem() doesn't
	   dereference the passed node unless it is found in the
	   list. So we potentially waste a bit of time searching the
	   queue for the uninitialised entry->link but we don't do
	   anything actually unsafe.
	*/
	if (entry->deadline != 0) {
		sq_rem(&entry->link, &callout_queue);
	}

	entry->deadline = deadline;
	entry->period = interval;
	entry->callout = callout;
	entry->arg = arg;

	hrt_call_enter(entry);

	hrt_unlock();
}

/**
 * If this returns true, the call has been invoked and removed from the callout list.
 *
 * Always returns false for repeating callouts.
 */
bool
hrt_called(struct hrt_call *entry)
{
	return (entry->deadline == 0);
}

/**
 * Remove the entry from the callout list.
 */
void
hrt_cancel(struct hrt_call *entry)
{
	hrt_lock();

	sq_rem(&entry->link, &callout_queue);
	entry->deadline = 0;

	/* if this is a periodic call being removed by the callout, prevent it from
	 * being re-entered when the callout returns.
	 */
	entry->period = 0;

	hrt_unlock();
}

static void
hrt_call_enter(struct hrt_call *entry)
{
	struct hrt_call	*call, *next;

	call = (struct hrt_call *)sq_peek(&callout_queue);

	if ((call == nullptr) || (entry->deadline < call->deadline)) {
		sq_addfirst(&entry->link, &callout_queue);
		hrtinfo("call enter at head, reschedule\n");
		/* we changed the next deadline, reschedule the timer event */
		hrt_call_reschedule();

	} else {
		do {
			next = (struct hrt_call *)sq_next(&call->link);

			if ((next == nullptr) || (entry->deadline < next->deadline)) {
				hrtinfo("call enter after head\n");
				sq_addafter(&call->link, &entry->link, &callout_queue);
				break;
			}
		} while ((call = next) != nullptr);
	}

	hrtinfo("scheduled\n");
}

static void
hrt_call_invoke(void)
{
	struct hrt_call	*call;
	hrt_abstime deadline;

	hrt_lock();

	while (true) {
		/* get the current time */
		hrt_abstime now = hrt_absolute_time();

		call = (struct hrt_call *)sq_peek(&callout_queue);

		if (call == nullptr) {
			break;
		}

		if (call->deadline > now) {
			break;
		}

		sq_rem(&call->link, &callout_queue);
		hrtinfo("call pop\n");

		/* save the intended deadline for periodic calls */
		deadline = call->deadline;

		if (now > deadline) {
			PX4_ERR("call pop now: %lu, deadline: %lu, late: %lu", now, deadline, now - deadline);
		}

		/* zero the deadline, as the call has occurred */
		call->deadline = 0;

		/* invoke the callout (if there is one) */
		if (call->callout) {
			hrtinfo("call %p: %p(%p)\n", call, call->callout, call->arg);
			hrt_unlock();

			call->callout(call->arg);

			hrt_lock();
		}

		/* if the callout has a non-zero period, it has to be re-entered */
		if (call->period != 0) {
			// re-check call->deadline to allow for
			// callouts to re-schedule themselves
			// using hrt_call_delay()
			if (call->deadline <= now) {
				call->deadline = deadline + call->period;
			}

			hrt_call_enter(call);
		}
	}

	hrt_unlock();
}

/**
 * Reschedule the next timer interrupt.
 *
 * This routine must be called with interrupts disabled.
 */
static void
hrt_call_reschedule()
{
	hrt_abstime	now = hrt_absolute_time();
	struct hrt_call	*next = (struct hrt_call *)sq_peek(&callout_queue);
	hrt_abstime	deadline = now + HRT_INTERVAL_MAX;

	/*
	 * Determine what the next deadline will be.
	 *
	 * Note that we ensure that this will be within the counter
	 * period, so that when we truncate all but the low 16 bits
	 * the next time the compare matches it will be the deadline
	 * we want.
	 *
	 * It is important for accurate timekeeping that the compare
	 * interrupt fires sufficiently often that the base_time update in
	 * hrt_absolute_time runs at least once per timer period.
	 */
	if (next != nullptr) {
		hrtinfo("entry in queue\n");

		if (next->deadline <= (now + HRT_INTERVAL_MIN)) {
			PX4_INFO("pre-expired\n");
			/* set a minimal deadline so that we call ASAP */
			deadline = now + HRT_INTERVAL_MIN;

		} else if (next->deadline < deadline) {
			hrtinfo("due soon\n");
			deadline = next->deadline;
		}
	}

	hrtinfo("schedule for %u at %u\n", (unsigned)(deadline & 0xffffffff), (unsigned)(now & 0xffffffff));

	/* set the new compare value and remember it for latency tracking */
	latency_baseline = deadline;

	// Delay until work performed
	g_hrt_deadline.store(deadline);

	// signal worker thread, only need to wake up if called from a different thread
	if (px4_getpid() != g_hrt_work_pid) {
		int sem_val;

		if (px4_sem_getvalue(&g_hrt_work_lock, &sem_val) == 0 && sem_val <= 0) {
			px4_sem_post(&g_hrt_work_lock);
		}
	}
}

static void
hrt_latency_update(void)
{
	uint16_t latency = latency_actual - latency_baseline;
	unsigned	index;

	/* bounded buckets */
	for (index = 0; index < LATENCY_BUCKET_COUNT; index++) {
		if (latency <= latency_buckets[index]) {
			latency_counters[index]++;
			return;
		}
	}

	/* catch-all at the end */
	latency_counters[index]++;
}

void
hrt_call_init(struct hrt_call *entry)
{
	memset(entry, 0, sizeof(*entry));
}

void
hrt_call_delay(struct hrt_call *entry, hrt_abstime delay)
{
	entry->deadline = hrt_absolute_time() + delay;
}

int px4_clock_gettime(clockid_t clk_id, struct timespec *tp)
{
	if (clk_id == CLOCK_MONOTONIC) {
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
		const uint64_t abstime = lockstep_scheduler->get_absolute_time();
		abstime_to_ts(tp, abstime - px4_timestart_monotonic);
		return 0;
#else // defined(ENABLE_LOCKSTEP_SCHEDULER)
#if defined(__PX4_DARWIN)
		// We don't have CLOCK_MONOTONIC on macOS, so we just have to
		// resort back to CLOCK_REALTIME here.
		return system_clock_gettime(CLOCK_REALTIME, tp);
#else // defined(__PX4_DARWIN)
		return system_clock_gettime(clk_id, tp);
#endif // defined(__PX4_DARWIN)
#endif // defined(ENABLE_LOCKSTEP_SCHEDULER)

	} else {
		return system_clock_gettime(clk_id, tp);
	}
}

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
int px4_clock_settime(clockid_t clk_id, const struct timespec *ts)
{
	if (clk_id == CLOCK_REALTIME) {
		return system_clock_settime(clk_id, ts);

	} else {
		const uint64_t time_us = ts_to_abstime(ts);

		if (px4_timestart_monotonic == 0) {
			px4_timestart_monotonic = time_us;
		}

		lockstep_scheduler->set_absolute_time(time_us);
		return 0;
	}
}


int px4_usleep(useconds_t usec)
{
	if (px4_timestart_monotonic == 0) {
		// Until the time is set by the simulator, we fallback to the normal
		// usleep;
		return system_usleep(usec);
	}

	const uint64_t time_finished = lockstep_scheduler->get_absolute_time() + usec;

	return lockstep_scheduler->usleep_until(time_finished);
}

unsigned int px4_sleep(unsigned int seconds)
{
	if (px4_timestart_monotonic == 0) {
		// Until the time is set by the simulator, we fallback to the normal
		// sleep;
		return system_sleep(seconds);
	}

	const uint64_t time_finished = lockstep_scheduler->get_absolute_time() +
				       ((uint64_t)seconds * 1000000);

	return lockstep_scheduler->usleep_until(time_finished);
}

int px4_pthread_cond_timedwait(pthread_cond_t *cond,
			       pthread_mutex_t *mutex,
			       const struct timespec *ts)
{
	const uint64_t time_us = ts_to_abstime(ts);
	const uint64_t scheduled = time_us + px4_timestart_monotonic;
	return lockstep_scheduler->cond_timedwait(cond, mutex, scheduled);
}

int px4_lockstep_register_component()
{
	return lockstep_scheduler->components().register_component();
}

void px4_lockstep_unregister_component(int component)
{
	lockstep_scheduler->components().unregister_component(component);
}

void px4_lockstep_progress(int component)
{
	lockstep_scheduler->components().lockstep_progress(component);
}

void px4_lockstep_wait_for_components()
{
	lockstep_scheduler->components().wait_for_components();
}
#endif
