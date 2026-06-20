#pragma once

// Lightweight per-task timing recorder for measuring
//   WCET - Worst Case Execution Time  (pure CPU time of the task body)
//   WCRT - Worst Case Response Time   (wall-clock from release to completion)
//
// WCET uses CLOCK_THREAD_CPUTIME_ID: it only advances while this thread is
// actually on a CPU, so preemption by other tasks is excluded. Compare it
// against the CBS runtime budget (sched_runtime).
//
// WCRT uses CLOCK_MONOTONIC measured from the job's *intended* release time,
// so it includes scheduling latency, preemption and blocking on mutexes.
// Compare it against the relative deadline (sched_deadline).
//
// Caveat: if a task body spawns worker threads (e.g. OpenCV parallelises
// detect()), their CPU time runs on other threads and is NOT counted by
// CLOCK_THREAD_CPUTIME_ID, so the vision WCET may be underestimated.

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <time.h>

struct RtStats {
    const char *name = "";
    long long budgetNs = 0; // expected WCET bound  (CBS runtime)
    long long deadlineNs = 0; // expected WCRT bound  (relative deadline)

    uint64_t jobs = 0;
    uint64_t wcetMaxNs = 0; // worst execution time seen
    uint64_t wcrtMaxNs = 0; // worst response time seen
    uint64_t wcetSumNs = 0; // running totals, for the mean
    uint64_t wcrtSumNs = 0;
    uint64_t budgetMisses = 0; // jobs whose WCET exceeded the runtime budget
    uint64_t deadlineMisses = 0; // jobs whose WCRT exceeded the deadline
};

// Read a clock as nanoseconds. clk is CLOCK_MONOTONIC or CLOCK_THREAD_CPUTIME_ID.
static inline uint64_t nowNs(clockid_t clk) {
    timespec ts;
    clock_gettime(clk, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ull + ts.tv_nsec;
}

// Convert an absolute CLOCK_MONOTONIC timespec (e.g. a periodic task's intended
// wake-up time) to nanoseconds, so it can serve as the release timestamp.
static inline uint64_t timespecToNs(const timespec &t) {
    return static_cast<uint64_t>(t.tv_sec) * 1'000'000'000ull + t.tv_nsec;
}

// Record one job. Capture the four timestamps around the task body:
//   releaseNs  - intended release (CLOCK_MONOTONIC): periodic wakeup or signal arrival
//   cpuStartNs - CLOCK_THREAD_CPUTIME_ID just before the body
//   cpuEndNs   - CLOCK_THREAD_CPUTIME_ID just after the body
//   finishNs   - CLOCK_MONOTONIC just after the body
static inline void rtRecord(RtStats &s, uint64_t releaseNs, uint64_t cpuStartNs, uint64_t cpuEndNs, uint64_t finishNs) {
    uint64_t wcet = cpuEndNs - cpuStartNs;
    uint64_t wcrt = finishNs - releaseNs;
    ++s.jobs;
    s.wcetSumNs += wcet;
    s.wcrtSumNs += wcrt;
    s.wcetMaxNs = std::max(s.wcetMaxNs, wcet);
    s.wcrtMaxNs = std::max(s.wcrtMaxNs, wcrt);
    if (wcet > static_cast<uint64_t>(s.budgetNs))
        ++s.budgetMisses;
    if (wcrt > static_cast<uint64_t>(s.deadlineNs))
        ++s.deadlineMisses;
}

// Print a one-task summary (microseconds). Call after the threads have joined.
static inline void rtReport(const RtStats &s) {
    std::cerr << "[rt] " << s.name << ": ";
    if (s.jobs == 0) {
        std::cerr << "no samples\n";
        return;
    }
    std::cerr << s.jobs << " jobs\n"
              << "       WCET  max=" << s.wcetMaxNs << " ns"
              << "  mean=" << (s.wcetSumNs / s.jobs) << " ns"
              << "  budget=" << s.budgetNs << " ns"
              << "  overruns=" << s.budgetMisses << "\n"
              << "       WCRT  max=" << s.wcrtMaxNs << " ns"
              << "  mean=" << (s.wcrtSumNs / s.jobs) << " ns"
              << "  deadline=" << s.deadlineNs << " ns"
              << "  misses=" << s.deadlineMisses << "\n";
}
