#ifndef OPTSCHED_MACROS_H
#define OPTSCHED_MACROS_H

#include "opt-sched/Scheduler/logger.h"

#ifdef DEBUG_BESTFS
#define BESTFS_LOG(someString, ...) Logger::Info(someString, __VA_ARGS__)
#endif
#ifndef DEBUG_BESTFS
#define BESTFS_LOG(someString, ...) (void *)0
#endif

#endif
