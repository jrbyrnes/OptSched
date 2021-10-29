#ifndef OPTSCHED_MACROS_H
#define OPTSCHED_MACROS_H

#include "opt-sched/Scheduler/logger.h"

// /#define DEBUG_BESTFS

#ifdef DEBUG_BESTFS
#define BESTFS_LOG(someString, ...) Logger::Info(someString, ##__VA_ARGS__)
#endif
#ifndef DEBUG_BESTFS
#define BESTFS_LOG(someString, ...) (void *)0
#endif


  #ifndef IS_DEBUG_SEARCH_ORDER2
    #define IS_DEBUG_SEARCH_ORDER2
  #endif

  //#ifndef IS_CORRECT_LOCALPOOL
  //  #define IS_CORRECT_LOCALPOOL
  //#endif

  //#ifndef IS_DEBUG_SEARCH_ORDER
  //  #define IS_DEBUG_SEARCH_ORDER
  //#endif

  //#ifndef DEBUG_GP_HISTORY
  //  #define DEBUG_GP_HISTORY
  //#endif

  //ifndef WORK_STEAL
  //  #define WORK_STEAL
  //#endif

  //#ifndef INSERT_ON_BACKTRACK
  //  #define INSERT_ON_BACKTRACK
  //#endif

  //#ifndef INSERT_ON_STEPFRWRD
  //  #define INSERT_ON_STEPFRWRD
  //#endif

  //#ifndef IS_DEBUG_METADATA
  //  #define IS_DEBUG_METADATA
  //#endif

  //#ifndef IS_SYNCH_ALLOC
  //  #define IS_SYNCH_ALLOC
  //#endif



#endif
