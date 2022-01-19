add_llvm_target(
  OptSched Scheduler/aco.cpp
  Scheduler/bb_thread.cpp
  Scheduler/buffers.cpp
  Scheduler/config.cpp
  Scheduler/data_dep.cpp
  Scheduler/enumerator.cpp
  Scheduler/gen_sched.cpp
  Scheduler/graph.cpp
  Scheduler/graph_trans.cpp
  Scheduler/hist_table.cpp
  Scheduler/list_sched.cpp
  Scheduler/logger.cpp
  Scheduler/reg_alloc.cpp
  Scheduler/utilities.cpp
  Scheduler/machine_model.cpp
  Scheduler/random.cpp
  Scheduler/ready_list.cpp
  Scheduler/register.cpp
  Scheduler/relaxed_sched.cpp
  Scheduler/sched_basic_data.cpp
  Scheduler/sched_region.cpp
  Scheduler/stats.cpp
  Wrapper/OptimizingScheduler.cpp
  Wrapper/OptSchedMachineWrapper.cpp
  Wrapper/OptSchedDDGWrapperBasic.cpp
  Wrapper/OptSchedGenericTarget.cpp

  STATIC
)

#set(OPTSCHED_EXTRA_DEFINITIONS "-DINSERT_ON_STEPFRWRD")

target_compile_definitions(LLVMOptSched PRIVATE INSERT_ON_STEPFRWRD)
target_include_directories(LLVMOptSched PRIVATE ${OPTSCHED_TARGET_DEPS})


#add_definitions(${OPTSCHED_EXTRA_DEFINITIONS})

add_compile_options(-g)

