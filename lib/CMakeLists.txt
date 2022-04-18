set(AMDGPU_TARGETS "gfx906" CACHE STRING "AMD GPU targets to compile for" FORCE)
find_package(hip REQUIRED CONFIG PATHS /opt/rocm-4.5.2/)
find_package(hiprand REQUIRED CONFIG PATHS /opt/rocm/4.5.2 REQUIRED)

add_llvm_target(
  OptSched Scheduler/aco.cpp
  Scheduler/bb_spill.cpp
  Scheduler/buffers.cpp
  Scheduler/config.cpp
  Scheduler/data_dep.cpp
  Scheduler/enumerator.cpp
  Scheduler/gen_sched.cpp
  Scheduler/graph.cpp
  Scheduler/graph_trans.cpp
  Scheduler/graph_trans_ilp.cpp
  Scheduler/graph_trans_ilp_occupancy_preserving.cpp
  Scheduler/HIPcode.cpp
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
  Wrapper/AMDGPU/GCNOptSched.cpp
  Wrapper/AMDGPU/OptSchedGCNTarget.cpp
  Wrapper/AMDGPU/OptSchedDDGWrapperGCN.cpp

  STATIC
)

#set(OPTSCHED_EXTRA_DEFINITIONS "-DINSERT_ON_STEPFRWRD")

#add_dependencies(OptSched ${OPTSCHED_TARGET_DEPS})

target_compile_definitions(LLVMOptSched PRIVATE INSERT_ON_STEPFRWRD)
target_include_directories(LLVMOptSched PRIVATE ${OPTSCHED_TARGET_DEPS})
target_compile_options(LLVMOptSched PRIVATE -Wno-suggest-override)

target_include_directories(LLVMOptSched PRIVATE "/opt/rocm-4.5.2/include")
target_include_directories(LLVMOptSched PRIVATE "/opt/rocm-4.5.2/hiprand/include" "/opt/rocm-4.5.2/rocrand/include")

target_link_libraries (LLVMOptSched hip::device)

#add_definitions(${OPTSCHED_EXTRA_DEFINITIONS})
