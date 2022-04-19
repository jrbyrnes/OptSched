cmake_minimum_required(VERSION 3.8)

set(AMDGPU_TARGETS "gfx906" CACHE STRING "AMD GPU targets to compile for" FORCE)
find_package(HIP REQUIRED CONFIG PATHS /opt/rocm/ REQUIRED)
find_package(hiprand REQUIRED CONFIG PATHS /opt/rocm/ REQUIRED)

option(OPTSCHED_INCLUDE_TESTS "Generate build targets for the OptSched unit tests." ON)
set(OPTSCHED_ENABLE_AMDGPU "Build the AMDGPU code. Requires that the AMDGPU target is supported." ON)
set(OPTSCHED_LIT_ARGS "-sv" CACHE STRING "Arguments to pass to lit")
set(OPTSCHED_EXTRA_LINK_LIBRARIES "" CACHE STRING "Extra link_libraries to pass to OptSched, ;-separated")
set(OPTSCHED_EXTRA_INCLUDE_DIRS "" CACHE STRING "Extra include_directories to pass to OptSched, ;-separated")
# To add OptSched debug defines, e.g.:
# '-DOPTSCHED_EXTRA_DEFINITIONS=-DIS_DEBUG_DEFS_AND_USES;-DIS_DEBUG_DEF_USE_COUNT'
set(OPTSCHED_EXTRA_DEFINITIONS "" CACHE STRING "Extra add_definitions to pass to OptSched, ;-separated")

set(llvm_subproject TRUE)


# If we are requested to build the AMDGPU functionality, ensure that we can do so.
if(OPTSCHED_ENABLE_AMDGPU)
  if(NOT "AMDGPU" IN_LIST LLVM_ALL_TARGETS)
    message(FATAL_ERROR "Trying to build the AMDGPU code, but AMDGPU is not supported by this build of LLVM")
  elseif(LLVM_VERSION VERSION_LESS 7.0)
    message(FATAL_ERROR "OptSched requries LLVM version >= 7.0 to build the AMDGPU scheduler.")
  endif()
endif()

# If asserts are enabled opt-sched must be built with "IS_DEBUG".
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DIS_DEBUG")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(OPTSCHED_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(OPTSCHED_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})

list(APPEND CMAKE_MODULE_PATH "${LLVM_CMAKE_DIR}" ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(AddLLVM)

include_directories(
  ${OPTSCHED_SOURCE_DIR}/include
  ${OPTSCHED_SOURCE_DIR}/lib
  ${LLVM_INCLUDE_DIRS}
  ${OPTSCHED_EXTRA_INCLUDE_DIRS}
)
add_definitions(${OPTSCHED_EXTRA_DEFINITIONS})
link_directories(${OPTSCHED_EXTRA_LINK_LIBRARIES})



add_subdirectory(lib)
