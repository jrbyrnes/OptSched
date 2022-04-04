#ifndef OPT_SCHED_REG
#define OPT_SCHED_REG

#include "llvm/CodeGen/MachineScheduler.h"
#include "GCNOptSched.h"

using namespace llvm;

namespace llvm {
namespace opt_sched {

// Create OptSched ScheduleDAG.
static ScheduleDAGInstrs *createOptSchedGCN(MachineSchedContext *C) {
  ScheduleDAGMILive *DAG = new ScheduleDAGOptSchedGCN(
      C, std::make_unique<GCNMaxOccupancySchedStrategy>(C));
  //DAG->addMutation(createLoadClusterDAGMutation(DAG->TII, DAG->TRI));
  //DAG->addMutation(createAMDGPUMacroFusionDAGMutation());
  //DAG->addMutation(createAMDGPUExportClusteringDAGMutation());
  return DAG;
}

// Register the machine scheduler.
static MachineSchedRegistry OptSchedGCNTargetRegistry("amdgcn",
                                                 createOptSchedGCNTarget);

static MachineSchedRegistry OptSchedGCNHSATargetRegistry("amdgcn-amd-amdhsa",
                                                    createOptSchedGCNTarget);

}
}