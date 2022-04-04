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

static MachineSchedRegistry 
    OptSchedGCNMIRegistry("gcn-optsched", "Use the GCN OptSched scheduler.", 
                       createOptSchedGCN); 


}
}

#endif
