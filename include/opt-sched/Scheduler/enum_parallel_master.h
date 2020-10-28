/******************************************
Description: Class for parallel enumeration
********************************************/
#ifndef ENUM_PARALLEL_MASTER_H
#define ENUM_PARALLEL_MASTER_H

#include "opt-sched/Scheduler/enumerator.h"

namespace llvm {
namespace opt_sched {

class EnumParallelMaster : public LengthCostEnumerator {

protected:
    int NumThreads_;

public: 
    EnumParallelMaster(DataDepGraph *dataDepGraph, MachineModel *machMdl, 
        InstCount schedUprBound, int16_t sigHashSize, SchedPriorities prirts,
        Pruning PruningStrategy, bool SchedForRPOnly, bool enblStallEnum, Milliseconds timeout,
        SPILL_COST_FUNCTION spillCostFunc, InstCount preFxdInstCnt,
        SchedInstruction *preFxdInsts[], int numThreads);
};

} // opt_sched namespace
} // llvm namespace

#endif