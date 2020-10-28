#include "opt-sched/Scheduler/enumerator.h"
#include "opt-sched/Scheduler/enum_parallel_master.h"
#include "opt-sched/Scheduler/logger.h"

using namespace llvm::opt_sched;

// should I use constructor or allocEnumrtr???


EnumParallelMaster::EnumParallelMaster(DataDepGraph *dataDepGraph, MachineModel *machMdl, 
    InstCount schedUprBound, int16_t sigHashSize, SchedPriorities prirts, 
    Pruning PruningStrategy, bool SchedForRPOnly, bool enblStallEnum, Milliseconds timeout,
    SPILL_COST_FUNCTION spillCostFunc, InstCount preFxdInstCnt, SchedInstruction *preFxdInsts[],
    int numThreads) 
    : LengthCostEnumerator(dataDepGraph,  machMdl,  schedUprBound, sigHashSize,  prirts,  
        PruningStrategy, SchedForRPOnly,  enblStallEnum,  timeout, spillCostFunc,  
        preFxdInstCnt, preFxdInsts) {

    NumThreads_ = numThreads;
    Logger::Info("Is constructed: %d", isCnstrctd_);
}