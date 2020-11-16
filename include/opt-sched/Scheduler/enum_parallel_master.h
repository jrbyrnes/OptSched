/******************************************
Description: Class for parallel enumeration
********************************************/
#ifndef ENUM_PARALLEL_MASTER_H
#define ENUM_PARALLEL_MASTER_H

#include "opt-sched/Scheduler/enumerator.h"

namespace llvm {
namespace opt_sched {


class EnumThread : public ConstrainedScheduler {    
protected:
    bool isCnstrctd_;

    Pruning prune_;
    bool SchedForRPOnly_;
    bool enblStallEnum_;
    InstCount prevTrgtLngth_;

    int memAllocBlkSize_;
    BinHashTable<HistEnumTreeNode> *exmndSubProbs_; // history table
    void SetInstSigs_(); // how do we use this?
    
    // "master level" counters, each thread will also have these
    uint64_t maxNodeCnt_;
    uint64_t createdNodeCnt_;
    uint64_t exmndNodeCnt_;
    long backTrackCnt_;
    int fsblSchedCnt_;
    int imprvmntCnt_;

    SPILL_COST_FUNCTION spillCostFunc_;


    // not sure if we need the following in the master thread
    RJ_RelaxedScheduler *rlxdSchdulr_;
    bool isEarlySubProbDom_;
    InstCount neededSlots_[MAX_ISSUTYPE_CNT];
    InstCount fxdInstCnt_;
    InstCount minUnschduldTplgclOrdr_;
    LinkedList<SchedInstruction> *tightndLst_;
    LinkedList<SchedInstruction> *bkwrdTightndLst_;
    LinkedList<SchedInstruction> *dirctTightndLst_;
    LinkedList<SchedInstruction> *fxdLst_;
    InstCount *tmpLwrBounds_;
    int iterNum_;
    InstCount preFxdInstCnt_;
    SchedInstruction **preFxdInsts_;


    inline bool IsHistDom();
    


    // Initiliaze the components initialized by enumetarator constructor
    void initialize(DataDepGraph *dataDepGraph, MachineModel *machMdl, 
    InstCount schedUprBound, int16_t sigHashSize, SchedPriorities prirts, 
    Pruning PruningStrategy, bool SchedForRPOnly, bool enblStallEnum, Milliseconds timeout,
    SPILL_COST_FUNCTION spillCostFunc, InstCount preFxdInstCnt, SchedInstruction *preFxdInsts[]);

public: 
    EnumThread(DataDepGraph *dataDepGraph, MachineModel *machMdl,
        InstCount schedUprBound, int16_t sigHashSize, SchedPriorities prirts,
        Pruning PruningStrategy, bool SchedForRPOnly, bool enblStallEnum, Milliseconds timeout,
        SPILL_COST_FUNCTION spillCostFunc, InstCount preFxdInstCnt,
        SchedInstruction *preFxdInsts[]);
    
};


bool EnumThread::IsHistDom() { return prune_.histDom; }





class EnumWorker : public EnumThread {
protected:
    /* LENGTH COST STUFF */
    int costChkCnt_;
    int costPruneCnt_;
    int costLwrBound_;

    bool WasObjctvMet_();
    bool BackTrack_();
    InstCount GetBestCost_();
    void CreateRootNode_();

    // Check if branching from the current node by scheduling this instruction
    // in the current slot is feasible or not
    bool ProbeBranch_(SchedInstruction *inst, EnumTreeNode *&newNode,
                    bool &isNodeDmntd, bool &isRlxInfsbl, bool &isLngthFsbl);
    bool Initialize_(InstSchedule *preSched, InstCount trgtLngth);
    bool ChkCostFsblty_(SchedInstruction *inst, EnumTreeNode *&newNode);
    bool EnumStall_();
    void InitNewNode_(EnumTreeNode *newNode);

    /* END LENGTH COST STUFF */

    /*
    void initialize(DataDepGraph *dataDepGraph, MachineModel *machMdl, 
    InstCount schedUprBound, int16_t sigHashSize, SchedPriorities prirts, 
    Pruning PruningStrategy, bool SchedForRPOnly, bool enblStallEnum, Milliseconds timeout,
    SPILL_COST_FUNCTION spillCostFunc, InstCount preFxdInstCnt, SchedInstruction *preFxdInsts[]);
    */

public: 
    EnumWorker(DataDepGraph *dataDepGraph, MachineModel *machMdl,
        InstCount schedUprBound, int16_t sigHashSize, SchedPriorities prirts,
        Pruning PruningStrategy, bool SchedForRPOnly, bool enblStallEnum, Milliseconds timeout,
        SPILL_COST_FUNCTION spillCostFunc, InstCount preFxdInstCnt,
        SchedInstruction *preFxdInsts[]);

};



class EnumParallelMaster : public EnumThread {
private:
    int NumThreads_;

    // History Table Allocation
    MemAlloc<CostHistEnumTreeNode> *histNodeAlctr_;
    bool alctrsSetup_;
    MemAlloc<BinHashTblEntry<HistEnumTreeNode>> *hashTblEntryAlctr_;
    EnumTreeNodeAlloc *nodeAlctr_;

    BitVector *bitVctr1_;
    BitVector *bitVctr2_;
    SchedInstruction **lastInsts_;
    SchedInstruction **othrLastInsts_;

    // functions to manage the allocators
    void setupAllocators_();
    void freeAllocators_();
    void resetAllocators_();
    
    void reset();

    HistEnumTreeNode *allocHistNode_(EnumTreeNode *node);
    HistEnumTreeNode *allocTempHistNode_(EnumTreeNode *node);
    void freeHistNode_(HistEnumTreeNode *histNode);

public: 
    EnumParallelMaster(DataDepGraph *dataDepGraph, MachineModel *machMdl,
        InstCount schedUprBound, int16_t sigHashSize, SchedPriorities prirts,
        Pruning PruningStrategy, bool SchedForRPOnly, bool enblStallEnum, Milliseconds timeout,
        SPILL_COST_FUNCTION spillCostFunc, InstCount preFxdInstCnt,
        SchedInstruction *preFxdInsts[], InstCount nThreads);
    
    // need to overwrite Constrained Scheduler's virtual
    FUNC_RESULT FindSchedule(InstSchedule *sched, SchedRegion *rgn) {
        return RES_ERROR;
    }

    void UpdtRdyLst_(InstCount cycleNum, int slotNum) {}
    bool ChkInstLglty_(SchedInstruction *inst) const {return false};

    

};



} // opt_sched namespace
} // llvm namespace

#endif