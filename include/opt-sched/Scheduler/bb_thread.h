#ifndef BB_THREAD_H
#define BB_THREAD_H

#include "opt-sched/Scheduler/OptSchedTarget.h"
#include "opt-sched/Scheduler/defines.h"
#include "opt-sched/Scheduler/sched_region.h"
#include "llvm/ADT/SmallVector.h"
#include <map>
#include <set>
#include <vector>
#include <queue>
#include <thread>

namespace llvm {
namespace opt_sched {

class LengthCostEnumerator;
class EnumTreeNode;
class Register;
class RegisterFile;
class BitVector;


class BBThread {
private:
  // The target machine
  const OptSchedTarget *OST;

  int IssueRate;
  
  int EntryInstCnt_;
  int ExitInstCnt_;
  int NumberOfInsts_;


  SPILL_COST_FUNCTION SpillCostFunc_;

  // A bit vector indexed by register number indicating whether that
  // register is live
  WeightedBitVector *LiveRegs_;

  // A bit vector indexed by physical register number indicating whether
  // that physical register is live
  WeightedBitVector *LivePhysRegs_;

  // Sum of lengths of live ranges. This vector is indexed by register type,
  // and each type will have its sum of live interval lengths computed.
  std::vector<int> SumOfLiveIntervalLengths_;

  int SchduldEntryInstCnt_;
  int SchduldExitInstCnt_;
  int SchduldInstCnt_;

  InstCount *SpillCosts_;
  // Current register pressure for each register type.
  SmallVector<unsigned, 8> RegPressures_;
  InstCount *PeakRegPressures_;
  InstCount CrntStepNum_;
  InstCount PeakSpillCost_;
  InstCount TotSpillCost_;
  InstCount SlilSpillCost_;
  bool TrackLiveRangeLngths_;

  // TODO(max): Document.
  InstCount CrntCycleNum_;
  // TODO(max): Document.
  InstCount CrntSlotNum_;


  // BBWithSpill-specific Functions:
  InstCount CmputCostLwrBound_(InstCount schedLngth);
  void InitForCostCmputtn_();
  InstCount CmputDynmcCost_();

  void UpdateSpillInfoForSchdul_(SchedInstruction *inst, bool trackCnflcts);
  void UpdateSpillInfoForUnSchdul_(SchedInstruction *inst);
  void SetupPhysRegs_();
  void CmputCrntSpillCost_();
  void CmputCnflcts_(InstSchedule *sched);

public:
  BBThread(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
              long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
              SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
              bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
              bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc,
              SchedulerType HeurSchedType);
  virtual ~BBThread();

  // virtual
  /*virtual FUNC_RESULT Enumerate_(Milliseconds startTime, Milliseconds rgnTimeout,
                                 Milliseconds lngthTimeout);

  virtual Enumerator *allocEnumrtr_(Milliseconds timeout);
  */

  // non-virtual

  int CmputCostLwrBound();

  bool ChkCostFsbltyBBThread(InstCount trgtLngth, EnumTreeNode *treeNode);
  void SchdulInstBBThread(SchedInstruction *inst, InstCount cycleNum, InstCount slotNum,
                  bool trackCnflcts);
  void UnschdulInstBBThread(SchedInstruction *inst, InstCount cycleNum,
                    InstCount slotNum, EnumTreeNode *trgtNode);
  void SetSttcLwrBoundsBBThread(EnumTreeNode *node);
  bool ChkInstLgltyBBThread(SchedInstruction *inst);

protected:
  LengthCostEnumerator *Enumrtr_;
  InstCount CrntSpillCost_;
  InstCount OptmlSpillCost_;

  bool SchedForRPOnly_;

  bool EnblStallEnum_;
  
  bool VrfySched_;

  int SCW_;
  int SchedCostFactor_;

  InstCount MaxLatency_;
  bool SimpleMachineModel_;

  int16_t RegTypeCnt_;
  RegisterFile *RegFiles_;

  InstCount StaticSlilLowerBound_ = 0;
  InstCount DynamicSlilLowerBound_ = 0;

  // Needed to override SchedRegion virtuals
  InstCount CmputNormCostBBThread_(InstSchedule *sched, COST_COMP_MODE compMode,
                           InstCount &execCost, bool trackCnflcts);
  
  void InitForSchdulngBBThread();

  bool EnableEnumBBThread_();

  InstCount CmputCostBBThread_(InstSchedule *sched, COST_COMP_MODE compMode,
                       InstCount &execCost, bool trackCnflcts);
  
  void SetupForSchdulngBBThread_();
  void FinishOptmlBBThread_();

  bool ChkScheduleBBThread_(InstSchedule *bestSched, InstSchedule *lstSched);

  // Virtual Functions:
  virtual int GetCostLwrBoundBBThread() = 0;

  virtual InstCount GetBestCostBBThread() = 0;

  virtual InstCount UpdtOptmlSched(InstSchedule *crntSched,
                           LengthCostEnumerator *enumrtr) = 0;

  // (Chris)
  /*inline virtual const std::vector<int> &GetSLIL_() const {
    return SumOfLiveIntervalLengths_;
  }*/
};

/******************************************************************/

class BBInterfacer : public SchedRegion, public BBThread {
private:
    void CmputAbslutUprBound_();
    void CmputSchedUprBound_();
    InstCount CmputCostLwrBound();

    static InstCount ComputeSLILStaticLowerBound(int64_t regTypeCnt_,
              RegisterFile *regFiles_, DataDepGraph *dataDepGraph_);

    InstCount UpdtOptmlSched(InstSchedule *crntSched,
              LengthCostEnumerator *enumrtr);


public:
    BBInterfacer(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
              long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
              SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
              bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
              bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc,
              SchedulerType HeurSchedType);

    FUNC_RESULT Enumerate_(Milliseconds startTime, Milliseconds rgnTimeout,
                           Milliseconds lngthTimeout) override;

    Enumerator *AllocEnumrtr_(Milliseconds timeout) override;

};

/******************************************************************/
class BBWithSpill : public BBInterfacer {
private:
    // override BBThread virtual
    inline InstCount GetBestCostBBThread() {return GetBestCost();};

    // Override virtuals in sched_region
    inline InstCount CmputNormCost_(InstSchedule *sched, COST_COMP_MODE compMode,
                                   InstCount &execCost, bool trackCnflcts) 
    {
      return CmputNormCostBBThread_(sched, compMode, execCost, trackCnflcts);
    }
    inline InstCount CmputCost_(InstSchedule *sched, COST_COMP_MODE compMode,
                       InstCount &execCost, bool trackCnflcts)
    {
      return CmputCostBBThread_(sched, compMode, execCost, trackCnflcts);                  
    }

    inline bool ChkCostFsblty(InstCount trgtLngth, EnumTreeNode *treeNode)
    {
      return ChkCostFsbltyBBThread(trgtLngth, treeNode);
    }

    inline void InitForSchdulng() {return InitForSchdulngBBThread();}
    inline void SetupForSchdulng_() {return SetupForSchdulngBBThread_();}

    inline void SchdulInst(SchedInstruction *inst, InstCount cycleNum, 
                           InstCount slotNum, bool trackCnflcts)
    {
      return SchdulInstBBThread(inst, cycleNum, slotNum, trackCnflcts);
    }

    inline void UnschdulInst(SchedInstruction *inst, InstCount cycleNum,
                             InstCount slotNum, EnumTreeNode *trgtNode)
    {
      return UnschdulInstBBThread(inst, cycleNum, slotNum, trgtNode);
    }

    inline void SetSttcLwrBounds(EnumTreeNode *node)
    {
      return SetSttcLwrBoundsBBThread(node);
    }

    inline bool ChkInstLglty(SchedInstruction *inst)
    {
      return ChkInstLgltyBBThread(inst);
    }

    inline bool ChkSchedule_(InstSchedule *bestSched, InstSchedule *lstSched)
    {
      return ChkScheduleBBThread_(bestSched, lstSched);
    }

    inline bool EnableEnum_()
    {
      return EnableEnumBBThread_();
    }

    inline void FinishOptml_()
    {
      return FinishOptmlBBThread_();
    }

    int GetCostLwrBoundBBThread()
    {
      return GetCostLwrBound();
    }

public:
    BBWithSpill(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
                long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
                SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
                bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
                bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc,
                SchedulerType HeurSchedType);

};


/******************************************************************/
class BBWorker : public BBThread{
private:
    DataDepGraph *DataDepGraph_;
    MachineModel  *MachMdl_; 

    InstCount SchedUprBound_;   // set by master (using schedRegion interface)
    int16_t SigHashSize_;       // set by master (using schedRegion interface)

    Pruning PruningStrategy_;
    SchedPriorities EnumPrirts_;
    SPILL_COST_FUNCTION SpillCostFunc_;

    vector<int> TakenArr;
    vector<BBWorker> *local_pool = NULL;

    InstSchedule *enumCrntSched_;
    InstSchedule *enumBestSched_;

    void handlEnumrtrRslt_(FUNC_RESULT rslt, InstCount trgtLngth);

    // overrides
    inline InstCount GetBestCostBBThread() {return getBestCost();};

    int getCostLwrBound()
    {
      return 0;
    }

    InstCount getBestCost() {return 0;}

    inline int GetCostLwrBoundBBThread()
    {
      return getCostLwrBound();
    };

    InstCount BBWorker::UpdtOptmlSched(InstSchedule *crntSched,
                           LengthCostEnumerator *enumrtr)
    {
      return 0;
    };

public:
    BBWorker(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
              long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
              SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
              bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
              bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc,
              SchedulerType HeurSchedType, InstCount SchedUprBound, 
              int16_t SigHashSize);

    InstSchedule *allocSched();
    void allocEnumrtr_(Milliseconds timeout);
    inline void initForSchdulng() {return InitForSchdulngBBThread();}
    inline void scheduleAndSetAsRoot(SchedInstruction *inst) { Enumrtr_->scheduleAndSetAsRoot_(inst);}
    FUNC_RESULT enumerate_(Milliseconds startTime, Milliseconds rgnTimeout,
                           Milliseconds lngthTimeout) {};
};

/******************************************************************/

class BBMaster : public BBInterfacer {
private:
    vector<BBWorker *> Workers;
    vector<thread> ThreadManager;
    queue<BBWorker *> GPQ;
    int NumThreads_;
    int PoolSize_;

    void initWorkers(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
             long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
             SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
             bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
             bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc,
             SchedulerType HeurSchedType);

    inline void initForSchdulng() {return InitForSchdulngBBThread();}

    void initGPQ();

public:
    BBMaster(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
             long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
             SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
             bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
             bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc,
             SchedulerType HeurSchedType, int NumThreads, int PoolSize);
    
    Enumerator *allocEnumHierarchy_(Milliseconds timeout);
    FUNC_RESULT enumerate_(Milliseconds startTime, Milliseconds rgnTimeout,
                           Milliseconds lngthTimeout);
    void init();

//TODO: destructors, handle resource allocation & deaallocation
//cleanup the virtual funcs in sched_region
    

};


} //optsched namespace
} //llvm namespace

#endif
