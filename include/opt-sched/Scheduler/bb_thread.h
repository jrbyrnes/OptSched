#ifndef BB_THREAD_H
#define BB_THREAD_H

#include "opt-sched/Scheduler/OptSchedTarget.h"
#include "opt-sched/Scheduler/defines.h"
#include "opt-sched/Scheduler/sched_region.h"
#include "opt-sched/Scheduler/enum_parallel_master.h"
#include "llvm/ADT/SmallVector.h"
#include <map>
#include <set>
#include <vector>


namespace llvm {
namespace opt_sched {

class LengthCostEnumerator;
class EnumTreeNode;
class Register;
class RegisterFile;
class BitVector;


class BBThread {
private:
  LengthCostEnumerator *Enumrtr_;

  InstCount CrntSpillCost_;
  InstCount OptmlSpillCost_;

  // I think I need a register allocator

  // The target machine
  const OptSchedTarget *OST;

  bool EnblStallEnum_;
  int SCW_;
  int SchedCostFactor_;

  bool SchedForRPOnly_;

  int16_t RegTypeCnt_;
  RegisterFile *RegFiles_;

  InstCount MaxLatency_;
  bool SimpleMachineModel_;
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

  InstCount StaticSlilLowerBound_ = 0;

  // (Chris): The dynamic lower bound for SLIL is calculated differently from
  // the other cost functions. It is first set when the static lower bound is
  // calculated.
  InstCount DynamicSlilLowerBound_ = 0;

  int EntryInstCnt_;
  int ExitInstCnt_;
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

  // Virtual Functions:
  virtual FUNC_RESULT enumerate_(Milliseconds startTime, Milliseconds rgnDeadline,
                         Milliseconds lngthDeadline);

  virtual Enumerator *allocEnumrtr_(Milliseconds timeout);

  virtual int GetCostLwrBound();

  virtual int GetBestCost();

  virtual InstCount UpdtOptmlSched(InstSchedule *crntSched,
                           LengthCostEnumerator *enumrtr);

  // Non Virtual Functions
  InstCount CmputNormCost_(InstSchedule *sched, COST_COMP_MODE compMode,
                           InstCount &execCost, bool trackCnflcts);
  InstCount CmputCost_(InstSchedule *sched, COST_COMP_MODE compMode,
                       InstCount &execCost, bool trackCnflcts);
  
  void SetupForSchdulng_();
  void FinishOptml_();

  // BBWithSpill-specific Functions:
  InstCount CmputCostLwrBound_(InstCount schedLngth);
  void InitForCostCmputtn_();
  InstCount CmputDynmcCost_();

  void UpdateSpillInfoForSchdul_(SchedInstruction *inst, bool trackCnflcts);
  void UpdateSpillInfoForUnSchdul_(SchedInstruction *inst);
  void SetupPhysRegs_();
  void CmputCrntSpillCost_();
  bool ChkSchedule_(InstSchedule *bestSched, InstSchedule *lstSched);
  void CmputCnflcts_(InstSchedule *sched);

public:
  BBThread(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
              long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
              SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
              bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
              bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc,
              SchedulerType HeurSchedType);
  ~BBThread();

  // virtual

  virtual InstCount UpdtOptmlSched(InstSchedule *crntSched,
                           LengthCostEnumerator *enumrtr);

  // non-virtual

  int CmputCostLwrBound();

  bool ChkCostFsblty(InstCount trgtLngth, EnumTreeNode *treeNode);
  void SchdulInst(SchedInstruction *inst, InstCount cycleNum, InstCount slotNum,
                  bool trackCnflcts);
  void UnschdulInst(SchedInstruction *inst, InstCount cycleNum,
                    InstCount slotNum, EnumTreeNode *trgtNode);
  void SetSttcLwrBounds(EnumTreeNode *node);
  bool ChkInstLglty(SchedInstruction *inst);
  void InitForSchdulng();

protected:
  // (Chris)
  inline virtual const std::vector<int> &GetSLIL_() const {
    return SumOfLiveIntervalLengths_;
  }
};

/**********************************/

class BBInterfacer : public SchedRegion, public BBThread {
private:
    FUNC_RESULT Enumerate_(Milliseconds startTime, Milliseconds rgnDeadline,
                         Milliseconds lngthDeadline);

    Enumerator *allocEnumrtr_(Milliseconds timeout);

    void CmputAbslutUprBound_();

    InstCount CmputCostLwrBound();

    static InstCount ComputeSLILStaticLowerBound(int64_t regTypeCnt_,
                                             RegisterFile *regFiles_,
                                             DataDepGraph *dataDepGraph_);


public:
    BBInterfacer(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
              long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
              SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
              bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
              bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc,
              SchedulerType HeurSchedType);
};

} //optsched namespace
} //llvm namespace

#endif
