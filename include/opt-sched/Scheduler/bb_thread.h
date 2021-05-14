#ifndef BB_THREAD_H
#define BB_THREAD_H

#include "opt-sched/Scheduler/OptSchedTarget.h"
#include "opt-sched/Scheduler/defines.h"
#include "opt-sched/Scheduler/gen_sched.h"
#include "opt-sched/Scheduler/sched_region.h"
#include "opt-sched/Scheduler/enumerator.h"
#include "llvm/ADT/SmallVector.h"
#include <map>
#include <set>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

namespace llvm {
namespace opt_sched {

class LengthCostEnumerator;
class EnumTreeNode;
class Register;
class RegisterFile;
class BitVector;


class InstPool {
private:
  std::queue<std::pair<EnumTreeNode *, unsigned long >> pool;
public:
  InstPool();
  void push(std::pair<EnumTreeNode *, unsigned long> n) {pool.push(n);}
  int size() {return pool.size();}
  std::pair<EnumTreeNode *, unsigned long> front() {return pool.front();}
  void pop() {pool.pop();}
  bool empty() {return pool.empty();}
  void sort();
};


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

  // non-virtual

  int CmputCostLwrBound();

  bool ChkCostFsblty(InstCount trgtLngth, EnumTreeNode *treeNode, bool isGlobalPoolNode = false);
  void SchdulInstBBThread(SchedInstruction *inst, InstCount cycleNum, InstCount slotNum,
                  bool trackCnflcts);
  void UnschdulInstBBThread(SchedInstruction *inst, InstCount cycleNum,
                    InstCount slotNum, EnumTreeNode *trgtNode);
  void UpdateSpillInfoForUnSchdul_(SchedInstruction *inst);
  void setSttcLwrBounds(EnumTreeNode *node);
  bool ChkInstLgltyBBThread(SchedInstruction *inst);

  void InitForSchdulngBBThread();

  InstSchedule *allocNewSched_();
  
  InstCount cmputNormCostBBThread_(InstSchedule *sched, COST_COMP_MODE compMode,
                           InstCount &execCost, bool trackCnflcts);

  // virtuals
  virtual InstCount getBestCost() = 0;

  virtual InstCount UpdtOptmlSched(InstSchedule *crntSched,
                           LengthCostEnumerator *enumrtr) = 0;

  virtual bool isSecondPass() = 0;

  virtual bool isWorker() = 0;

  virtual void histTableLock(UDT_HASHVAL key) = 0;
  virtual void histTableUnlock(UDT_HASHVAL key) = 0;

  virtual void allocatorLock() = 0;
  virtual void allocatorUnlock() = 0;

  virtual void incrementImprvmntCnt() = 0;
  

  // Needed by aco
  virtual InstCount getHeuristicCost() = 0;

  void SetupForSchdulngBBThread_();
  

protected:
  LengthCostEnumerator *Enumrtr_;
  InstCount CrntSpillCost_;
  InstCount OptmlSpillCost_;

  DataDepGraph *DataDepGraph_;
  MachineModel  *MachMdl_; 

  int SolverID_;

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
  InstCount StaticLowerBound_ = 0;

  // Needed to override SchedRegion virtuals
  bool EnableEnumBBThread_();

  InstCount CmputCost_(InstSchedule *sched, COST_COMP_MODE compMode,
                       InstCount &execCost, bool trackCnflcts);
  

  void FinishOptmlBBThread_();

  bool ChkScheduleBBThread_(InstSchedule *bestSched, InstSchedule *lstSched);

  // Returns the static lower bound
  inline int getCostLwrBound() {return StaticLowerBound_;};

  // Virtual Functions:
  virtual void setBestCost(InstCount BestCost) = 0;


};

/******************************************************************/

class BBInterfacer : public SchedRegion, public BBThread {
private:
    void CmputAbslutUprBound_();

    InstCount CmputCostLwrBound();

protected:
    InstCount *BestCost_;
    InstCount *CostLwrBound_;

    int NumSolvers_;

    void CmputSchedUprBound_();

      // override SchedRegion virtual
    void InitForSchdulng() override {return InitForSchdulngBBThread();}

    void SetupForSchdulng_() override {return SetupForSchdulngBBThread_();}

    bool ChkInstLglty(SchedInstruction *inst) override
    {
      return ChkInstLgltyBBThread(inst);
    }

    bool ChkSchedule_(InstSchedule *bestSched, InstSchedule *lstSched) override
    {
      return ChkScheduleBBThread_(bestSched, lstSched);
    }

    bool EnableEnum_() override
    {
      return EnableEnumBBThread_();
    }

    void FinishOptml_() override
    {
      return FinishOptmlBBThread_();
    }

  // override BBThread virtual
  InstCount getBestCost() override {return *BestCost_;}
 
  void setBestCost(InstCount BestCost) override { *BestCost_ = BestCost; }

  InstCount UpdtOptmlSched(InstSchedule *crntSched,
                             LengthCostEnumerator *enumrtr);


public:
    BBInterfacer(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
              long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
              SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
              bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
              bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc,
              SchedulerType HeurSchedType);


    inline void SchdulInst(SchedInstruction *inst, InstCount cycleNum, InstCount slotNum,
                  bool trackCnflcts)
    {
      SchdulInstBBThread(inst, cycleNum, slotNum, trackCnflcts);
    }

    inline void UnschdulInst(SchedInstruction *inst, InstCount cycleNum,
                    InstCount slotNum, EnumTreeNode *trgtNode)
    {
      UnschdulInstBBThread(inst, cycleNum, slotNum, trgtNode);
    }

    inline InstCount CmputNormCost_(InstSchedule *sched, COST_COMP_MODE compMode,
                           InstCount &execCost, bool trackCnflcts)
    {
      return cmputNormCostBBThread_(sched, compMode, execCost, trackCnflcts);
    }

    static InstCount ComputeSLILStaticLowerBound(int64_t regTypeCnt_,
                                                 RegisterFile *regFiles_, 
                                                 DataDepGraph *dataDepGraph_);

    bool isSecondPass() override { return isSecondPass_; }

    bool isWorker() override {return false;}

    void histTableLock(UDT_HASHVAL key) override {/*nothing*/; }
    void histTableUnlock(UDT_HASHVAL key) override {/*nothing*/; }

    void incrementImprvmntCnt() override {/*nothing*/;}

    void allocatorLock() override {/*nothing*/;}
    void allocatorUnlock() override {/*nothing*/;}

    inline InstCount getHeuristicCost() {return GetHeuristicCost();}

};

/******************************************************************/
class BBWithSpill : public BBInterfacer {
private:

protected:

public:
    BBWithSpill(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
                long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
                SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
                bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
                bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc,
                SchedulerType HeurSchedType);

    
    FUNC_RESULT Enumerate_(Milliseconds startTime, Milliseconds rgnTimeout,
                           Milliseconds lngthTimeout, int *OptimalSolverID) override;

    Enumerator *AllocEnumrtr_(Milliseconds timeout);

    uint64_t getExaminedNodeCount() override {return Enumrtr_->GetNodeCnt(); }

};


/******************************************************************/
class BBWorker : public BBThread {
private:

    InstCount SchedUprBound_;   // set by master (using schedRegion interface)
    int16_t SigHashSize_;       // set by master (using schedRegion interface)

    Pruning PruningStrategy_;
    SchedPriorities EnumPrirts_;
    SPILL_COST_FUNCTION SpillCostFunc_;

    vector<int> TakenArr;
    vector<BBWorker> *local_pool = NULL;

    vector<FUNC_RESULT> *RsltAddr_;


    InstSchedule *EnumCrntSched_;
    InstSchedule *EnumBestSched_;

    // local variable holding cost of best schedule for current enumerator
    InstCount BestCost_;
    // cost of the heuristic schedule
    InstCount HeuristicCost_;
    // lower bound of schedule length
    InstCount SchedLwrBound_;

    // shared variable of best schedule
    InstSchedule *MasterSched_;
    // shared variable of the best cost found so far
    InstCount *MasterCost_;       
    // shared variable of the best spill cost found so far
    InstCount *MasterSpill_;
    // share variable of the best sched elgnth found so far
    InstCount *MasterLength_;

    // the best found schedule for the region
    InstSchedule *RegionSched_;

    // TODO replace Count with Cnt
    uint64_t *NodeCount_;
    int *MasterImprvCount_;

    // are we in the second apss
    bool IsSecondPass_;

    // A reference to the shared GlobalPool
    InstPool *GlobalPool_;

    // References to the locks on shared data
    std::mutex **HistTableLock_;
    std::mutex *GlobalPoolLock_; 
    std::mutex *BestSchedLock_;
    std::mutex *NodeCountLock_;
    std::mutex *ImprvmntCntLock_;
    std::mutex *RegionSchedLock_;
    std::mutex *AllocatorLock_;

    int *IdleTime_;
    

    void handlEnumrtrRslt_(FUNC_RESULT rslt, InstCount trgtLngth);

    // overrides
    inline InstCount getBestCost() {return *MasterCost_;}
    inline void setBestCost(InstCount BestCost) {
      BestCost_ = BestCost;
      }

    // needs to write to master
    InstCount UpdtOptmlSched(InstSchedule *crntSched, LengthCostEnumerator *enumrtr);

    void writeBestSchedToMaster(InstSchedule *BestSchedule, InstCount BestCost, InstCount BestSpill);

public:
    BBWorker(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
              long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
              SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
              bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
              bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc,
              SchedulerType HeurSchedType, bool IsSecondPass, 
              InstSchedule *MasterSched, InstCount *MasterCost, 
              InstCount *MasterSpill, InstCount *MasterLength, 
              InstPool *GlobalPool, 
              uint64_t *NodeCount, int SolverID, std::mutex **HistTableLock, 
              std::mutex *GlobalPoolLock, std::mutex *BestSchedLock, std::mutex *NodeCountLock,
              std::mutex *ImprCountLock, std::mutex *RegionSchedLock, std::mutex *AllocatorLock,
              vector<FUNC_RESULT> *resAddr, int *idleTimes);

    /*
    BBWorker (const BBWorker&) = delete;
    BBWorker& operator= (const BBWorker&) = delete;
    */

    void setHeurInfo(InstCount SchedUprBound, InstCount HeuristicCost, InstCount SchedLwrBound);

    void allocEnumrtr_(Milliseconds timeout);
    void initEnumrtr_();
    void setLCEElements_(InstCount costLwrBound);
    inline void setEnumHistTable(BinHashTable<HistEnumTreeNode> *histTable)  {
      Enumrtr_->setHistTable(histTable);
    }

    void allocSched_();

    void setBestSched(InstSchedule *sched);
    void setCrntSched(InstSchedule *sched);

    inline bool scheduleArtificialRoot() {return Enumrtr_->scheduleArtificialRoot();}
    
    inline void scheduleAndSetAsRoot(SchedInstruction *inst, 
                                     LinkedList<SchedInstruction> *frstList,
                                     LinkedList<SchedInstruction> *scndList) { 
      Enumrtr_->scheduleAndSetAsRoot_(inst, frstList, scndList);
    }
    
    inline InstCount getRootInstNum() {return Enumrtr_->getRootInstNum();}

    inline int getSolverID() {return SolverID_;}

    inline void appendToRdyLst(LinkedList<SchedInstruction> *lst) {
      Enumrtr_->appendToRdyLst(lst);
    }

    inline void setRootRdyLst() {Enumrtr_->setRootRdyLst();}

    bool generateStateFromNode(EnumTreeNode *GlobalPoolNode);

    FUNC_RESULT enumerate_(EnumTreeNode *GlobalPoolNode, Milliseconds StartTime, 
                           Milliseconds RgnTimeout, Milliseconds LngthTimeout);

    //TODO - clean this up
    inline InstCount CmputNormCost_(InstSchedule *sched, COST_COMP_MODE compMode,
                           InstCount &execCost, bool trackCnflcts)
    {
      return cmputNormCostBBThread_(sched, compMode, execCost, trackCnflcts);
    }

    bool isSecondPass() override { return IsSecondPass_;}

    bool isWorker() override {return true;}

    inline InstCount getHeuristicCost() {return HeuristicCost_;}

    inline void setCostLowerBound(InstCount StaticLowerBound) {
      StaticLowerBound_ = StaticLowerBound;
    }

    inline void setMasterSched(InstSchedule *MasterSched) {MasterSched_ = MasterSched;}
    inline void setMasterImprvCount(int *ImprvCount) {MasterImprvCount_ = ImprvCount; }

    inline void setRegionSchedule(InstSchedule *RegionSched) {RegionSched_ = RegionSched;}

    void histTableLock(UDT_HASHVAL key) override;
    void histTableUnlock(UDT_HASHVAL key) override; 

    void allocatorLock() override;
    void allocatorUnlock() override;

    void incrementImprvmntCnt() override;

};

/******************************************************************/

class BBMaster : public BBInterfacer {
private:
    vector<BBWorker *> Workers;
    vector<std::thread> ThreadManager;
    InstPool *GlobalPool; 
    int firstLevelSize_;
    int NumThreads_;
    int PoolSize_;
    uint64_t MasterNodeCount_;
    vector<FUNC_RESULT> results;

    std::mutex **HistTableLock;
    std::mutex GlobalPoolLock;
    std::mutex BestSchedLock;
    std::mutex NodeCountLock;
    std::mutex ImprvCountLock;
    std::mutex RegionSchedLock;
    std::mutex AllocatorLock;

    int64_t HistTableSize_;

    int *idleTimes;


    void initWorkers(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
             long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
             SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
             bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
             bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc,
             SchedulerType HeurSchedType, InstCount *BestCost, InstCount SchedLwrBound,
             InstSchedule *BestSched, InstCount *BestSpill, 
             InstCount *BestLength, 
             InstPool *GlobalPool, 
             uint64_t *NodeCount,  std::mutex **HistTableLock, std::mutex *GlobalPoolLock, std::mutex *BestSchedLock, 
             std::mutex *NodeCountLock, std::mutex *ImprvCountLock, std::mutex *RegionSchedLock, 
             std::mutex *AllocatorLock, vector<FUNC_RESULT> *results, int *idleTimes);

  
    bool initGlobalPool();
    bool init();
    void setWorkerHeurInfo();
    Enumerator *allocEnumHierarchy_(Milliseconds timeout, bool *fsbl);

    inline BinHashTable<HistEnumTreeNode> *getEnumHistTable() {
      return Enumrtr_->getHistTable(); 
    }

public:
    BBMaster(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
             long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
             SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
             bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
             bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc,
             SchedulerType HeurSchedType, int NumThreads, int PoolSize, 
             int NumSolvers);

    ~BBMaster();
    
    BBMaster (const BBMaster&) = delete;
    BBMaster& operator= (const BBMaster&) = delete;

    Enumerator *AllocEnumrtr_(Milliseconds timeout);


    FUNC_RESULT Enumerate_(Milliseconds startTime, Milliseconds rgnTimeout,
                           Milliseconds lngthTimeout, int *OptimalSolverID) override;

    
    uint64_t getExaminedNodeCount() override {return MasterNodeCount_; }




//TODO: destructors, handle resource allocation & deaallocation
//cleanup the virtual funcs in sched_region
    

};


} //optsched namespace
} //llvm namespace

#endif
