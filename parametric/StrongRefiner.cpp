/*
 * This file is part of PARAM.
 *
 * PARAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * PARAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with PARAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2009-2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#include <tr1/unordered_map>
#include "rationalFunction/RationalFunction.h"
#include "PMC.h"
#include "StrongRefiner.h"
#include "Partition.h"

namespace parametric {
  template<class Key, class Entry>
  class HashMap : public std::tr1::unordered_map<Key,Entry> {
  };
  
  using namespace std;
  using namespace rational;
  
  StrongRefiner::StrongRefiner() {
  }
  
  void StrongRefiner::refineClass(EqClass &eqClass) {
    /* enumerate neighbor partitions */
    map<EqClass*, unsigned> classMap;
    unsigned numClasses = 0;
    for (EqClass::iterator eqI = eqClass.begin(); eqI != eqClass.end(); eqI++) {
      PMM::state state(*eqI);
      
      for (unsigned succ(0); succ < oldPMC->getNumSuccStates(state); succ++) {
	PMM::state succState = oldPMC->getSuccState(state, succ);
        EqClass *succClass = &*(*partition)[succState];
        if (classMap.find(succClass) == classMap.end()) {
          classMap[succClass] = numClasses;
          numClasses++;
        }
      }
    }
    
    /* for each vertex, each neighbored equivalence class, calculate
     * probabilities to go to this class. */
    typedef pair<RationalFunction,RationalFunction> RationalPair;
    HashMap<RationalPair, unsigned> rMap;
    unsigned valueNumber(1);
    for (EqClass::iterator eqI = eqClass.begin(); eqI != eqClass.end(); eqI++) {
      PMM::state state(*eqI);
      map<EqClass*, RationalPair> uMap;
      for (unsigned succ(0); succ < oldPMC->getNumSuccStates(state); succ++) {
        const RationalFunction val(oldPMC->getSuccProb(state, succ));
	RationalFunction reward;
	if (isRewardAnalysis) {
	  reward = oldPMC->getSuccReward(state, succ);
	}
	PMM::state succState(oldPMC->getSuccState(state, succ));
        EqClass *vClass = &*(*partition)[succState];
        map<EqClass*, RationalPair>::iterator uMapIt(uMap.find(vClass));
        if (uMapIt == uMap.end()) {
          uMap.insert(make_pair(vClass, make_pair(val, reward)));
        } else {
          RationalPair oldVal(uMapIt->second);
          RationalPair newVal
            (make_pair(oldVal.first + val, oldVal.second + reward));
          uMap.erase(uMapIt);
          uMap.insert(make_pair(vClass, newVal));
        }
      }
      vector<unsigned> &vec = refined[state];
      vec.clear();
      vec.resize(numClasses, 0);
      
      map<EqClass*, RationalPair>::iterator uMapIt(uMap.find(&eqClass));
      for (uMapIt = uMap.begin(); uMapIt != uMap.end(); uMapIt++) {
        EqClass *eqClass = uMapIt->first;
        unsigned classNum = classMap[eqClass];
        const RationalPair &val(uMapIt->second);
        HashMap<RationalPair,unsigned>::iterator rMapIt(rMap.find(val));
        if (rMapIt == rMap.end()) {
          rMap.insert(make_pair(val, valueNumber));
          vec[classNum] = valueNumber;
          valueNumber++;
        } else {
          vec[classNum] = rMapIt->second;
        }
      }
    }
    
    /* map vertices of same values to same new class*/
    map<vector<unsigned>,EqClass> newClassMap;
    for (EqClass::iterator it = eqClass.begin(); it != eqClass.end(); it++) {
      PMM::state state(*it);
      vector<unsigned> &eqp = refined[state];
      EqClass &newEqClass = newClassMap[eqp];
      newEqClass.insert(state);
    }
    
    /* append these classes to list of new classes */
    map<vector<unsigned>,EqClass>::iterator it;
    if (newClassMap.size() == 1) {
      it = newClassMap.begin();
      partition->insert(it->second, false);
    } else {
      for (it = newClassMap.begin(); it != newClassMap.end(); it++) {
        partition->insert(it->second, true);
      }
    }
  }
  
  void StrongRefiner::createQuotient(Partition &partition) {
    PartitionList &P = partition.P;
    /* build new graph from blocks */
    PartitionList::iterator P_it;
    
    /* create state map */
    vector<PMM::state> oldToNew(oldPMC->getNumStates());
    vector<PMM::state> newToOld(P.size());
    unsigned newState(0);
    for (P_it = P.begin(); P_it != P.end(); P_it++) {
      EqClass &entry(*P_it);
      newToOld[newState] = *entry.begin();
      EqClass::iterator entry_it;
      bool is_init(false);
      bool rewardAlreadySet(false);
      for (entry_it = entry.begin(); entry_it != entry.end(); entry_it++) {	
	PMM::state entryState(*entry_it);
	oldToNew[entryState] = newState;

        if (isRewardAnalysis) {
          if (!rewardAlreadySet) {
	    //            RationalFunction reward(oldPMC->getStateReward(entryState));
	    //            newPMC->setStateReward(entryState, reward);
            rewardAlreadySet = true;
          }
        }
        if (oldPMC->isInit(entryState)) {
          is_init = true;
        }
      }
      if (is_init) {
        newPMC->addInit(newState);
      }
      newState++;
    }
    
    /* count transitions */
    newState = 0;
    unsigned numTrans(0);
    for (P_it = P.begin(); P_it != P.end(); P_it++) {
      PMC::state oldState(newToOld[newState]);
      set<PMM::state> succStates;
      for (unsigned oldSucc(0); oldSucc < oldPMC->getNumSuccStates(oldState);
	   oldSucc++) {
	PMM::state oldSuccState(oldPMC->getSuccState(oldState, oldSucc));
	PMM::state newSuccState(oldToNew[oldSuccState]);
	succStates.insert(newSuccState);
      }
      numTrans += succStates.size();
      newState++;
    }

    newPMC->reserveRowsMem(newState);
    newPMC->reserveColsMem(numTrans);
    if (isRewardAnalysis) {
      newPMC->reserveStateRewardsMem(newState);
      newPMC->reserveTransRewardsMem(numTrans);
    }
    
    /* create transitions of new graph */
    newState = 0;
    for (P_it = P.begin(); P_it != P.end(); P_it++) {
      PMC::state oldState(newToOld[newState]);
      map<PMM::state,RationalFunction> newSuccProbs;
      map<PMM::state,RationalFunction> newSuccRewards;
      for (unsigned oldSucc(0); oldSucc < oldPMC->getNumSuccStates(oldState);
	   oldSucc++) {
	PMM::state oldSuccState(oldPMC->getSuccState(oldState, oldSucc));
	RationalFunction succProb(oldPMC->getSuccProb(oldState, oldSucc));
	PMM::state newSuccState(oldToNew[oldSuccState]);
	newSuccProbs[newSuccState] += succProb;
	if (isRewardAnalysis) {
	  RationalFunction succReward(oldPMC->getSuccReward(oldState, oldSucc));	  
	  newSuccRewards[newSuccState] += succProb * succReward;
	}
      }
      for (map<PMM::state,RationalFunction>::iterator it = newSuccProbs.begin();
	   it != newSuccProbs.end(); it++) {
	newPMC->addSucc(it->first, it->second);
	if (isRewardAnalysis) {
	  newPMC->addSuccReward(newSuccRewards[it->first] / it->second);
	}
      }
      newPMC->finishState();
      newState++;
    }
  }
}
