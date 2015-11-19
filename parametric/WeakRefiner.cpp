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

#include <stack>
#include <boost/dynamic_bitset.hpp>
#include "rationalFunction/RationalFunction.h"
#include "PMC.h"
#include "Partition.h"
#include "WeakRefiner.h"

namespace parametric {
  template<class Key, class Entry>
  class HashMap : public std::tr1::unordered_map<Key,Entry> {
  };
  
  using namespace std;
  using namespace rational;
  
  WeakRefiner::WeakRefiner() {
  }
  
  /**
   * Checks whether @a u is a silent vertex.
   *
   * @param state state to be checked
   * @param A parition of @a u
   * @return true iff @a u is silent
   */
  bool WeakRefiner::weakIsSilent(PMM::state state, EqClass &A) {
    for (unsigned succ(0); succ < oldPMC->getNumSuccStates(state); succ++) {
      PMM::state succState(oldPMC->getSuccState(state, succ));
      /* transition to outside partition */
      if (A.find(succState) == A.end()) {
        return false;
      }
    }
    
    return true;
  }
  
  /**
   * Calculates P_Chi(u,C).
   *
   * @param state state
   * @param A partition of @a u
   * @param C target partition
   * @return P_Chi(u,C)
   */
  RationalFunction WeakRefiner::weakP_Chi
  (PMM::state state, EqClass &A, EqClass &C) {
    RationalFunction loop_el(0);
    RationalFunction out_el(0);
    for (unsigned succ(0); succ < oldPMC->getNumSuccStates(state); succ++) {
      PMM::state succState(oldPMC->getSuccState(state, succ));
      RationalFunction prob(oldPMC->getSuccProb(state, succ));
      if (A.find(succState) != A.end()) {
        loop_el += prob;
      } else if (C.find(succState) != C.end()) {
        out_el += prob;
      }
    }
    
    RationalFunction result(out_el / (RationalFunction(1) - loop_el));
    
    return result;
  }
  
  void WeakRefiner::silentSplit
  (EqClass &B, list<PMM::state> &nonSilent,
   map<PMM::state,vector<bool> > &label,
   map<PMM::state,unsigned> &nonSilentLabel) {
    
    for (list<PMM::state>::iterator nonSilentIt = nonSilent.begin();
         nonSilentIt != nonSilent.end(); nonSilentIt++) {
      PMM::state s(*nonSilentIt);
      unsigned classNr(nonSilentLabel[s]);
      stack<PMM::state> stack;
      stack.push(s);
      while (!stack.empty()) {
	PMM::state u(stack.top());
        stack.pop();
	for (unsigned pred(0); pred < oldPMC->getNumPredStates(u); pred++) {
	  PMM::state predState(oldPMC->getPredState(u, pred));
          if ((B.find(predState) != B.end())
              && weakIsSilent(predState, B)
              && !label[predState][classNr]) {
            label[predState][classNr] = true;
            stack.push(predState);
          }
        }
      }
    }
  }
  
  void WeakRefiner::refineClass(EqClass &eqClass) {
    /* enumerate neighbor partitions */
    map<EqClass*, unsigned> classMap;
    unsigned numClasses = 0;
    for (EqClass::iterator eqI = eqClass.begin(); eqI != eqClass.end(); eqI++) {
      PMM::state u = *eqI;
      for (unsigned succ(0); succ < oldPMC->getNumSuccStates(u); succ++) {
	PMM::state v = oldPMC->getSuccState(u, succ);
        EqClass *vClass = &*(*partition)[v];
        if ((vClass != &eqClass)
            && (classMap.find(vClass) == classMap.end())) {
          classMap[vClass] = numClasses;
          numClasses++;
        }
      }
    }
    
    /* for each non-silent state, each neighbored equivalence class, calculate
     * probabilities to go to this class. */
    HashMap<RationalFunction, unsigned> rMap;
    unsigned valueNumber = 1;
    for (EqClass::iterator eqI = eqClass.begin(); eqI != eqClass.end(); eqI++) {
      PMM::state u = *eqI;
      if (!weakIsSilent(u, eqClass)) {
        map<EqClass*, RationalFunction> uMap;
	for (unsigned succ(0); succ < oldPMC->getNumSuccStates(u); succ++) {
          const RationalFunction val(oldPMC->getSuccProb(u, succ));
	  PMM::state v(oldPMC->getSuccState(u, succ));
          EqClass *vClass = &*(*partition)[v];
          map<EqClass*, RationalFunction>::iterator uMapIt(uMap.find(vClass));
          if (uMapIt == uMap.end()) {
            uMap.insert(make_pair(vClass, val));
          } else {
            RationalFunction oldVal(uMapIt->second);
            uMap.erase(uMapIt);
            uMap.insert(make_pair(vClass, oldVal + val));
          }
        }
        // TODO check following
        map<EqClass*, RationalFunction>::iterator uMapIt(uMap.find(&eqClass));
        if (uMap.end() != uMapIt) {
          RationalFunction dividor(RationalFunction(1) - uMapIt->second);
          uMap.erase(uMapIt);
          for (map<EqClass*, RationalFunction>::iterator it = uMap.begin();
               it != uMap.end(); it++) {
            it->second = it->second /  dividor;
          }
        }
        vector<unsigned> &vec = refined[u];
        vec.clear();
        vec.resize(numClasses, 0);
        
        for (uMapIt = uMap.begin(); uMapIt != uMap.end(); uMapIt++) {
          EqClass *eqClass = uMapIt->first;
          unsigned classNum = classMap[eqClass];
          const RationalFunction &val(uMapIt->second);
          HashMap<RationalFunction,unsigned>::iterator rMapIt =
            rMap.find(val);
          if (rMapIt == rMap.end()) {
            rMap.insert(make_pair(val, valueNumber));
            vec[classNum] = valueNumber;
            valueNumber++;
          } else {
            vec[classNum] = rMapIt->second;
          }
        }
      }
    }
    
    /* enumerate classes */
    int classNr = 0;
    map<vector<unsigned>,unsigned> classes;
    for (EqClass::iterator it = eqClass.begin(); it != eqClass.end(); it++) {
      PMM::state u(*it);
      if (!weakIsSilent(u, eqClass)) {
        vector<unsigned> &eqp = refined[u];
        if (classes.find(eqp) == classes.end()) {
          classes.insert(make_pair(eqp, classNr));
          classNr++;
        }
      }
    }
    
    /* create label structure and remember non-silent states*/
    list<PMM::state> nonSilent;
    map<PMM::state,vector<bool> > label;
    map<PMM::state,unsigned> nonSilentLabel;
    for (EqClass::iterator it = eqClass.begin(); it != eqClass.end(); it++) {
      PMM::state u(*it);
      vector<bool> &vec = label[u];
      vec.resize(classes.size());
      if (!weakIsSilent(u, eqClass)) {
        vector<unsigned> &eqp = refined[u];
        unsigned classNr = classes[eqp];
        vec[classNr] = true;
        nonSilentLabel[u] = classNr;
        nonSilent.insert(nonSilent.begin(), u);
      }
    }
    
    /* handle silent states */
    silentSplit(eqClass, nonSilent, label, nonSilentLabel);
    
    /* create new classes */
    map<vector<bool>,EqClass> newClassMap;
    for (EqClass::iterator it = eqClass.begin(); it != eqClass.end(); it++) {
      PMM::state u = *it;
      vector<bool> &vec = label[u];
      EqClass &eqClass = newClassMap[vec];
      eqClass.insert(u);
    }
    
    /* append these classes to list of new classes */
    map<vector<bool>,EqClass>::iterator it;
    if (newClassMap.size() == 1) {
      it = newClassMap.begin();
      partition->insert(it->second, false);
    } else {
      for (it = newClassMap.begin(); it != newClassMap.end(); it++) {
        partition->insert(it->second, true);
      }
    }
  }
  
  void WeakRefiner::createQuotient(Partition &partition) {
    PartitionList &P = partition.P;
    /* build new graph from blocks */
    PartitionList::iterator P_it;
    
    /* create state map */
    vector<PMM::state> oldToNew(oldPMC->getNumStates());
    vector<PMM::state> newToOld(P.size());
    unsigned newState(0);
    for (P_it = P.begin(); P_it != P.end(); P_it++) {
      EqClass &entry(*P_it);
      /* find non-silent entry if existing */
      newToOld[newState] = *entry.begin();
      EqClass::iterator entry_it;
      for (entry_it = entry.begin(); entry_it != entry.end(); entry_it++) {
	PMM::state state(*entry_it);
	for (unsigned succ(0); succ < oldPMC->getNumSuccStates(state); succ++) {
	  PMM::state succState(oldPMC->getSuccState(state, succ));
	  if (entry.find(succState) == entry.end()) {
	    newToOld[newState] = state;
	  }
	}
      }
      bool is_init(false);
      for (entry_it = entry.begin(); entry_it != entry.end(); entry_it++) {	
	PMM::state entryState(*entry_it);
	oldToNew[entryState] = newState;
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
      EqClass &entry(*P_it);
      PMC::state oldState(newToOld[newState]);
      set<PMM::state> succStates;
      for (unsigned oldSucc(0); oldSucc < oldPMC->getNumSuccStates(oldState);
	   oldSucc++) {
	PMM::state oldSuccState(oldPMC->getSuccState(oldState, oldSucc));
	if (entry.find(oldSuccState) == entry.end()) {
	  PMM::state newSuccState(oldToNew[oldSuccState]);
	  succStates.insert(newSuccState);
	}
      }
      numTrans += succStates.size();
      newState++;
    }

    newPMC->reserveRowsMem(newState);
    newPMC->reserveColsMem(numTrans);
    
    /* create transitions of new graph */
    newState = 0;
    for (P_it = P.begin(); P_it != P.end(); P_it++) {
      EqClass &entry(*P_it);

      PMC::state oldState(newToOld[newState]);
      map<PMM::state,RationalFunction> newSuccProbs;
      RationalFunction sumOut(0);
      for (unsigned oldSucc(0); oldSucc < oldPMC->getNumSuccStates(oldState);
	   oldSucc++) {
	PMM::state oldSuccState(oldPMC->getSuccState(oldState, oldSucc));
	if (entry.find(oldSuccState) == entry.end()) {
	  RationalFunction succProb(oldPMC->getSuccProb(oldState, oldSucc));
	  sumOut += succProb;
	  PMM::state newSuccState(oldToNew[oldSuccState]);
	  newSuccProbs[newSuccState] += succProb;
	}
      }
      for (map<PMM::state,RationalFunction>::iterator it = newSuccProbs.begin();
	   it != newSuccProbs.end(); it++) {
	newPMC->addSucc(it->first, it->second / sumOut);
      }
      if (0 == sumOut) {
	newPMC->addSucc(newState, 1);
      }
      newPMC->finishState();
      newState++;
    }
  }
  
  void WeakRefiner::createInitialPartition(Partition &partition) {
    Refiner::createInitialPartition(partition);
    /* split off divergent states */
    PartitionList &P = partition.P;
    
    PartitionList P_div;
    PartitionList::iterator P_it;
    for (P_it = P.begin(); P_it != P.end(); P_it++) {
      EqClass &A = *P_it;
      boost::dynamic_bitset<> marked(oldPMC->getNumStates());
      for (EqClass::iterator A_it = A.begin(); A_it != A.end(); A_it++) {
	PMM::state u = *A_it;
        bool non_silent  = false;
        /* check whether outgoing edges to other partitions exist */
	for (unsigned succ(0); succ < oldPMC->getNumSuccStates(u); succ++) {
	  PMM::state v = oldPMC->getSuccState(u, succ);
          /* transition to other partition? */
          if (A.find(v) == A.end()) {
            non_silent = true;
            break;
          }
        }
        
        /* mark vertex able to reach other partions */
        if (non_silent) {
	  backSearch(marked, u);
        }
      }

      /* place vertices not able to reach other partitions in new partition */
      set<PMM::state> s;
      list<set<PMM::state> >::iterator iA_div =
        P_div.insert(P_div.begin(), s);
      set<PMM::state> &A_div = *iA_div;
      EqClass::iterator A_it;
      A_it = A.begin();
      while (A_it != A.end()) {
	PMM::state u(*A_it);
        if (!marked[u]) {
          set<PMM::state>::iterator A_del_it = A_it;
          A_it++;
          A.erase(A_del_it);
          A_div.insert(u);
        } else {
          A_it++;
        }
      }
    }
    
    P.splice(P.begin(), P_div);
    /* remove empty partitions */
    P_it = P.begin();
    while (P_it != P.end()) {
      set<PMM::state> &A = *P_it;
      if (A.empty()) {
        list<set<PMM::state> >::iterator P_del_it = P_it;
        P_it++;
        P.erase(P_del_it);
      } else {
        P_it++;
      }
    }
  }

  void WeakRefiner::backSearch
  (boost::dynamic_bitset<> &seen, PMM::state state) {
    list<PMM::state> work;
    seen[state] = true;
    work.push_back(state);
    while (!work.empty()) {
      PMM::state state(work.front());
      work.pop_front();
      for (unsigned succ(0); succ < oldPMC->getNumSuccStates(state); succ++) {
	PMM::state succState(oldPMC->getSuccState(state, succ));
	if (!seen[succState]) {
	  work.push_back(succState);
	}
	seen[succState] = true;
      }
    }
  }
}
