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

#include <map>
#include <list>
#include <stdexcept>
#include <boost/dynamic_bitset.hpp>
#include "PMDP.h"
#include "Controller.h"
#include "GPMC.h"
#include "Eliminator.h"

using namespace std;
using namespace rational;

namespace parametric {
  Eliminator::Eliminator() {
    pmc = NULL;
    initStates = NULL;
    targetStates = NULL;
    needValue = NULL;
    pmdp = NULL;
  }

  Eliminator::~Eliminator() {
  }

  void Eliminator::setPMC(GPMC &pmc__) {
    pmc = &pmc__;
  }
  
  void Eliminator::setInitStates(StateSet &initStates__) {
    initStates = &initStates__;
  }
  
  void Eliminator::setTargetStates(StateSet &targetStates__) {
    targetStates = &targetStates__;
  }
  
  
  void Eliminator::setRewardAnalysis(bool rewardAnalysis__) {
    rewardAnalysis = rewardAnalysis__;
  }
  
  void Eliminator::setEliminationOrder(const std::string &eliminationOrder_) {
    eliminationOrder = eliminationOrder_;
  }

  void Eliminator::setNeedValue(const boost::dynamic_bitset<> &needValue_) {
    needValue = &needValue_;
  }

  void Eliminator::setPMDP(const PMDP &__pmdp) {
    pmdp = &__pmdp;
  }

  void Eliminator::setBackMap(vector<unsigned> &__backMap) {
    backMap = &__backMap;
  }

  RationalFunction Eliminator::leavingSum(PMM::state state) {
    unsigned loopNr(pmc->getSuccNrBySuccState(state, state));
    if (loopNr == pmc->getInvalidState()) {
      return 1;
    } else {
      return 1-pmc->getSuccProb(state, loopNr);
    }
  }
  
  void Eliminator::eliminateState(PMM::state elimState) {
    RationalFunction leaving(leavingSum(elimState));
    RationalFunction loopReward(0);
    
    unsigned loopNr(pmc->getSuccNrBySuccState(elimState, elimState));
    if (pmc->getInvalidState() != loopNr) {
      if (0 != leaving) {
	if (rewardAnalysis) {
	  loopReward = ((1 - leaving) / leaving)
	    * pmc->getSuccReward(elimState, loopNr);
	}
	pmc->removeSuccTrans(elimState, loopNr);
      }
    }

    if (0 == leaving) {
      return;
    }

    /* calculate outgoing probabilities */
    vector<RationalFunction> outVals;
    vector<PMM::state> outStates;
    vector<RationalFunction> outRewards;
    for (unsigned succ(0); succ < pmc->getNumSuccStates(elimState); succ++) {
      PMM::state succState(pmc->getSuccState(elimState, succ));
      RationalFunction outProb(pmc->getSuccProb(elimState, succ) / leaving);      
      outVals.push_back(outProb);
      if ((*needValue)[elimState]) {
	pmc->setSuccProb(elimState, succ, outProb);
      }
      if (rewardAnalysis) {
	RationalFunction outReward(loopReward + pmc->getSuccReward(elimState, succ));
        outRewards.push_back(outReward);
	if ((*needValue)[elimState]) {
	  pmc->setSuccReward(elimState, succ, outReward);
	}
      }
      outStates.push_back(succState);
    }

    vector<RationalFunction> inVals;
    vector<PMM::state> inStates;
    vector<RationalFunction> inRewards;
    vector<unsigned> remove;
    for (unsigned pred(0); pred < pmc->getNumPredStates(elimState); pred++) {
      PMM::state predState(pmc->getPredState(elimState, pred));
      unsigned predSuccNr(pmc->getSuccNrBySuccState(predState, elimState));
      RationalFunction inVal(pmc->getSuccProb(predState, predSuccNr));
      inVals.push_back(inVal);
      if (rewardAnalysis) {
	RationalFunction inReward(pmc->getSuccReward(predState, predSuccNr));
	inRewards.push_back(inReward);
      }
      inStates.push_back(predState);
      remove.push_back(predSuccNr);
    }
    for (unsigned nr(0); nr < inStates.size(); nr++) {
      PMM::state predState(inStates[nr]);
      pmc->removeSuccTrans(predState, remove[nr]);
    }

    for (unsigned inNr(0); inNr < inStates.size(); inNr++) {
      RationalFunction inProb(inVals[inNr]);
      PMM::state inState(inStates[inNr]);
      for (unsigned outNr(0); outNr < outStates.size(); outNr++) {
	RationalFunction outProb(outVals[outNr]);
	PMM::state outState(outStates[outNr]);
	RationalFunction prob(inProb * outProb);
	unsigned inToOut(pmc->getSuccNrBySuccState(inState, outState));
	if (pmc->getInvalidState() != inToOut) {
	  RationalFunction oldProb(pmc->getSuccProb(inState, inToOut));
	  pmc->setSuccProb(inState, inToOut, prob + oldProb);
	  if (rewardAnalysis) {
	    RationalFunction newRew(inRewards[inNr] + outRewards[outNr]);
	    RationalFunction oldRew(pmc->getSuccReward(inState, inToOut));
	    RationalFunction rew((newRew * prob + oldRew * oldProb) / (prob + oldProb));
	    pmc->setSuccReward(inState, inToOut, rew);
	  }
	} else {
	  if (rewardAnalysis) {
	    RationalFunction rew(inRewards[inNr] + outRewards[outNr]);
	    pmc->addSucc(inState, outState, prob, rew);
	  } else {
	    pmc->addSucc(inState, outState, prob);
	  }
	}
      }
    }

    if (!(*needValue)[elimState]) {
      pmc->makeAbsorbing(elimState);
    }
  }
  
  void Eliminator::eliminateStates(StateList &elemStates) {
    for (StateList::iterator it = elemStates.begin();
         it != elemStates.end(); it++) {
      PMM::state state(*it);
      eliminateState(state);
    }
  }
  
  void Eliminator::collectStatesForward(StateList &stateList) {
    if (pmdp == NULL) {
      boost::dynamic_bitset<> seen(pmc->getNumStates());
      list<PMM::state> work;
      for (StateSet::iterator it(initStates->begin()); it != initStates->end();
	   it++) {
	work.push_back(*it);
	seen[*it] = true;
	stateList.push_back(*it);
      }
      while (!work.empty()) {
	PMM::state state(work.front());
	work.pop_front();
	for (unsigned succ(0); succ < pmc->getNumSuccStates(state); succ++) {
	  PMM::state succState(pmc->getSuccState(state, succ));
	  if ((0 == targetStates->count(succState))
	      && (!seen[succState])) {
	    work.push_back(succState);
	    stateList.push_back(succState);
	    seen[succState] = true;
	  }
	}
      }
      for (unsigned state(0); state < pmc->getNumStates(); state++) {
	if ((0 == targetStates->count(state)) && !seen[state]) {
	  stateList.push_back(state);
	}
      }
    } else {
      StateList pmdpStates;
      boost::dynamic_bitset<> seen(pmdp->getNumStates());
      list<PMM::state> work;
      for (unsigned state(0); state < pmdp->getNumStates(); state++) {
	if (pmdp->isInit(state)) {
	  work.push_back(state);
	  seen[state] = true;
	  pmdpStates.push_back(state);
	}
      }
      while (!work.empty()) {
	PMM::state state(work.front());
	work.pop_front();
	for (unsigned choice(0); choice < pmdp->getNumSuccChoices(state); choice++) {
	  for (unsigned succ(0); succ < pmdp->getNumSuccStates(state, choice); succ++) {
	    PMM::state succState(pmdp->getSuccState(state, choice, succ));
	    if ((0 == targetStates->count((*backMap)[succState]))
		&& (!seen[succState])) {
	      work.push_back(succState);
	      pmdpStates.push_back(succState);
	    }
	    seen[succState] = true;
	  }
	}
      }
      seen.resize(0);
      seen.resize(pmc->getNumStates(), false);
      for (StateList::iterator it(pmdpStates.begin()); it != pmdpStates.end();
	   it++) {
	const unsigned state(*it);
	const unsigned pmcState((*backMap)[state]);
	if (!seen[pmcState]) {
	  seen[pmcState] = true;
	  stateList.push_back(pmcState);
	}
      }
    }
    //    assert(stateList.size() == pmc->getNumStates());
    // TODO instead of eliminating all states for, mark states which were not
    // needed

  }

  void Eliminator::collectStatesBackwardOpt(StateList &stateList) {
    boost::dynamic_bitset<> seen(pmc->getNumStates());
    list<PMM::state> work;
    for (StateSet::iterator it(targetStates->begin());
	 it != targetStates->end(); it++) {
      work.push_back(*it);
      seen[*it] = true;
      //      stateList.push_back(*it);
    }
    while (!work.empty()) {
      PMM::state state(work.front());
      work.pop_front();
      for (unsigned pred(0); pred < pmc->getNumPredStates(state); pred++) {
	PMM::state predState(pmc->getPredState(state, pred));
	if ((0 == targetStates->count(predState))
	    && (!seen[predState])) {
	  work.push_back(predState);
	  //	  stateList.push_back(predState);
	}
	seen[predState] = true;
      }
    }

    for (PMM::state state(0); state < pmc->getNumStates(); state++) {
      if (!seen[state]) {
	pmc->makeAbsorbing(state);
      }
      for (unsigned succ(0); succ < pmc->getNumSuccStates(state); succ++) {
	PMM::state succState(pmc->getSuccState(state, succ));
	if (!seen[succState]) {
	  pmc->removeSuccTrans(state, succ);
	}
      }
    }

    seen.clear();
    seen.resize(pmc->getNumStates(), false);
    work.clear();

    for (StateSet::iterator it(targetStates->begin());
	 it != targetStates->end(); it++) {
      work.push_back(*it);
      seen[*it] = true;
      stateList.push_back(*it);
    }

    bool changed(true);
    while (changed) {
      changed = false;
      for (PMM::state state(0); state < pmc->getNumStates(); state++) {
	if (seen[state]) {
	  continue;
	}
	bool allSeen(true);
	for (unsigned succ(0); succ < pmc->getNumSuccStates(state); succ++) {
	  const PMM::state succState(pmc->getSuccState(state, succ));
	  if (!seen[succState]) {
	    allSeen = false;
	  }
	}  
	if (allSeen) {
	  seen[state] = true;
	  stateList.push_back(state);
	  changed = true;
	}
      }
    }

#if 0
    while (!work.empty()) {
      PMM::state state(work.front());
      work.pop_front();
      for (unsigned pred(0); pred < pmc->getNumPredStates(state); pred++) {
	PMM::state predState(pmc->getPredState(state, pred));
	bool hasNotSeenSucc(false);
	for (unsigned predSucc(0); predSucc < pmc->getNumSuccStates(predState); predSucc++) {
	  PMM::state predSuccState(pmc->getSuccState(predState, predSucc));
	  if (!seen[predSuccState]) {
	    hasNotSeenSucc = true;
	    break;
	  }
	}

	if (!hasNotSeenSucc) {
	  if ((0 == targetStates->count(predState))
	      && (!seen[predState])) {
	    work.push_back(predState);
	    stateList.push_back(predState);
	  }
	  seen[predState] = true;
	} else {
	  if ((0 == targetStates->count(predState))
	      && (!seen[predState])) {
	    work.push_back(predState);
	  }
	}
      }
    }
#endif
  }

  void Eliminator::collectStatesOrdered(StateList &stateList) {
    if (("forward" == eliminationOrder)
	|| ("forward-reversed" == eliminationOrder)
	|| ("random" == eliminationOrder)) {
      collectStatesForward(stateList);
    } else if ("backward-opt" == eliminationOrder) {
      collectStatesBackwardOpt(stateList);
    }
    
    if (("forward-reversed" == eliminationOrder)
	|| ("backward-reversed" == eliminationOrder)) {
      stateList.reverse();
    } else if ("random" == eliminationOrder) {
      vector<PMM::state> stateVector(stateList.begin(), stateList.end());
      random_shuffle(stateVector.begin(), stateVector.end());
      stateList.assign(stateVector.begin(), stateVector.end());
    }

    if (0 == stateList.size()) {
      throw runtime_error("Invalid elimination order given.");
    }
  }
    
  void Eliminator::eliminate(std::vector<RationalFunction> &result) {
    /* remove all states except initial and target ones */
    StateList stateList;
    collectStatesOrdered(stateList);
    eliminateStates(stateList);

    result.resize(pmc->getNumStates());
    /* now collect results */

    for (PMM::state state(0); state < pmc->getNumStates(); state++) {
      if ((*needValue)[state]) {
        RationalFunction vValue(0);
        if (targetStates->find(state) != targetStates->end()) {
          if (!rewardAnalysis) {
            vValue = 1;
          } else {
	    vValue = 0;
	  }
        } else {
	  for (unsigned succ(0); succ < pmc->getNumSuccStates(state); succ++) {
	    PMM::state succState(pmc->getSuccState(state, succ));
	    if (targetStates->find(succState) != targetStates->end()) {
	      RationalFunction prob(pmc->getSuccProb(state, succ));
	      if (rewardAnalysis) {
		vValue += prob * pmc->getSuccReward(state, succ);
	      } else {
		vValue += prob;
	      }
	    } else {
	    }
          }
        }
        result[state] = vValue;
      } else {
	result[state] = -1;
      }
    }
  }
}
