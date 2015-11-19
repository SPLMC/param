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

 * You should have received a copy of the GNU General Public License
 * along with PARAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2010-2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#include <cassert>
#include <boost/dynamic_bitset.hpp>
#include "GPMC.h"

namespace parametric {
  using namespace std;
  using namespace rational;

  GPMC::GPMC() {
    numStates = 0;
    succStates = NULL;
    succProbs = NULL;
    succRewards = NULL;
    stateRewards = NULL;
    predStates = NULL;
  }

  GPMC::~GPMC() {
    if (NULL != succStates) {
      delete[] succStates;
    }
    if (NULL != succProbs) {
      delete[] succProbs;
    }
    if (NULL != succRewards) {
      delete[] succRewards;
    }
    if (NULL != stateRewards) {
      delete[] stateRewards;
    }
    if (NULL != predStates) {
      delete[] predStates;
    }
  }

  void GPMC::reserveRowsMem(unsigned numStates_) {
    succStates = new vector<unsigned>[numStates_];
    succProbs = new vector<RationalFunction>[numStates_];
    succRewards = new vector<RationalFunction>[numStates_];
    absorbing = new boost::dynamic_bitset<>(numStates_);
  }
  
  void GPMC::reserveColsMem(unsigned numCols) {
  }

  void GPMC::reserveTransRewardsMem(unsigned numCols) {
  }

  void GPMC::reserveStateRewardsMem(unsigned numStates_) {
    assert(NULL == stateRewards);
    stateRewards = new RationalFunction[numStates_];
  }

  void GPMC::setStateReward(RationalFunction rat) {
    stateRewards[numStates] = rat;
  }

  void GPMC::setStateReward(PMM::state state, RationalFunction rat) {
    stateRewards[state] = rat;
  }

  void GPMC::addSucc(state succState, RationalFunction prob) {
    succStates[numStates].push_back(succState);
    succProbs[numStates].push_back(prob);
  }

  void GPMC::addSuccReward(RationalFunction rew) {
    succRewards[numStates].push_back(rew);
  }

  void GPMC::finishState() {
    numStates++;
  }

  void GPMC::computeBackTransitions() {
    assert(NULL == predStates);
    predStates = new vector<PMM::state>[numStates];
    for (PMM::state state(0); state < numStates; state++) {
      if (!(*absorbing)[state]) {
	for (unsigned succNr(0); succNr < succStates[state].size(); succNr++) {
	  unsigned succState(succStates[state][succNr]);
	  predStates[succState].push_back(state);
	}
      } else {
	predStates[state].push_back(state);	
      }
    }
  }
  
  unsigned GPMC::getNumStates() const {
    return numStates;
  }

  unsigned GPMC::getNumTrans() const {
    unsigned numTrans(0);
    for (PMM::state state(0); state < numStates; state++) {
      numTrans += succStates[state].size();
    }
    
    return numTrans;
  }

  unsigned GPMC::getNumSuccStates(state state) const {
    if (!(*absorbing)[state]) {
      return succStates[state].size();
    } else {
      return 1;
    }
  }

  PMC::state GPMC::getSuccState(state state, unsigned number) const {
    if (!(*absorbing)[state]) {
      return succStates[state][number];
    } else {
      return state;
    }
  }

  RationalFunction GPMC::getSuccProb(state state, unsigned number) const {
    if (!(*absorbing)[state]) {
      return succProbs[state][number];
    } else {
      return 1;
    }
  }

  RationalFunction GPMC::getStateReward(state state) const {
    return stateRewards[state];
  }

  RationalFunction GPMC::getSuccReward(state state, unsigned number) const {
    if (!(*absorbing)[state]) {
      return succRewards[state][number];
    } else {
      return 0;
    }
  }

  unsigned GPMC::getNumPredStates(state state) const {
    return predStates[state].size();
  }
  
  PMC::state GPMC::getPredState(state state, unsigned number) const {
    return predStates[state][number];
  }

  RationalFunction GPMC::getPredProb(state state, unsigned number) const {
    PMC::state pred = getPredState(state, number);
    unsigned numSucc = getNumSuccStates(pred);
    for (unsigned succNr(0); succNr < numSucc; succNr++) {
      if (state == getSuccState(pred, succNr)) {
	return getSuccProb(pred, succNr);
      }
    }
    assert(false);
  }

  RationalFunction GPMC::getPredReward(PMC::state state, unsigned number) const {
    PMC::state pred = getPredState(state, number);
    unsigned numSucc = getNumSuccStates(pred);
    for (unsigned succNr(0); succNr < numSucc; succNr++) {
      if (state == getSuccState(pred, succNr)) {
	return getSuccReward(pred, succNr);
      }
    }
    assert(false);
  }
  
  void GPMC::removeSuccTrans(PMC::state state, unsigned number) {
    if ((*absorbing)[state]) {
       return;
    }
    PMM::state succState(succStates[state][number]);
    succStates[state][number] = succStates[state].back();
    succStates[state].pop_back();
    succProbs[state][number] = succProbs[state].back();
    succProbs[state].pop_back();
    if (0 != succRewards[state].size()) {
      succRewards[state][number] = succRewards[state].back();
      succRewards[state].pop_back();
    }

    if (NULL != predStates) {
      for (unsigned predNr(0); predNr < predStates[succState].size(); predNr++) {
	if (predStates[succState][predNr] == state) {
	  predStates[succState][predNr] = predStates[succState].back();
	  predStates[succState].pop_back();
	  break;
	}
      }
    }
  }

  void GPMC::addSucc(PMC::state state, PMC::state succState, RationalFunction prob) {
    succStates[state].push_back(succState);
    succProbs[state].push_back(prob);
    predStates[succState].push_back(state);
  }

  void GPMC::addSucc(PMC::state state, PMC::state succState, RationalFunction prob,
		     RationalFunction rew) {
    succStates[state].push_back(succState);
    succProbs[state].push_back(prob);
    if (NULL != succRewards) {
      succRewards[state].push_back(rew);
    }
    predStates[succState].push_back(state);
  }

  void GPMC::setSuccProb(state state, unsigned number,
			 RationalFunction newVal) {
    if (!(*absorbing)[state]) {
      succProbs[state][number] = newVal;
    }
  }

  void GPMC::setSuccReward(state state, unsigned number,
			   RationalFunction newVal) {
    if (!(*absorbing)[state]) {
      succRewards[state][number] = newVal;
    }
  }

  PMM::state GPMC::getSuccNrBySuccState(PMM::state state, PMM::state succState) const {
    if ((*absorbing)[state]) {
      if (state == succState) {
	return 0;
      } else {
	return getInvalidState();
      }
    }

    for (unsigned succNr(0); succNr < succStates[state].size(); succNr++) {
      if (succStates[state][succNr] == succState) {
	return succNr;
      }
    }

    return getInvalidState();
  }

  void GPMC::makeAbsorbing(PMC::state state) {
    if (NULL != predStates) {
      for (unsigned succ(0); succ < succStates[state].size(); succ++) {
	PMM::state succState(succStates[state][succ]);
	for (unsigned pred(0); pred < predStates[succState].size(); pred++) {
	  if (predStates[succState][pred] == state) {
	    predStates[succState][pred] = predStates[succState].back();
	    predStates[succState].pop_back();
	  }
	}
      }
    }

    succStates[state].clear();
    succStates[state].push_back(state);
    succProbs[state].clear();
    succProbs[state].push_back(1);

    if (NULL != succRewards) {
      succRewards[state].clear();
      succRewards[state].push_back(0);
    }
  }

  bool GPMC::useRewards() const {
    return succRewards != NULL;
  }
}
