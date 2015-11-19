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
 * Copyright 2010-2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#include <cassert>
#include <boost/dynamic_bitset.hpp>
#include "SPMC.h"

namespace parametric {
  using namespace rational;

  SPMC::SPMC() {
    numStates = 0;
    rows = NULL;
    cols = NULL;
    nonZeros = NULL;
    transRewards = NULL;
    stateRewards = NULL;
    backRows = NULL;
    backCols = NULL;
    colIndex = 0;
    rewColIndex = 0;
  }

  SPMC::~SPMC() {
    if (NULL != rows) {
      delete[] rows;
    }
    if (NULL != cols) {
      delete[] cols;
    }
    if (NULL != nonZeros) {
      delete[] nonZeros;
    }
    if (NULL != transRewards) {
      delete[] transRewards;
    }
    if (NULL != stateRewards) {
      delete[] stateRewards;
    }
    if (NULL != backRows) {
      delete[] backRows;
    }
    if (NULL != backCols) {
      delete[] backCols;
    }
  }

  void SPMC::reserveRowsMem(unsigned numStates_) {
    assert(NULL == rows);
    rows = new unsigned[numStates_ + 1];
    rows[0] = 0;
    absorbing = new boost::dynamic_bitset<>(numStates_);
  }
  
  void SPMC::reserveColsMem(unsigned numCols) {
    assert(NULL == cols);
    cols = new unsigned[numCols];
    nonZeros = new RationalFunction[numCols];
  }

  void SPMC::reserveTransRewardsMem(unsigned numCols) {
    assert(NULL == transRewards);
    transRewards = new RationalFunction[numCols];
  }

  void SPMC::reserveStateRewardsMem(unsigned numStates_) {
    assert(NULL == stateRewards);
    stateRewards = new RationalFunction[numStates_];
  }

  void SPMC::setStateReward(RationalFunction rat) {
    stateRewards[numStates] = rat;
  }

  void SPMC::setStateReward(PMM::state state, RationalFunction rat) {
    stateRewards[state] = rat;
  }

  void SPMC::addSucc(state succState, RationalFunction prob) {
    cols[colIndex] = succState;
    nonZeros[colIndex] = prob;
    colIndex++;
  }

  void SPMC::addSuccReward(RationalFunction rew) {
    transRewards[rewColIndex] = rew;
    rewColIndex++;
  }

  void SPMC::finishState() {
    numStates++;
    rows[numStates] = colIndex;
  }
  
  unsigned SPMC::getNumStates() const {
    return numStates;
  }

  unsigned SPMC::getNumTrans() const {
    return rows[numStates];
  }

  unsigned SPMC::getNumSuccStates(state state) const {
    if (!(*absorbing)[state]) {
      return rows[state + 1] - rows[state];
    } else {
      return 1;
    }
  }

  PMC::state SPMC::getSuccState(state state, unsigned number) const {
    if (!(*absorbing)[state]) {
      return cols[rows[state] + number];
    } else {
      return state;
    }
  }

  RationalFunction SPMC::getSuccProb(state state, unsigned number) const {
    if (!(*absorbing)[state]) {
      return nonZeros[rows[state] + number];    
    } else {
      return 1;
    }
  }

  RationalFunction SPMC::getStateReward(state state) const {
    if (!(*absorbing)[state]) {
      return stateRewards[state];
    } else {
      return 0;
    }
  }

  RationalFunction SPMC::getSuccReward(state state, unsigned number) const {    
    if (!(*absorbing)[state]) {
      return transRewards[rows[state] + number];
    } else {
      return 0;
    }
  }

  unsigned SPMC::getNumPredStates(state state) const {
    assert(NULL != backRows);
    return backRows[state + 1] - backRows[state];
  }
  
  PMC::state SPMC::getPredState(state state, unsigned number) const {
    return backCols[backRows[state] + number];
  }

  RationalFunction SPMC::getPredProb(state state, unsigned number) const {
    PMC::state pred = getPredState(state, number);
    unsigned numSucc = getNumSuccStates(pred);
    for (unsigned succNr(0); succNr < numSucc; succNr++) {
      if (state == getSuccState(pred, succNr)) {
	return getSuccProb(pred, succNr);
      }
    }
    assert(false);
  }

  RationalFunction SPMC::getPredReward(state state, unsigned number) const {
    PMC::state pred = getPredState(state, number);
    unsigned numSucc = getNumSuccStates(pred);
    for (unsigned succNr(0); succNr < numSucc; succNr++) {
      if (state == getSuccState(pred, succNr)) {
	return getSuccReward(pred, succNr);
      }
    }
    assert(false);
  }

  void SPMC::computeBackTransitions() {
    if (NULL != backRows) {
      delete[] backRows;
    }
    if (NULL != backCols) {
      delete[] backCols;
    }
    backRows = new unsigned[numStates + 1];
    for (unsigned state(0); state < numStates; state++) {
      backRows[state] = 0;
    }
    backRows[numStates] = 0;
    for (unsigned state(0); state < numStates; state++) {
      if (!(*absorbing)[state]) {
	for (unsigned succNr(rows[state]); succNr < rows[state + 1]; succNr++) {
	  unsigned succState(cols[succNr]);
	  backRows[succState + 1]++;
	}
      } else {
	backRows[state + 1]++;
      }
    }
    for (unsigned state(0); state < numStates; state++) {
      backRows[state + 1] += backRows[state];
    }
    backCols = new unsigned[backRows[numStates]];
    for (unsigned state(0); state < numStates; state++) {
      if (!(*absorbing)[state]) {
	for (unsigned succNr(rows[state]); succNr < rows[state + 1]; succNr++) {
	  unsigned succState(cols[succNr]);
	  backCols[backRows[succState]] = state;
	  backRows[succState]++;
	}
      } else {
	backCols[backRows[state]] = state;
	backRows[state]++;	
      }
    }
    delete[] backRows;
    backRows = new unsigned[numStates + 1];
    for (unsigned state(0); state < numStates; state++) {
      backRows[state] = 0;
    }
    backRows[numStates] = 0;
    for (unsigned state(0); state < numStates; state++) {
      if (!(*absorbing)[state]) {
	for (unsigned succNr(rows[state]); succNr < rows[state + 1]; succNr++) {
	  unsigned succState(cols[succNr]);
	  backRows[succState + 1]++;
	}
      } else {
	backRows[state + 1]++;
      }
    }
    for (unsigned state(0); state < numStates; state++) {
      backRows[state + 1] += backRows[state];
    }
  }

  void SPMC::setSuccProb(state state, unsigned number,
			 RationalFunction newVal) {
    if (!(*absorbing)[state]) {
      nonZeros[rows[state] + number] = newVal;
    }
  }

  void SPMC::setSuccReward(state state, unsigned number,
			   RationalFunction newVal) {
    if (!(*absorbing)[state]) {
      transRewards[rows[state] + number] = newVal;
    }
  }

  PMM::state SPMC::getSuccNrBySuccState(state state, unsigned succState) const {
    if ((*absorbing)[state]) {
      if (state == succState) {
	return 0;
      } else {
	return getInvalidState();
      }
    }

    for (unsigned succNr(rows[state]); succNr < rows[state + 1]; succNr++) {
      if (cols[succNr] == succState) {
	return succNr - rows[state];
      }
    }

    return getInvalidState();
  }

  bool SPMC::useRewards() const {
    return transRewards != NULL;
  }
}
