/*
 * This file is part of PARAM.
 *
 * PARAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * Free Software Foundation, either version 3 of the License, or
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

#include <cstdlib>
#include <cassert>
#include "MC.h"

namespace parametric {
  MC::MC() {
    numStates = 0;
    rows = NULL;
    cols = NULL;
    nonZeros = NULL;
    stateRewards = NULL;
    colIndex = 0;
    rewColIndex = 0;
  }

  MC::~MC() {
    if (NULL != rows) {
      delete[] rows;
    }
    if (NULL != cols) {
      delete[] cols;
    }
    if (NULL != nonZeros) {
      delete[] nonZeros;
    }
    if (NULL != stateRewards) {
      delete[] stateRewards;
    }
  }

  void MC::reserveRowsMem(unsigned numStates_) {
    assert(NULL == rows);
    rows = new unsigned[numStates_ + 1];
    rows[0] = 0;
  }
  
  void MC::reserveColsMem(unsigned numCols) {
    assert(NULL == cols);
    cols = new unsigned[numCols];
    nonZeros = new double[numCols];
  }

  void MC::reserveStateRewardsMem(unsigned numStates_) {
    assert(NULL == stateRewards);
    stateRewards = new double[numStates_];
  }

  void MC::setStateReward(double rat) {
    stateRewards[numStates] = rat;
  }

  void MC::setStateReward(unsigned state, double rat) {
    stateRewards[state] = rat;
  }

  void MC::addSucc(unsigned succState, double prob) {
    cols[colIndex] = succState;
    nonZeros[colIndex] = prob;
    colIndex++;
  }

  void MC::finishState() {
    numStates++;
    rows[numStates] = colIndex;
  }
  
  unsigned MC::getNumStates() const {
    return numStates;
  }

  unsigned MC::getNumTrans() const {
    return rows[numStates];
  }

  unsigned MC::getNumSuccStates(unsigned state) const {
    return rows[state + 1] - rows[state];
  }

  unsigned MC::getSuccState(unsigned state, unsigned number) const {
    return cols[rows[state] + number];
  }

  double MC::getSuccProb(unsigned state, unsigned number) const {
    return nonZeros[rows[state] + number];    
  }

  double MC::getStateReward(unsigned state) const {
    return stateRewards[state];
  }

  void MC::setSuccProb(unsigned state, unsigned number,
		       double newVal) {
    nonZeros[rows[state] + number] = newVal;
  }

  bool MC::useRewards() const {
    return stateRewards != NULL;
  }

  unsigned MC::getStateBegin(unsigned state) const {
    return rows[state];
  }

  unsigned MC::getStateEnd(unsigned state) const {
    return rows[state + 1];
  }

  unsigned MC::getSuccState(unsigned succ) const {
    return cols[succ];
  }

  double MC::getSuccProb(unsigned succ) const {
    return nonZeros[succ];
  }
}
