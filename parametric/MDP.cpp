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
#include "MDP.h"

namespace parametric {
  using namespace std;

  MDP::MDP() {
    numStates = 0;
    numChoices = 0;
    rows = NULL;
    choices = NULL;
    cols = NULL;
    nonZeros = NULL;
    transRewards = NULL;
    stateRewards = NULL;
    backRows = NULL;
    backCols = NULL;
    colIndex = 0;
    rewColIndex = 0;
  }

  MDP::~MDP() {
    if (NULL != rows) {
      delete[] rows;
    }
    if (NULL != choices) {
      delete[] choices;
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

  void MDP::reserveRowsMem(unsigned numStates_) {
    assert(NULL == rows);
    rows = new unsigned[numStates_ + 1];
    rows[0] = 0;
  }

  void MDP::reserveChoicesMem(unsigned numChoices_) {
    assert(NULL == choices);
    choices = new unsigned[numChoices_ + 1];
    choices[0] = 0;
  }

  void MDP::reserveColsMem(unsigned numCols_) {
    assert(NULL == cols);
    cols = new unsigned[numCols_];
    nonZeros = new double[numCols_];
  }

  void MDP::reserveStateRewardsMem(unsigned numStates_) {
    assert(NULL == stateRewards);
    stateRewards = new double[numStates_];
  }

  void MDP::reserveTransRewardsMem(unsigned numChoices_) {
    assert(NULL == transRewards);
    transRewards = new double[numChoices_];
  }

  void MDP::addSucc(unsigned succState, double prob) {
    cols[colIndex] = succState;
    nonZeros[colIndex] = prob;
    colIndex++;
  }

  void MDP::addSuccReward(double rew) {
    transRewards[rewColIndex] = rew;
    rewColIndex++;
  }

  void MDP::finishState() {
    numStates++;
    rows[numStates] = numChoices;
  }

  void MDP::finishChoice() {
    numChoices++;
    choices[numChoices] = colIndex;
  }

  unsigned MDP::getNumChoices() const {
    return numChoices;
  }

  unsigned MDP::getNumCols() const {
    return colIndex;
  }

  unsigned MDP::getNumSuccChoices(unsigned state) const {
    return rows[state + 1] - rows[state];
  }

  unsigned MDP::getNumSuccStates(unsigned state, unsigned choice) const {
    return (choices[rows[state] + choice + 1]
	    - choices[rows[state] + choice]);
  }

  unsigned MDP::getSuccState(unsigned state, unsigned choice,
			       unsigned succ) const {
    return cols[choices[rows[state] + choice] + succ];
  }

  double MDP::getSuccProb(unsigned state, unsigned choice,
				      unsigned succ) const {
    return nonZeros[choices[rows[state] + choice] + succ];
  }

  double MDP::getStateReward(unsigned state) const {
    return stateRewards[state];
  }

  double MDP::getSuccReward(unsigned state, unsigned choice, unsigned succ) const {
    return transRewards[choices[rows[state] + choice] + succ];
  }

  void MDP::setStateReward(double rat) {
    stateRewards[numStates] = rat;
  }

  void MDP::setStateReward(unsigned state, double rat) {
    stateRewards[state] = rat;
  }

  unsigned MDP::getNumStates() const {
    return numStates;
  }
  
  bool MDP::useRewards() const {
    return (NULL != transRewards);
  }
}
