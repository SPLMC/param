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
#include "SPMDP.h"

namespace parametric {
  using namespace std;
  using namespace rational;

  SPMDP::SPMDP() {
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

  SPMDP::~SPMDP() {
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

  void SPMDP::reserveRowsMem(unsigned numStates_) {
    assert(NULL == rows);
    rows = new unsigned[numStates_ + 1];
    rows[0] = 0;
    absorbing = new boost::dynamic_bitset<>(numStates_);
  }

  void SPMDP::reserveChoicesMem(unsigned numChoices_) {
    assert(NULL == choices);
    choices = new unsigned[numChoices_ + 1];
    choices[0] = 0;
  }

  void SPMDP::reserveColsMem(unsigned numCols_) {
    assert(NULL == cols);
    cols = new unsigned[numCols_];
    nonZeros = new RationalFunction[numCols_];
  }

  void SPMDP::reserveStateRewardsMem(unsigned numStates_) {
    assert(NULL == stateRewards);
    stateRewards = new RationalFunction[numStates_];
  }

  void SPMDP::reserveTransRewardsMem(unsigned numChoices_) {
    assert(NULL == transRewards);
    transRewards = new RationalFunction[numChoices_];
  }

  void SPMDP::addSucc(state succState, RationalFunction prob) {
    cols[colIndex] = succState;
    nonZeros[colIndex] = prob;
    colIndex++;
  }

  void SPMDP::addSuccReward(RationalFunction rew) {
    transRewards[rewColIndex] = rew;
    rewColIndex++;
  }

  void SPMDP::finishState() {
    numStates++;
    rows[numStates] = numChoices;
  }

  void SPMDP::finishChoice() {
    numChoices++;
    choices[numChoices] = colIndex;
  }

  unsigned SPMDP::getNumChoices() const {
    return numChoices;
  }

  unsigned SPMDP::getNumCols() const {
    return colIndex;
  }

  unsigned SPMDP::getNumSuccChoices(state state) const {
    if (!(*absorbing)[state]) {
      return rows[state + 1] - rows[state];
    } else {
      return 1;
    }
  }

  unsigned SPMDP::getNumSuccStates(state state, unsigned choice) const {
    if (!(*absorbing)[state]) {
      return (choices[rows[state] + choice + 1]
	      - choices[rows[state] + choice]);
    } else {
      return 1;
    }
  }

  unsigned SPMDP::getSuccState(state state, unsigned choice,
			       unsigned succ) const {
    if (!(*absorbing)[state]) {
      return cols[choices[rows[state] + choice] + succ];
    } else {
      return state;
    }
  }

  RationalFunction SPMDP::getSuccProb(state state, unsigned choice,
				      unsigned succ) const {
    if (!(*absorbing)[state]) {
      return nonZeros[choices[rows[state] + choice] + succ];
    } else {
      return 1;
    }
  }

  RationalFunction SPMDP::getStateReward(state state) const {
    if (!(*absorbing)[state]) {
      return stateRewards[state];
    } else {
      return 0;
    }
  }

  RationalFunction SPMDP::getSuccReward(state state, unsigned choice, unsigned succ) const {
    if (!(*absorbing)[state]) {
      return transRewards[choices[rows[state] + choice] + succ];
    } else {
      return 0;
    }
  }

  void SPMDP::setStateReward(RationalFunction rat) {
    stateRewards[numStates] = rat;
  }

  void SPMDP::setStateReward(PMM::state state, RationalFunction rat) {
    stateRewards[state] = rat;
  }

  unsigned SPMDP::getNumStates() const {
    return numStates;
  }
  
  bool SPMDP::useRewards() const {
    return (NULL != transRewards);
  }
}
