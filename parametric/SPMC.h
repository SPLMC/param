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

#ifndef SPMC_H
#define SPMC_H

#include "rationalFunction/RationalFunction.h"
#include "PMC.h"

namespace parametric {
  class SPMC : public PMC {
  public:
    SPMC();
    virtual ~SPMC();
    void reserveRowsMem(unsigned);
    void reserveColsMem(unsigned);
    void reserveTransRewardsMem(unsigned);
    void reserveStateRewardsMem(unsigned);
    void setStateReward(state, rational::RationalFunction);
    void setStateReward(rational::RationalFunction);
    void addSucc(state, rational::RationalFunction);
    void addSuccReward(rational::RationalFunction);
    void finishState();
    void computeBackTransitions();

    unsigned getNumStates() const;
    unsigned getNumTrans() const;
    unsigned getNumSuccStates(state) const;
    state getSuccState(state, unsigned) const;
    rational::RationalFunction getSuccProb(state, unsigned) const;
    rational::RationalFunction getStateReward(state) const;
    rational::RationalFunction getSuccReward(state, unsigned) const;

    unsigned getNumPredStates(state) const;
    state getPredState(state, unsigned) const;
    state getSuccNrBySuccState(state, state) const;
    rational::RationalFunction getPredProb(state, unsigned) const;
    rational::RationalFunction getPredReward(state, unsigned) const;

    void setSuccProb(state, unsigned, rational::RationalFunction);
    void setSuccReward(state, unsigned, rational::RationalFunction);
    bool useRewards() const;
  private:
    unsigned numStates;
    unsigned *rows;
    unsigned *cols;
    rational::RationalFunction *nonZeros;
    rational::RationalFunction *transRewards;
    rational::RationalFunction *stateRewards;
    unsigned colIndex;
    unsigned rewColIndex;
    unsigned *backRows;
    unsigned *backCols;
  };
}

#endif
