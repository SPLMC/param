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

#ifndef SPMDP_H
#define SPMDP_H

#include "rationalFunction/RationalFunction.h"
#include "PMDP.h"

namespace parametric {  
  class SPMDP : public PMDP {
  public:
    SPMDP();
    virtual ~SPMDP();
    void reserveRowsMem(unsigned);
    void reserveChoicesMem(unsigned);
    void reserveColsMem(unsigned);
    void reserveStateRewardsMem(unsigned);
    void reserveTransRewardsMem(unsigned);
    void addSucc(state, rational::RationalFunction);
    void addSuccReward(rational::RationalFunction);
    void finishState();
    void finishChoice();

    unsigned getNumChoices() const;
    unsigned getNumCols() const;
    unsigned getNumSuccChoices(state) const;
    unsigned getNumSuccStates(state, unsigned) const;
    unsigned getSuccState(state, unsigned, unsigned) const;
    rational::RationalFunction getSuccProb(state, unsigned, unsigned) const;
    rational::RationalFunction getStateReward(state) const;
    rational::RationalFunction getSuccReward(state, unsigned, unsigned) const;
    void setStateReward(state, rational::RationalFunction);
    void setStateReward(rational::RationalFunction);
    unsigned getNumStates() const;
    bool useRewards() const;
  private:
    unsigned numStates;
    unsigned numChoices;
    unsigned *rows;
    unsigned *choices;
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
