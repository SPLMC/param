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

#ifndef MDP_H
#define MDP_H

namespace parametric {  
  class MDP {
  public:
    MDP();
    ~MDP();
    void reserveRowsMem(unsigned);
    void reserveChoicesMem(unsigned);
    void reserveColsMem(unsigned);
    void reserveStateRewardsMem(unsigned);
    void reserveTransRewardsMem(unsigned);
    void addSucc(unsigned, double);
    void addSuccReward(double);
    void finishState();
    void finishChoice();

    unsigned getNumChoices() const;
    unsigned getNumCols() const;
    unsigned getNumSuccChoices(unsigned) const;
    unsigned getNumSuccStates(unsigned, unsigned) const;
    unsigned getSuccState(unsigned, unsigned, unsigned) const;
    double getSuccProb(unsigned, unsigned, unsigned) const;
    double getStateReward(unsigned) const;
    double getSuccReward(unsigned, unsigned, unsigned) const;
    void setStateReward(unsigned, double);
    void setStateReward(double);
    unsigned getNumStates() const;
    bool useRewards() const;
  private:
    unsigned numStates;
    unsigned numChoices;
    unsigned *rows;
    unsigned *choices;
    unsigned *cols;
    double *nonZeros;
    double *transRewards;
    double *stateRewards;
    unsigned colIndex;
    unsigned rewColIndex;
    unsigned *backRows;
    unsigned *backCols;
  };
}

#endif
