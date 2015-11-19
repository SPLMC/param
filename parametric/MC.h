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

#ifndef MC_H
#define MC_H

namespace parametric {
  class MC {
  public:
    MC();
    virtual ~MC();
    void reserveRowsMem(unsigned);
    void reserveColsMem(unsigned);
    void reserveStateRewardsMem(unsigned);
    void setStateReward(unsigned, double);
    void setStateReward(double);
    void addSucc(unsigned, double);
    void finishState();

    unsigned getNumStates() const;
    unsigned getNumTrans() const;
    unsigned getStateBegin(unsigned) const;
    unsigned getStateEnd(unsigned) const;
    unsigned getNumSuccStates(unsigned) const;
    unsigned getSuccState(unsigned, unsigned) const;
    double getSuccProb(unsigned, unsigned) const;
    unsigned getSuccState(unsigned) const;
    double getSuccProb(unsigned) const;
    double getStateReward(unsigned) const;
    void setSuccProb(unsigned, unsigned, double);
    void setSuccReward(unsigned, unsigned, double);
    bool useRewards() const;
  private:
    unsigned numStates;
    unsigned *rows;
    unsigned *cols;
    double *nonZeros;
    double *stateRewards;
    unsigned colIndex;
    unsigned rewColIndex;
  };
}

#endif
