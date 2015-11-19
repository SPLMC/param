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

#ifndef PMC_H
#define PMC_H

#include "rationalFunction/RationalFunction.h"
#include "PMM.h"

namespace parametric {  
  class PMC : public PMM {
  public:
    virtual void reserveStateRewardsMem(unsigned) = 0;
    virtual void setStateReward(state, rational::RationalFunction) = 0;
    virtual void setStateReward(rational::RationalFunction) = 0;
    virtual void finishState() = 0;
    virtual void computeBackTransitions() = 0;

    virtual unsigned getNumStates() const = 0;
    ModelType getModelType() const;
    virtual unsigned getNumTrans() const = 0;
    virtual unsigned getNumSuccStates(state) const = 0;
    virtual state getSuccState(state, unsigned) const = 0;
    virtual rational::RationalFunction getSuccProb(state, unsigned) const = 0;
    virtual rational::RationalFunction getSuccReward(state, unsigned) const = 0;
    virtual rational::RationalFunction getStateReward(state) const = 0;

    virtual unsigned getNumPredStates(state) const = 0;
    virtual state getPredState(state, unsigned) const = 0;
    virtual state getSuccNrBySuccState(state, state) const = 0;
    virtual rational::RationalFunction getPredProb(state, unsigned) const = 0;
    virtual rational::RationalFunction getPredReward(state, unsigned) const = 0;

    virtual void setSuccProb(state, unsigned, rational::RationalFunction) = 0;
    virtual void setSuccReward(state, unsigned, rational::RationalFunction) = 0;
  };

  void copy(const PMC &, PMC &);
}

#endif
