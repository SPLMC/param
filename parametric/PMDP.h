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

#ifndef PMDP_H
#define PMDP_H

#include "rationalFunction/RationalFunction.h"
#include "PMM.h"

namespace parametric {  
  class PMDP : public PMM {
  public:
    virtual void reserveChoicesMem(unsigned) = 0;
    virtual void reserveColsMem(unsigned) = 0;
    virtual void reserveTransRewardsMem(unsigned) = 0;
    virtual void finishChoice() = 0;
    ModelType getModelType() const;
    virtual unsigned getNumChoices() const = 0;
    virtual unsigned getNumCols() const = 0;
    virtual unsigned getNumSuccChoices(state) const = 0;
    virtual unsigned getNumSuccStates(state, unsigned) const = 0;
    virtual unsigned getSuccState(state, unsigned, unsigned) const = 0;
    virtual rational::RationalFunction getSuccProb(state, unsigned, unsigned) const = 0;
    virtual rational::RationalFunction getStateReward(state) const = 0;
    virtual rational::RationalFunction getSuccReward(state, unsigned, unsigned) const = 0;
  };
}

#endif
