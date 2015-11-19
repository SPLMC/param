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
 * Copyright 2009-2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#ifndef ELIMINATOR_H
#define ELIMINATOR_H

#include <string>
#include <map>
#include <set>
#include <list>
#include <boost/dynamic_bitset_fwd.hpp>
#include "rationalFunction/RationalFunction.h"

namespace rational {
  class RationalFunction;
}

namespace parametric {
  typedef std::set<unsigned> StateSet;
  typedef std::list<unsigned> StateList;
  class GPMC;
  class PMDP;

  class Eliminator {
  public:
    Eliminator();
    ~Eliminator();
    void setPMC(GPMC &);
    void setInitStates(StateSet &);
    void setTargetStates(StateSet &);
    void setRewardAnalysis(bool);
    void setEliminationOrder(const std::string &);
    void setNeedValue(const boost::dynamic_bitset<> &);
    void setPMDP(const PMDP &);
    void setBackMap(std::vector<unsigned> &);
    void eliminate(std::vector<rational::RationalFunction> &);
  private:
    void collectStatesOrdered(StateList &);
    void collectStatesForward(StateList &);
    void collectStatesBackwardOpt(StateList &);
    rational::RationalFunction leavingSum(unsigned);
    void eliminateState(unsigned);
    void eliminateStates(StateList &);
    
    GPMC *pmc;
    StateSet *initStates;
    StateSet *targetStates;
    bool rewardAnalysis;
    std::string eliminationOrder;
    const boost::dynamic_bitset<> *needValue;
    const PMDP *pmdp;
    const std::vector<unsigned> *backMap;
  };
}

#endif
