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
 * Copyright 2009-2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#ifndef QUOTIENT_H
#define QUOTIENT_H

#include <map>
#include <vector>
#include <string>
#include "PMC.h"
#include "Partition.h"
#include "rationalFunction/RationalFunction.h"

typedef std::map<parametric::PMM::state, rational::RationalFunction> RewardMap;

namespace parametric {
  
  class SparseMC;
  class Partition;
  
  class Quotient {
  public:
    Quotient();
    ~Quotient();
    void setBisim(const std::string &);
    void setRewardAnalysis(bool);
    void setOldPMC(PMC &);
    void setNewPMC(PMC &);
    void setPartRefOrder(const std::string &);
    void setInitialPartition(const std::vector<unsigned> &);
    void setBackMap(std::vector<unsigned> &);
    Partition &getPartition();
    QuotMap &getQuotMap();
    void quot();
  private:
    const std::vector<unsigned> *initialPartition;
    std::vector<unsigned> *backMap;
    Refiner *refiner;
    bool rewardAnalysis;
    PMC *oldPMC;
    StateSet *oldInitStates;
    StateSet *oldTargetStates;
    RewardMap *oldStateRewards;
    PMC *newPMC;
    StateSet *newInitStates;
    StateSet *newTargetStates;
    RewardMap *newStateRewards;
    std::string bisim;
    std::string partRefOrder;
    Partition *partition;
  };
}

#endif
