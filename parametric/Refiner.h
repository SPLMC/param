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

#ifndef REFINER_H
#define REFINER_H

#include <set>
#include <vector>
#include "Controller.h"
#include "Partition.h"

namespace parametric {
  typedef std::set<unsigned> StateSet;
  class Refiner {
  public:
    Refiner();
    virtual ~Refiner();
    void setInitialPartition(const std::vector<unsigned> &);
    virtual void createInitialPartition(Partition &);
    virtual void refineClass(EqClass &) = 0;
    virtual void createQuotient(Partition &) = 0;
    void setPartition(Partition &);
    void setRewardAnalysis(bool);
    void setOldPMC(PMC &);
    void setNewPMC(PMC &);
    QuotMap &getQuotMap();
    
  protected:
    const std::vector<unsigned> *initialPartition;
    Partition *partition;
    bool isRewardAnalysis;
    PMC *oldPMC;
    PMC *newPMC;
    QuotMap quotMap;
  };
}

#endif
