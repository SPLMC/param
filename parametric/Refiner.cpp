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

#include <set>
#include "Quotient.h"
#include "Refiner.h"

namespace parametric {
  using namespace std;
  using namespace rational;
  
  Refiner::Refiner() {
  }
  
  Refiner::~Refiner() {
  }
  
  void Refiner::setInitialPartition
  (const vector<unsigned> &initialPartition_) {
    initialPartition = &initialPartition_;
  }

  void Refiner::setPartition(Partition &__partition) {
    partition = &__partition;
  }

  void Refiner::setRewardAnalysis(bool _isRewardAnalysis) {
    isRewardAnalysis = _isRewardAnalysis;
  }

  void Refiner::setOldPMC(PMC &_oldPMC) {
    oldPMC = &_oldPMC;
  }

  void Refiner::setNewPMC(PMC &_newPMC) {
    newPMC = &_newPMC;
  }

  QuotMap &Refiner::getQuotMap() {
    return quotMap;
  }

  /**
   * Creates default initial partition.
   * Notice: This may not be enough for some equivalence classes as an
   * initial partition, for example for weak bisimulation one must split
   * off divergent states. This means you may have to extend this method
   * for some equivalence classes!
   */
  void Refiner::createInitialPartition(Partition &partition) {
    PartitionList &P = partition.P;
    set<PMM::state> s;
    vector<list<StateSet>::iterator> rewardBlocks;
    
    unsigned highestPartNr(0);
    for (PMM::state state(0); state < initialPartition->size(); state++) {
      highestPartNr = max(highestPartNr, (*initialPartition)[state]);
    }
    vector<list<StateSet>::iterator> blocks;
    for (unsigned part(0); part < highestPartNr+1; part++) {
      blocks.push_back(P.insert(P.begin(), s));
    }
    for (PMM::state state(0); state < initialPartition->size(); state++) {
      blocks[(*initialPartition)[state]]->insert(state);
    }    
  }
}
