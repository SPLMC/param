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

#include <cassert>
#include "Controller.h"
#include "Partition.h"
#include "Quotient.h"
#include "Refiner.h"
#include "StrongRefiner.h"
#include "WeakRefiner.h"

namespace parametric {
  
  using namespace std;
  using namespace rational;

  void Quotient::setBisim(const string &_bisim) {
    bisim = _bisim;
  }
  
  void Quotient::setOldPMC(PMC &_oldPMC) {
    oldPMC = &_oldPMC;
  }

  void Quotient::setNewPMC(PMC &_newPMC) {
    newPMC = &_newPMC;
  }

  void Quotient::setPartRefOrder(const string &_partRefOrder) {
    partRefOrder = _partRefOrder;
  }

  void Quotient::setRewardAnalysis(bool _rewardAnalysis) {
    rewardAnalysis = _rewardAnalysis;
  }

  QuotMap &Quotient::getQuotMap() {
    return refiner->getQuotMap();
  }

  Quotient::Quotient() {
    partition = NULL;
    refiner = NULL;
  }

  Quotient::~Quotient() {
    if (NULL != partition) {
      delete partition;
    }
    if (NULL != refiner) {
      delete refiner;
    }
  }

  Partition &Quotient::getPartition() {
    assert(NULL != partition);
    return *partition;
  }

  void Quotient::setInitialPartition
  (const vector<unsigned> &initialPartition_) {
    initialPartition = &initialPartition_;
  }

  void Quotient::setBackMap(vector<unsigned> &backMap_) {
    backMap = &backMap_;
  }

  /**
   * Lump the Markov model.
   */
  void Quotient::quot() {
    if ("weak" == bisim) {
      refiner = new WeakRefiner();
    } else if ("strong" == bisim) {
      refiner = new StrongRefiner();
    } else {
      throw runtime_error("Unsupported bisimulation type \"" + bisim + "\"");
    }
    refiner->setRewardAnalysis(rewardAnalysis);
    refiner->setOldPMC(*oldPMC);
    refiner->setNewPMC(*newPMC);
    
    partition = new Partition(partRefOrder);
    partition->pmc = oldPMC;
    refiner->setInitialPartition(*initialPartition);
    refiner->createInitialPartition(*partition);
    refiner->setPartition(*partition);
    partition->remapAll();
    PartitionList &P = partition->P;
    for (PartitionList::iterator P_it = P.begin(); P_it != P.end(); P_it++) {
      partition->mayChange->push(P_it);
    }
    
    while (!partition->mayChange->empty()) {
      PartitionList::iterator P_it = partition->mayChange->top();
      partition->mayChange->pop();
      EqClass &p = *P_it;
      refiner->refineClass(p);
      partition->afterRefine();
      partition->erase(P_it);
    }
    PMM::state quotState(0);
    backMap->resize(oldPMC->getNumStates());
    for (PartitionList::const_iterator it(partition->begin());
	 it != partition->end(); it++) {
      const EqClass &eqClass(*it);
      for (EqClass::const_iterator it2(eqClass.begin());
	   it2 != eqClass.end(); it2++) {
	const PMM::state originalState(*it2);
	(*backMap)[originalState] = quotState;
      }
      quotState++;
    }
    
    refiner->createQuotient(*partition);
  }
}
