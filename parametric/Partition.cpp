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

#include "PMC.h"
#include "Partition.h"

namespace parametric {
  
  using namespace std;
  
  Partition::Partition(const std::string &partRefOrder) {
    mayChange.reset(new MayChange(partRefOrder));
  }
  
  void Partition::calcMayChange(EqClass *uEqClass) {
    EqClass::iterator uIt;
    for (uIt = uEqClass->begin(); uIt != uEqClass->end(); uIt++) {    
      PMM::state state(*uIt);
      for (unsigned pred(0); pred < pmc->getNumPredStates(state); pred++) {
	PMM::state predState(pmc->getPredState(state, pred));
        EqClass *vEqClass = &*P_map[predState];
        if (vEqClass->size() > 0) {
          mayChange->push(P_map[predState]);
        }
      }
    }
  }
}
