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
#include "BoundedIterator.h"

namespace parametric {  
  using namespace std;
  using namespace rational;

  void BoundedIterator::setPMC(PMC &pmc__) {
    pmc = &pmc__;
  }

  void BoundedIterator::setPresValues
  (const vector<RationalFunction> &presValues_) {
    presValues = &presValues_;
  }

  void BoundedIterator::setNextValues(vector<RationalFunction> &nextValues_) {
    nextValues = &nextValues_;
  }

  void BoundedIterator::multiply() {
    nextValues->resize(pmc->getNumStates());
    for (PMM::state state(0); state < pmc->getNumStates(); state++) {
      (*nextValues)[state] = 0;
      for (unsigned succ(0); succ < pmc->getNumSuccStates(state); succ++) {
	PMM::state succState = pmc->getSuccState(state, succ);
	RationalFunction succProb = pmc->getSuccProb(state, succ);
	(*nextValues)[state] += succProb * (*presValues)[succState];
      }
    }
  }
}
