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

#include <cassert>
#include <cmath>
#include <limits>
#include <boost/dynamic_bitset.hpp>
#include <boost/tuple/tuple_comparison.hpp>
#include "RegionResult.h"
#include "rationalFunction/RationalFunction.h"
#include "PMC.h"
#include "PMDP.h"
#include "MC.h"
#include "MDP.h"
#include "MDPIterator.h"

namespace parametric {
  using namespace std;
  using namespace boost;
  using namespace rational;

  MDPIterator::MDPIterator() {
    pmdp = NULL;
    mdp = NULL;
  }

  MDPIterator::~MDPIterator() {
    if (NULL != mdp) {
      delete mdp;
    }
  }

  void MDPIterator::setPMDP(const PMDP &__pmdp) {
    pmdp = &__pmdp;
  }

  void MDPIterator::setPoint(const vector<mpq_class> &__point) {
    point = __point;
  }

  void MDPIterator::setPrecision(const double __precision) {
    precision = __precision;
  }

  void MDPIterator::setToleranceFactor(const double __toleranceFactor) {
    toleranceFactor = __toleranceFactor;
  }

  bool MDPIterator::better(const bool min,
			   const double val1, const double val2) const {
    if (min) {
      return val1 < val2;
    } else {
      return val1 > val2;
    }
  }

  void MDPIterator::instantiateMDP(bool useReward) {
    assert(pmdp != NULL);

    if (NULL != mdp) {
      delete mdp;
    }

    unsigned numChoices(0);
    unsigned numCols(0);
    for (unsigned state(0); state < pmdp->getNumStates(); state++) {
      for (unsigned choice(0); choice < pmdp->getNumSuccChoices(state); choice++) {
	numChoices++;
	for (unsigned succ(0); succ < pmdp->getNumSuccStates(state, choice); succ++) {
	  numCols++;
	}
      }
    }
    mdp = new MDP();
    mdp->reserveRowsMem(pmdp->getNumStates());
    mdp->reserveChoicesMem(numChoices);
    mdp->reserveColsMem(numCols);
    if (useReward) {
      mdp->reserveTransRewardsMem(numCols);
    }

    for (unsigned state(0); state < pmdp->getNumStates(); state++) {
      for (unsigned choice(0); choice < pmdp->getNumSuccChoices(state); choice++) {
	for (unsigned succ(0); succ < pmdp->getNumSuccStates(state, choice); succ++) {
	  unsigned succState = pmdp->getSuccState(state, choice, succ);
	  RationalFunction succProb(pmdp->getSuccProb(state, choice, succ));
	  const double succDoubleProb(succProb.evaluate(point).get_d());
	  mdp->addSucc(succState, succDoubleProb);
	  if (useReward) {
	    const double succRew(pmdp->getSuccReward(state, choice, succ).evaluate(point).get_d());
	    mdp->addSuccReward(succRew);
	  }
	}
	mdp->finishChoice();
      }
      mdp->finishState();
    }
  }

  void MDPIterator::next
  (const bool min, const dynamic_bitset<> &targets,
   std::vector<unsigned> &scheduler) {
    instantiateMDP(false);
    scheduler.clear();
    scheduler.resize(targets.size(), numeric_limits<unsigned>::max());
    for (unsigned state(0); state < mdp->getNumStates(); state++) {
      double optVal(min ? 2.0 : -1.0);
      for (unsigned choice(0); choice < mdp->getNumSuccChoices(state); choice++) {
	double choiceProb(0.0);
	for (unsigned succ(0); succ < mdp->getNumSuccStates(state, choice); succ++) {
	  unsigned succState = mdp->getSuccState(state, choice, succ);
	  double succProb = mdp->getSuccProb(state, choice, succ);
	  if (targets[succState]) {
	    choiceProb += succProb;
	  }
	}
	if (better(min, choiceProb, optVal)) {
	  optVal = choiceProb;
	  scheduler[state] = choice;
	}
      }
    }
  }

  void MDPIterator::bounded
  (const bool min, const unsigned steps, const Result &pPres,
   std::vector<unsigned> &scheduler) {
    instantiateMDP(false);
    scheduler.clear();
    const unsigned numStates(pPres.size());
    scheduler.resize(numStates * steps, numeric_limits<unsigned>::max());

    vector<double> pres;
    instantiateVector(pPres, pres);
    vector<double> next(numStates);
    for (unsigned step(0); step < steps; step++) {
      for (unsigned state(0); state < numStates; state++) {
	double optVal(min ? 2.0 : -1.0);
	for (unsigned choice(0); choice < mdp->getNumSuccChoices(state); choice++) {
	  double choiceProb(0.0);
	  for (unsigned succ(0); succ < mdp->getNumSuccStates(state, choice); succ++) {
	    unsigned succState = mdp->getSuccState(state, choice, succ);
	    double succProb = mdp->getSuccProb(state, choice, succ);
	    choiceProb += succProb * pres[succState];
	  }
	  if (better(min, choiceProb, optVal)) {
	    optVal = choiceProb;
	    scheduler[step * numStates + state] = choice;
	  }
	}
	next[state] = optVal;
      }
      pres.swap(next);
    }
  }

  mpq_class MDPIterator::distance(const Point &p1, const Point &p2) {
    mpq_class result(0);

    for (unsigned varNr(0); varNr < p1.size(); varNr++) {
      result += (p2[varNr] - p1[varNr]) * (p2[varNr] - p1[varNr]);
    }

    return result;
  }

  void MDPIterator::unbounded
  (const bool min, const dynamic_bitset<> &targets,
   std::vector<unsigned> &scheduler) {
    if (0 != optSched.count(make_tuple(prop, propNr, point))) {
      scheduler = optSched[make_tuple(prop, propNr, point)].first;
      return;
    }

    scheduler.resize(0);
    scheduler.resize(pmdp->getNumStates(), 0);
    vector<double> prob(pmdp->getNumStates(), 0.0);
    mpq_class minDist(1000);
    for (OptSched::iterator it(optSched.begin()); it != optSched.end(); it++) {
      const tuple<const prismparser::Property*, unsigned, Point> &left(it->first);
      if ((left.get<0>() == prop) && (left.get<1>() == propNr)) {
	if (distance(point, left.get<2>()) < minDist) {
	  scheduler = (it->second).first;
	  prob = (it->second).second;
	  minDist = distance(point, left.get<2>());
	}
      }
    }

    instantiateMDP(false);
    const unsigned numStates(mdp->getNumStates());
    bool changed(true);
    unsigned numIters(0);
    while (changed) {
      numIters++;
      MC mc;
      computeInducedMC(scheduler, false, mc);
      unbounded(mc, targets, prob);
      changed = false;
      for (unsigned state(0); state < numStates; state++) {
	if (targets[state]) {
	  continue;
	}
	double optProb(prob[state]);
	for (unsigned choice(0); choice < mdp->getNumSuccChoices(state); choice++) {
	  double choiceVal(0.0);
	  for (unsigned succ(0); succ < mdp->getNumSuccStates(state, choice); succ++) {
	    const double succProb(mdp->getSuccProb(state, choice, succ));
	    const unsigned succState(mdp->getSuccState(state, choice, succ));
	    choiceVal += succProb * prob[succState];
	  }
	  if (better(min, choiceVal, optProb)) {
	    if (diff(choiceVal, optProb) > toleranceFactor * precision) {
	      changed = true;
	      scheduler[state] = choice;
	      optProb = choiceVal;
	    }
	  }
	}
      }
    }
    optSched.insert(make_pair(make_tuple(prop, propNr, point), make_pair(scheduler, prob)));

#if 0
    instantiateMDP(false);
    const unsigned numStates(targets.size());
    scheduler.resize(numStates);

    vector<double> prob(targets.size());
    for (unsigned state(0); state < numStates; state++) {
      prob[state] = targets[state] ? 1.0 : 0.0;
    }

    bool done(false);
    while (!done) {
      done = true;
      for (unsigned state(0); state < numStates; state++) {
	double optVal(min ? 2.0 : -1.0);
	for (unsigned choice(0); choice < mdp->getNumSuccChoices(state); choice++) {
	  double choiceProb(0.0);
	  for (unsigned succ(0); succ < mdp->getNumSuccStates(state, choice); succ++) {
	    unsigned succState = mdp->getSuccState(state, choice, succ);
	    double succProb = mdp->getSuccProb(state, choice, succ);
	    choiceProb += succProb * prob[succState];
	  }
	  if (better(min, choiceProb, optVal)) {
	    optVal = choiceProb;
	    scheduler[state] = choice;
	  }
	}
	if (diff(prob[state], optVal) >= precision) {
	  done = false;
	}
	prob[state] = optVal;
      }
    }

    if (!min) {
      computeScheduler(targets, prob, scheduler);
    }
#endif
  }

  void MDPIterator::reachReward
  (const bool min, vector<unsigned> &scheduler) {
    if (0 != optSched.count(make_tuple(prop, propNr, point))) {
      scheduler = optSched[make_tuple(prop, propNr, point)].first;
      return;
    }
    scheduler.resize(0);
    scheduler.resize(pmdp->getNumStates(), 0);
    mpq_class minDist(1000);
    vector<double> prob(pmdp->getNumStates(), 0.0);
    for (OptSched::iterator it(optSched.begin()); it != optSched.end(); it++) {
      const tuple<const prismparser::Property*, unsigned, Point> &left(it->first);
      if ((left.get<0>() == prop) && (left.get<1>() == propNr)) {
	if (distance(point, left.get<2>()) < minDist) {
	  scheduler = (it->second).first;
	  prob = (it->second).second;
	  minDist = distance(point, left.get<2>());
	}
      }
    }

    instantiateMDP(true);
    const unsigned numStates(mdp->getNumStates());
    bool changed(true);
    while (changed) {
      MC mc;
      computeInducedMC(scheduler, true, mc);
      reachReward(mc, prob);
      changed = false;
      for (unsigned state(0); state < numStates; state++) {
	double optProb(prob[state]);
	for (unsigned choice(0); choice < mdp->getNumSuccChoices(state); choice++) {
	  double choiceVal(0.0);
	  for (unsigned succ(0); succ < mdp->getNumSuccStates(state, choice); succ++) {
	    const double succProb(mdp->getSuccProb(state, choice, succ));
	    choiceVal += mdp->getSuccReward(state, choice, succ) * succProb;
	    const unsigned succState(mdp->getSuccState(state, choice, succ));
	    choiceVal += succProb * prob[succState];
	  }
	  if (better(min, choiceVal, optProb)) {
	    if (diff(choiceVal, optProb) > toleranceFactor * precision) {
	      changed = true;
	      scheduler[state] = choice;
	      optProb = choiceVal;
	    }
	  }
	}
      }
    }
    optSched.insert(make_pair(make_tuple(prop, propNr, point), make_pair(scheduler, prob)));
  }

  bool MDPIterator::checkUnbounded
  (const bool min, const dynamic_bitset<> &targets,
   const vector<unsigned> &scheduler) {
    if (0 != schedValid.count(make_tuple(prop, propNr, point, scheduler))) {
      return schedValid[make_tuple(prop, propNr, point, scheduler)];
    }

    mpq_class minDist(1000);
    vector<double> prob(pmdp->getNumStates(), 0.0);
    for (OptSched::iterator it(optSched.begin()); it != optSched.end(); it++) {
      const tuple<const prismparser::Property*, unsigned, Point> &left(it->first);
      if ((left.get<0>() == prop) && (left.get<1>() == propNr)) {
	if (distance(point, left.get<2>()) < minDist) {
	  prob = (it->second).second;
	  minDist = distance(point, left.get<2>());
	}
      }
    }

    instantiateMDP(false);
    const unsigned numStates(mdp->getNumStates());
    MC mc;
    computeInducedMC(scheduler, false, mc);
    unbounded(mc, targets, prob);
    for (unsigned state(0); state < numStates; state++) {
      if (targets[state]) {
	continue;
      }
      double optProb(prob[state]);
      for (unsigned choice(0); choice < mdp->getNumSuccChoices(state); choice++) {
	double choiceVal(0.0);
	for (unsigned succ(0); succ < mdp->getNumSuccStates(state, choice); succ++) {
	  const double succProb(mdp->getSuccProb(state, choice, succ));
	  const unsigned succState(mdp->getSuccState(state, choice, succ));
	  choiceVal += succProb * prob[succState];
	}
	if (better(min, choiceVal, optProb)) {
	  if (diff(choiceVal, optProb) > toleranceFactor * precision) {
	    schedValid.insert(make_pair(make_tuple(prop, propNr, point, scheduler), false));
	    return false;
	  }
	}
      }
    }

    schedValid.insert(make_pair(make_tuple(prop, propNr, point, scheduler), true));
    optSched.insert(make_pair(make_tuple(prop, propNr, point), make_pair(scheduler, prob)));

    return true;
  }
  

  bool MDPIterator::checkReachReward
  (const bool min, const vector<unsigned> &scheduler) {
    if (0 != schedValid.count(make_tuple(prop, propNr, point, scheduler))) {
      return schedValid[make_tuple(prop, propNr, point, scheduler)];
    }

    mpq_class minDist(1000);
    vector<double> prob(pmdp->getNumStates(), 0.0);
    for (OptSched::iterator it(optSched.begin()); it != optSched.end(); it++) {
      const tuple<const prismparser::Property*, unsigned, Point> &left(it->first);
      if ((left.get<0>() == prop) && (left.get<1>() == propNr)) {
	if (distance(point, left.get<2>()) < minDist) {
	  prob = (it->second).second;
	  minDist = distance(point, left.get<2>());
	}
      }
    }

    instantiateMDP(true);
    const unsigned numStates(mdp->getNumStates());
    MC mc;
    computeInducedMC(scheduler, true, mc);
    reachReward(mc, prob);
    for (unsigned state(0); state < numStates; state++) {
      double optProb(prob[state]);
      for (unsigned choice(0); choice < mdp->getNumSuccChoices(state); choice++) {
	double choiceVal(0.0);
	for (unsigned succ(0); succ < mdp->getNumSuccStates(state, choice); succ++) {
	  const double succProb(mdp->getSuccProb(state, choice, succ));
	  choiceVal += mdp->getSuccReward(state, choice, succ) * succProb;
	  const unsigned succState(mdp->getSuccState(state, choice, succ));
	  choiceVal += succProb * prob[succState];
	}
	if (better(min, choiceVal, optProb)) {
	  if (diff(choiceVal, optProb) > toleranceFactor * precision) {
	    schedValid.insert(make_pair(make_tuple(prop, propNr, point, scheduler), false));
	    return false;
	  }
	}
      }
    }

    schedValid.insert(make_pair(make_tuple(prop, propNr, point, scheduler), true));
    optSched.insert(make_pair(make_tuple(prop, propNr, point), make_pair(scheduler, prob)));

    return true;
  }

  void MDPIterator::reachReward(const MC &mc, vector<double> &val) const {
    bool done(false);
    while (!done) {
      done = true;
      for (unsigned state(0); state < mc.getNumStates(); state++) {
	double stateVal(mc.getStateReward(state));
	for (unsigned succ(mc.getStateBegin(state)); succ < mc.getStateEnd(state); succ++) {
	  unsigned succState = mc.getSuccState(succ);
	  double succProb = mc.getSuccProb(succ);
	  stateVal += succProb * val[succState];
	}
	if (diff(val[state], stateVal) >= precision) {
	  done = false;
	}
	val[state] = stateVal;
      }
    }
  }

  void MDPIterator::unbounded(const MC &mc, const dynamic_bitset<> &targets,
			      vector<double> &val) const {
    bool done(false);
    for (unsigned state(0); state < mc.getNumStates(); state++) {
      if (targets[state]) {
	val[state] = 1.0;
      } else {
	val[state] = 0.0;
      }
    }
    while (!done) {
      done = true;
      for (unsigned state(0); state < mc.getNumStates(); state++) {
	if (targets[state]) {
	  val[state] = 1.0;
	} else {
	  double stateVal(0.0);
	  for (unsigned succ(mc.getStateBegin(state)); succ < mc.getStateEnd(state); succ++) {
	    unsigned succState = mc.getSuccState(succ);
	    double succProb = mc.getSuccProb(succ);
	    stateVal += succProb * val[succState];
	  }
	  if (diff(val[state], stateVal) >= precision) {
	    done = false;
	  }
	  val[state] = stateVal;
	}
      }
    }
  }

  double MDPIterator::diff(const double val1, const double val2) const {
#if 1
    if ((0.0 == val1) && (0.0 == val2)) {
      return 0.0;
    } else if ((0.0 == val1) || (0.0 == val2)) {
      return numeric_limits<double>::infinity();
    } else {
      return min(abs((val1 - val2) / val1), abs((val1 - val2) / val2));
    }
#else
    return abs(val1 - val2);
#endif
  }

  /**
   * Compute scheduler from unbounded reachability probabilities.
   * Algorithm taken from orange model checking book.
   * TODO cite original source
   */
  void MDPIterator::computeScheduler
  (const dynamic_bitset<> &targets, const vector<double> &prob,
   vector<unsigned> &scheduler) {
    const unsigned numStates(prob.size());
    scheduler.assign(numStates, 0);

    unsigned numChoices(0);
    vector<unsigned> stateStart(numStates);
    for (unsigned state(0); state < pmdp->getNumStates(); state++) {
      stateStart[state] = numChoices;
      numChoices += mdp->getNumSuccChoices(state);
    }

    dynamic_bitset<> maxActions(numChoices, false);

    /* compute optimizing actions */
    for (unsigned state(0); state < numStates; state++) {
      for (unsigned choice(0); choice < mdp->getNumSuccChoices(state); choice++) {
	double cProb = 0.0;
	for (unsigned succ(0); succ < mdp->getNumSuccStates(state, choice); succ++) {
	  const unsigned succState(mdp->getSuccState(state, choice, succ));
	  const double succProb(mdp->getSuccProb(state, choice, succ));
	  cProb += succProb * prob[succState];
	}
	if (abs(cProb - prob[state]) < (precision * toleranceFactor)) {
	  maxActions[stateStart[state] + choice] = true;
	}
      }
    }

    /* compute scheduler */
    /* TODO for large matrices, it would be more efficient to 
       use backward edges, but that's not the bottleneck
        currently.
     */
    dynamic_bitset<> included(numStates, false);

    for (unsigned state(0); state < mdp->getNumStates(); state++) {
      if (targets[state]) {
	included[state] = true;
      }
    }

    bool changed;
    do {
      changed = false;
      for (unsigned state(0); state < numStates; state++) {
	if (!included[state]) {
	  for (unsigned choice(0); choice < mdp->getNumSuccChoices(state); choice++) {
	    if (maxActions[stateStart[state] + choice]) {
	      for (unsigned succ(0); succ < mdp->getNumSuccStates(state, choice); succ++) {
		const unsigned succState(mdp->getSuccState(state, choice, succ));
		if (included[succState]) {
		  included[state] = true;
		  changed = true;
		  scheduler[state] = choice;
		  break;
		}
	      }
	      if (included[state]) {
		break;
	      }
	    }
	  }
	}
      }
    } while (changed);
  }

  void MDPIterator::instantiateVector
  (const Result &pVec, vector<double> &vec) const {
    vec.resize(pVec.size());
    for (unsigned state(0); state < pVec.size(); state++) {
      vec[state] = pVec[state].evaluate(point).get_d();
    }
  }

  void MDPIterator::computeInducedPMC
  (const vector<unsigned> &scheduler, const bool useReward, PMC &pmc) const {
    unsigned colsSize(0);
    for (unsigned state(0); state < pmdp->getNumStates(); state++) {
      const unsigned choice(scheduler[state]);
      colsSize += pmdp->getNumSuccStates(state, choice);
    }
    pmc.reserveRowsMem(pmdp->getNumStates());
    pmc.reserveColsMem(colsSize);
    if (useReward) {
      pmc.reserveTransRewardsMem(colsSize);
    }
    for (unsigned state(0); state < pmdp->getNumStates(); state++) {
      const unsigned choice(scheduler[state]);
      for (unsigned succ(0); succ < pmdp->getNumSuccStates(state, choice); succ++) {
	const unsigned succState(pmdp->getSuccState(state, choice, succ));
	RationalFunction succProb = pmdp->getSuccProb(state, choice, succ);
	pmc.addSucc(succState, succProb);
	if (useReward) {
	  RationalFunction succReward = pmdp->getSuccReward(state, choice, succ);
	  pmc.addSuccReward(succReward);
	}
      }
      pmc.finishState();
      if (pmdp->isInit(state)) {
	pmc.addInit(state);
      }
    }
  }

  void MDPIterator::computeInducedMC
  (const vector<unsigned> &scheduler, bool useReward, MC &mc) const {
    unsigned colsSize(0);
    for (unsigned state(0); state < mdp->getNumStates(); state++) {
      const unsigned choice(scheduler[state]);
      colsSize += mdp->getNumSuccStates(state, choice);
    }
    mc.reserveRowsMem(mdp->getNumStates());
    mc.reserveColsMem(colsSize);
    if (useReward) {
      mc.reserveStateRewardsMem(mdp->getNumStates());
    }
    for (unsigned state(0); state < mdp->getNumStates(); state++) {
      const unsigned choice(scheduler[state]);
      double stateReward(0.0);
      for (unsigned succ(0); succ < mdp->getNumSuccStates(state, choice); succ++) {
	const unsigned succState(mdp->getSuccState(state, choice, succ));
	const double succProb(mdp->getSuccProb(state, choice, succ));
	mc.addSucc(succState, succProb);
	if (useReward) {
	  const double succRew(mdp->getSuccReward(state, choice, succ) * succProb);
	  stateReward += succRew;
	}
      }
      mc.finishState();
      if (useReward) {
	mc.setStateReward(state, stateReward);
      }
    }    
  }

  void MDPIterator::setProperty
  (const prismparser::Property &__prop, const unsigned __propNr) {
    prop = &__prop;
    propNr = __propNr;
  }
}
