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

#include <fstream>
#include <cassert>
#include <cmath>
#include <limits>
#include <vector>
#include <tr1/unordered_map>
#include <tr1/unordered_set>
#include <boost/dynamic_bitset.hpp>
#include "rationalFunction/RationalFunction.h"
#include "rationalFunction/ExprConverter.h"
#include "Eliminator.h"
#include "RegionResult.h"
#include "BoundedIterator.h"
#include "ExprToNumber.h"
#include "PMC.h"
#include "GPMC.h"
#include "PMDP.h"
#include "Quotient.h"
#include "prismparser/Model.h"
#include "prismparser/Property.h"
#include "RegionsTODO.h"
#include "IneqChecker.h"
#include "Cache.h"
#include "MDPIterator.h"
#include "ModelChecker.h"

namespace parametric {
  using namespace std;
  using namespace std::tr1;
  using namespace boost;
  using namespace rational;
  using namespace prismparser;

  ofstream out("regions.tex", ios::out);
  map<vector<unsigned>, unsigned> schedMap;

  ModelChecker::ModelChecker() {
    if (2 == RationalFunction::getNumSymbols()) {
      out << "\\begin{tikzpicture}[scale=4.4]\n";
      out << "\\draw[rectangle,fill,color=gray] ";
      out << "(" << RationalFunction::getBoundLeft(0) << ",";
      out << RationalFunction::getBoundLeft(1) << ") rectangle (";
      out << RationalFunction::getBoundRight(0) << ",";
      out << RationalFunction::getBoundRight(1) << ");\n";
    }
      
    pmm = NULL;
    pmc = NULL;
    result = NULL;
    cache = new Cache();
    maxUnknown = -1.0;
    ineqChecker = new IneqChecker();
    ineqChecker->setAssumeNoDenomSignChange(true);
    mdpIterator = new MDPIterator();
  }
  
  ModelChecker::~ModelChecker() {
    delete ineqChecker;
    delete cache;
    delete mdpIterator;
    if (2 == RationalFunction::getNumSymbols()) {
      out << "\\end{tikzpicture}\n";
    }
  }

  void ModelChecker::setModel(PMM &pmm_) {
    assert(NULL == pmm);
    pmm = &pmm_;
    if (PMM::PMC == pmm->getModelType()) {
      pmc = (PMC *) pmm;
    } else if (PMM::PMDP == pmm->getModelType()) {
      pmdp = (PMDP *) pmm;
      mdpIterator->setPMDP(*pmdp);
    }
  }

  void ModelChecker::setProperties(Properties &props_) {
    props = &props_;
  }

  void ModelChecker::setResult(RegionResult &result_) {
    result = &result_;
  }

  void ModelChecker::setExprToNumber(ExprToNumber &exprToNumber_) {
    exprToNumber = &exprToNumber_;
  }

  void ModelChecker::setLumpMethod(const string &lumpMethod_) {
    lumpMethod = lumpMethod_;
  }

  void ModelChecker::setMaxUnknown(const mpq_class &maxUnknown_) {
    maxUnknown = maxUnknown_;
  }

  void ModelChecker::setEliminationOrder
  (const string &eliminationOrder_) {
    eliminationOrder = eliminationOrder_;
  }

  void ModelChecker::setRefineOrder(const string &refineOrder_) {
    refineOrder = refineOrder_;
  }

  void ModelChecker::setRegionSplitMode(const string &regionSplitMode_) {
    regionSplitMode = regionSplitMode_;
  }

  void ModelChecker::setSolver(const string &solver_) {
    solver = solver_;
  }

  void ModelChecker::setISATBinary(const string &iSATBinary_) {
    iSATBinary = iSATBinary_;
  }

  void ModelChecker::setRSolverBinary(const string &RSolverBinary_) {
    RSolverBinary = RSolverBinary_;
  }

  void ModelChecker::setRAHDBinary(const string &RAHDBinary_) {
    RAHDBinary = RAHDBinary_;
  }

  void ModelChecker::setIteratePrecision(const double __iteratePrecision) {
    iteratePrecision = __iteratePrecision;
  }

  void ModelChecker::setToleranceFactor(const double __toleranceFactor) {
    toleranceFactor = __toleranceFactor;
  }

  void ModelChecker::setRandomSchedCheck(const unsigned __randomSchedCheck) {
    randomSchedCheck = __randomSchedCheck;
  }

  void ModelChecker::execute() {
    ineqChecker->setSolver(solver);
    ineqChecker->setISATBinary(iSATBinary);
    ineqChecker->setRSolverBinary(RSolverBinary);
    ineqChecker->setRAHDBinary(RAHDBinary);

    for (unsigned propNr(0); propNr < props->size(); propNr++) {
      const Property *prop = (*props)[propNr].get();
      modelCheckProperty(*prop);
    }
  }

  void ModelChecker::printProgress(const mpq_class &unknown) const {
    cout << "\r                                                           ";
    cout << "\rUnknown area remaining: " << unknown.get_d() << flush;
  }

  void ModelChecker::modelCheckProperty(const Property &prop) {
    RegionsTODO todo;
    todo.createInitialRegion();
    mdpIterator->setPrecision(iteratePrecision);
    mdpIterator->setToleranceFactor(toleranceFactor);
    
    dynamic_bitset<> needResult;
    needResult.resize(pmm->getNumStates(), true);
    while (todo.measure() >= maxUnknown) {
      printProgress(todo.measure());
      Result regionResult;
      Region region;
      todo.popLargestRegion(region);
      if (modelCheckRegion(prop, region, false, needResult, regionResult)) {	
	result->push_back(make_pair(region, regionResult));
      } else {
	split(region, todo);
      }
    }
    printProgress(todo.measure());
    cout << endl;
  }

  bool ModelChecker::modelCheckRegion
  (const Property &prop, const Region &region, bool min,
   const dynamic_bitset<> &needResult, Result &result) {
    switch (prop.kind) {
    case expr:
      return modelCheckExpr(prop, region, min, needResult, result);
      break;
    case binary:
      return modelCheckBinary(prop, region, min, needResult, result);
      break;
    case neg:
      return modelCheckNeg(prop, region, min, needResult, result);
      break;
    case quant:
      return modelCheckQuant(prop, region, min, needResult, result);
      break;
    case next:
      return modelCheckNext(prop, region, min, needResult, result);
      break;
    case until: {
      return modelCheckUntil(prop, region, min, needResult, result);
    }
    case reachability_reward:
      return modelCheckReachReward(prop, region, min, needResult, result);
    default:
      throw runtime_error("Cannot handle this property");
      break;
    }
  }

  bool ModelChecker::modelCheckExpr
  (const Property &prop, const Region &region, bool min,
   const dynamic_bitset<> &needResult, Result &result) {
    const PropExpr &propExpr((const PropExpr &) prop);
    const Expr &expr(propExpr.getExpr());
    const unsigned ap(exprToNumber->getNumberByExpr(expr));
    result.resize(pmm->getNumStates());
    for (PMM::state state(0); state < pmm->getNumStates(); state++) {
      result[state] = (pmm->isAP(state, ap) ? 1 : 0);
    }

    return true;
  }

  bool ModelChecker::modelCheckNeg
  (const Property &prop, const Region &region, bool min,
   const dynamic_bitset<> &needResult, Result &result) {
    const PropNeg &propNeg((const PropNeg &) prop);
    Result innerResult;
    if (!modelCheckRegion(propNeg[0], region, min, needResult, innerResult)) {
      return false;
    }

    result.resize(pmm->getNumStates());
    for (PMM::state state(0); state < pmm->getNumStates(); state++) {
      result[state] = 1 - innerResult[state];
    }

    return true;
  }

  bool ModelChecker::modelCheckBinary
  (const Property &prop, const Region &region, bool min,
   const dynamic_bitset<> &needResult, Result &result) {
    RationalFunction oneHalf(RationalFunction(1) / 2);
    const PropBinary &propBinary((const PropBinary &) prop);
    Result leftResult;
    Result rightResult;
    if (!modelCheckRegion(propBinary[0], region, min, needResult, leftResult)) {
      return false;
    }
    dynamic_bitset<> needResultRight(pmm->getNumStates());
    for (PMM::state state(0); state < pmm->getNumStates(); state++) {
      bool left(leftResult[state] == 1);
      if (PropBinary::OR == propBinary.getOp()) {
	needResultRight[state] = !left;
      } else if (PropBinary::AND == propBinary.getOp()) {
	needResultRight[state] = left;
      } else if (PropBinary::IMPL == propBinary.getOp()) {
	needResultRight[state] = left;
      }
    }
    if (!modelCheckRegion(propBinary[1], region, min, 
			  needResultRight, rightResult)) {
      return false;
    }

    for (PMM::state state(0); state < pmm->getNumStates(); state++) {
      bool left(leftResult[state] == 1);
      bool right(rightResult[state] == 1);
      bool value;
      if (PropBinary::OR == propBinary.getOp()) {
	value = left || right;
      } else if (PropBinary::AND == propBinary.getOp()) {
	value = left && right;
      } else if (PropBinary::IMPL == propBinary.getOp()) {
	value = !left || right;
      }
      result.push_back(value ? 1 : 0);
    }

    return true;
  }

  bool ModelChecker::modelCheckQuant
  (const Property &prop, const Region &region, bool min,
   const dynamic_bitset<> &needResult, Result &result) {
    cache->lookupProperty(prop, region, result);
    if (0 != result.size()) {
      return true;
    }

    const Quant &propQuant((const Quant &) prop);
    dynamic_bitset<> needInnerResult;
    const prismparser::Bound &bound(propQuant.getBound());
    computeNeedInnerResultFromBound(bound, needResult, needInnerResult);
    Result innerResult;
    if ((PMM::PMDP == pmm->getModelType())
	&& (!bound.isMinOrMax())
	&& ((Bound::DK == bound.kind)
	    || (Bound::EQ == bound.kind))) {
      throw runtime_error("Properties of the form \"P=?\" or \"P=<prob>\" "
			  "are not supported for non-deterministic models.");
    }
    if (!modelCheckRegion(propQuant[0], region, propQuant.isMin(),
			  needInnerResult, innerResult)) {
      return false;
    }

    bool fRes(filterByBound(bound, needResult, innerResult, region, result));
    if (fRes) {
      cache->insertProperty(prop, region, result);
    }

    return fRes;
  }

  void ModelChecker::computeNeedInnerResultFromBound
  (const Bound &bound, const dynamic_bitset<> &needResult,
   dynamic_bitset<> &needInnerResult) {
    if (Bound::DK == bound.kind) {
      needInnerResult.resize(pmm->getNumStates(), false);
      for (unsigned state(0); state < needInnerResult.size(); state++) {
	if (pmm->isInit(state)) {
	  needInnerResult[state] = true;
	}
      }
    } else {
      needInnerResult = needResult;
    }
  }


  bool ModelChecker::filterByBound
  (const Bound &bound, const dynamic_bitset<> &needResult,
   const vector<RationalFunction> &innerResult, const Region &region,
   vector<RationalFunction> &result) {
    if (Bound::DK == bound.kind) {
      result = innerResult;
      return true;
    } else {
      bool strict((bound.kind == Bound::LE) || (bound.kind == Bound::GR));
      ExprConverter exprconv;
      RationalFunction probBound(exprconv(bound.getBound()));
      result.resize(pmm->getNumStates());
      bool retVal(true);
      for (unsigned state(0); state < innerResult.size(); state++) {
	if (!needResult[state]) {
	  result[state] = -1;
	  continue;
	}
	RationalFunction checkExprFulf;
	if ((bound.kind == Bound::LE) || (bound.kind == Bound::LEQ)) {
	  checkExprFulf = probBound - innerResult[state];
	} else {
	  checkExprFulf = innerResult[state] - probBound;
	}
	if (ineqChecker->check(checkExprFulf, strict, region)) {
	  result[state] = 1;
	} else {
	  RationalFunction checkExprNotFulf;
	  if ((bound.kind == Bound::LE) || (bound.kind == Bound::LEQ)) {
	    checkExprNotFulf = innerResult[state] - probBound;
	  } else {
	    checkExprNotFulf = probBound - innerResult[state];
	  }
	  if (ineqChecker->check(checkExprNotFulf, !strict, region)) {
	    result[state] = 0;
	  } else {
	    result[state] = -1;
	    retVal = false;
	  }
	}
      }

      return retVal;
    }
  }
  
  bool ModelChecker::modelCheckNext
  (const Property &prop, const Region &region, bool min,
   const dynamic_bitset<> &needResult, Result &result) {
    if (PMM::PMC == pmm->getModelType()) {
      return modelCheckNextPMC(prop, region, min, needResult, result);
    } else if (PMM::PMDP == pmm->getModelType()) {
      return modelCheckNextPMDP(prop, region, min, needResult, result);      
    } else {
      assert(false);
    }
  }

  void ModelChecker::needInnerResultForNextPMC
  (const Property &prop, const dynamic_bitset<> &needResult,
   dynamic_bitset<> &needInnerResult) {
    needInnerResult.resize(0);
    needInnerResult.resize(pmc->getNumStates());

    for (unsigned state(0); state < pmc->getNumStates(); state++) {
      if (needResult[state]) {
	for (unsigned succ(0); succ < pmc->getNumSuccStates(state); succ++) {
	  PMM::state succState = pmc->getSuccState(state, succ);
	  needInnerResult[succState] = true;
	}
      }
    }
  }

  bool ModelChecker::modelCheckNextPMC
  (const Property &prop, const Region &region, bool min,
   const dynamic_bitset<> &needResult, Result &result) {
    cache->lookupProperty(prop, result);
    if (0 != result.size()) {
      return true;
    }

    dynamic_bitset<> needInnerResult;
    needInnerResultForNextPMC(prop, needResult, needInnerResult);

    Result innerResult;
    if (!modelCheckRegion(prop[0], region, min, needInnerResult, innerResult)) {
      return false;
    }
    result.resize(pmc->getNumStates());
    for (unsigned state(0); state < pmc->getNumStates(); state++) {
      result[state] = 0;
      if (needResult[state]) {
	for (unsigned succ(0); succ < pmc->getNumSuccStates(state); succ++) {
	  PMM::state succState = pmc->getSuccState(state, succ);
	  RationalFunction succProb = pmc->getSuccProb(state, succ);
	  result[state] += succProb * innerResult[succState];
	}
      }
    }

    cache->insertProperty(prop, result);
    return true;
  }

  void ModelChecker::needInnerResultForNextPMDP
  (const Property &prop, const dynamic_bitset<> &needResult,
   dynamic_bitset<> &needInnerResult) {
    needInnerResult.resize(0);
    needInnerResult.resize(pmdp->getNumStates());

    for (unsigned state(0); state < pmdp->getNumStates(); state++) {
      if (needResult[state]) {
	for (unsigned choice(0); choice < pmdp->getNumSuccChoices(state); choice++) {
	  for (unsigned succ(0); succ < pmdp->getNumSuccStates(state, choice); succ++) {
	    PMM::state succState = pmdp->getSuccState(state, choice, succ);
	    needInnerResult[succState] = true;
	  }
	}
      }
    }
  }

  void ModelChecker::computeNextPMDPResultByScheduler
  (const Scheduler &scheduler, const dynamic_bitset<> &needResult,
   const Result &innerResult, Result &result) {
    result.resize(pmdp->getNumStates());

    for (unsigned state(0); state < pmdp->getNumStates(); state++) {
      result[state] = 0;
      if (needResult[state]) {
	unsigned choice = scheduler[state];
	for (unsigned succ(0); succ < pmdp->getNumSuccStates(state, choice); succ++) {
	  PMM::state succState = pmdp->getSuccState(state, choice, succ);
	  RationalFunction succProb = pmdp->getSuccProb(state, choice, succ);
	  result[state] += succProb * innerResult[succState];
	}
      }
    }    
  }

  bool ModelChecker::verifyNextPMDPResult
  (const Property &prop, const Scheduler &scheduler,
   const dynamic_bitset<> &needResult, const Result &innerResult,
   const Result &result, const Region &region, const bool min) {
    CheckSet checkSet;
    cache->lookupCheckSet(prop, 0, scheduler, checkSet);
    if (0 == checkSet.size()) {
      for (unsigned state(0); state < pmdp->getNumStates(); state++) {
	if (needResult[state]) {
	  for (unsigned choice(0); choice < pmdp->getNumSuccChoices(state); choice++) {
	    RationalFunction compare(0);
	    for (unsigned succ(0); succ < pmdp->getNumSuccStates(state, choice); succ++) {
	      PMM::state succState = pmdp->getSuccState(state, choice, succ);
	      RationalFunction succProb = pmdp->getSuccProb(state, choice, succ);
	      compare += succProb * innerResult[succState];
	    }
	    compare = min ? compare - result[state] : result[state] - compare;
	    checkSet.insert(compare);
	  }
	}
      }
      cache->insertCheckSet(prop, 0, scheduler, checkSet);
    }
    bool retVal(true);
    for (CheckSetIter it = checkSet.begin(); it != checkSet.end(); it++) {
      if (!ineqChecker->check(*it, false, region)) {
	retVal = false;
      }
    }

    return retVal;
  }

  bool ModelChecker::modelCheckNextPMDP
  (const Property &prop, const Region &region, bool min,
   const dynamic_bitset<> &needResult, Result &result) {
    result.clear();
    cache->lookupProperty(prop, region, result);
    if (0 != result.size()) {
      return true;
    }

    dynamic_bitset<> needInnerResult;
    needInnerResultForNextPMDP(prop, needResult, needInnerResult);
    Result innerResult;
    if (!modelCheckRegion(prop[0], region, min, needInnerResult, innerResult)) {
      return false;
    }

    Scheduler scheduler;
    computeLocalOptNextScheduler(region, min, innerResult, scheduler);
    computeNextPMDPResultByScheduler(scheduler, needResult, innerResult, result);
    if (!verifyNextPMDPResult
	(prop, scheduler, needResult, innerResult,result, region, min)) {
      return false;
    }

    cache->insertProperty(prop, region, result);    
    return true;
  }

  bool ModelChecker::modelCheckUntil
  (const Property &prop, const Region &region, bool min,
   const dynamic_bitset<> &needResult, Result &result) {
    if (PMM::PMC == pmm->getModelType()) {
      return modelCheckUntilPMC(prop, region, min, needResult, result);
    } else if (PMM::PMDP == pmm->getModelType()) {
      return modelCheckUntilPMDP(prop, region, min, needResult, result);      
    } else {
      assert(false);
    }
  }

  void ModelChecker::needInnerResultForUntilPMC
  (const Property &prop, const dynamic_bitset<> &needResult,
   dynamic_bitset<> &needInnerResult) {
    // TODO exclude unneeded states
    needInnerResult.resize(pmm->getNumStates(), true);
  }

  bool ModelChecker::modelCheckUntilPMC
  (const Property &prop, const Region &region, bool min,
   const dynamic_bitset<> &needResult, Result &result) {
    result.clear();
    cache->lookupProperty(prop, result);
    if (0 != result.size()) {
      return true;
    }

    const Until &propUntil((const Until &) prop);
    const Time &time(propUntil.getTime());

    dynamic_bitset<> needInnerResult;
    needInnerResultForUntilPMC(prop, needResult, needInnerResult);
    Result leftResult;
    Result rightResult;
    if (!modelCheckRegion(propUntil[0], region, min, needInnerResult, leftResult)) {
      return false;
    }
    if (!modelCheckRegion(propUntil[1], region, min, needInnerResult, rightResult)) {
      return false;
    }
    result = rightResult;
    for (PMM::state state(0); state < pmm->getNumStates(); state++) {
      pmm->setAbsorbing(state, (0 == leftResult[state]) || (1 == rightResult[state]));
    }
    if (numeric_limits<double>::infinity() == time.t2) {
      unboundedIteratePMC(prop, 0, region, min, needResult, result);
    } else {
      boundedIteratePMC(prop, 0, region, min, time.t2 - time.t1, needResult, result);
    }

    if (time.t1 > 0) {
      boundedIteratePMC(prop, 1, region, min, 1, needResult, result);
      for (unsigned state(0); state < pmm->getNumStates(); state++) {
	result[state] = (1 == leftResult[state]) ? result[state] : 0;
	pmm->setAbsorbing(state, (0 == leftResult[state]));
      }
      boundedIteratePMC(prop, 1, region, min, time.t1 - 1, needResult, result);
    }
    
    cache->insertProperty(prop, result);
    return true;
  }

  void ModelChecker::needInnerResultForUntilPMDP
  (const Property &prop, const dynamic_bitset<> &needResult,
   dynamic_bitset<> &needInnerResult) {
    // TODO exclude unneeded states
    needInnerResult.resize(pmm->getNumStates(), true);
  }

  bool ModelChecker::modelCheckUntilPMDP
  (const Property &prop, const Region &region, bool min,
   const dynamic_bitset<> &needResult, Result &result) {
    result.clear();
    cache->lookupProperty(prop, region, result);
    if (0 != result.size()) {
      return true;
    }

    const Until &propUntil((const Until &) prop);
    const Time &time(propUntil.getTime());

    dynamic_bitset<> needInnerResult;
    needInnerResultForUntilPMDP(prop, needResult, needInnerResult);
    Result leftResult;
    Result rightResult;
    if (!modelCheckRegion(propUntil[0], region, min, needInnerResult, leftResult)) {
      return false;
    }
    if (!modelCheckRegion(propUntil[1], region, min, needInnerResult, rightResult)) {
      return false;
    }

    bool resVal(true);
    result = rightResult;
    for (PMM::state state(0); state < pmm->getNumStates(); state++) {
      pmm->setAbsorbing(state, (0 == leftResult[state]) || (1 == rightResult[state]));
    }
    if (numeric_limits<double>::infinity() == time.t2) {
      resVal = resVal && unboundedIteratePMDP(prop, 0, region, min, needResult, result);
    } else {
      resVal = resVal && boundedIteratePMDP(prop, 0, region, min, time.t2 - time.t1, needResult, result);
    }

    if (time.t1 > 0) {
      resVal = resVal && boundedIteratePMDP(prop, 1, region, min, 1, needResult, result);
      for (unsigned state(0); state < pmm->getNumStates(); state++) {
	result[state] = (1 == leftResult[state]) ? result[state] : 0;
	pmm->setAbsorbing(state, (0 == leftResult[state]));
      }
      resVal = resVal && boundedIteratePMDP(prop, 1, region, min, time.t1 - 1, needResult, result);
    }

    if (!resVal) {
      return false;
    }

    cache->insertProperty(prop, region, result);    
    return true;
  }

  void ModelChecker::computeIdentityBackMap(vector<PMM::state> &backMap) {
    backMap.resize(pmm->getNumStates());
    for (unsigned state(0); state < pmm->getNumStates(); state++) {
      backMap[state] = state;
    }
  }

  void ModelChecker::boundedIteratePMC
  (const Property &prop, const unsigned number, const Region &region, bool min, double time,
   const dynamic_bitset<> &needResult, Result &result) {
    GPMC apmc;
    vector<PMM::state> backMap;
    if ("none" == lumpMethod) {
      copy(*pmc, apmc);
      computeIdentityBackMap(backMap);
    } else {
      RationalFunction::setCleanupMethod(RationalFunction::never);
      Quotient quot;
      if ("weak" == lumpMethod) {
	throw runtime_error
	  ("Weak bisimulation invalid for time-bounded analyses");
      }
      quot.setBisim("strong");
      pmc->computeBackTransitions();
      vector<unsigned> initPart;
      createInitialPartition(result, initPart);
      quot.setInitialPartition(initPart);
      quot.setRewardAnalysis(false);
      quot.setPartRefOrder(refineOrder);
      quot.setOldPMC(*pmc);
      quot.setNewPMC(apmc);
      quot.setBackMap(backMap);
      quot.quot();
    }
    RationalFunction::setCleanupMethod(RationalFunction::always);
    // RationalFunction::setCleanupMethod(RationalFunction::never);
    BoundedIterator iterator;
    vector<RationalFunction> pres;
    pres.resize(apmc.getNumStates());
    for (unsigned state(0); state < pmm->getNumStates(); state++) {
      pres[backMap[state]] = result[state];
    }
    vector<RationalFunction> next;
    iterator.setPresValues(pres);
    iterator.setNextValues(next);
    iterator.setPMC(apmc);
    for (unsigned step(0); step < time; step++) {
      iterator.multiply();
      pres.swap(next);
    }
    
    for (PMM::state state(0); state < pmc->getNumStates(); state++) {
      result[state] = pres[backMap[state]];
    }
  }

  void ModelChecker::unboundedIteratePMC
  (const Property &prop, const unsigned number, const Region &region,
   bool min, const dynamic_bitset<> &needResult,
   Result &result) {
    GPMC apmc;
    vector<PMM::state> backMap;
    if ("none" == lumpMethod) {
      copy(*pmc, apmc);
      computeIdentityBackMap(backMap);
    } else {
      RationalFunction::setCleanupMethod(RationalFunction::never);
      Quotient quot;
      if ("auto" == lumpMethod) {
	quot.setBisim("weak");
      } else {
	quot.setBisim(lumpMethod);
      }
      pmc->computeBackTransitions();
      vector<unsigned> initPart;
      createInitialPartition(result, initPart);
      quot.setInitialPartition(initPart);
      quot.setRewardAnalysis(false);
      quot.setPartRefOrder(refineOrder);
      quot.setOldPMC(*pmc);
      quot.setNewPMC(apmc);
      quot.setBackMap(backMap);
      quot.quot();
    }
    RationalFunction::setCleanupMethod(RationalFunction::always);
    // RationalFunction::setCleanupMethod(RationalFunction::never);
    Eliminator eliminator;
    eliminator.setPMC(apmc);
    set<PMM::state> targetStates;
    eliminator.setTargetStates(targetStates);
    set<PMM::state> initStates;
    boost::dynamic_bitset<> needValue;
    needValue.resize(apmc.getNumStates());
    eliminator.setNeedValue(needValue);
    for (PMM::state state(0); state < apmc.getNumStates(); state++) {
      if (apmc.isInit(state)) {
	initStates.insert(state);
      }
    }
    for (PMM::state state(0); state < pmm->getNumStates(); state++) {
      if (needResult[state]) {
	needValue[backMap[state]] = 1;
      }
    }

    for (PMM::state state(0); state < pmm->getNumStates(); state++) {
      if (1 == result[state]) {
	targetStates.insert(backMap[state]);
      }
    }
    apmc.computeBackTransitions();
    eliminator.setInitStates(initStates);      
    eliminator.setEliminationOrder(eliminationOrder);
    eliminator.setRewardAnalysis(false);
    Result elimResult;
    eliminator.eliminate(elimResult);
    for (PMM::state state(0); state < pmc->getNumStates(); state++) {
      result[state] = elimResult[backMap[state]];
    }
  }

  void ModelChecker::computeBoundedPMDPResultByScheduler
  (const Property &prop, const unsigned number, const Scheduler &scheduler,
   const bool min, Result &result,
   CheckSet &checkSet) {
    Result lookupResult;
    cache->lookup(prop, number, scheduler, lookupResult);
    if (0 == lookupResult.size()) {
      double time(scheduler.size() / pmdp->getNumStates());
      Result next;
      next.resize(result.size());
      for (unsigned step(0); step < (unsigned) time; step++) {
	for (unsigned state(0); state < pmdp->getNumStates(); state++) {
	  unsigned choice(scheduler[step * pmdp->getNumStates() + state]);
	  RationalFunction newVal(0);
	  for (unsigned succ(0); succ < pmdp->getNumSuccStates(state, choice); succ++) {
	    PMM::state succState = pmdp->getSuccState(state, choice, succ);
	    RationalFunction succProb = pmdp->getSuccProb(state, choice, succ);
	    newVal += succProb * result[succState];
	  }
	  next[state] = newVal;
	  for (unsigned cChoice(0); cChoice < pmdp->getNumSuccChoices(state); cChoice++) {
	    RationalFunction compare(0);
	    for (unsigned succ(0); succ < pmdp->getNumSuccStates(state, cChoice); succ++) {
	      PMM::state succState = pmdp->getSuccState(state, cChoice, succ);
	      RationalFunction succProb = pmdp->getSuccProb(state, cChoice, succ);
	      compare += succProb * result[succState];
	    }
	    // TODO only valid for bounded reachabilty!
	    if (min || (0 != compare)) {
	      compare = min ? compare - newVal : newVal - compare;
	      checkSet.insert(compare);
	    }
	  }
	}
	result.swap(next);
      }
      cache->insertCheckSet(prop, number, scheduler, checkSet);
    } else {
      result.swap(lookupResult);
      cache->lookupCheckSet(prop, number, scheduler, checkSet);
    }
  }

  bool ModelChecker::verifyBoundedPMDPResult
  (const CheckSet &checkSet, const Region &region) {
    bool resVal(true);
    for (ConstCheckSetIter it(checkSet.begin()); it != checkSet.end(); it++) {
      if (!ineqChecker->check(*it, false, region)) {
	resVal = false;
      }      
    }

    return resVal;
  }

  bool ModelChecker::boundedIteratePMDP
  (const Property &prop, const unsigned number, const Region &region,
   bool min, double time, const dynamic_bitset<> &needResult, Result &result) {
    Scheduler scheduler;
    mdpIterator->setProperty(prop, number);
    computeLocalOptBoundedScheduler(region, min, result, time, scheduler);
    CheckSet checkSet;
    computeBoundedPMDPResultByScheduler(prop, number, scheduler, min, result, checkSet);
    if (!verifyBoundedPMDPResult(checkSet, region)) {
      return false;
    }

    return true;
  }

  bool ModelChecker::verifyUnboundedPMDPResult
  (const Property &prop, const unsigned number, const Scheduler &scheduler,
   const dynamic_bitset<> &needResult,
   const Result &result, const Region &region, const bool min) {
    CheckSet checkSet;
    cache->lookupCheckSet(prop, number, scheduler, checkSet);
    if (0 == checkSet.size()) {
      for (PMM::state state(0); state < pmdp->getNumStates(); state++) {
	for (unsigned cChoice(0); cChoice < pmdp->getNumSuccChoices(state); cChoice++) {
	  RationalFunction compare(0);
	  for (unsigned succ(0); succ < pmdp->getNumSuccStates(state, cChoice); succ++) {
	    PMM::state succState = pmdp->getSuccState(state, cChoice, succ);
	    RationalFunction succProb = pmdp->getSuccProb(state, cChoice, succ);
	    compare += succProb * result[succState];
	  }
	  compare = min ? compare - result[state] : result[state] - compare;
	  checkSet.insert(compare);
	}
      }
      cache->insertCheckSet(prop, number, scheduler, checkSet);
    }
    bool retVal(true);
    for (CheckSetIter it = checkSet.begin(); it != checkSet.end(); it++) {
      if (!ineqChecker->check(*it, false, region)) {
	retVal = false;
      }
    }
    
    if (retVal) {
      unsigned schedNr;
      if (0 != schedMap.count(scheduler)) {	
	schedNr = schedMap[scheduler];
      } else {
	schedMap[scheduler] = schedMap.size();
	schedNr = schedMap.size();
      }
      out << "\\draw[rectangle,fill,color=white,draw=black] ";
      out << "(" << region[0].first << ","
	  << region[1].first << ")";
      out << " rectangle ";
      out << "(" << region[0].second << ","
	  << region[1].second << ");\n";
      out << "\\node at ("
	  << ((region[0].first + region[0].second) / 2)
	  << "," << ((region[1].first + region[1].second) / 2) << ") {$"
	  << schedNr << "$};\n";
    }

    return retVal;
  }

  void ModelChecker::computeUnboundedPMDPResultByScheduler
  (const Property &prop, const unsigned number, const Scheduler &scheduler, Result &result) {
    Result lookupResult;
    cache->lookup(prop, number, scheduler, lookupResult);
    if (0 == lookupResult.size()) {
      GPMC induced;
      mdpIterator->computeInducedPMC(scheduler, false, induced);    
      GPMC apmc;
      vector<PMM::state> backMap;
      if ("none" == lumpMethod) {
	copy(induced, apmc);
	computeIdentityBackMap(backMap);
      } else {
	RationalFunction::setCleanupMethod(RationalFunction::never);
	Quotient quot;
	if ("auto" == lumpMethod) {
	  quot.setBisim("weak");
	} else {
	  quot.setBisim(lumpMethod);
	}
	induced.computeBackTransitions();
	vector<unsigned> initPart;
	createInitialPartition(result, initPart);
	quot.setInitialPartition(initPart);
	quot.setRewardAnalysis(false);
	quot.setPartRefOrder(refineOrder);
	quot.setOldPMC(induced);
	quot.setNewPMC(apmc);
	quot.setBackMap(backMap);
	quot.quot();
      }
      RationalFunction::setCleanupMethod(RationalFunction::always);
      // RationalFunction::setCleanupMethod(RationalFunction::never);
      Eliminator eliminator;
      eliminator.setPMC(apmc);
      eliminator.setPMDP(*pmdp);
      eliminator.setBackMap(backMap);
      set<PMM::state> targetStates;
      eliminator.setTargetStates(targetStates);
      set<PMM::state> initStates;
      boost::dynamic_bitset<> needValue;
      // TODO if possible, exclude values not really needed
      needValue.resize(apmc.getNumStates(), true);
      eliminator.setNeedValue(needValue);
      for (PMM::state state(0); state < apmc.getNumStates(); state++) {
	if (apmc.isInit(state)) {
	  initStates.insert(state);
	}
      }
      
      for (PMM::state state(0); state < pmm->getNumStates(); state++) {
	if (1 == result[state]) {
	  targetStates.insert(backMap[state]);
	}
      }
      apmc.computeBackTransitions();
      eliminator.setInitStates(initStates);      
      eliminator.setEliminationOrder(eliminationOrder);
      eliminator.setRewardAnalysis(false);
      Result elimResult;
      eliminator.eliminate(elimResult);
      for (PMM::state state(0); state < induced.getNumStates(); state++) {
	result[state] = elimResult[backMap[state]];
      }
      cache->insert(prop, number, scheduler, result);
    } else {
      result.swap(lookupResult);
    }
  }

  bool ModelChecker::unboundedIteratePMDP
  (const Property &prop, const unsigned number, const Region &region,
   bool min, const dynamic_bitset<> &needResult,
   Result &result) {
    Scheduler scheduler;
    dynamic_bitset<> targets(result.size());
    for (unsigned state(0); state < result.size(); state++) {
      targets[state] = (1 == result[state]);
    }
    vector<mpq_class> point;
    list<Scheduler> &schedList(cache->lookupScheduler(prop));
    list<Scheduler>::iterator it2;
    bool candidateFound(false);
    mdpIterator->setProperty(prop, number);
    for (it2 = schedList.begin(); it2 != schedList.end(); it2++) {
      const Scheduler &listScheduler = *it2;
      region.getMidPoint(point);
      mdpIterator->setPoint(point);
      if (mdpIterator->checkUnbounded(min, targets, listScheduler)) {
	candidateFound = true;
	for (unsigned edgeNr(0); edgeNr < region.getNumEdges(); edgeNr++) {
	  region.getEdgePoint(edgeNr, point);
	  mdpIterator->setPoint(point);
	  if (!mdpIterator->checkUnbounded(min, targets, listScheduler)) {
	    candidateFound = false;
	    break;
	  }
	}
	if (candidateFound) {
	  for (unsigned i(0); i < randomSchedCheck; i++) {
	    region.getRandomPoint(point);
	    mdpIterator->setPoint(point);
	    if (!mdpIterator->checkUnbounded(min, targets, listScheduler)) {
	      candidateFound = false;
	      break;
	    }
	  }
	}
	if (candidateFound) {
	  scheduler = listScheduler;
	  break;
	}
      }
    }

    if (!candidateFound) {
      candidateFound = true;
      region.getMidPoint(point);
      mdpIterator->setPoint(point);
      mdpIterator->unbounded(min, targets, scheduler);
      for (unsigned edgeNr(0); edgeNr < region.getNumEdges(); edgeNr++) {
	region.getEdgePoint(edgeNr, point);
	mdpIterator->setPoint(point);
	if (!mdpIterator->checkUnbounded(min, targets, scheduler)) {
	  candidateFound = false;
	  break;
	}
      }
      if (candidateFound) {
	for (unsigned i(0); i < randomSchedCheck; i++) {
	  region.getRandomPoint(point);
	  mdpIterator->setPoint(point);
	  if (!mdpIterator->checkUnbounded(min, targets, scheduler)) {
	    candidateFound = false;
	    break;
	  }
	}
      }
      if (candidateFound) {
	cache->insertScheduler(prop, scheduler);
      }
    }

    if (!candidateFound) {
      return false;
    }

    computeUnboundedPMDPResultByScheduler(prop, 0, scheduler, result);

    return verifyUnboundedPMDPResult
      (prop, number, scheduler, needResult, result, region, min);
  }

  bool ModelChecker::modelCheckReachReward
  (const Property &prop, const Region &region, bool min,
   const dynamic_bitset<> &needResult, Result &result) {
    if (PMM::PMC == pmm->getModelType()) {
      return modelCheckReachRewardPMC(prop, region, min, needResult, result);
    } else if (PMM::PMDP == pmm->getModelType()) {
      return modelCheckReachRewardPMDP(prop, region, min, needResult, result);      
    } else {
      assert(false);
    }
  }

  bool ModelChecker::modelCheckReachRewardPMC
  (const Property &prop, const Region &region, bool min,
   const dynamic_bitset<> &needResult, Result &result) {
    cache->lookupProperty(prop, region, result);
    if (0 != result.size()) {
      return true;
    }

    Result rewards;
    cache->lookup(prop, 0, rewards);
    const ReachabilityReward &propReachRew((const ReachabilityReward &) prop);
    if (0 == rewards.size()) {
      Result innerResult;
      // TODO restrict to relevant states
      dynamic_bitset<> needInnerResult(pmm->getNumStates(), true);
      if (!modelCheckRegion(propReachRew[0], region, min, needInnerResult, innerResult)) {
	return false;
      }
      
      GPMC apmc;
      vector<PMM::state> backMap;
      if ("none" == lumpMethod) {
	copy(*pmc, apmc);
	computeIdentityBackMap(backMap);
      } else {
	RationalFunction::setCleanupMethod(RationalFunction::never);
	Quotient quot;
	if ("weak" == lumpMethod) {
	  throw runtime_error
	    ("Weak bisimulation is not supported for reachability rewards.");
	}
	quot.setBisim("strong");
	for (PMM::state state(0); state < pmc->getNumStates(); state++) {
	  pmc->setAbsorbing(state, (1 == innerResult[state]));
	}
	pmc->computeBackTransitions();
	vector<unsigned> initPart;
	createInitialPartition(innerResult, initPart);
	quot.setInitialPartition(initPart);
	quot.setRewardAnalysis(true);
	quot.setPartRefOrder(refineOrder);
	quot.setOldPMC(*pmc);
	quot.setNewPMC(apmc);
	quot.setBackMap(backMap);
	quot.quot();
      }
      RationalFunction::setCleanupMethod(RationalFunction::always);
      // RationalFunction::setCleanupMethod(RationalFunction::never);
      RationalFunction::removeUnusedSymbols();
      Eliminator eliminator;
      eliminator.setPMC(apmc);
      set<PMM::state> targetStates;
      eliminator.setTargetStates(targetStates);
      set<PMM::state> initStates;
      boost::dynamic_bitset<> needValue(apmc.getNumStates());
      eliminator.setNeedValue(needValue);
      for (PMM::state state(0); state < apmc.getNumStates(); state++) {
	if (apmc.isInit(state)) {
	  initStates.insert(state);
	}
      }
      for (PMM::state state(0); state < pmc->getNumStates(); state++) {
	if (1 == innerResult[state]) {
	  targetStates.insert(backMap[state]);
	}
	apmc.setAbsorbing(backMap[state], (1 == innerResult[state]));
	if (needResult[state]) {
	  needValue[backMap[state]] = true;
	}
      }
      apmc.computeBackTransitions();
      eliminator.setInitStates(initStates);      
      eliminator.setEliminationOrder(eliminationOrder);
      eliminator.setRewardAnalysis(true);
      Result elimResult;
      eliminator.eliminate(elimResult);

      rewards.resize(pmc->getNumStates());
      for (PMM::state state(0); state < pmc->getNumStates(); state++) {
	rewards[state] = elimResult[backMap[state]];
      }
      cache->insert(prop, 0, rewards);
    }

    const Bound &bound(propReachRew.getBound());
    dynamic_bitset<> needInnerResult;
    computeNeedInnerResultFromBound(bound, needResult, needInnerResult);
    bool fRes(filterByBound(bound, needInnerResult, rewards, region, result));
    if (fRes) {
      cache->insertProperty(prop, region, result);
    }

    return fRes;
  }

  bool ModelChecker::modelCheckReachRewardPMDP
  (const Property &prop, const Region &region, bool min,
   const dynamic_bitset<> &needResult, Result &result) {
    mdpIterator->setProperty(prop, 0);
    const ReachabilityReward &propReachRew((const ReachabilityReward &) prop);

    const Bound &bound(propReachRew.getBound());
    min = bound.isMin();

    cache->lookupProperty(prop, region, result);
    if (0 != result.size()) {
      return true;
    }

    Result innerResult;
    dynamic_bitset<> needInnerResult(pmdp->getNumStates(), true);
    //    findReachable(needResult, needInnerResult);

    if (!modelCheckRegion(propReachRew[0], region, min, needInnerResult, innerResult)) {
      return false;
    }

    for (unsigned state(0); state < pmdp->getNumStates(); state++) {
      pmdp->setAbsorbing(state, (1 == innerResult[state]));
    }

    vector<mpq_class> point;
    list<Scheduler> &schedList(cache->lookupScheduler(prop));
    list<Scheduler>::iterator it2;
    bool candidateFound(false);
    Scheduler scheduler;
    for (it2 = schedList.begin(); it2 != schedList.end(); it2++) {
      const Scheduler &listScheduler = *it2;
      region.getMidPoint(point);
      mdpIterator->setPoint(point);
      if (mdpIterator->checkReachReward(min, listScheduler)) {
	candidateFound = true;
	for (unsigned edgeNr(0); edgeNr < region.getNumEdges(); edgeNr++) {
	  region.getEdgePoint(edgeNr, point);
	  mdpIterator->setPoint(point);
	  if (!mdpIterator->checkReachReward(min, listScheduler)) {
	    candidateFound = false;
	    break;
	  }
	}
	if (candidateFound) {
	  for (unsigned i(0); i < randomSchedCheck; i++) {
	    region.getRandomPoint(point);
	    mdpIterator->setPoint(point);
	    if (!mdpIterator->checkReachReward(min, listScheduler)) {
	      candidateFound = false;
	      break;
	    }
	  }
	}
	if (candidateFound) {
	  scheduler = listScheduler;
	  break;
	}
      }
    }

    if (!candidateFound) {
      candidateFound = true;
      region.getMidPoint(point);
      mdpIterator->setPoint(point);
      mdpIterator->reachReward(min, scheduler);
      for (unsigned edgeNr(0); edgeNr < region.getNumEdges(); edgeNr++) {
	region.getEdgePoint(edgeNr, point);
	mdpIterator->setPoint(point);
	if (!mdpIterator->checkReachReward(min, scheduler)) {
	  candidateFound = false;
	  break;
	}
      }
      if (candidateFound) {
	for (unsigned i(0); i < randomSchedCheck; i++) {
	  region.getRandomPoint(point);
	  mdpIterator->setPoint(point);
	  if (!mdpIterator->checkReachReward(min, scheduler)) {
	    candidateFound = false;
	    break;
	  }
	}
      }
      if (candidateFound) {
	cache->insertScheduler(prop, scheduler);
      }
    }

    if (!candidateFound) {
      return false;
    }

    Result rewards;
    cache->lookup(prop, 0, scheduler, rewards);

    if (0 == rewards.size()) {
      GPMC induced;
      mdpIterator->computeInducedPMC(scheduler, true, induced);    
      GPMC apmc;
      vector<PMM::state> backMap;
      
      if ("none" == lumpMethod) {
	copy(induced, apmc);
	computeIdentityBackMap(backMap);
      } else {
	RationalFunction::setCleanupMethod(RationalFunction::never);
	Quotient quot;
	if ("weak" == lumpMethod) {
	  throw runtime_error
	    ("Weak bisimulation is not supported for reachability rewards.");
	}
	quot.setBisim("strong");
	
	induced.computeBackTransitions();
	vector<unsigned> initPart;
	createInitialPartition(innerResult, initPart);
	quot.setInitialPartition(initPart);
	quot.setRewardAnalysis(true);
	quot.setPartRefOrder(refineOrder);
	quot.setOldPMC(induced);
	quot.setNewPMC(apmc);
	quot.setBackMap(backMap);
	quot.quot();
      }
      RationalFunction::setCleanupMethod(RationalFunction::always);
      // RationalFunction::setCleanupMethod(RationalFunction::never);
      RationalFunction::removeUnusedSymbols();
      Eliminator eliminator;
      eliminator.setPMC(apmc);
      //	eliminator.setPMDP(*pmdp);
      eliminator.setBackMap(backMap);
      set<PMM::state> targetStates;
      eliminator.setTargetStates(targetStates);
      set<PMM::state> initStates;
      //      findReachable(needResult, needInnerResult);
      needInnerResult.clear();
      needInnerResult.resize(pmdp->getNumStates(), true);
      boost::dynamic_bitset<> needValue(apmc.getNumStates(), false);
      for (PMM::state state(0); state < pmdp->getNumStates(); state++) {
	if (needInnerResult[state]) {
	  needValue[backMap[state]] = true;
	}
      }

      for (PMM::state state(0); state < apmc.getNumStates(); state++) {
	if (apmc.isInit(state)) {
	  initStates.insert(state);
	}
      }
      for (PMM::state state(0); state < induced.getNumStates(); state++) {
	if (1 == innerResult[state]) {
	  targetStates.insert(backMap[state]);
	}
      }
      eliminator.setNeedValue(needValue);
      apmc.computeBackTransitions();
      eliminator.setInitStates(initStates);      
      eliminator.setEliminationOrder(eliminationOrder);
      eliminator.setRewardAnalysis(true);
      Result elimResult;
      eliminator.eliminate(elimResult);
      rewards.resize(induced.getNumStates());
      for (PMM::state state(0); state < induced.getNumStates(); state++) {
	rewards[state] = elimResult[backMap[state]];
      }
      cache->insert(prop, 0, scheduler, rewards);
    }
    
    CheckSet checkSet;
    cache->lookupCheckSet(prop, 0, scheduler, checkSet);
    if (0 == checkSet.size()) {
      for (PMM::state state(0); state < pmm->getNumStates(); state++) {
	if (1 == innerResult[state]) {
	  continue;
	}
	if (!needInnerResult[state]) {
	  continue;
	}
	RationalFunction rew(0);
	for (unsigned succ(0); succ < pmdp->getNumSuccStates(state, scheduler[state]); succ++) {
	  PMM::state succState = pmdp->getSuccState(state, scheduler[state], succ);
	  RationalFunction succProb = pmdp->getSuccProb(state, scheduler[state], succ);
	  rew += succProb * rewards[succState];
	  rew += pmdp->getSuccReward(state, scheduler[state], succ) * succProb;
	}
	for (unsigned cChoice(0); cChoice < pmdp->getNumSuccChoices(state); cChoice++) {
	  RationalFunction compare(0);
	  for (unsigned succ(0); succ < pmdp->getNumSuccStates(state, cChoice); succ++) {
	    PMM::state succState = pmdp->getSuccState(state, cChoice, succ);
	    RationalFunction succProb = pmdp->getSuccProb(state, cChoice, succ);
	    compare += succProb * rewards[succState];
	    compare += pmdp->getSuccReward(state, cChoice, succ) * succProb;
	  }
	  compare = min ? compare - rew : rew - compare;
	  checkSet.insert(compare);
	}
      }
      cache->insertCheckSet(prop, 0, scheduler, checkSet);
    }
    bool resValue(true);
    for (CheckSetIter it = checkSet.begin(); it != checkSet.end(); it++) {
      if (!ineqChecker->check(*it, false, region)) {
	resValue = false;
	break;
      }
    }

    if (!resValue) {
      return false;
    }

    unsigned schedNr;
    if (0 != schedMap.count(scheduler)) {	
      schedNr = schedMap[scheduler];
    } else {
      schedMap[scheduler] = schedMap.size();
      schedNr = schedMap.size();
    }
    out << "\\draw[rectangle,fill,color=white,draw=black] ";
    out << "(" << region[0].first << ","
	<< region[1].first << ")";
    out << " rectangle ";
    out << "(" << region[0].second << ","
	<< region[1].second << ");\n";
    out << "\\node at ("
	<< ((region[0].first + region[0].second) / 2)
	<< "," << ((region[1].first + region[1].second) / 2) << ") {\\small $"
	<< schedNr << "$};\n";

    computeNeedInnerResultFromBound(bound, needResult, needInnerResult);
    bool fRes(filterByBound(bound, needInnerResult, rewards, region, result));
    if (fRes) {
      cache->insertProperty(prop, region, result);
    }

    return fRes;
  }

  void ModelChecker::createInitialPartition
  (const Result &values, vector<unsigned> &partition) {
    partition.clear();
    unordered_map<RationalFunction,unsigned> valuesEnum;
    unsigned number(0);
    for (unsigned state(0); state < values.size(); state++) {
      if (0 == valuesEnum.count(values[state])) {
	valuesEnum.insert(make_pair(values[state], number));
	number++;
      }
    }

    for (unsigned state(0); state < values.size(); state++) {
      partition.push_back(valuesEnum[values[state]]);
    }
  }

  void ModelChecker::splitAll(const Region &region, RegionsTODO &todo) {
    unsigned numParts(2);

    const unsigned numSymbols(RationalFunction::getNumSymbols());
    const unsigned numNewRegions(unsigned(pow(numParts, numSymbols)));

    for (unsigned newRegionNr(0); newRegionNr < numNewRegions; newRegionNr++) {
      unsigned regionRest(newRegionNr);
      Region newRegion;
      for (unsigned symNr(0); symNr < numSymbols; symNr++) {
	unsigned lower(regionRest % numParts);
	regionRest /= numParts;
	const Interval &oldInterval(region[symNr]);
	Interval newInterval;
	mpq_class partLength((oldInterval.second - oldInterval.first) / numParts);
	newInterval.first = oldInterval.first + partLength * lower;
	newInterval.second = oldInterval.first + partLength * (lower + 1);	
	newRegion.push_back(newInterval);
      }
      todo.insertRegion(newRegion);
    }
  }

  void ModelChecker::splitLongest(const Region &region, RegionsTODO &todo) {
    const unsigned numSymbols(RationalFunction::getNumSymbols());
    mpq_class longestSize(0);
    unsigned longestSym(0);
    for (unsigned symNr(0); symNr < numSymbols; symNr++) {
      const Interval &oldInterval(region[symNr]);
      mpq_class length(oldInterval.second - oldInterval.first);
      if (length > longestSize) {
	longestSize = length;
	longestSym = symNr;
      }
    }

    Region newRegion1;
    Region newRegion2;
    for (unsigned symNr(0); symNr < numSymbols; symNr++) {
      const Interval &oldInterval(region[symNr]);
      mpq_class length(oldInterval.second - oldInterval.first);
      if (longestSym == symNr) {
	mpq_class midPoint((oldInterval.first + oldInterval.second) / 2);
	Interval newInterval1;
	newInterval1.first = oldInterval.first;
	newInterval1.second = midPoint;
	newRegion1.push_back(newInterval1);
	Interval newInterval2;
	newInterval2.first = midPoint;
	newInterval2.second = oldInterval.second;
	newRegion2.push_back(newInterval2);
      } else {
	newRegion1.push_back(oldInterval);
	newRegion2.push_back(oldInterval);
      }
    }

    todo.insertRegion(newRegion1);
    todo.insertRegion(newRegion2);
  }

  void ModelChecker::split(const Region &region, RegionsTODO &todo) {
    if ("all" == regionSplitMode) {
      splitAll(region, todo);
    } else if ("longest" == regionSplitMode) {
      splitLongest(region, todo);
    } else {
      throw runtime_error("split mode \"" + regionSplitMode
			  + "\" not supported");
    }
  }

  void ModelChecker::computeLocalOptNextScheduler
  (const Region &region, const bool min, const Result &ptargets,
   Scheduler &scheduler) const {
    vector<mpq_class> point;
    region.getMidPoint(point);
    mdpIterator->setPoint(point);
    dynamic_bitset<> targets(ptargets.size());
    for (unsigned state(0); state < ptargets.size(); state++) {
      targets[state] = (1 == ptargets[state]);
    }
    mdpIterator->next(min, targets, scheduler);
  }

  void ModelChecker::computeLocalOptUnboundedScheduler
  (const Region &region, const bool min, const Result &ptargets,
   Scheduler &scheduler) const {
    vector<mpq_class> point;
    region.getMidPoint(point);
    mdpIterator->setPoint(point);
    dynamic_bitset<> targets(ptargets.size());
    for (unsigned state(0); state < ptargets.size(); state++) {
      targets[state] = (1 == ptargets[state]);
    }
    mdpIterator->unbounded(min, targets, scheduler);
  }

  void ModelChecker::computeLocalOptBoundedScheduler
  (const Region &region, const bool min, const Result &ptargets,
   const double time, Scheduler &scheduler) const {
    vector<mpq_class> point;
    region.getMidPoint(point);
    mdpIterator->setPoint(point);
    mdpIterator->bounded(min, (unsigned) time, ptargets, scheduler);
  }

  void ModelChecker::findReachable
  (const dynamic_bitset<> &init, dynamic_bitset<> &reach) const {
    if (pmc != NULL) {
      reach.resize(0);
      reach.resize(pmc->getNumStates());
      list<unsigned> work;
      for (unsigned state(0); state < init.size(); state++) {
	if (init[state]) {
	  work.push_back(state);
	  reach[state] = true;
	}
      }
      while (!work.empty()) {
	PMM::state state(work.front());
	work.pop_front();
	for (unsigned succ(0); succ < pmc->getNumSuccStates(state); succ++) {
	  PMM::state succState(pmc->getSuccState(state, succ));
	  if (!reach[succState]) {
	    work.push_back(succState);
	    reach[succState] = true;
	  }
	}
      }
    } else {
      reach.resize(0);
      reach.resize(pmdp->getNumStates());
      list<unsigned> work;
      for (unsigned state(0); state < init.size(); state++) {
	if (init[state]) {
	  work.push_back(state);
	  reach[state] = true;
	}
      }
      while (!work.empty()) {
	PMM::state state(work.front());
	work.pop_front();
	for (unsigned choice(0); choice < pmdp->getNumSuccChoices(state); choice++) {
	  for (unsigned succ(0); succ < pmdp->getNumSuccStates(state, choice); succ++) {
	    PMM::state succState(pmdp->getSuccState(state, choice, succ));
	    if (!reach[succState]) {
	      work.push_back(succState);
	      reach[succState] = true;
	    }
	  }
	}
      }
    }
  }
}
