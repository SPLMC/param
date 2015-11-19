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

#include <stdlib.h>
#include <sys/types.h>
#include <iostream>
#include <fstream>
#include <limits>
#include <vector>
#include "prismparser/PRISMParser.h"
#include "prismparser/Property.h"
#include "prismparser/Model.h"
#include "Model2XExplorer.h"
#include "rationalFunction/RationalFunction.h"
#include "SPMC.h"
#include "SPMDP.h"
#include "ExprToNumber.h"
#include "model2x/Model2X.h"

namespace parametric {  
  using namespace std;
  using namespace prismparser;
  using namespace model2x;
  using namespace rational;
  
  Model2XExplorer::Model2XExplorer() {
    model = new Model();
  }
  
  Model2XExplorer::~Model2XExplorer() {
    delete model;
  }
    
  /**
   * Load a PASS model and appendant property file.
   * @param model_filename model file to be loaded
   * @param property_filename property file to be loaded
   */
  void Model2XExplorer::loadModel
  (const string &model_filename, const string &property_filename) {
    prismparser::PRISMParser p;
    p.run(model_filename, *model);
    fclose(stdin);
    model->flatten();
    model->fixDeadlocks();
    p.run(property_filename, *model);
    fclose(stdin);
    *props = model->getProperties();
  }

  void Model2XExplorer::constructMC(Model2X &model2x) {
    ModelType modelType = model->getModelType();
    unsigned numInitStates(model2x.getNumInitStates());
    const unsigned *init(model2x.getInitStates());
    for (unsigned initNr(0); initNr < numInitStates; initNr++) {
      unsigned initState(init[initNr]);
      pmm->addInit(initState);
    }

    vector<unsigned> pres(init, init + numInitStates);
    vector<unsigned> next;
    
    const RationalFunction *succRewardsList(NULL);
    const unsigned *succStatesList(model2x.getSuccStatesList());
    const RationalFunction *succRatesList(model2x.getSuccRatesList());
    const unsigned *nonDetBounds(model2x.getNonDetBounds());
    if (useReward) {
      succRewardsList = model2x.getSuccRewardsList();
    }

    unsigned numStates(model2x.getNumStates());
    for (unsigned state(0); state < numStates; state++) {
      for (unsigned ap(0); ap < exprToNumber->getNumExprs(); ap++) {
	pmm->setAP(state, ap, model2x.inStateSet(state, ap));
      }
      model2x.getStateSuccessors(state);
      RationalFunction stateReward(useReward ? model2x.getStateReward(state) : 0);
      map<PMM::state,RationalFunction> probMap;
      map<PMM::state,RationalFunction> rewardMap;
      unsigned lastNonDet(0);
      unsigned numSuccStates(model2x.getNumSuccStates());
      RationalFunction sumRate(0);
      if (CTMC == modelType) {
	for (unsigned succNr(0); succNr < numSuccStates; succNr++) {
	  sumRate += succRatesList[succNr];
	}
      } else {
	sumRate = 1;
      }
      for (unsigned succNr(0); succNr < numSuccStates; succNr++) {
	unsigned succState(succStatesList[succNr]);
	RationalFunction prob(succRatesList[succNr]);
	probMap[succState] += prob;
	if (useReward) {
	  rewardMap[succState] += (succRewardsList[succNr] + stateReward / sumRate) * prob;
	}
	if (nonDetBounds[lastNonDet + 1] == succNr + 1) {
	  lastNonDet++;
	  for (map<PMM::state,RationalFunction>::iterator it(probMap.begin());
	       it != probMap.end(); it++) {
	    unsigned succState(it->first);
	    RationalFunction prob(it->second);
	    pmm->addSucc(succState, prob);
	    if (useReward) {
	      RationalFunction reward(rewardMap[it->first]);
	      pmm->addSuccReward(reward / prob);
	    }	  
	  }
	  probMap.clear();
	  rewardMap.clear();
	  if (PMM::PMDP == pmm->getModelType()) {
	    pmdp->finishChoice();
	  }
	}
      }
      pmm->finishState();
    }
  }
  
  void Model2XExplorer::exploreAllStates(Model2X &model2x) {
    unsigned numInitStates(model2x.getNumInitStates());
    const unsigned *init(model2x.getInitStates());
    vector<unsigned> pres(init, init + numInitStates);
    vector<unsigned> next;

    unsigned numTrans(0);
    unsigned numChoices(0);
    const unsigned *succStatesList(model2x.getSuccStatesList());
    do {
      for (unsigned stateNr(0); stateNr < pres.size(); stateNr++) {
        const unsigned state(pres[stateNr]);
	const unsigned lastNumStates(model2x.getNumStates());
        model2x.getStateSuccessors(state);
	numTrans += model2x.getNumSuccStates();
	set<PMM::state> succStates(succStatesList, succStatesList
				   + model2x.getNumSuccStates());
	for (set<unsigned>::iterator it(succStates.begin());
	     it != succStates.end(); it++) {
	  const unsigned succState(*it);
	  if (succState >= lastNumStates) {
	    next.push_back(succState);
	  }
	}
	if (PMM::PMDP == pmm->getModelType()) {
	  numChoices += model2x.getNumNonDet();
	}
      }      
      pres.swap(next);
      next.clear();
    } while (0 != pres.size());
    const unsigned numStates(model2x.getNumStates());
    pmm->reserveRowsMem(numStates);
    pmm->reserveColsMem(numTrans);
    pmm->reserveAPMem(numStates, exprToNumber->getNumExprs());
    if (PMM::PMDP == pmm->getModelType()) {
      pmdp->reserveChoicesMem(numChoices);
    }
    if (useReward) {
      pmm->reserveStateRewardsMem(numStates);
      pmm->reserveTransRewardsMem(numTrans);
    }
  }

  void Model2XExplorer::checkUseReward(const Property &prop) {
    if (reachability_reward == prop.kind) {
      const ReachabilityReward &reachRew((ReachabilityReward &) prop);
      useReward = true;
      rewardStruct = reachRew.getRewardStruct();
    } else {
      for (unsigned childNr(0); childNr < prop.arity(); childNr++) {
	checkUseReward(prop[childNr]);
      }
    }
  }

  void Model2XExplorer::checkUseReward() {
    useReward = false;
    for (unsigned propNr(0); propNr < props->size(); propNr++) {
      const Property *prop = (*props)[propNr].get();
      checkUseReward(*prop);
    }
  }

  void Model2XExplorer::embed() {
    for (PMM::state state(0); state < pmc->getNumStates(); state++) {
      RationalFunction sum(0);
      for (unsigned succ(0); succ < pmc->getNumSuccStates(state); succ++) {
	RationalFunction succProb = pmc->getSuccProb(state, succ);
	sum += succProb;
      }
      for (unsigned succ(0); succ < pmc->getNumSuccStates(state); succ++) {
	RationalFunction succProb = pmc->getSuccProb(state, succ);
	succProb /= sum;
	pmc->setSuccProb(state, succ, succProb);
      }
    }
  }

  void Model2XExplorer::explore() {
    loadModel(modelFilename, propertyFilename);
    if (model->getModelType() == MDP) {
      pmdp = new SPMDP();
      pmm = pmdp;
    } else {
      pmc = new SPMC();
      pmm = pmc;
    }

    for (unsigned varNr = 0; varNr < model->getNumVariables(); varNr++) {
      const Expr &var = model->getVariable(varNr);
      if (model->isParameterVariable(var)) {
	RationalFunction::addSymbol(var.toString());
	mpq_class leftBound(0);
	mpq_class rightBound(1);
	if (model->hasBounds(var)) {
	  pair<pair<int,int>, pair<int,int> > bounds(model->getBounds(var));
	  leftBound = mpq_class(bounds.first.first) / mpq_class(bounds.first.second);
	  rightBound = mpq_class(bounds.second.first) / mpq_class(bounds.second.second);
	}
	RationalFunction::setBounds(var.toString(), leftBound, rightBound);
      }
    }
    RationalFunction::start();
    Model2X model2x;
    model2x.setModel(*model);
    checkUseReward();
    exprToNumber->setProperties(*props);
    exprToNumber->build();
    for (unsigned exprNr(0); exprNr < exprToNumber->getNumExprs(); exprNr++) {
      model2x.addStateSet(exprToNumber->getExprByNumber(exprNr));
    }
    model2x.setUseRewards(useReward);
    model2x.setRewardStruct(rewardStruct);
    model2x.build();
    model2x.addInitStates();
    exploreAllStates(model2x);
    constructMC(model2x);
    pmm->setTimeType((model->getModelType() == CTMC) ? PMM::CONT : PMM::DISC);
    if (model->getModelType() == CTMC) {
      embed();
    }
    if (NULL != pmc) {
      pmc->computeBackTransitions();
    }
    ofstream statesfile("states");
    statesfile << pmm->getNumStates() << endl;
  }
}
