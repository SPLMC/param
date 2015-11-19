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

#include <string>
#include "prismparser/Model.h"
#include "prismparser/Property.h"
#include "rationalFunction/Base.h"
#include "PMM.h"
#include "ProgramOptions.h"
#include "Statistics.h"
#include "Model2XExplorer.h"
#include "LowLevelExplorer.h"
#include "RegionResult.h"
#include "ResultExporter.h"
#include "ExprToNumber.h"
#include "ModelExporter.h"
#include "ModelChecker.h"
#include "Controller.h"

namespace parametric {
  using namespace std;
  using namespace rational;
  using namespace prismparser;
  
  Controller::Controller(int argc, char *argv[]) {
    
    pmm = NULL;
    statistics = NULL;
    props = new Properties();
    exprToNumber = new ExprToNumber();
    pair<boost::program_options::variables_map,
      boost::program_options::options_description *>
      opt = parseCommandLine(argc, argv);
    vm = new boost::program_options::variables_map();
    *vm = opt.first;
    od = opt.second;
    if (vm->count("no-startup-message") == 0) {
      printStartMessage();
    }

    /* exit if requested or wrong syntax used */
    if (!vm->count("help")) {
      if (0 == vm->count("model-file")) {
	throw runtime_error("No model file given");
      }
      if (0 == vm->count("property-file")) {
	throw runtime_error("No property file given");
      }
      statistics = new Statistics();
      statistics->totalTime.start();
      parse();

      if (0 != vm->count("model-dot")) {
	ModelExporter exporter;
	exporter.setModel(*pmm);
	exporter.setFormat("dot");
	exporter.setOutput((*vm)["model-dot"].as<string>());
	exporter.setUseRewards(false);
	exporter.setExprToNumber(*exprToNumber);
	exporter.write();
      }
      RegionResult result;
      execute(result);
      ResultExporter resultExporter;
      resultExporter.setPlotStep((*vm)["plot-step"].as<string>());
      resultExporter.setOutputPrefix((*vm)["result-file"].as<string>());
      resultExporter.setOutputFormat((*vm)["result-format"].as<string>());
      resultExporter.setRegionsPlotLines
	((*vm)["regions-plot-lines"].as<bool>());
      resultExporter.setRegionsTrueColor
	((*vm)["regions-true-color"].as<string>());
      resultExporter.setRegionsFalseColor
	((*vm)["regions-false-color"].as<string>());
      resultExporter.setRegionsDontknowColor
	((*vm)["regions-dontknow-color"].as<string>());
      bool min;
      filterResult(result, min);
      resultExporter.setMinimize(min);
      resultExporter.setResult(result);
      resultExporter.write();
      printStatistics();
    } else {
      showHelp();
    }
  }

  Controller::~Controller() {
    if (NULL != pmm) {
      delete pmm;
    }
    if (NULL != statistics) {
      delete statistics;
    }
    if (NULL != vm) {
      delete vm;
    }
    if (NULL != exprToNumber) {
      delete exprToNumber;
    }
    if (NULL != od) {
      delete od;
    }
    RationalFunction::cleanup();
  }

  void Controller::filterResult(RegionResult &regionResult, bool &min) {
    // TODO handle multiple formulas
    const Property &prop(*((*props)[0].get()));
    for (RegionResult::iterator it(regionResult.begin());
	 it != regionResult.end(); it++) {
      Result &result(it->second);
      Result newResult;
      bool quantitative(false);
      min = false;
      if (quant == prop.kind) {
	const Quant &quant((const prismparser::Quant &) prop);
	const Bound &bound(quant.getBound());
	min = quant.isMin();
	quantitative = (Bound::DK == bound.kind);
      } else if (reachability_reward == prop.kind) {
	const ReachabilityReward &reach((const ReachabilityReward &) prop);	
	quantitative = true;
	min = reach.getBound().isMin();
      }
      if (quantitative) {
	for (PMM::state state(0); state < pmm->getNumStates(); state++) {
	  if (pmm->isInit(state)) {
	    newResult.push_back(result[state]);
	  }
	}
      } else {
	RationalFunction val(1);
	RationalFunction oneHalf(val / 2);
	for (PMM::state state(0); state < pmm->getNumStates(); state++) {
	  if (result[state] == oneHalf) {
	    val = 0;
	  } else if (result[state] == 0) {
	    val = 0;
	    break;
	  }
	}
	newResult.push_back(val);	
      }
      result = newResult;
    }
  }

  /**
   * Show help on program usage.
   *
   * @param calledName name under which PARAM was called
   * @param visible named program options visible to user
   */
  void Controller::showHelp() {
    cout << "Usage: " << (*vm)["called-name"].as<string>()
         << " <model-file> <properties-file [options]" << endl;
    cout << *od << endl;
  }

  void Controller::printStartMessage() const {
    cout << 
  " ------------------------------------------------------------------- \n"
  "|                 PARAMetric Markov Model Analyser                  |\n"
  "|                     PARAM version 2.3 Alpha                       |\n"
  "|           Copyright (C) Saarland University, 2008-2011.           |\n"
  "|                             Author:                               |\n"
  "|              E. Moritz Hahn (emh@cs.uni-saarland.de)              |\n"
  "|         Authors of parser for extension of PRISM language:        |\n"
  "|          Bjoern Wachter (bjoern.wachter@comlab.ox.ac.uk)          |\n"
  "|              E. Moritz Hahn (emh@cs.uni-saarland.de)              |\n"
  "|          PARAM  is distributed under the GPL conditions           |\n"
  "|           (GPL stands for GNU General Public License)             |\n"
  "|          The product comes with ABSOLUTELY NO WARRANTY.           |\n"
  "|  This is a free software, and you are welcome to redistribute it. |\n"
  " ------------------------------------------------------------------- \n\n";
  }
  
  /**
   * Executes the analysis.
   * The model and formula must have been loaded. Statespace must have been
   * explored, model type and analysis type etc. must have been set.
   */
  void Controller::execute(RegionResult& result) {
    statistics->analysisTime.start();
    ModelChecker modelChecker;
    modelChecker.setModel(*pmm);
    modelChecker.setProperties(*props);
    modelChecker.setResult(result);
    modelChecker.setExprToNumber(*exprToNumber);
    modelChecker.setMaxUnknown((*vm)["region-max-unknown"].as<double>());
    modelChecker.setEliminationOrder((*vm)["elimination-order"].as<string>());
    modelChecker.setLumpMethod((*vm)["lump-method"].as<string>());
    modelChecker.setRefineOrder((*vm)["refine-order"].as<string>());
    modelChecker.setRegionSplitMode((*vm)["split-mode"].as<string>());
    modelChecker.setSolver((*vm)["solver"].as<string>());
    modelChecker.setISATBinary((*vm)["isat-binary"].as<string>());
    modelChecker.setRSolverBinary((*vm)["rsolver-binary"].as<string>());
    modelChecker.setRAHDBinary((*vm)["rahd-binary"].as<string>());
    modelChecker.setIteratePrecision((*vm)["iterate-precision"].as<double>());
    modelChecker.setToleranceFactor((*vm)["tolerance-factor"].as<double>());
    modelChecker.setRandomSchedCheck((*vm)["random-sched-check"].as<unsigned>());
    modelChecker.execute();
    statistics->analysisTime.stop();
  }
  
  void Controller::parse() {
    RationalFunction::setCleanupMethod(RationalFunction::never);
    statistics->exploreTime.start();
    ModelExplorer *explorer;
    if (0 == vm->count("low-level-input")) {
      explorer = new Model2XExplorer();
    } else {
      explorer = new LowLevelExplorer();
    }
    explorer->setModelFilename((*vm)["model-file"].as<string>());
    explorer->setPropertyFilename((*vm)["property-file"].as<string>());
    explorer->setProperties(*props);
    explorer->setExprToNumber(*exprToNumber);
    explorer->explore();
    pmm = explorer->getPMMModel();
    delete explorer;
    statistics->exploreTime.stop();
    statistics->numStatesModel = pmm->getNumStates();
    //    statistics->numTransitionsModel = pmm->getNumTrans();
  }

  /**
   * Print statistics to file specified by program options.
   */
  void Controller::printStatistics() {
    if (0 != vm->count("statistics-file")) {
      statistics->totalTime.stop();
      statistics->print((*vm)["statistics-file"].as<std::string>());
    }
  }
}
