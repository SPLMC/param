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
 * along with PARAM.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2010-2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#include <stdlib.h>
#include <sys/types.h>
#include <iterator>
#include <iostream>
#include <iomanip>
#include <istream>
#include <fstream>
#include <boost/program_options.hpp>
#include <limits>
#include "PMM.h"
#include "PMC.h"
#include "Controller.h"
#include "LowLevelExplorer.h"
#include "rationalFunction/RationalFunction.h"
#include "RationalParser.h"

namespace parametric {
  using namespace std;
  using namespace rational;
  using namespace boost;

  LowLevelExplorer::LowLevelExplorer() {
  }

  LowLevelExplorer::~LowLevelExplorer() {
  }

  unsigned LowLevelExplorer::readNumStates(istream &stream) const {
    string line;
    getline(stream, line);
    vector<string> splitLine;
    istringstream iss(line);
    
    copy(std::istream_iterator<string>(iss),
         std::istream_iterator<string>(),
         back_inserter<vector<string> >(splitLine));
    
    if (2 != splitLine.size() || ("STATES" != splitLine[0])) {
      throw runtime_error("first line of low-level input file must be of"
			  " the form \"STATES <num-states>\"");
    }

    unsigned result(atoi(splitLine[1].c_str()));
    if (0 == result) {
      throw runtime_error("Invalid number of states.");
    }

    return result;
  }
  
  unsigned LowLevelExplorer::readNumTransitions(istream &stream) const {
    string line;
    getline(stream, line);
    vector<string> splitLine;
    istringstream iss(line);
    
    copy(std::istream_iterator<string>(iss),
         std::istream_iterator<string>(),
         back_inserter<vector<string> >(splitLine));
    
    if (2 != splitLine.size() || ("TRANSITIONS" != splitLine[0])) {
      throw runtime_error("second line of low-level input file must be of"
			  " the form \"TRANSITIONS <num-transitions>\"");
    }

    unsigned result(atoi(splitLine[1].c_str()));
    if (0 == result) {
      throw runtime_error("Invalid number of transitions.");
    }

    return result;
  }

  void LowLevelExplorer::readParameters(istream &stream, RationalParser &parser) {
    string line;
    getline(stream, line);
    if ("PARAM " != line.substr(0, 6)) {
      throw runtime_error("fifth line of low-level input file must be of"
			  " the form \"PARAM <param1> ... <paramp>\"");

    }
    parser.parseSymbols(line.substr(6));
  }

  void LowLevelExplorer::readInitStates(istream &stream) {
    string line;
    getline(stream, line);
    vector<string> splitLine;
    istringstream iss(line);
    
    copy(std::istream_iterator<string>(iss),
         std::istream_iterator<string>(),
         back_inserter<vector<string> >(splitLine));
    
    if (2 > splitLine.size() || ("INIT" != splitLine[0])) {
      throw runtime_error("third line of low-level input file must be of"
			  " the form \"INIT <state1> ... <staten>\"");
    }

    for (unsigned stateNr(1); stateNr < splitLine.size(); stateNr++) {
      unsigned init(atoi(splitLine[stateNr].c_str()));      
      if (0 == init) {
	throw runtime_error("Invalid initial state \"" + splitLine[stateNr] + "\"");
      }
      init--;
      pmm->addInit(init);
    }
  }

  void LowLevelExplorer::readTargetStates
  (istream &stream) {
    string line;
    getline(stream, line);
    vector<string> splitLine;
    istringstream iss(line);
    
    copy(std::istream_iterator<string>(iss),
         std::istream_iterator<string>(),
         back_inserter<vector<string> >(splitLine));
    
    if (2 > splitLine.size() || ("TARGET" != splitLine[0])) {
      throw runtime_error("fourth line of low-level input file must be of"
			  " the form \"TARGET <state1> ... <statem>\"");
    }

    for (unsigned stateNr(1); stateNr < splitLine.size(); stateNr++) {
      unsigned target(atoi(splitLine[stateNr].c_str()));      
      if (0 == target) {
	throw runtime_error("Invalid target state \"" + splitLine[stateNr] + "\"");
      }
      target--;
      // TODO add AP labels
      //      mc.targetStates.insert(target);
    }
  }

  void LowLevelExplorer::readTransitions
  (istream &stream, unsigned numStates, unsigned numTransitions, RationalParser &parser) {
    unsigned lastFrom(1);
    while (!stream.eof()) {
      string line;
      getline(stream, line);
      if ("" != line) {
	istringstream iss(line);
	unsigned from;
	iss >> from;
	from--;
	unsigned to;
	iss >> to;
	to--;
	string restString;
	getline(iss, restString);
	RationalFunction prob(parser.parseRational(restString));
	if (lastFrom != from) {
	  pmc->finishState();
	  if (lastFrom != from - 1) {
	    throw new runtime_error("Transitions must be ordered by source state.");
	  }
	}
	pmc->addSucc(to, prob);
      }
    }
    pmc->finishState();
  }

  void LowLevelExplorer::explore() {
    RationalParser parser;
    ifstream file(modelFilename.c_str(), ifstream::in);
    const unsigned numStates(readNumStates(file));
    pmc->reserveRowsMem(numStates);
    const unsigned numTransitions(readNumTransitions(file));
    pmc->reserveColsMem(numTransitions);
    readInitStates(file);
    readTargetStates(file);
    readParameters(file, parser);
    readTransitions(file, numStates, numTransitions, parser);
    RationalFunction::removeUnusedSymbols();
  }
}
