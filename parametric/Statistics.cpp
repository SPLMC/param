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

#include <iostream>
#ifndef __APPLE__
#include <malloc.h>
#endif
#include <iomanip>
#include "Statistics.h"

namespace parametric {
  
  using namespace std;
  
  /**
   * Print statistics to stream @a stream.
   *
   * @param stream output stream to print statistics to
   */
  void Statistics::print(std::ostream &stream) const {
    stream << "total-time: " << totalTime.read() << endl;
    stream << "exploration-time: " << exploreTime.read() << endl;
    stream << "lumping-time: " << lumpTime.read() << endl;
    stream << "analysis-time: " << analysisTime.read() << endl;
    stream << "number-model-states: " << numStatesModel << endl;
    stream << "number-model-transitions: " << numTransitionsModel << endl;
    stream << "number-quotient-states: " << numStatesQuotient << endl;
    stream << "number-quotient-transitions: "
           << numTransitionsQuotient << endl;
  }
  
  /**
   * Print statistics to standard output.
   */
  void Statistics::print() const {
    print(cout);
  }
  
  /**
   * Print statistics to file @a filename.
   *
   * @param filename filename of file to print statistics to
   */
  void Statistics::print(const std::string &filename) const {
    ofstream file(filename.c_str(), ios::out);
    print(file);
    file.close();
  }
}
