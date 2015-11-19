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

 * You should have received a copy of the GNU General Public License
 * along with PARAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2009-2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#ifndef STATISTICS_H
#define STATISTICS_H

#include <fstream>
#include "Timer.h"

namespace parametric {
  
  class SparseMC;
  class PASSStateExplorer;
  class LowLevelStateExplorer;
  class Quotient;
  
  /**
   * Manage and print out statistics about program runs.
   */
  class Statistics {
    friend class Controller;
    friend class PASSStateExplorer;
    friend class LowLevelStateExplorer;
    friend class Quotient;
  private:
    Timer totalTime;
    Timer exploreTime;
    Timer lumpTime;
    Timer analysisTime;
    unsigned numStatesModel;
    unsigned numTransitionsModel;
    unsigned numStatesQuotient;
    unsigned numTransitionsQuotient;
    
    void print() const;
    void print(std::ostream &) const;
    void print(const std::string &) const;
  };
}

#endif
