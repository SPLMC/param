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

#ifndef LOW_LEVEL_EXPLORER_H
#define LOW_LEVEL_EXPLORER_H

#include <vector>
#include <string>
#include <iosfwd>
#include "ModelExplorer.h"

namespace rational {
  class RationalFunction;
}

namespace prismparser {
  class Properties;
}

namespace parametric {
  class ExprToNumber;
  class PMM;
  class PMC;
  class RationalParser;
  
  class LowLevelExplorer : public ModelExplorer {
  public:
    LowLevelExplorer();
    ~LowLevelExplorer();
    void explore();
  private:
    rational::RationalFunction parseRationalFunction(std::string &);
    unsigned readNumStates(std::istream &) const;
    unsigned readNumTransitions(std::istream &) const;
    void readParameters(std::istream &, RationalParser &);
    void readInitStates(std::istream &);
    void readTargetStates(std::istream &);
    void readTransitions(std::istream &, unsigned, unsigned, RationalParser &);
  };
}

#endif
