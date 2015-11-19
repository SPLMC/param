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

#ifndef CONTROLLER_H
#define CONTROLLER_H

namespace boost {
  namespace program_options {
    class variables_map;
    class options_description;
  }
}

namespace prismparser {
  class Properties;
}

namespace parametric {
  class PMM;
  class RegionResult;
  class RegionResult;
  class Statistics;
  class ExprToNumber;
  class Controller {
  public:
    Controller(int, char **);
    ~Controller();
  private:
    void parse();
    void printStatistics();
    void execute(RegionResult &);
    void printStartMessage() const;
    void filterResult(RegionResult &, bool &);
    void showHelp();
    
    Statistics *statistics;
    boost::program_options::variables_map *vm;
    boost::program_options::options_description *od;
    PMM *pmm;
    prismparser::Properties *props;
    ExprToNumber *exprToNumber;
  };
}

#endif
