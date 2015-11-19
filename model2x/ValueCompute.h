/*
 * This file is part of Model2X.
 *
 * Model2X is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Model2X is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Model2X. If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#ifndef VALUE_COMPUTE_H
#define VALUE_COMPUTE_H

#include <vector>
#include <tr1/unordered_map>

namespace prismparser {
  class Expr;
}

namespace rational {
  class RationalFunction;
}

namespace model2x {
  class ValueCompute {
  public:
    typedef unsigned entry_t;
    typedef std::vector<int> state_t;
    ValueCompute();
    ~ValueCompute();
    void setStateVariables(const std::vector<prismparser::Expr> &);
    entry_t addEntry(const prismparser::Expr &);
    void compute(const entry_t, const state_t &, rational::RationalFunction &) const;
  private:
    typedef std::tr1::unordered_map<std::string, unsigned> ParamNumbersMap;
    typedef std::vector<unsigned> ProgramInsts;
    typedef std::vector<void *> ProgramParams;
    typedef std::vector<prismparser::Expr> StateVariables;

    const StateVariables *stateVariables;
    ParamNumbersMap *paramNumbers;
    std::vector<ProgramInsts> programInsts;
    std::vector<ProgramParams> programParams;

    void expr2program(const prismparser::Expr &, ProgramInsts &, ProgramParams &);
    void fold(rational::RationalFunction &, const rational::RationalFunction, const unsigned) const;
  };
}

#endif
