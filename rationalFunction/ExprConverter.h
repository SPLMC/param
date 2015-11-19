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

#ifndef EXPR_CONVERTER_H
#define EXPR_CONVERTER_H

#include <string>
#include <tr1/unordered_map>

namespace prismparser {
  class Expr;
}

namespace rational {
  class RationalFunction;
  class ExprConverter {
  public:
    ExprConverter();
    RationalFunction operator()(const prismparser::Expr &);
    void addSymbols(const prismparser::Expr &);
  private:
    RationalFunction convert(const prismparser::Expr &);
    typedef std::tr1::unordered_map<std::string, unsigned> ParamNumbersMap;
    ParamNumbersMap paramNumbers;
  };
}

#endif
