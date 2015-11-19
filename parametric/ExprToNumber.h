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

#ifndef EXPR_TO_NUMBER_H
#define EXPR_TO_NUMBER_H

#include <vector>


namespace prismparser {
  template<class T> class ExprHashMap;
  class Expr;
  class Properties;
  class Property;
}

namespace parametric {
  class ExprToNumber {
  public:
    ExprToNumber();
    ~ExprToNumber();
    void setProperties(const prismparser::Properties &);
    void build();
    const prismparser::Expr &getExprByNumber(unsigned) const;
    unsigned getNumberByExpr(const prismparser::Expr &) const;
    unsigned getNumExprs() const;
  private:
    void buildExprToNumber(const prismparser::Property *);

    const prismparser::Properties *props;
    prismparser::ExprHashMap<unsigned> *mapExpr;
    std::vector<prismparser::Expr> *mapNumber;
  };
}

#endif
