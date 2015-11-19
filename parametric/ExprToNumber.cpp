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

#include "prismparser/Model.h"
#include "prismparser/Property.h"
#include "ExprToNumber.h"

namespace parametric {
  using namespace prismparser;

  ExprToNumber::ExprToNumber() {
    mapExpr = new ExprHashMap<unsigned>();
    mapNumber = new std::vector<Expr>();
    props = NULL;
  }

  ExprToNumber::~ExprToNumber() {
    delete mapExpr;
    delete mapNumber;
  }

  void ExprToNumber::setProperties(const Properties &props_) {
    props = &props_;
  }

  void ExprToNumber::build() {
    for (unsigned propNr(0); propNr < props->size(); propNr++) {
      const Property *prop = (*props)[propNr].get();
      buildExprToNumber(prop);
    }
  }

  void ExprToNumber::buildExprToNumber(const Property *prop) {
    if (expr == prop->kind) {
      const PropExpr *propExpr((const PropExpr *) prop);
      Expr expr(propExpr->getExpr());
      if (0 == mapExpr->count(expr)) {
	(*mapExpr)[expr] = mapNumber->size();;
	mapNumber->push_back(expr);
      }
    } else {
      for (unsigned childNr(0); childNr < prop->arity(); childNr++) {
	buildExprToNumber(&((*prop)[childNr]));
      }
    }
  }

  const Expr &ExprToNumber::getExprByNumber(unsigned number) const {
    return (*mapNumber)[number];
  }

  unsigned ExprToNumber::getNumberByExpr(const Expr &expr) const {
    return (*mapExpr)[expr];
  }

  unsigned ExprToNumber::getNumExprs() const {
    return mapNumber->size();
  }
}
