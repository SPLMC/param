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

#include <iostream>
#include <cstdlib>
#include <limits>
#include "InitStatesComp.h"
#include "prismparser/Expr.h"

namespace model2x {
  using namespace std;
  using namespace prismparser;

  InitStatesComp::InitStatesComp() {
    initExpr = NULL;
    vars = NULL;
    lowerBounds = NULL;
    upperBounds = NULL;
  }

  InitStatesComp::~InitStatesComp() {
  }

  void InitStatesComp::setExpr(const Expr &_initExpr) {
    initExpr = &_initExpr;
  }

  void InitStatesComp::setVars(const vector<Expr> &_vars) {
    vars = &_vars;
  }
  void InitStatesComp::setBounds
  (const vector<int> &_lowerBounds, const std::vector<int> &_upperBounds) {
    lowerBounds = &_lowerBounds;
    upperBounds = &_upperBounds;
  }

  Expr InitStatesComp::negToInner(const Expr &expr, const bool neg) {
    unsigned kind = expr.getKind();
    if (Expr::Not == kind) {
      return negToInner(expr[0], !neg);
    } else if (Expr::And == kind) {
      unsigned arity(expr.arity());
      vector<Expr> childs;
      
      for (unsigned partNr(0); partNr < arity; partNr++) {
	childs.push_back(negToInner(expr[partNr], neg));
      }

      if (!neg) {
	return Expr::andExpr(childs);
      } else {
	return Expr::orExpr(childs);
      }
    } else if (Expr::Or == kind) {
      unsigned arity(expr.arity());
      vector<Expr> childs;
      
      for (unsigned partNr(0); partNr < arity; partNr++) {
	childs.push_back(negToInner(expr[partNr], neg));
      }

      if (!neg) {
	return Expr::orExpr(childs);
      } else {
	return Expr::andExpr(childs);
      }
    } else if (Expr::Impl == kind) {
      vector<Expr> childs;
      childs.push_back(negToInner(expr[0], !neg));
      childs.push_back(negToInner(expr[1], neg));
      if (!neg) {
	return Expr::orExpr(childs);
      } else {
	return Expr::andExpr(childs);
      }
    } else if (Expr::Ite == kind) {
      Expr a(negToInner(expr[0], neg));
      Expr nega(negToInner(expr[0], !neg));
      Expr b(negToInner(expr[1], neg));
      Expr c(negToInner(expr[2], neg));
      if (!neg) {
	return Expr::orExpr(Expr::andExpr(a, b), Expr::andExpr(nega, c));
      } else {
	return Expr::andExpr(Expr::orExpr(a, b), Expr::orExpr(nega, c));
      }
    } else if (Expr::Eq == kind) {
      if (!neg) {
	return expr;
      } else {
	return Expr::notExpr(expr);
      }
    } else if (Expr::Lt == kind) {
      if (!neg) {
	return expr;
      } else {
	return Expr::geExpr(expr[0], expr[1]);
      }
    } else if (Expr::Le == kind) {
      if (!neg) {
	return expr;
      } else {
	return Expr::gtExpr(expr[0], expr[1]);	
      }      
    } else if (Expr::Gt == kind) {
      if (!neg) {
	return expr;
      } else {
	return Expr::leExpr(expr[0], expr[1]);
      }
    } else if (Expr::Ge == kind) {
      if (!neg) {
	return expr;
      } else {
	return Expr::ltExpr(expr[0], expr[1]);
      }
    } else {
      if (!neg) {
	return expr;
      } else {
	return Expr::notExpr(expr);
      }
    }
  }

  int InitStatesComp::fromTree
  (const Expr &expr, unsigned varNr, const vector<int> &varVals) {
    unsigned kind = expr.getKind();

    if (Expr::And == kind) {
      unsigned arity(expr.arity());
      int res = numeric_limits<int>::min();
      for (unsigned partNr(0); partNr < arity; partNr++) {
	res = max(res, fromTree(expr[partNr], varNr, varVals));
      }
      return res;
    } else if (Expr::Or == kind) {
      unsigned arity(expr.arity());
      int res = numeric_limits<int>::max();
      for (unsigned partNr(0); partNr < arity; partNr++) {
	res = min(res, fromTree(expr[partNr], varNr, varVals));
      }
      return res;
    } else {
      vector<Expr> uvars;
      vector<Expr> uvals;
      for (unsigned i = 0; i <= varNr; i++) {
	uvars.push_back((*vars)[i]);
	if (BoolVarType == Expr::getVarType((*vars)[i])) {
	  uvals.push_back(varVals[i] ? Expr::trueExpr() : Expr::falseExpr());
	} else {
	  uvals.push_back(Expr::ratExpr(varVals[i], 1));
	}
      }
      if (Expr::simplify((expr.substExpr(uvars, uvals))).isFalse()) {
	// TODO could improve here
	return varVals[varNr] + 1;
      } else {
	return varVals[varNr];
      }
    }
  }

  void InitStatesComp::compInits
  (vector<vector<int> > &initStates, unsigned varNr, vector<int> &varVals, const Expr &expr) {
    if (varNr < vars->size()) {
      varVals[varNr] = (*lowerBounds)[varNr];
      while (varVals[varNr] <= (*upperBounds)[varNr]) {
	int next = fromTree(expr, varNr, varVals);
	if (varVals[varNr] == next) {
	  compInits(initStates, varNr + 1, varVals, expr);
	  next++;
	}
	varVals[varNr] = next;
      }
    } else {
      initStates.push_back(varVals);
    }
  }

  void InitStatesComp::compInits(vector<vector<int> > &initStates) {
    Expr initExprI = negToInner(*initExpr, false);

    vector<int> varVals(vars->size());
    compInits(initStates, 0, varVals, initExprI);
  }
}
