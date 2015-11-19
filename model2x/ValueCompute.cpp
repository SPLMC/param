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

#include <cassert>
#include <cstddef>
#include <stdexcept>
#include "rationalFunction/Base.h"
#include "ValueCompute.h"
#include "prismparser/Expr.h"

namespace model2x {
  using namespace std;
  using namespace rational;
  using namespace prismparser;

  ValueCompute::ValueCompute() {
    stateVariables = NULL;
    paramNumbers = new ParamNumbersMap();
    vector<string> &symbols(Base::getSymbols());
    for (unsigned symbolNr(0); symbolNr < symbols.size(); symbolNr++) {
      paramNumbers->insert(make_pair(symbols[symbolNr], symbolNr));
    }
  }

  ValueCompute::~ValueCompute() {
    delete paramNumbers;
    for (unsigned pNr = 0; pNr < programInsts.size(); pNr++) {
      const ProgramInsts &proginsts(programInsts[pNr]);
      const ProgramParams &progpars(programParams[pNr]);
      for (unsigned i = 0; i < proginsts.size(); i++) {
	if (Expr::Rat == proginsts[i]) {
	  delete (RationalFunction *) progpars[i];
	}
      }
    }
  }

  void ValueCompute::setStateVariables(const vector<Expr> &_stateVariables) {
    stateVariables = &_stateVariables;
  }

  void ValueCompute::fold
  (RationalFunction &acc, const RationalFunction next, unsigned inst) const {
    switch (inst) {
    case Expr::Plus:
      acc += next;
      break;
    case Expr::Minus:
      acc -= next;
      break;
    case Expr::Mult:
      acc *= next;
      break;
    case Expr::Div:
      acc /= next;
      break;
    case Expr::Pow: {
      RationalFunction &pow(acc);
      RationalFunction r(next);
      if (!pow.isInt()) {
	throw runtime_error("Expression \"" + pow.toString() + "\" is not an integer.");
      }
      int powInt(pow.toInt());
      unsigned upow(powInt >= 0 ? powInt : -powInt);
      
      RationalFunction res = 1;
      if (0 == upow) {
	res = 1;
      } else {
	while (1 != upow) {
	  if (1 & upow) {
	    res *= r;
	  }
	  upow >>= 1;
	  r *= r;
	}
	res *= r;
      }
      
      if (0 > powInt) {
	res = 1 / res;
      }
      acc = res;
      break;
    }
    case Expr::Eq:
      acc = (acc == next) ? 1 : 0;
      break;
    case Expr::Neq:
      acc = (acc != next) ? 1 : 0;
      break;
    case Expr::Lt:
      acc = (acc.getIntNum() * next.getIntDen() < next.getIntNum() * acc.getIntDen()) ? 1 : 0;
      break;
    case Expr::Gt:
      acc = (acc.getIntNum() * next.getIntDen() > next.getIntNum() * acc.getIntDen()) ? 1 : 0;
      break;
    case Expr::Le:
      acc = (acc.getIntNum() * next.getIntDen() <= next.getIntNum() * acc.getIntDen()) ? 1 : 0;
      break;
    case Expr::Ge:
      acc = (acc.getIntNum() * next.getIntDen() >= next.getIntNum() * acc.getIntDen()) ? 1 : 0;
      break;
    case Expr::Min:
      if (acc.getIntNum() * next.getIntDen() < next.getIntNum() * acc.getIntDen()) {
	acc = acc;
      } else {
	acc = next;
      }
      break;
    case Expr::Max:
      if (acc.getIntNum() * next.getIntDen() > next.getIntNum() * acc.getIntDen()) {
	acc = acc;
      } else {
	acc = next;
      }
      break;
    case Expr::Or:
      acc = (acc.toInt() || next.toInt()) ? 1 : 0;
      break;
    case Expr::And:
      acc = (acc.toInt() && next.toInt()) ? 1 : 0;
      break;
    case Expr::Impl:
      acc = (!acc.toInt() || next.toInt()) ? 1 : 0;
      break;
    default:
      assert(false);
      break;
    }
  }

  void ValueCompute::expr2program(const Expr &expr, ProgramInsts &proginsts, ProgramParams &progpars) {
    RationalFunction one(1);

    if (expr.isRational()) {
      RationalFunction num(expr.getNumerator());
      RationalFunction den(expr.getDenominator());
      proginsts.push_back(Expr::Rat);
      progpars.push_back((void *) (new RationalFunction(num / den)));
    } else if (expr.isVar()) {
      string name(expr.toString());
      ParamNumbersMap::iterator iter(paramNumbers->find(name));
      if (paramNumbers->end() != iter) {
        vector<unsigned> monomial;
        monomial.resize(Base::getNumSymbols());
        monomial[iter->second] = 1;
        RationalFunction res(1, monomial);
	proginsts.push_back(Expr::Rat);
	progpars.push_back((void *) (new RationalFunction(res)));
      } else {
	unsigned var;
	for (var = 0; var < stateVariables->size(); var++) {
	  if ((*stateVariables)[var] == expr) {
	    break;
	  }
	}
	if (var < stateVariables->size()) {
	  proginsts.push_back(Expr::Var);
	  progpars.push_back((void *) (long) var);
	} else {
	  throw runtime_error("Variable \"" + name + "\" was not found in parameter list.");
	}
      }
    } else if (expr.isTrue()) {
      proginsts.push_back(Expr::Rat);
      progpars.push_back((void *) (new RationalFunction(1)));
    } else if (expr.isFalse()) {
      proginsts.push_back(Expr::Rat);
      progpars.push_back((void *) (new RationalFunction(0)));
    } else { // operator
      Expr::Kind inst = expr.getKind();
      unsigned arity(expr.arity());
      
      for (unsigned partNr(0); partNr < arity; partNr++) {
	expr2program(expr[arity - 1 - partNr], proginsts, progpars);
      }
      bool canReduce(true);
      for (unsigned i = 0; i < arity; i++) {
	if (Expr::Rat != proginsts[proginsts.size() - 1 - i]) {
	  canReduce = false;
	}
      }
      if (canReduce) {
	RationalFunction res;
	switch (inst) {
	case Expr::Uminus: {
	    RationalFunction val(*((RationalFunction *) progpars.back()));
	    delete (RationalFunction *) progpars[progpars.size() - 1];
	    proginsts.pop_back();
	    progpars.pop_back();
	    res = -val;
	    break;
	  }
	case Expr::Not: {
	    RationalFunction val(*((RationalFunction *) progpars.back()));
	    delete (RationalFunction *) progpars[progpars.size() - 1];
	    proginsts.pop_back();
	    progpars.pop_back();
	    res = (one == val) ? 0 : 1;
	    break;
	  }
	case Expr::Ite: {
	    bool isTrue(*((RationalFunction *) progpars.back()) == one);
	    delete (RationalFunction *) progpars[progpars.size() - 1];
	    proginsts.pop_back();
	    progpars.pop_back();
	    RationalFunction ifVal(*((RationalFunction *) progpars.back()));
	    delete (RationalFunction *) progpars[progpars.size() - 1];
	    proginsts.pop_back();
	    progpars.pop_back();
	    RationalFunction elseVal(*((RationalFunction *) progpars.back()));
	    delete (RationalFunction *) progpars[progpars.size() - 1];
	    proginsts.pop_back();
	    progpars.pop_back();
	    res = isTrue ? ifVal : elseVal;
	    break;
	  }
	case Expr::Plus: case Expr::Minus: case Expr::Mult: case Expr::Div: case Expr::Pow: case Expr::Eq:
	case Expr::Neq: case Expr::Lt:	case Expr::Gt: case Expr::Le: case Expr::Ge: case Expr::Or:
	case Expr::And: case Expr::Impl: case Expr::Min: case Expr::Max: {
	    res = *((RationalFunction *) progpars.back());
	    delete (RationalFunction *) progpars[progpars.size() - 1];
	    proginsts.pop_back();
	    progpars.pop_back();
	    for (unsigned i = 1; i < arity; i++) {
	      fold(res, *((RationalFunction *) progpars.back()), inst);
	      delete (RationalFunction *) progpars[progpars.size() - 1];
	      proginsts.pop_back();
	      progpars.pop_back();
	    }
	    break;
	}
	case Expr::Var:
	case Expr::Bool:
	case Expr::Rat:
	  assert(false);
	}
	
	proginsts.push_back(Expr::Rat);
	progpars.push_back((void *) new RationalFunction(res));
      } else {
	progpars.push_back((void *) (long) arity);
	proginsts.push_back(inst);
      }
    }
  }

  ValueCompute::entry_t ValueCompute::addEntry(const Expr &expr) {
    ProgramInsts proginsts;
    ProgramParams progpars;

    expr2program(expr, proginsts, progpars);
    programInsts.push_back(proginsts);
    programParams.push_back(progpars);
    
    return programInsts.size() - 1;
  }

  void ValueCompute::compute(const ValueCompute::entry_t entry, const ValueCompute::state_t &state, RationalFunction &result) const {
    const ProgramInsts &proginsts(programInsts[entry]);
    const ProgramParams &progpars(programParams[entry]);
    vector<RationalFunction> stack;
    stack.reserve(proginsts.size());
    RationalFunction one(1);

    for (unsigned instNr = 0; instNr < proginsts.size(); instNr++) {
      switch (proginsts[instNr]) {
      case Expr::Rat:
	stack.push_back(*((RationalFunction *)progpars[instNr]));
	break;
      case Expr::Var:
	stack.push_back(state[(unsigned) ((long) progpars[instNr])]);
	break;
      case Expr::Uminus: {
	  RationalFunction res(-stack.back());
	  stack.pop_back();
	  stack.push_back(res);
	  break;
	}
      case Expr::Not: {
	  RationalFunction res(one == stack.back() ? 0 : 1);
	  stack.pop_back();
	  stack.push_back(res);
	  break;
	}
      case Expr::Ite: {
	  bool isTrue(stack.back() == one);
	  stack.pop_back();
	  RationalFunction ifVal(stack.back());
	  stack.pop_back();
	  RationalFunction elseVal(stack.back());
	  stack.pop_back();
	  stack.push_back(isTrue ? ifVal : elseVal);
	  break;
	}
      case Expr::Plus: case Expr::Minus: case Expr::Mult: case Expr::Div: case Expr::Pow: case Expr::Eq: case Expr::Lt:
      case Expr::Gt: case Expr::Le: case Expr::Ge: case Expr::Or: case Expr::And: case Expr::Impl:
      case Expr::Min: case Expr::Max: {
	  RationalFunction res(stack.back());
	  stack.pop_back();
	  for (unsigned i = 1; i < (long) progpars[instNr]; i++) {
	    fold(res, stack.back(), proginsts[instNr]);
	    stack.pop_back();
	  }
	  stack.push_back(res);
	  break;
	}
      default:
	assert(false);
	break;
      }
    }
    result = stack.back();
  }
}
