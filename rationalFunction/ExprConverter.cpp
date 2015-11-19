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

#include <cassert>
#include "ExprConverter.h"
#include "Base.h"
#include "RationalFunction.h"
#include "prismparser/Expr.h"

namespace rational {
  using namespace std;
  using namespace prismparser;

  ExprConverter::ExprConverter() {
  }

  RationalFunction ExprConverter::convert(const Expr &expr) {
    if (expr.isRational()) {
      RationalFunction num(expr.getNumerator());
      RationalFunction den(expr.getDenominator());
      return num / den;
    } else if (expr.isVar()) {
      string name(expr.toString());
      ParamNumbersMap::iterator iter(paramNumbers.find(name));
      if (paramNumbers.end() != iter) {
        vector<unsigned> monomial;
        monomial.resize(Base::getNumSymbols());
        monomial[iter->second] = 1;
        RationalFunction res(1, monomial);
        return res;
      } else {
	throw runtime_error("Variable \"" + name + "\" was not found in parameter list.");
      }
    } else if (expr.getKind() == 3004) { // PLUS
      RationalFunction result(0);
      unsigned arity(expr.arity());
      for (unsigned partNr(0); partNr < arity; partNr++) {
	result += convert(expr[partNr]);
      }
      return result;
    } else if (expr.getKind() == 3005) { // MINUS
      RationalFunction result(convert(expr[0]));
      unsigned arity(expr.arity());
      for (unsigned partNr(1); partNr < arity; partNr++) {
	result -= convert(expr[partNr]);
      }
      return result;
    } else if (expr.getKind() == 3006) { // TIMES
      RationalFunction result(1);
      unsigned arity(expr.arity());
      for (unsigned partNr(0); partNr < arity; partNr++) {
	result *= convert(expr[partNr]);
      }
      return result;
    } else if (expr.getKind() == 3007) { // DIV
      RationalFunction result(convert(expr[0]));
      unsigned arity(expr.arity());
      for (unsigned partNr(1); partNr < arity; partNr++) {
	result /= convert(expr[partNr]);
      }
      return result;
    } else if (expr.getKind() == 3008) { // POWER
      Expr pow(prismparser::Expr::simplify(expr[0]));
      if (!pow.isRational() || (1 != pow.getDenominator())) {
	throw runtime_error("Expression \"" + pow.toString() + "\" in \""
			    + expr.toString() + "\" is not an integer.");
      }
      int powInt(pow.getNumerator());
      unsigned upow(powInt >= 0 ? powInt : -powInt);

      RationalFunction res(1);
      if (0 == upow) {
	res = 1;
      } else {
	RationalFunction r(convert(expr[1]));
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

      return res;
    } else if (expr.getKind() == 121) { // ITE
      Expr truthVal(prismparser::Expr::simplify(expr[0]));
      if (truthVal.isTrue()) {
	return convert(expr[1]);
      } else if (truthVal.isFalse()) {
	return convert(expr[2]);
      } else {
	cout << "truth value of \"" << expr[0] << "\" could not be evaluated." << endl;
	assert(false);
      }
    } else {
      cout << "TO BE DONE: " << expr << " IS " << expr.getKind() << endl;
      assert(false);
    }
  }

  RationalFunction ExprConverter::operator()(const Expr &expr) {
    vector<string> &symbols(Base::symbols);
    paramNumbers.clear();
    for (unsigned symbolNr(0); symbolNr < symbols.size(); symbolNr++) {
      paramNumbers.insert(make_pair(symbols[symbolNr], symbolNr));
    }

    return convert(expr);
  }

  void ExprConverter::addSymbols(const Expr &expr) {
    if (expr.isVar()) {
      string name(expr.toString());
      ParamNumbersMap::iterator iter(paramNumbers.find(name));
      if (paramNumbers.end() == iter) {
        Base::symbols.push_back(name);
        paramNumbers.insert(make_pair(name, Base::getNumSymbols() - 1));
      }
    } else {
      for (unsigned childNr = 0; childNr < expr.arity(); childNr++) {
        addSymbols(expr[childNr]);
      }
    }
  }
}
