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

 * You should have received a copy of the GNU General Public License
 * along with Model2X. If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2010-2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#include <stdexcept>
#include <iostream>
#include "expr2lua.h"
#include "prismparser/Expr.h"

namespace model2x {
  using namespace std;
  using namespace prismparser;

  std::tr1::unordered_map<unsigned,string>
  expr2lua::kindToString(prepareKindToString());
  expr2lua::expr2lua(const Expr &__expr) {
    expr = new Expr(prismparser::Expr::simplify(__expr));
  }

  expr2lua::~expr2lua() {
    delete expr;
  }
  
  /**
   * Prints expression to @a stream as Lua expression.
   *
   * @param expr expression to print
   */
  void expr2lua::printExprC(std::ostream &stream, const Expr &expr)
    const {
    if (0 != expr.arity()) {
      unsigned kind = expr.getKind();
      if (0 != kindToString.count(kind)) {
        string &op = kindToString[kind];
        if (1 == expr.arity()) {
          stream << op << "(";
          printExprC(stream, expr[0]);
          stream << ")";
        } else {
          stream << "(";
          printExprC(stream, expr[0]);
          stream << ")";
          for (unsigned childNr = 1u; childNr < unsigned(expr.arity());
               childNr++) {
            stream << " " << op << " (";
            printExprC(stream, expr[childNr]);
            stream << ")";
          }
        }
      } else if (expr.isITE()) {
	stream << "(function() if ";
        printExprC(stream, expr[0]);
	stream << " then return ";
        printExprC(stream, expr[1]);
	stream << " else return ";
        printExprC(stream, expr[2]);
	stream << " end end)()";
      } else if (Expr::Pow == expr.getKind()) {
        stream << "math.pow( ";
        printExprC(stream, expr[1]);
        stream << ",  ";
        printExprC(stream, expr[0]);
        stream << ")";
      } else {
        throw runtime_error("Unsupported operator in expression\"" + expr.toString() + "\".");
      }
    } else {
      if (expr.isTrue()) {
        stream << "true";
      } else if (expr.isFalse()) {
        stream << "false";
      } else if (expr.isRational()) {
	const int numerator(expr.getNumerator());
	const int denominator(expr.getDenominator());
	if (1 == denominator) {
	  stream << numerator;
	} else {
	  stream << "(" << numerator << " / " << denominator << ")";
	}
      } else {
        stream << expr;
      }
    }      
  }
  
  /**
   * Prepares the hash-table mapping Expr to C operators.
   */
  std::tr1::unordered_map<unsigned,std::string>
  expr2lua::prepareKindToString() {
    std::tr1::unordered_map<unsigned,std::string> kindToString;

    kindToString[Expr::And] = "and";
    kindToString[Expr::Or] = "or";
    kindToString[Expr::Not] = "not";
    kindToString[Expr::Eq] = "==";
    kindToString[Expr::Neq] = "~=";
    kindToString[Expr::Uminus] = "-";
    kindToString[Expr::Plus] = "+";
    kindToString[Expr::Minus] = "-";
    kindToString[Expr::Mult] = "*";
    kindToString[Expr::Div] = "/";
    kindToString[Expr::Lt] = "<";
    kindToString[Expr::Gt] = ">";
    kindToString[Expr::Le] = "<=";
    kindToString[Expr::Ge] = ">=";

    return kindToString;
  }
}
