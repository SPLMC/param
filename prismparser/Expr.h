/*
 * This file is part of a parser for an extension of the PRISM language.
 *
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The parser is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the program this parser part of.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#ifndef PP_EXPR_H
#define PP_EXPR_H

#include <vector>
#include <iosfwd>
#include <tr1/unordered_set>
#include <tr1/unordered_map>
#include "VarType.h"

namespace prismparser_ast {
  class Expr;
}

namespace boost {
  template<class T> class shared_ptr;
}

namespace prismparser {
  class ExprContent;
  struct eqExpr;
  struct hashExpr;
  template<class T> class ExprHashMap;
  class Expr;
  bool operator==(const Expr &, const Expr &);

  class Expr {
    friend class ExprContent;
    friend struct eqExpr;
    friend struct hashExpr;
    friend std::ostream &operator<<(std::ostream &, const Expr &);
    friend bool operator==(const Expr &, const Expr &);
  public:
    enum Kind {
      Var,
      Bool,
      Rat,
      
      Not,
      And,
      Or,
      Impl,
      
      Eq,
      Neq,
      Lt,
      Gt,
      Le,
      Ge,
      
      Plus,
      Minus,
      Uminus,
      Mult,
      Div,
      Pow,
      
      Ite,
      Min,
      Max
    };
    
    Expr();
    Expr(const boost::shared_ptr<prismparser_ast::Expr> &);
    Expr(const Expr &);
    ~Expr();
    unsigned arity() const;
    const Expr &operator[](const unsigned) const;
    std::string toString() const;
    bool isNull() const;
    const std::string &getName() const;
    bool isBoolConst() const;
    bool isTrue() const;
    bool isFalse() const;
    bool isRational() const;
    bool isInteger() const;
    bool isVar() const;
    bool isITE() const;
    long long getNumerator() const;
    long long getDenominator() const;
    Kind getKind() const;
    Expr &operator=(const Expr &);

    static Expr multExpr(const Expr &, const Expr &);
    static Expr falseExpr();
    static Expr trueExpr();
    static Expr andExpr(const Expr &, const Expr &);
    static Expr andExpr(const std::vector<Expr> &);
    static Expr orExpr(const Expr &, const Expr &);
    static Expr orExpr(const std::vector<Expr> &);
    static Expr notExpr(const Expr &);
    static Expr eqExpr(const Expr &, const Expr &);
    static Expr ltExpr(const Expr &, const Expr &);
    static Expr gtExpr(const Expr &, const Expr &);
    static Expr leExpr(const Expr &, const Expr &);
    static Expr geExpr(const Expr &, const Expr &);
    static Expr varExpr(const std::string &);
    static Expr ratExpr(long long, long long);
    static Expr ratExpr(const std::string &, int);
    static Expr ratExpr(const std::string &);
    static Expr simplify(const Expr &);
    static void setVarType(const Expr &, VarType);
    static VarType getVarType(const Expr &);
    static void setVarBounds(const Expr &, int, int, int, int);
    static int getVarLowerBoundNum(const Expr &);
    static int getVarLowerBoundDen(const Expr &);
    static int getVarUpperBoundNum(const Expr &);
    static int getVarUpperBoundDen(const Expr &);
    static const std::string &varTypeToString(VarType);
    Expr substExpr(const ExprHashMap<Expr> &) const;
    Expr substExpr(const std::vector<Expr> &, const std::vector<Expr> &) const;
  private:
    const ExprContent *content;
  };

  struct eqExpr {
    bool operator()(const Expr &e1, const Expr &e2) const {
      return (e1.content == e2.content);
    }
  };
  struct hashExpr {
    size_t operator()(const Expr &e) const {
      return reinterpret_cast<size_t>(e.content);
    }
  };

  template<class T>
    class ExprHashMap : public std::tr1::unordered_map<Expr,T,hashExpr,eqExpr> {
  };

  typedef std::tr1::unordered_set<Expr,hashExpr,eqExpr> ExprHashSet;
}

#endif
