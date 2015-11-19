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

#include <tr1/unordered_map>
#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>
#include "AST.h"
#include "Expr.h"
#include "Util.h"
#include "VarType.h"

namespace prismparser {
  using namespace std;

  static inline long long gcd(long long a, long long b) {
    long long c = a % b;
    while (c != 0) {
	a = b;
	b = c;
	c = a % b;
    }
    return b;
  }

  static inline long long lcm(long long a, long long b) {
    return (a / gcd(a, b)) * b;
  }

  static inline void normalise(long long &num, long long &den) {
    if (0 == num) {
      den = 1;
    } else {
      long long sign = 1;
      if ((num < 0) && (den < 0)) {
	num *= -1;
	den *= -1;
      } else if ((num < 0) || (den < 0)) {
	sign = -1;
	if (den < 0) {
	  den *= -1;
	} else {
	  num *= -1;
	}
      }
      long long c = gcd(num, den);
      num /= c;
      den /= c;
      num *= sign;
    }
  }

  static inline void double2ratnoexpr(const string &double_val, long long &num, long long &den) {
    size_t dotpos = double_val.find(".");
    if (string::npos == dotpos) {
      num = atoll(double_val.c_str());
      den = 1;
    } else {
      string before_str = double_val.substr(0, dotpos);
      string after_str = double_val.substr(dotpos + 1);
      unsigned divpow = after_str.length();
      string num_str = before_str + after_str;
      num = atoll(num_str.c_str());
      den = 1;
      for (unsigned i = 0; i < divpow; i++) {
	den *= 10;
      }
      normalise(num, den);
    }
  }

  static inline void double2rat(const string &double_val, long long &num, long long &den) {
    size_t divpos = double_val.find("/");
    size_t epos = double_val.find("e");
    if (string::npos == epos) {
      epos = double_val.find("E");
    }

    if (string::npos != divpos) {
      string num_str = double_val.substr(0, divpos);
      string den_str = double_val.substr(divpos + 1);
      num = atoll(num_str.c_str());
      den = atoll(den_str.c_str());
    } else if (string::npos == epos) {
      double2ratnoexpr(double_val, num, den);
    } else {
      string mant_str = double_val.substr(0, epos);
      string expr_str = double_val.substr(epos + 1);
      long long expr = atoll(expr_str.c_str());
      double2ratnoexpr(mant_str, num, den);
      if (expr >= 0) {
	for (int i = 0; i < expr; i++) {
	  num *= 10;
	}
      } else {
	expr = -expr;
	for (int i = 0; i < expr; i++) {
	  den *= 10;
	}
      }
      normalise(num, den);
    }
  }

  class ExprContent {
    friend class Expr;
  private:
    static ExprContent *fromAST(const boost::shared_ptr<prismparser_ast::Expr> &);
    static void incRecRefs(const ExprContent *);
    static void decRecRefs(const ExprContent *);
    static ExprContent *makeUnique(ExprContent *);
    static string op2string(Expr::Kind);
    static void cleanupTable();
    static void setEntriesNull();

    ExprContent();
    string optEmbrace() const;
    string toString() const;
    const ExprContent *substExpr(const ExprHashMap<Expr> &) const;
    const ExprContent *simplify() const;
    bool isConst() const;
    bool isInteger() const;

    struct hashstate {
      size_t operator()(const ExprContent &e) const {
	size_t hashval = 0;

	hashval = e.kind + (hashval << 6) + (hashval << 16) - hashval;
	switch (e.kind) {
	case Expr::Var:
	  for (unsigned charNr = 0; charNr < e.identifier.length(); charNr++) {
	    hashval = e.identifier[charNr] + (hashval << 6) + (hashval << 16) - hashval;
	  }
	  break;
	case Expr::Bool:
	  hashval = e.bool_value + (hashval << 6) + (hashval << 16) - hashval;
	  break;
	case Expr::Rat:
	  hashval = e.num_value + (hashval << 6) + (hashval << 16) - hashval;
	  hashval = e.den_value + (hashval << 6) + (hashval << 16) - hashval;
	default:
	  break;
	}
    
	for (unsigned i = 0; i < e.children.size(); i++) {
	  hashval = reinterpret_cast<size_t>(e.children[i])
	    + (hashval << 6) + (hashval << 16) - hashval;
	}

	return hashval;
      }
    };
    struct eqstate {
      bool operator()(const ExprContent &e1, const ExprContent &e2) const {
	if (e1.kind != e2.kind) {
	  return false;
	}

	switch (e1.kind) {
	case Expr::Var:
	  if (e1.identifier != e2.identifier) {
	    return false;
	  }
	  break;
	case Expr::Bool:
	  if (e1.bool_value != e2.bool_value) {
	    return false;
	  }
	  break;
	case Expr::Rat:
	  if (e1.num_value != e2.num_value) {
	    return false;
	  } else if (e1.den_value != e2.den_value) {
	    return false;
	  }
	  break;
	default:
	  break;
	}

	if (e1.children.size() != e2.children.size()) {
	  return false;
	}
	for (unsigned childNr = 0; childNr < e1.children.size(); childNr++) {
	  if (e1.children[childNr] != e2.children[childNr]) {
	    return false;
	  }
	}
	return true;
      }
    };
    typedef std::tr1::unordered_map<const ExprContent,const Expr,hashstate,eqstate> UniqTab;
    static UniqTab uniqTab;
    static int cleanupint;
    static ExprHashMap<VarType> varTypes;
    static ExprHashMap<pair<pair<int,int>,pair<int,int> > > bounds;
    static std::tr1::unordered_map<VarType, const string,std::tr1::hash<unsigned> > varTypeStrings;

    Expr::Kind kind;
    string identifier;
    union {
      bool bool_value;
      long long num_value;
    };
    long long den_value;
    vector<const ExprContent *> children;
    mutable unsigned refs;
  };

  ExprContent::UniqTab ExprContent::uniqTab;
  int ExprContent::cleanupint = atexit(ExprContent::setEntriesNull);
  ExprHashMap<VarType> ExprContent::varTypes;
  ExprHashMap<pair<pair<int,int>,pair<int,int> > > ExprContent::bounds;
  
  static std::tr1::unordered_map<VarType, const string, std::tr1::hash<unsigned> > prepareVarTypeStrings() {
    std::tr1::unordered_map<VarType, const string, std::tr1::hash<unsigned> > result;

    result.insert(make_pair(NullVarType, "null"));
    result.insert(make_pair(BoolVarType, "bool"));
    result.insert(make_pair(IntVarType, "int"));
    result.insert(make_pair(RangeVarType, "range"));
    result.insert(make_pair(RealVarType, "real"));
    
    return result;
  }

  std::tr1::unordered_map<VarType, const string, std::tr1::hash<unsigned> > ExprContent::varTypeStrings = prepareVarTypeStrings();

  ExprContent::ExprContent() {
    identifier = "";
    bool_value = false;
    num_value = 0;
    den_value = 1;
    refs = 0;
  }

  void ExprContent::setEntriesNull() {
    for (UniqTab::iterator it = uniqTab.begin(); it != uniqTab.end(); it++) {
      ((Expr &) it->second).content = NULL;
    }
  }

  void ExprContent::incRecRefs(const ExprContent *e) {
    assert(NULL != e);
    for (unsigned childNr = 0; childNr < e->children.size(); childNr++) {
      ExprContent::incRecRefs(e->children[childNr]);
    }
    e->refs++;
  }
   
  void ExprContent::decRecRefs(const ExprContent *e) {
    assert(NULL != e);
    for (unsigned childNr = 0; childNr < e->children.size(); childNr++) {
      ExprContent::decRecRefs(e->children[childNr]);
    }
    e->refs--;
    if (0 == e->refs) {
      UniqTab::iterator it = uniqTab.find(*e);
      ((Expr &) it->second).content = NULL;
      uniqTab.erase(it);
    }
  }

  ExprContent *ExprContent::makeUnique(ExprContent *cont) {
    assert(NULL != cont);
    ExprContent *res;
    
    UniqTab::iterator it = uniqTab.find(*cont);
    if (it != uniqTab.end()) {
      res = (ExprContent *) &(it->first);
    } else {
      Expr defaultExpr;

      uniqTab.insert(make_pair(*cont, defaultExpr));
      UniqTab::iterator it = uniqTab.find(*cont);
      res = (ExprContent *) &(it->first);
      ((Expr &) it->second).content = res;
    }
    if (res != cont) {
      delete cont;
    }

    return res;
  }

  ExprContent *ExprContent::fromAST(const boost::shared_ptr<prismparser_ast::Expr> &ast) {
    ExprContent *cont = new ExprContent();

    for (unsigned childNr = 0; childNr < ast->arity(); childNr++) {
      cont->children.push_back(fromAST(ast->children[childNr]));
    }
    
    switch (ast->getKind()) {
    case prismparser_ast::Null:
      assert(false);
      break;

    case prismparser_ast::Var:
      cont->kind = Expr::Var;
      break;
    case prismparser_ast::Bool:
      cont->kind = Expr::Bool;
      break;
    case prismparser_ast::Int:
      cont->kind = Expr::Rat;
      break;
    case prismparser_ast::Rat:
      cont->kind = Expr::Rat;
      break;
    case prismparser_ast::Double:
      cont->kind = Expr::Rat;
      break;
    
    case prismparser_ast::Not:
      cont->kind = Expr::Not;
      break;
    case prismparser_ast::And:
      cont->kind = Expr::And;
      break;
    case prismparser_ast::Or:
      cont->kind = Expr::Or;
      break;
    case prismparser_ast::Impl:
      cont->kind = Expr::Impl;
      break;
        
    case prismparser_ast::Eq:
      cont->kind = Expr::Eq;
      break;
    case prismparser_ast::Neq:
      cont->kind = Expr::Neq;
      break;
    case prismparser_ast::Lt:
      cont->kind = Expr::Lt;
      break;
    case prismparser_ast::Gt:
      cont->kind = Expr::Gt;
      break;
    case prismparser_ast::Le:
      cont->kind = Expr::Le;
      break;
    case prismparser_ast::Ge:
      cont->kind = Expr::Ge;
      break;
    
    case prismparser_ast::Plus:
      cont->kind = Expr::Plus;
      break;
    case prismparser_ast::Minus:
      cont->kind = Expr::Minus;
      break;
    case prismparser_ast::Uminus:
      cont->kind = Expr::Uminus;
      break;
    case prismparser_ast::Mult:
      cont->kind = Expr::Mult;
      break;
    case prismparser_ast::Div:
      cont->kind = Expr::Div;
      break;
    case prismparser_ast::Pow:
      cont->kind = Expr::Pow;
      break;
    
    case prismparser_ast::Ite:
      cont->kind = Expr::Ite;
      break;
    case prismparser_ast::Min:
      cont->kind = Expr::Min;
      break;
    case prismparser_ast::Max:
      cont->kind = Expr::Max;
      break;
    default:
      assert(false);
    }

    if (prismparser_ast::Bool == ast->getKind()) {
      cont->bool_value = ast->getBool();
    } else if (prismparser_ast::Int == ast->getKind()) {
      cont->num_value = ast->getInt();
      cont->kind = Expr::Rat;
    } else if (prismparser_ast::Double == ast->getKind()) {
      double2rat(ast->getDoubleAsString(), cont->num_value, cont->den_value);
    } else if (prismparser_ast::Var == ast->getKind()) {
      cont->identifier = ast->getIdentifier();
    }
    cont = makeUnique(cont);

    return cont;
  }

  bool ExprContent::isConst() const {
    return ((Expr::Bool == kind) || (Expr::Rat == kind));
  }

  bool ExprContent::isInteger() const {
    return ((Expr::Rat == kind) && (1 == den_value));
  }

  string ExprContent::op2string(Expr::Kind kind) {
    switch (kind) {
    case Expr::Not:
      return "!";
    case Expr::And:
      return "&";
    case Expr::Or:
      return "|";
    case Expr::Impl:
      return "=>";
    case Expr::Eq:
      return "=";
    case Expr::Neq:
      return "!=";
    case Expr::Lt:
      return "<";
    case Expr::Gt:
      return ">";
    case Expr::Le:
      return "<=";
    case Expr::Ge:
      return ">=";
    case Expr::Plus:
      return "+";
    case Expr::Minus:
      return "-";
    case Expr::Uminus:
      return "-";
    case Expr::Mult:
      return "*";
    case Expr::Div:
      return "/";
    default:
      assert(false);
    }
  }

  string ExprContent::optEmbrace() const {
    string result;
    if ((Expr::Var == kind)
	|| (Expr::Not == kind)
	|| (Expr::Pow == kind)
	|| (Expr::Min == kind)
	|| (Expr::Max == kind)) {
      return toString();
    } else {
      if (children.size() > 1) {
	result += "(";
      }
      result += toString();
      if (children.size() > 1) {
	result += ")";
      }
    }
    return result;
  }

  string ExprContent::toString() const {
    string result;
    switch (kind) {
    case Expr::Var:
      result = identifier;
      break;
    case Expr::Bool:
      result = bool_value ? "true" : "false";
      break;
    case Expr::Rat:
      if (1 == den_value) {
	result = intToString(num_value);
      } else {
	result = intToString(num_value) + "/" + intToString(den_value);
      }
      break;
    case Expr::Not:
      result += "!" + children[0]->optEmbrace();
      break;
    case Expr::Pow:
      result = "pow(" + children[0]->toString() + "," + children[1]->toString() + ")";
    case Expr::Ite:
      result = children[0]->optEmbrace() + "?" + children[1]->optEmbrace()
	+ ":" + children[2]->optEmbrace();
      break;
    case Expr::Min:
      result = "min(" + children[0]->toString() + "," + children[1]->toString() + ")";
      break;
    case Expr::Max:
      result = "max(" + children[0]->toString() + "," + children[1]->toString() + ")";
      break;
    case Expr::Uminus:
      result = "-" + children[0]->optEmbrace();
      break;
    case Expr::And:
    case Expr::Or:
    case Expr::Impl:
    case Expr::Eq:
    case Expr::Neq:
    case Expr::Lt:
    case Expr::Gt:
    case Expr::Le:
    case Expr::Ge:    
    case Expr::Plus:
    case Expr::Minus:
    case Expr::Mult:
    case Expr::Div:
      for (unsigned childNr = 0; childNr < children.size(); childNr++) {
	result += children[childNr]->optEmbrace();
	if (childNr < children.size() - 1) {
	  result += op2string(kind);
	}
      }
      break;
    }

    return result;
  }

  const ExprContent *ExprContent::substExpr(const ExprHashMap<Expr> &table) const {
    Expr finder;
    finder.content = this;
    ExprHashMap<Expr>::const_iterator it = table.find(finder);
    if (it != table.end()) {
      finder.content = NULL;
      return it->second.content;
    } else {
      ExprContent *res = new ExprContent;
      res->identifier = identifier;
      res->bool_value = bool_value;
      res->num_value = num_value;
      res->den_value = den_value;
      res->kind = kind;
      for (unsigned childNr = 0; childNr < children.size(); childNr++) {
	res->children.push_back(children[childNr]->substExpr(table));
      }
      res = makeUnique(res);
      finder.content = NULL;
      return res;
    }
  }

  void ExprContent::cleanupTable() {
    vector<UniqTab::iterator> remove;
    for (UniqTab::iterator it = uniqTab.begin(); it != uniqTab.end(); it++) {
      if (0 == it->first.refs) {
	remove.push_back(it);
      }
    }
    for (unsigned remNr = 0; remNr < remove.size(); remNr++) {
      uniqTab.erase(remove[remNr]);
    }
  }

  const ExprContent *ExprContent::simplify() const {
    ExprContent *res = new ExprContent;
    res->identifier = identifier;
    res->bool_value = bool_value;
    res->num_value = num_value;
    res->den_value = den_value;
    res->kind = kind;

    switch (kind) {
    case Expr::Var:
    case Expr::Bool:
    case Expr::Rat:
      delete res;
      return this;
    case Expr::Not: {
      const ExprContent *c = children[0]->simplify();
      if (!c->isConst()) {
	res->children.push_back(c);
      } else {
	res->kind = Expr::Bool;
	res->bool_value = !c->bool_value;
      }
      break;
    }
    case Expr::And: {
      vector<const ExprContent *> cs;
      bool oneFalse = false;
      for (unsigned childNr = 0; childNr < children.size(); childNr++) {
	const ExprContent *c = children[childNr]->simplify();
	if (!c->isConst()) {
	  cs.push_back(c);
	} else if (!c->bool_value) {
	  oneFalse = true;
	}
      }
      if (oneFalse) {
	res->kind = Expr::Bool;
	res->bool_value = false;
      } else if (0 == cs.size()) {
	res->kind = Expr::Bool;
	res->bool_value = true;
      } else if (1 == cs.size()) {
	delete res;
	return (ExprContent *) cs[0];
      } else {
	for (unsigned childNr = 0; childNr < cs.size(); childNr++) {
	  res->children.push_back(cs[childNr]);
	}
      }
      break;
    }
    case Expr::Or: {
      vector<const ExprContent *> cs;
      bool oneTrue = false;
      for (unsigned childNr = 0; childNr < children.size(); childNr++) {
	const ExprContent *c = children[childNr]->simplify();
	if (!c->isConst()) {
	  cs.push_back(c);
	} else if (c->bool_value) {
	  oneTrue = true;
	}
      }
      if (oneTrue) {
	res->kind = Expr::Bool;
	res->bool_value = true;
      } else if (0 == cs.size()) {
	res->kind = Expr::Bool;
	res->bool_value = false;
      } else if (1 == cs.size()) {
	delete res;
	return (ExprContent *) cs[0];
      } else {
	for (unsigned childNr = 0; childNr < cs.size(); childNr++) {
	  res->children.push_back(cs[childNr]);
	}
      }
      break;
    }
    case Expr::Impl: {
      const ExprContent *c1 = children[0]->simplify();
      const ExprContent *c2 = children[1]->simplify();
      if (c1->isConst() && c2->isConst()) {
	res->kind = Expr::Bool;
	res->bool_value = !c1->bool_value || c2->bool_value;
      } else if (c1->isConst()) {
	if (c1->bool_value) {
	  delete res;
	  res = (ExprContent *) c2;
	} else {
	  res->kind = Expr::Bool;
	  res->bool_value = true;
	}
      } else if (c2->isConst()) {
	if (c2->bool_value) {
	  res->kind = Expr::Bool;
	  res->bool_value = true;	  
	} else {
	  res->kind = Expr::Not;
	  res->children.push_back(c1);
	}
      } else {
	res->children.push_back(c1);
	res->children.push_back(c2);
      }
      break;
    }
    case Expr::Eq: {
      const ExprContent *c1 = children[0]->simplify();
      const ExprContent *c2 = children[1]->simplify();
      if (c1 != c2) {
	if (c1->isConst() && c2->isConst()) {
	  res->kind = Expr::Bool;
	  res->bool_value = false;
	} else {
	  res->children.push_back(c1);
	  res->children.push_back(c2);
	}
      } else {
	res->kind = Expr::Bool;
	res->bool_value = true;
      }
      break;
    }
    case Expr::Neq: {
      const ExprContent *c1 = children[0]->simplify();
      const ExprContent *c2 = children[1]->simplify();
      if (c1 == c2) {
	res->kind = Expr::Bool;
	res->bool_value = false;
      } else {
	if (c1->isConst() && c2->isConst()) {
	  res->kind = Expr::Bool;
	  res->bool_value = true;
	} else {
	  res->children.push_back(c1);
	  res->children.push_back(c2);
	}
      }
      break;
    }
    case Expr::Lt: {
      const ExprContent *c1 = children[0]->simplify();
      const ExprContent *c2 = children[1]->simplify();
      if (c1 == c2) {
	res->kind = Expr::Bool;
	res->bool_value = false;
      } else if (c1->isConst() && c2->isConst()) {
	res->kind = Expr::Bool;
	res->bool_value = (c1->num_value * c2->den_value < c2->num_value * c1->den_value);
      } else {
	res->children.push_back(c1);
	res->children.push_back(c2);
      }
      break;
    }
    case Expr::Gt: {
      const ExprContent *c1 = children[0]->simplify();
      const ExprContent *c2 = children[1]->simplify();
      if (c1 == c2) {
	res->kind = Expr::Bool;
	res->bool_value = false;
      } else if (c1->isConst() && c2->isConst()) {
	res->kind = Expr::Bool;
	res->bool_value = (c1->num_value * c2->den_value > c2->num_value * c1->den_value);
      } else {
	res->children.push_back(c1);
	res->children.push_back(c2);
      }
      break;
    }
    case Expr::Le: {
      const ExprContent *c1 = children[0]->simplify();
      const ExprContent *c2 = children[1]->simplify();
      if (c1 == c2) {
	res->kind = Expr::Bool;
	res->bool_value = true;
      } else if (c1->isConst() && c2->isConst()) {
	res->kind = Expr::Bool;
	res->bool_value = (c1->num_value * c2->den_value <= c2->num_value * c1->den_value);
      } else {
	res->children.push_back(c1);
	res->children.push_back(c2);
      }
      break;
    }
    case Expr::Ge: {
      const ExprContent *c1 = children[0]->simplify();
      const ExprContent *c2 = children[1]->simplify();
      if (c1 == c2) {
	res->kind = Expr::Bool;
	res->bool_value = true;
      } else if (c1->isConst() && c2->isConst()) {
	res->kind = Expr::Bool;
	res->bool_value = (c1->num_value * c2->den_value >= c2->num_value * c1->den_value);
      } else {
	res->children.push_back(c1);
	res->children.push_back(c2);
      }
      break;
    }
    case Expr::Plus: {
      vector<const ExprContent *> cs;
      long long values = 0;
      long long lmult = 1;
      for (unsigned childNr = 0; childNr < children.size(); childNr++) {
	const ExprContent *c = children[childNr]->simplify();
	if (Expr::Rat != c->kind) {
	  cs.push_back(c);
	} else {
	  lmult = lcm(lmult, c->den_value);
	}
      }
      for (unsigned childNr = 0; childNr < children.size(); childNr++) {
	const ExprContent *c = children[childNr]->simplify();
	if (Expr::Rat == c->kind) {
	  values += (lmult / c->den_value) * c->num_value;
	}
      }
      if (0 == cs.size()) {
	res->kind = Expr::Rat;
	res->num_value = values;
	res->den_value = lmult;
      } else if (1 == cs.size()) {
	if (0 == values) {
	  delete res;
	  res = (ExprContent *) cs[0];
	} else {
	  res->children.push_back(cs[0]);
	  ExprContent *rest = new ExprContent;
	  rest->kind = Expr::Rat;
	  rest->num_value = values;
	  rest->den_value = lmult;
	  rest = makeUnique(rest);
	  res->children.push_back(rest);
	}
      } else {
	for (unsigned childNr = 0; childNr < cs.size(); childNr++) {
	  res->children.push_back(cs[childNr]);
	}
	if (0 != values) {
	  ExprContent *rest = new ExprContent;
	  rest->kind = Expr::Rat;
	  rest->num_value = values;
	  rest->den_value = lmult;
	  rest = makeUnique(rest);
	  res->children.push_back(rest);
	}
      }
      break;
    }
    case Expr::Uminus: {
      assert(1 == children.size());
      const ExprContent *c = children[0]->simplify();
      if (c->isConst()) {
	res->kind = Expr::Rat;
	res->num_value = -c->num_value;
	res->den_value = c->den_value;
      } else {
	res->children.push_back(children[0]->simplify());
      }
      break;
    }
    case Expr::Minus: {
      assert(2 == children.size());
      const ExprContent *c1 = children[0]->simplify();
      const ExprContent *c2 = children[1]->simplify();
      if (c1->isConst() && c2->isConst()) {
	res->kind = Expr::Rat;
	res->num_value = c1->num_value * c2->den_value - c2->num_value * c1->den_value;
	res->den_value = c1->den_value * c2->den_value;
	normalise(res->num_value, res->den_value);
      } else {
	res->children.push_back(c1);
	res->children.push_back(c2);
      }
      break;
    }
    case Expr::Mult: {
      vector<const ExprContent *> cs;
      long long num = 1;
      long long den = 1;
      for (unsigned childNr = 0; childNr < children.size(); childNr++) {
	const ExprContent *c = children[childNr]->simplify();
	if (Expr::Rat != c->kind) {
	  cs.push_back(c);
	} else {
	  num *= c->num_value;
	  den *= c->den_value;
	  normalise(num, den);
	}
      }
      if (0 == cs.size()) {
	res->kind = Expr::Rat;
	res->num_value = num;
	res->den_value = den;
      } else if (1 == cs.size()) {
	if ((1 == num) && (1 == den)) {
	  delete res;
	  res = (ExprContent *) cs[0];
	} else {
	  res->children.push_back(cs[0]);
	  ExprContent *rest = new ExprContent;
	  rest->kind = Expr::Rat;
	  rest->num_value = num;
	  rest->den_value = den;
	  res->children.push_back(rest);
	}
      } else {
	for (unsigned childNr = 0; childNr < cs.size(); childNr++) {
	  res->children.push_back(cs[childNr]);
	}
	if ((1 != num) || (1 != den)) {
	  ExprContent *rest = new ExprContent;
	  rest->kind = Expr::Rat;
	  rest->num_value = num;
	  rest->den_value = den;
	  rest = makeUnique(rest);
	  res->children.push_back(rest);
	}
      }
      break;
    }
    case Expr::Div: {
      assert(2 == children.size());
      const ExprContent *c1 = children[0]->simplify();
      const ExprContent *c2 = children[1]->simplify();
      if (c1->isConst() && c2->isConst()) {
	res->kind = Expr::Rat;
	res->num_value = c1->num_value * c2->den_value;
	res->den_value = c1->den_value * c2->num_value;
	normalise(res->num_value, res->den_value);
      } else {
	res->children.push_back(c1);
	res->children.push_back(c2);
      }
      break;
    }
    case Expr::Pow: {
      assert(2 == children.size());
      const ExprContent *c1 = children[0]->simplify();
      const ExprContent *c2 = children[1]->simplify();
      if (c1->isConst() && c2->isInteger()) {
	long long times = c2->num_value;
	int sign = times < 0 ? -1 : 1;
	times *= sign;
	long long num = 1;
	long long den = 1;
	for (long long i = 0; i < times; i++) {
	  num *= c1->num_value;
	  den *= c1->den_value;
	}
	res->kind = Expr::Rat;
	if (-1 == sign) {
	  res->num_value = den;
	  res->den_value = num;
	} else {
	  res->num_value = num;
	  res->den_value = den;
	}
      } else {
	res->children.push_back(c1);
	res->children.push_back(c2);
      }
      break;
    }
    case Expr::Ite: {
      assert(3 == children.size());
      const ExprContent *c1 = children[0]->simplify();
      const ExprContent *c2 = children[1]->simplify();
      const ExprContent *c3 = children[1]->simplify();
      if (c1->isConst()) {
	delete res;
	if (c1->bool_value) {
	  res = (ExprContent *) c2;
	} else {
	  res = (ExprContent *) c3;
	}
      } else {
	res->children.push_back(c1);
	res->children.push_back(c2);
	res->children.push_back(c3);
      }
      break;
    }
    case Expr::Min: {
      assert(2 == children.size());
      const ExprContent *c1 = children[0]->simplify();
      const ExprContent *c2 = children[1]->simplify();
      if (c1->isConst() && c2->isConst()) {
	res->kind = Expr::Rat;
	delete res;
	if (c1->num_value * c2->den_value < c2->num_value * c2->den_value) {
	  res = (ExprContent *) c1;
	} else {
	  res = (ExprContent *) c2;
	}
      } else {
	res->children.push_back(c1);
	res->children.push_back(c2);
      }
      break;
    }
    case Expr::Max: {
      assert(2 == children.size());
      const ExprContent *c1 = children[0]->simplify();
      const ExprContent *c2 = children[1]->simplify();
      if (c1->isConst() && c2->isConst()) {
	res->kind = Expr::Rat;
	delete res;
	if (c1->num_value * c2->den_value > c2->num_value * c2->den_value) {
	  res = (ExprContent *) c1;
	} else {
	  res = (ExprContent *) c2;
	}
      } else {
	res->children.push_back(c1);
	res->children.push_back(c2);
      }
      break;
    }
    }
    
    res = makeUnique(res);
    return res;
  }

  Expr::Expr() {
    content = NULL;
  }

  Expr::Expr(const boost::shared_ptr<prismparser_ast::Expr> &ast) {
    content = ExprContent::fromAST(ast);
    ExprContent::incRecRefs(content);
  }

  Expr::Expr(const Expr &_expr) {
    content = _expr.content;
    if (NULL != content) {
      ExprContent::incRecRefs(content);
    }
  }

  Expr::~Expr() {
    if (NULL != content) {
      ExprContent::decRecRefs(content);
    }
  }

  unsigned Expr::arity() const {
    assert(NULL != content);
    return content->children.size();
  }

  const Expr &Expr::operator[](const unsigned index) const {
    assert(index < content->children.size());
    return ExprContent::uniqTab.find(*content->children[index])->second;
  }

  Expr Expr::multExpr(const Expr &e1, const Expr &e2) {
    assert(e1.content != NULL);
    assert(e2.content != NULL);
    ExprContent *cont = new ExprContent();
    cont->kind = Mult;
    cont->children.push_back(e1.content);
    cont->children.push_back(e2.content);
    cont = ExprContent::makeUnique(cont);
    ExprContent::incRecRefs(cont);
    return ExprContent::uniqTab[*cont];
  }

  Expr Expr::andExpr(const Expr &e1, const Expr &e2) {
    assert(e1.content != NULL);
    assert(e2.content != NULL);
    ExprContent *cont = new ExprContent();
    cont->kind = And;
    cont->children.push_back(e1.content);
    cont->children.push_back(e2.content);
    cont = ExprContent::makeUnique(cont);
    ExprContent::incRecRefs(cont);
    return ExprContent::uniqTab[*cont];
  }

  Expr Expr::andExpr(const vector<Expr> &es) {
    ExprContent *cont = new ExprContent();
    if (0 == es.size()) {
      cont->kind = Bool;
      cont->bool_value = true;
    } else if (1 == es.size()) {
      delete cont;
      assert(NULL != es[0].content);
      cont = (ExprContent *) es[0].content;
    } else {
      cont->kind = And;
      for (unsigned eNr = 0; eNr < es.size(); eNr++) {
	assert(NULL != es[eNr].content);
	cont->children.push_back(es[eNr].content);
      }
    }
    cont = ExprContent::makeUnique(cont);
    ExprContent::incRecRefs(cont);
    return ExprContent::uniqTab[*cont];
  }

  Expr Expr::orExpr(const Expr &e1, const Expr &e2) {
    assert(e1.content != NULL);
    assert(e2.content != NULL);
    ExprContent *cont = new ExprContent();
    cont->kind = Or;
    cont->children.push_back(e1.content);
    cont->children.push_back(e2.content);
    cont = ExprContent::makeUnique(cont);
    ExprContent::incRecRefs(cont);
    return ExprContent::uniqTab[*cont];
  }

  Expr Expr::orExpr(const vector<Expr> &es) {
    ExprContent *cont = new ExprContent();
    if (0 == es.size()) {
      cont->kind = Bool;
      cont->bool_value = false;
    } else if (1 == es.size()) {
      delete cont;
      assert(NULL != es[0].content);
      cont = (ExprContent *) es[0].content;
    } else {
      cont->kind = Or;
      for (unsigned eNr = 0; eNr < es.size(); eNr++) {
	assert(NULL != es[eNr].content);
	cont->children.push_back(es[eNr].content);
      }
    }
    cont = ExprContent::makeUnique(cont);
    ExprContent::incRecRefs(cont);
    return ExprContent::uniqTab[*cont];
  }

  Expr Expr::notExpr(const Expr &e) {
    assert(e.content != NULL);
    ExprContent *cont = new ExprContent();
    cont->kind = Not;
    cont->children.push_back(e.content);
    cont = ExprContent::makeUnique(cont);
    ExprContent::incRecRefs(cont);
    return ExprContent::uniqTab[*cont];
  }

  Expr Expr::varExpr(const string &__identifier) {
    assert("" != __identifier);
    ExprContent *cont = new ExprContent();
    cont->kind = Var;
    cont->identifier = __identifier;
    cont = ExprContent::makeUnique(cont);
    ExprContent::incRecRefs(cont);
    return ExprContent::uniqTab[*cont];
  }

  Expr Expr::eqExpr(const Expr &e1, const Expr &e2) {
    assert(e1.content != NULL);
    assert(e2.content != NULL);
    ExprContent *cont = new ExprContent();
    cont->kind = Eq;
    cont->children.push_back(e1.content);
    cont->children.push_back(e2.content);
    cont = ExprContent::makeUnique(cont);
    ExprContent::incRecRefs(cont);
    return ExprContent::uniqTab[*cont];
  }

  Expr Expr::ltExpr(const Expr &e1, const Expr &e2) {
    assert(e1.content != NULL);
    assert(e2.content != NULL);
    ExprContent *cont = new ExprContent();
    cont->kind = Lt;
    cont->children.push_back(e1.content);
    cont->children.push_back(e2.content);
    cont = ExprContent::makeUnique(cont);
    ExprContent::incRecRefs(cont);
    return ExprContent::uniqTab[*cont];
  }
  
  Expr Expr::gtExpr(const Expr &e1, const Expr &e2) {
    assert(e1.content != NULL);
    assert(e2.content != NULL);
    ExprContent *cont = new ExprContent();
    cont->kind = Gt;
    cont->children.push_back(e1.content);
    cont->children.push_back(e2.content);
    cont = ExprContent::makeUnique(cont);
    ExprContent::incRecRefs(cont);
    return ExprContent::uniqTab[*cont];
  }

  Expr Expr::leExpr(const Expr &e1, const Expr &e2) {
    assert(e1.content != NULL);
    assert(e2.content != NULL);
    ExprContent *cont = new ExprContent();
    cont->kind = Le;
    cont->children.push_back(e1.content);
    cont->children.push_back(e2.content);
    cont = ExprContent::makeUnique(cont);
    ExprContent::incRecRefs(cont);
    return ExprContent::uniqTab[*cont];
  }

  Expr Expr::geExpr(const Expr &e1, const Expr &e2) {
    assert(e1.content != NULL);
    assert(e2.content != NULL);
    ExprContent *cont = new ExprContent();
    cont->kind = Ge;
    cont->children.push_back(e1.content);
    cont->children.push_back(e2.content);
    cont = ExprContent::makeUnique(cont);
    ExprContent::incRecRefs(cont);
    return ExprContent::uniqTab[*cont];
  }

  Expr Expr::falseExpr() {
    ExprContent *cont = new ExprContent();
    cont->kind = Bool;
    cont->bool_value = false;
    cont = ExprContent::makeUnique(cont);
    ExprContent::incRecRefs(cont);
    return ExprContent::uniqTab[*cont];
  }

  Expr Expr::trueExpr() {
    ExprContent *cont = new ExprContent();
    cont->kind = Bool;
    cont->bool_value = true;
    cont = ExprContent::makeUnique(cont);
    ExprContent::incRecRefs(cont);
    return ExprContent::uniqTab[*cont];
  }

  Expr Expr::ratExpr(long long num, long long den) {
    assert(0 != den);
    ExprContent *cont = new ExprContent();

    normalise(num, den);
    cont->kind = Rat;
    cont->num_value = num;
    cont->den_value = den;
    cont = ExprContent::makeUnique(cont);
    ExprContent::incRecRefs(cont);
    return ExprContent::uniqTab[*cont];
  }

  Expr Expr::ratExpr(const string &str, int base) {
    assert(10 == base);
    ExprContent *cont = new ExprContent();
    cont->kind = Rat;
    cont->num_value = 0;
    long long digVal = 1;
    for (int charPos = str.size(); charPos > 0; charPos--) {
      digVal *= base;
      cont->num_value += (str[charPos] - '0');
    }
    cont = ExprContent::makeUnique(cont);
    ExprContent::incRecRefs(cont);
    return ExprContent::uniqTab[*cont];
  }

  Expr Expr::ratExpr(const string &str) {
    assert("" != str);
    ExprContent *cont = new ExprContent();
    double2rat(str, cont->num_value, cont->den_value);
    cont->kind = Rat;
    cont = ExprContent::makeUnique(cont);
    ExprContent::incRecRefs(cont);
    return ExprContent::uniqTab[*cont];
  }

  string Expr::toString() const {
    if (NULL == content) {
      return "Null";
    } else {
      return content->toString();
    }
  }

  bool Expr::isNull() const {
    return (NULL == content);
  }

  const string &Expr::getName() const {
    return content->identifier;
  }

  bool Expr::isBoolConst() const {
    return (Bool == content->kind);
  }

  bool Expr::isITE() const {
    return (Ite == content->kind);
  }

  bool Expr::isRational() const {
    if (NULL == content) {
      return false;
    }
    return (Rat == content->kind);
  }

  bool Expr::isInteger() const {
    return (isRational() && (1 == content->den_value));
  }

  bool Expr::isFalse() const {
    return ((Bool == content->kind)
	    && !content->bool_value);
  }

  bool Expr::isTrue() const {
    return ((Bool == content->kind)
	    && content->bool_value);
  }

  Expr Expr::simplify(const Expr &expr) {
    const ExprContent *resCont = expr.content->simplify();
    ExprContent::incRecRefs(resCont);
    ExprContent::cleanupTable();
    return ExprContent::uniqTab[*resCont];
  }

  Expr Expr::substExpr(const ExprHashMap<Expr> &table) const {
    const ExprContent *resCont = content->substExpr(table);
    ExprContent::incRecRefs(resCont);
    return ExprContent::uniqTab[*resCont];
  }

  Expr Expr::substExpr(const vector<Expr> &from, const vector<Expr> &to) const {
    assert(from.size() == to.size());
    ExprHashMap<Expr> table;
    for (unsigned exprNr = 0; exprNr < from.size(); exprNr++) {
      table.insert(make_pair(from[exprNr], to[exprNr]));
    }
    const ExprContent *resCont = content->substExpr(table);
    ExprContent::incRecRefs(resCont);
    return ExprContent::uniqTab[*resCont];
  }

  long long Expr::getNumerator() const {
    return content->num_value;
  }

  long long Expr::getDenominator() const {
    return content->den_value;
  }

  bool Expr::isVar() const {
    return (Var == content->kind);
  }

  Expr::Kind Expr::getKind() const {
    return content->kind;
  }

  ostream &operator<<(ostream &stream, const Expr &e) {
    stream << e.toString();
    return stream;
  }

  bool operator==(const Expr &e1, const Expr &e2) {
    return (e1.content == e2.content);
  }

  Expr &Expr::operator=(const Expr &from) {
    if (NULL != from.content) {
      ExprContent::incRecRefs(from.content);
      if (NULL != content) {
	ExprContent::decRecRefs(content);
      }
      content = from.content;
    }

    return *this;
  }

  void Expr::setVarType(const Expr &expr, VarType varType) {
    assert(NULL != expr.content);
    assert(Var == expr.content->kind);
    ExprContent::varTypes.insert(make_pair(expr,varType));
  }

  VarType Expr::getVarType(const Expr &expr) {
    assert(NULL != expr.content);
    assert(Var == expr.content->kind);
    return ExprContent::varTypes.find(expr)->second;
  }

  void Expr::setVarBounds
  (const Expr &expr,int lowerNum, int lowerDen,
   int upperNum, int upperDen) {
    assert(NULL != expr.content);
    assert(Var == expr.content->kind);
    pair<pair<int,int>,pair<int,int> > b = make_pair(make_pair(lowerNum, lowerDen), make_pair(upperNum, upperDen));
    ExprContent::bounds.insert(make_pair(expr, b));
  }
  
  int Expr::getVarLowerBoundNum(const Expr &expr) {
    assert(NULL != expr.content);
    assert(Var == expr.content->kind);
    return ExprContent::bounds.find(expr)->second.first.first;
  }

  int Expr::getVarLowerBoundDen(const Expr &expr) {
    assert(NULL != expr.content);
    assert(Var == expr.content->kind);
    return ExprContent::bounds.find(expr)->second.first.second;
  }

  int Expr::getVarUpperBoundNum(const Expr &expr) {
    assert(NULL != expr.content);
    assert(Var == expr.content->kind);
    return ExprContent::bounds.find(expr)->second.second.first;
  }

  int Expr::getVarUpperBoundDen(const Expr &expr) {
    assert(NULL != expr.content);
    assert(Var == expr.content->kind);
    return ExprContent::bounds.find(expr)->second.second.second;
  }

  const string &Expr::varTypeToString(VarType varType) {
    return ExprContent::varTypeStrings.find(varType)->second;
  }
}
