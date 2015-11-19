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
 * Copyright 2007-2010 Bjoern Wachter (Bjoern.Wachter@comlab.ox.ac.uk)
 * Copyright 2009-2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#include <stdexcept>
#include "AST.h"
#include "Util.h"

namespace prismparser_ast {
  Substitution dummy_subst;
  ActionSubstitution dummy_action_subst;

  Expr::Expr() : kind(Null) {
  }

  /** \brief Boolean constant */
  Expr::Expr(bool b) : kind(Bool), bool_value(b) {
  }

  /** \brief integer constant */
  Expr::Expr(int i) : kind(Int), int_value(i) {
  }

  Expr::Expr(Kind k, const char *val) {
    kind = k;
    if (Double == k) {
      double_value = val;
    } else if (Var == k) {
      identifier = val;
    } else {
      assert(false);
    }
  }

  /** \brief zeroary operation */
  Expr::Expr (Kind k) : kind (k) {
  }

  /** \brief unary operation */
  Expr::Expr (Kind k, Expr * e1) : kind(k) {
    assert(e1);
    addChild(e1);
  }

  /** \brief binary operation */
  Expr::Expr (Kind k, Expr *e1, Expr *e2) : kind (k) {
    assert(e1);
    assert(e2);
    addChild(e1);
    addChild(e2);
  }

  /** \brief ternary operation */
  Expr::Expr(Kind k, Expr *e1, Expr *e2, Expr *e3) : kind(k) {
    assert(e1);
    assert(e2);
    assert(e3);
    addChild(e1);
    addChild(e2);
    addChild(e3);
  }

  Expr::Expr(Kind k, Expr *e1, Expr *e2, Expr *e3, Expr *e4) : kind(k) {
    assert(e1);
    assert(e2);
    assert(e3);
    assert(e4);
    addChild(e1);
    addChild(e2);
    addChild(e3);
    addChild(e4);
  }

  Expr::Expr(Kind k, Expr *e1, Expr *e2, Expr *e3, Expr *e4, Expr *e5) : kind(k) {
    assert(e1);
    assert(e2);
    assert(e3);
    assert(e4);
    assert(e5);
    addChild(e1);
    addChild(e2);
    addChild(e3);
    addChild(e4);
    addChild(e5);
  }

  /**
   * Notice that eptr is not added to list of children of new expression.
   * You have to free it yourself.
   */
  Expr::Expr(const Expr *eptr, const Substitution &s) {
    assert(eptr);
    const Expr& e(*eptr);
    switch (e.kind) {
    case Var:
      {
	kind = e.kind;
	Substitution::const_iterator i (s.find (e.identifier));
	(*this) = (i == s.end ()) ? e : *(i->second);
      }
      break;
    case Bool: case Int: case Double:
      (*this) = e;
      break;
    default:
      kind = e.kind;
      for (Exprs::const_iterator i = e.children.begin (); i != e.children.end (); ++i)
      	{
	  addChild(new Expr (i->get(), s));
      	}
      break;
    }
  }

  Expr & Expr::operator=(const Expr &e) {
    kind = e.kind;
    identifier = e.identifier;
    children = e.children;
    bool_value = e.bool_value;
    double_value = e.double_value;
    int_value = e.int_value;

    return *this;
  }

  bool Expr::getBool() const {
    assert(kind==Bool);
    return bool_value;
  }

  int  Expr::getInt() const {
    assert(kind==Int);
    return int_value;

  }

  double Expr::getDoubleVal() const {
    assert((kind==Double) || (kind==Int));
    if (kind==Double) {
      return atof(double_value.c_str());
    } else {
      return int_value;
    }
  }

  const std::string &Expr::getDoubleAsString() const {
    assert(kind==Double);
    return double_value;
  }

  const std::string& Expr::getIdentifier() const {
    assert(kind == Var);
    return identifier;
  }

  std::string Expr::toString(const Kind& kind) const {
    std::string result;
    switch(kind) {
    case Null:
      result = "UNDEF";
      break;
    case Not:
      result = "!";
      break;
    case And:
      result = "&";
      break;
    case Or:
      result = "|";
      break;
    case Eq:
      result = "=";
      break;
    case Neq:
      result ="!=";
      break;
    case Lt:
      result = "<";
    case Gt:
      result = ">=";
      break;
    case Le:
      result = "<=";
      break;
    case Ge:
      result = ">=";
      break;
    case Plus:
      result = "+";
      break;
    case Minus:
    case Uminus:
      result = "-";
      break;
    case Mult:
      result = "*";
      break;
    case Div:
      result = "/";
      break;
    case Pow:
      result = "pow";
      break;
    case Ite:
      result = "ite";
      break;
    case Min:
      result = "min";
      break;
    case Max:
      result = "max";
      break;
    case P:
      result = "P";
      break;
    case Until:
      result = "U";
      break;
    case Impl:
      result = "=>";
      break;
    default:
      result = "?";
      break;
    }
    return result;
  }

  std::string Expr::toString() const {
    std::string result;

    std::vector<std::string> child_strings(children.size ());
    for (unsigned i = 0; i < children.size (); i++) {
      child_strings[i] = children[i]->toString ();
    }
    
    std::string kind_str(toString(kind));

    switch (kind) {
    case Null:
      result = kind_str;
      break;
    case Var:
      result = identifier;
      break;
    case Bool:
      result = bool_value ? "true" : "false";
      break;
    case Int:
      result = prismparser::intToString(int_value);
      break;
    case Double:
      result = double_value;
      break;
    case Not:
    case Next:
      assert(child_strings.size()==1);
      result = kind_str + "(" + child_strings[0] + ")";
      break;
    case And:
    case Or:

    case Eq:
    case Neq:
    case Lt:
    case Gt:
    case Le:
    case Ge:

    case Plus:
    case Minus:
    case Uminus:
    case Mult:
    case Div:
    case Until:
      result += "(";
      for (unsigned i = 0; i < child_strings.size (); ++i)
	{
	  result += ((i>0) ? " " + kind_str + " " : "") + child_strings[i];
	}
      result += ")";
      break;

    case Pow:
      result += kind_str + "(" + child_strings[0] + ","
	+ child_strings[1] + ")";
      break;

    case Ite:
    case Min:
    case Max:

    case P:
    case Steady:
    case ReachabilityReward:
    case CumulativeReward:
    case InstantaneousReward:
    case SteadyStateReward:
      {
	result += kind_str + "(";
	for (unsigned i = 0; i < child_strings.size (); i++) {
	  result += ( i>0 ? "," : "") +  child_strings[i];
	}
	result += ")";
      }
      break;
    case Impl:
      result = "(" + child_strings[0] + "=>" + child_strings[1] + ")";
      break;
    default:
      throw std::runtime_error("Property error in AST.");
    }
    return result;
  }

  Kind Expr::getKind() const {
    return kind;
  }

  unsigned Expr::arity() const {
    return children.size();
  }

  void Expr::addChild(Expr *e) {
    children.push_back (boost::shared_ptr < Expr > (e));
  }

  Update::Update(const Update &u, const Substitution &s) {
    for (Assignment::const_iterator i (u.assignment.begin ());
	 i != u.assignment.end (); i++) {
      Assign(new Expr((*i).first.get(),s), new Expr((*i).second.get(),s));
    }
  }

  std::string Update::toString() const {
    std::string result;
    for (Assignment::const_iterator i (assignment.begin ());
	 i != assignment.end (); ++i) {
      result += (i != assignment.begin() ? "&": "");
      result += "(" + (i->first)->toString () + "'=" + (i->second)->toString () + ")";
    }
    
    return result;
  }

  Alternative::Alternative(const Alternative &a, const Substitution &s)
    : update(a.update, s), weight(new Expr (a.weight.get(), s)) {
  }

  std::string Alternative::toString() const {
    return weight->toString() + " : " + update.toString();
  }

  Command::Command(const Command & c,
		   const Substitution & s,
		   const ActionSubstitution& as) : guard (new Expr (c.guard.get(), s))
  {
    for (Alternatives::const_iterator i (c.alternatives.begin ());
	 i != c.alternatives.end (); i++) {
	addAlternative (new Alternative (**i, s));
      }
    ActionSubstitution::const_iterator i (as.find (c.label));
    label = i == as.end ()? c.label : i->second;
  }

  std::string Command::toString() const {
    std::string result;
    
    result = "[" + label + "] " + guard->toString () + " -> ";

    for (Alternatives::const_iterator i (alternatives.begin ());
	 i != alternatives.end (); i++) {
      result += (i!=alternatives.begin() ? " + " : "") + (*i)->toString ();
    }
    result += ";";

    return result;
  }

  Variable::Variable( const Variable* v, const Substitution& s)
    : type(new Type(v->type.get(),s)) {
    is_parameter = v->is_parameter;
    Substitution::const_iterator i(s.find(v->identifier));
    if (i == s.end()) {
      identifier = v->identifier;
    } else {
      Expr* e(i->second.get());
      if(e->isVariable()) {
	identifier = e->getIdentifier();
      }
    }
    if (v->init.get()) {
      init = boost::shared_ptr<Expr>(new Expr(v->init.get(),s));
    }
  }

  std::string Variable::toString() const {
    return identifier + " : " + type->toString ();
  }

  Type::Type(const Type *t, const Substitution &s) : kind(t->kind) {
    switch(kind) {
    case Boolean:
    case Integer:
    case Double:
      (*this) = *t;
      break;
    case Range:
      range_data.lower = boost::shared_ptr<Expr> (new Expr(t->range_data.lower.get(),s));
      range_data.upper = boost::shared_ptr<Expr> (new Expr(t->range_data.upper.get(),s));
      break;
    }
  }

  Type &Type::operator=(const Type &t) {
    kind = t.kind;
    switch(t.kind) {
    case Boolean:
    case Integer:
    case Double:
      break;
    case Range:
      range_data = t.range_data;
      break;
    }
    return *this;
  }

  std::string Type::toString() const {
    std::string result;
    switch (kind) {
    case Boolean:
      result = "bool";
	break;
    case Integer:
      result = "int";
      break;
    case Double:
      result = "double";
      break;
    case Range:
      result = "[" + range_data.lower->toString () + ".." + range_data.upper->toString () + "]";
      break;
    }
    return result;
  }
  
  Module::Module(const std::string &n, const Module *m, const Substitution &s, const ActionSubstitution &as)
    : name(n) {
    assert(m);
    for (Commands::const_iterator i (m->commands.begin ());
	 i != m->commands.end (); i++) {
      addCommand (new Command (**i, s, as));
    }
    for (Variables::const_iterator i (m->locals.begin ());
	 i != m->locals.end (); i++) {
      const Variable* vptr(i->second.get());
      if (s.find(vptr->identifier)!=s.end()) { // only add if this is actually an instantiation of a local
	addVariable(new Variable(i->second.get(), s));
      } else {
	addVariable(new Variable(vptr, s));
      }
    }
  }


  std::string Module::toString() const {
    std::string result;
    result = "module " + name + "\n";
    for (Variables::const_iterator i (locals.begin ());
	 i != locals.end (); ++i) {
      result += (*i).second->toString() + ";\n";
    }
    
    for (Commands::const_iterator i (commands.begin ());
	 i != commands.end (); i++) {
      result += (*i)->toString() + "\n";
    }
    
    result += "endmodule";
    return result;
  }
  
  Model::Model(const Model &m, const Substitution &s) :
    model_type(m.model_type), actions(m.actions) {
    for (Variables::const_iterator i (m.globals.begin ());
	 i != m.globals.end (); i++) {
      addVariable (new Variable(i->second.get(), s));
    }

    if (initial.get()) {
      setInitial(new Expr(initial.get(), s));
    }

    for (Exprs::const_iterator i (invariants.begin ());
	 i != invariants.end (); i++) {
      addInvariant(new Expr((*i).get(),s));
    }
    
    for (Exprs::const_iterator i (properties.begin ());
	 i != properties.end (); i++) {
      addProperty(new Expr((*i).get(),s));
    }
  }

  std::string Model::toString() const {
    std::string result;

    for (Variables::const_iterator i (globals.begin ());
	 i != globals.end (); i++) {
      result += (*i).second->toString() + ";\n";
    }
    
    for (Modules::const_iterator i (modules.begin ());
	 i != modules.end (); i++) {
      result += ((*i).second)->toString() + "\n";
    }
    
    if (initial.get()) {
      result += "init " + (*initial).toString() + "endinit\n";
    }

    for (Exprs::const_iterator i (invariants.begin ());
	 i != invariants.end (); i++) {
      result += (**i).toString() + "\n";
    }
    
    for (Exprs::const_iterator i (properties.begin ());
	 i != properties.end (); i++) {
      result += (**i).toString() + "\n";
    }

    return result;
  }

  void Model::addVariable (Variable *vptr) {
    globals.insert(std::pair<std::string,boost::shared_ptr<Variable> > (vptr->identifier,boost::shared_ptr<Variable>(vptr)));
  }

  void compose(Substitution &result, const Substitution &s1, const Substitution &s2) {
    for (Substitution::const_iterator it = s2.begin(); it != s2.end(); it++) {
      const std::string& key(it->first);
      Substitution::const_iterator fit(s1.find(key));
      std::pair<std::string, boost::shared_ptr<Expr> > p( key, fit!=s1.end() ? (fit->second) : it->second);
      result.insert(p);
    }
  }
}
