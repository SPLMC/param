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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the program this parser part of.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2007-2010 Bjoern Wachter (Bjoern.Wachter@comlab.ox.ac.uk)
 * Copyright 2009-2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#include <limits>
#include <string>
#include <stdio.h>
#include "AST.h"
#include "Util.h"
#include "Node.h"
#include "Property.h"
#include "Model.h"
#include "PRISMParser.h"
extern void PRISMparse();
extern FILE *PRISMin;
#include "Expr.h"

std::string file_name;

namespace prismparser {
  extern prismparser_ast::RewNameToNr rewNameToNr;
  extern int line_number;
  using namespace std;

  prismparser_ast::Model PRISMParser::astModel;

  prismparser_ast::Substitution constants;

  void translateModel(prismparser_ast::Model& am, Model& m);

  PRISMParser::PRISMParser() {
  }

  PRISMParser::~PRISMParser() {
  }

  void PRISMParser::run(const string& file, prismparser::Model &model) {
    file_name = file;
    line_number = 1;
    if (!(PRISMin = fopen(file.c_str(), "r"))) {
      throw prismparser_error("File " + file + " not found\n");
    }

    PRISMparse();
    
    translateModel(PRISMParser::astModel, model);
    PRISMParser::astModel.clear(); // avoid double insertions into ::model
    fclose(PRISMin);
  }

  Expr translateBaseExpr(boost::shared_ptr<prismparser_ast::Expr> ae) {
    return Expr(ae);
  }

  /**
   * Transforms AST bound expression to Bound of concrete syntax tree.
   *
   * @param bound_expr abstract expression to be transformed
   * @param minimize true iff result should be for minimizing probabilities
   */
  Bound boundFromAST(const prismparser_ast::Expr& boundExpr, bool minimize,
		     bool minOrMax) {
    Bound::Kind k;
    Expr bound;
    if (2 == boundExpr.arity()) {
      bound = translateBaseExpr(boundExpr.children[1]);
    }

    switch (boundExpr.getKind()) {
    case prismparser_ast::Gt:
      k = Bound::GR;  // >  bound ... greater
      break;
    case prismparser_ast::Ge:
      k = Bound::GEQ; // >= bound ... greater or equal
      break;
    case prismparser_ast::Lt:
      k = Bound::LE;  // <  bound ... strictly less
      break;
    case prismparser_ast::Le:
      k = Bound::LEQ; // <= bound ... less or equal
      break;
    case prismparser_ast::Eq:
      k = Bound::EQ; // =  bound ... equal
      break;
    default:
      k = Bound::DK;   // = ? ... value to be computed
      break;
    }

    Bound b(k, bound, minimize, minOrMax);

    return b;
  }

  Filter filterFromAST(const prismparser_ast::Expr &filterExpr) {
    assert(prismparser_ast::Filter == filterExpr.getKind());
    assert(2 == filterExpr.arity());
    Expr filter(translateBaseExpr(filterExpr.children[0]));
    assert(prismparser_ast::Int == filterExpr.children[1]->getKind());
    int direction(filterExpr.children[1]->getInt());
    bool minimize(-1 == direction);
    bool minOrMax(0 != direction);
    Filter f(filter, minimize, minOrMax);

    return f;
  }

  static ExprHashMap<Expr> replaceInit;

  Property* translateProperty(boost::shared_ptr<prismparser_ast::Expr> ae) {
    Property* result(0);
    assert(ae.get());
    prismparser_ast::Expr e(*ae.get());

    switch (e.getKind()) {
    case prismparser_ast::Next:
      if (e.arity() == 1) {
	result = new Next(translateProperty(e.children[0]));
      } else if(e.arity() == 3) {
	double a(-1), b (-1);
	Time::Kind k;
	switch (e.children[1]->getKind()) {
	case prismparser_ast::Null:
	  break;
	case prismparser_ast::Int:
	  a = e.children[1]->getInt();
	  break;
	case prismparser_ast::Double:
	  a = e.children[1]->getDoubleVal();
	  break;
	default:
	  break;
	}

	switch(e.children[2]->getKind()) {
	case prismparser_ast::Null:
	  break;
	case prismparser_ast::Int:
	  b = e.children[2]->getInt();
	  break;
	case prismparser_ast::Double:
	  b = e.children[2]->getDoubleVal();
	  break;
	default:
	  break;
	}

	if ((a != -1.0) && (b != -1.0)) {
	  k = Time::INTERVAL;
	} else if (b != -1.0) {
	  a = 0;
	  k = Time::LE;
	} else if (a != -1.0) {
	  k = Time::GE;
	  b = numeric_limits<double>::infinity();
	} else {
	  throw prismparser_error("Bad time bound");
	}
	Time t(k,a,b);
	result = new Next(t, translateProperty(e.children[0]));
      }

      break;
    case prismparser_ast::Until:
      {
	Property* p1(translateProperty(e.children[0]));
	Property* p2(translateProperty(e.children[1]));

	if (e.arity() == 2) {
	  result = new Until(p1, p2);
	} else if (e.arity() == 4) {
	  double a(-1.0), b (-1.0);
	  Time::Kind k;
	  switch (e.children[2]->getKind()) {
	  case prismparser_ast::Null:
	    break;
	  case prismparser_ast::Int:
	    a = e.children[2]->getInt();
	    break;
	  case prismparser_ast::Double:
	    a = e.children[2]->getDoubleVal();
	    break;
	  default:
	    break;
	  }

	  switch(e.children[3]->getKind()) {
	  case prismparser_ast::Null:
	    break;
	  case prismparser_ast::Int:
	    b = e.children[3]->getInt();
	    break;
	  case prismparser_ast::Double:
	    b = e.children[3]->getDoubleVal();
	    break;
	  default:
	    break;
	  }

	  if ((a != -1.0) && (b != -1.0)) {
	    k = Time::INTERVAL;
	  } else if (b != -1.0) {
	    a = 0;
	    k = Time::LE;
	  } else if (a != -1.0) {
	    b = numeric_limits<double>::infinity();
	    k = Time::GE;
	  } else {
	    throw prismparser_error("Bad time bound");
	  }
	  Time t(k,a,b);
	  result = new Until(t,p1,p2);
	}
      }
      break;
    case prismparser_ast::P:
    case prismparser_ast::Steady:
    case prismparser_ast::ReachabilityReward:
    case prismparser_ast::CumulativeReward:
    case prismparser_ast::InstantaneousReward:
    case prismparser_ast::SteadyStateReward:
      {
      const int direction((e.children[0].get())->getInt());
      const prismparser_ast::Expr& bound_expr(*e.children[1].get());
      const boost::shared_ptr<prismparser_ast::Expr> &inner_expr(e.children[2]);
      const prismparser_ast::Expr& rewst_expr(*e.children[3].get());
      const prismparser_ast::Expr& filter(*e.children[4].get());
      unsigned rewNr;
      if (prismparser_ast::Int == rewst_expr.getKind()) {
	rewNr = rewst_expr.getInt();
	rewNr--;
      } else {
	const string rewName(rewst_expr.getIdentifier());
	if (0 == rewNameToNr.count(rewName)) {
	  throw prismparser_error("Reward structure \"" + rewName + "\" not specified.");
	}
	rewNr = rewNameToNr[rewName];
      }

      bool min(false);
      bool minOrMax(false);
      if (0 == direction) {
	if ((prismparser_ast::Gt == bound_expr.getKind())
	    || (prismparser_ast::Ge == bound_expr.getKind())) {
	  min = true;
	  minOrMax = false;
	} else if ((prismparser_ast::Lt == bound_expr.getKind())
		   || (prismparser_ast::Le == bound_expr.getKind())) {
	  min = false;
	  minOrMax = false;
	} else {
	  min = false;
	  minOrMax = false;
	}
      } else if (-1 == direction) {
	min = true;
	minOrMax = true;
      } else if (1 == direction) {
	min = false;
	minOrMax = true;
      }

      Bound b(boundFromAST(bound_expr, min, minOrMax));
      Filter f(filterFromAST(filter));
      if (prismparser_ast::P == e.getKind()) {
	result = new Quant(b, translateProperty(inner_expr), f);
      } else if (prismparser_ast::Steady == e.getKind()) {
	result = new SteadyState(b, translateProperty(inner_expr), f);
      } else if (prismparser_ast::ReachabilityReward == e.getKind()) {
	result = new ReachabilityReward(b, translateProperty(inner_expr), rewNr, f);
      } else if (prismparser_ast::CumulativeReward == e.getKind()) {
	result = new CumulativeReward(b, inner_expr->getDoubleVal(), rewNr, f);
      } else if (prismparser_ast::SteadyStateReward) {
	result = new SteadyStateReward(b, rewNr, f);
      } else if (prismparser_ast::InstantaneousReward == e.getKind()) {
	result = new InstantaneousReward(b, inner_expr->getDoubleVal(), rewNr, f);
      }
    }
      break;
    case prismparser_ast::Not:
      {
	Property *inner = translateProperty(e.children[0]);
	if (expr == inner->kind) {
	  PropExpr *innerProp = (PropExpr *) inner;
	  result = new PropExpr(Expr::notExpr(innerProp->getExpr()));
	  delete inner;
	} else {
	  result = new PropNeg(inner);
	}
	break;
      }
    case prismparser_ast::And:
      {
	Property *innerA = translateProperty(e.children[0]);
	Property *innerB = translateProperty(e.children[1]);
	if ((expr == innerA->kind) && (expr == innerB->kind)) {
	  PropExpr *innerAProp = (PropExpr *) innerA;
	  PropExpr *innerBProp = (PropExpr *) innerB;
	  result = new PropExpr(Expr::andExpr(innerAProp->getExpr(), innerBProp->getExpr()));
	  delete innerA;
	  delete innerB;
	} else {
	  result = new PropBinary(PropBinary::AND,innerA, innerB);
	}
	break;
      }
    case prismparser_ast::Or:
      {
	Property *innerA = translateProperty(e.children[0]);
	Property *innerB = translateProperty(e.children[1]);
	if ((expr == innerA->kind) && (expr == innerB->kind)) {
	  PropExpr *innerAProp = (PropExpr *) innerA;
	  PropExpr *innerBProp = (PropExpr *) innerB;
	  result = new PropExpr(Expr::orExpr(innerAProp->getExpr(), innerBProp->getExpr()));
	} else {
	  result = new PropBinary(PropBinary::OR,innerA, innerB);
	}
	break;
      }
    case prismparser_ast::Impl:
      {
	Property *innerA = translateProperty(e.children[0]);
	Property *innerB = translateProperty(e.children[1]);
	if ((expr == innerA->kind) && (expr == innerB->kind)) {
	  PropExpr *innerAProp = (PropExpr *) innerA;
	  PropExpr *innerBProp = (PropExpr *) innerB;
	  result = new PropExpr(Expr::orExpr(innerAProp->getExpr(), innerBProp->getExpr()));
	} else {
	  result = new PropBinary(PropBinary::IMPL,innerA, innerB);
	}
	break;
      }
    default:
      {
	Expr nested_expr(translateBaseExpr(ae));
	nested_expr = nested_expr.substExpr(replaceInit);
	result = new PropExpr(nested_expr);
	break;
      }
    }
    return result;
  }

  Expr translateExpr(boost::shared_ptr<prismparser_ast::Expr> ae) {
    Property *prop = translateProperty(ae);
    assert(prop->kind == expr);
    Expr result(((PropExpr *)prop)->getExpr());
    delete prop;

    return result;
  }

  Alternative* translateAlternative(boost::shared_ptr<prismparser_ast::Alternative> aa) {
    const prismparser_ast::Alternative& alternative(*aa.get());
    const prismparser_ast::Update& update (alternative.update);
    Alternative* result(new Alternative());

    for (prismparser_ast::Assignment::const_iterator i = update.assignment.begin();
	 i != update.assignment.end(); i++) {
      Expr lhs(translateExpr(i->first));
      Expr rhs(translateExpr(i->second));
      result->Assign(lhs,rhs);
    }
    Expr weight(translateExpr(alternative.weight));
    result->setWeight(weight);
    return result;
  }

  Command* translateCommand(boost::shared_ptr<prismparser_ast::Command> ac) {
    string label;
    boost::shared_ptr < prismparser_ast::Expr > guard;
    prismparser_ast::Alternatives alternatives;

    const prismparser_ast::Command& command (*ac.get());
    Command* result(new Command());

    for (prismparser_ast::Alternatives::const_iterator i(command.alternatives.begin());
    	 i != command.alternatives.end(); ++i) {
      try {
	result->addAlternative(translateAlternative(*i));
      } catch(prismparser_error& p) {
	throw prismparser_error("Alternative of command "+(*i)->toString() + "\n"
				+ " Reason: " +  p.toString() + "\n");
      }
    }
    
    try {
      result->setGuard(translateExpr(command.guard));
    } catch(prismparser_error& p) {
      throw prismparser_error("Guard "+command.guard->toString() + "\n"
			      + " Reason: " +  p.toString() + "\n");
    }

    result->setAction(command.label);
    return result;
  }

  Module* translateModule(boost::shared_ptr<prismparser_ast::Module> am) {
    const prismparser_ast::Module& module(*am.get());
    Module* result(new Module(module.name));
    for (prismparser_ast::Commands::const_iterator i (module.commands.begin ());
    	 i != module.commands.end (); ++i) {
      result->addCommand(translateCommand(*i)) ;
    }
    
    return result;
  }

  void translateVariables(const prismparser_ast::Variables& vars, Model& model) {
    for (prismparser_ast::Variables::const_iterator i(vars.begin ());
	 i != vars.end(); ++i) {
      const prismparser_ast::Variable& var(*i->second.get());

      Expr var_expr;

      switch (var.type->kind) {
      case prismparser_ast::Type::Boolean: {
	var_expr = Expr::varExpr(i->first);
	model.addVariable(var_expr, BoolVarType);
	Expr::setVarType(var_expr, BoolVarType);
	model.setDefaultInitialValue(var_expr, var.init.get() ? translateExpr(var.init) : Expr::falseExpr());
      }
	break;
      case prismparser_ast::Type::Integer: {
	var_expr = Expr::varExpr(i->first);
	model.addVariable(var_expr, IntVarType);
	Expr::setVarType(var_expr, IntVarType);
	model.setDefaultInitialValue(var_expr, var.init.get() ? translateExpr(var.init) : Expr::ratExpr(0ll,1ll));
      }
	break;
      case prismparser_ast::Type::Double: {
	var_expr = Expr::varExpr(i->first);
	model.addVariable(var_expr, RealVarType);
	Expr::setVarType(var_expr, RealVarType);
	model.setDefaultInitialValue(var_expr, var.init.get() ? translateExpr(var.init) : Expr::ratExpr(0ll,1ll));
      }
	break;
      case prismparser_ast::Type::Range: {
	Expr upper, lower;
	try {
	  lower = Expr::simplify(translateExpr(var.type->range_data.lower));
	  upper = Expr::simplify(translateExpr(var.type->range_data.upper));
	} catch (prismparser_error &p) {
	  throw prismparser_error("Range of variable "+ var.toString() + "\n"
				  + " Reason: " +  p.toString() + "\n");
	}
	var_expr = (Expr::varExpr(i->first));
	assert(lower.isRational());
	assert(upper.isRational());
	model.addVariable(var_expr, RangeVarType, lower.getNumerator(), lower.getDenominator(), 
			  upper.getNumerator(), upper.getDenominator());
	Expr::setVarType(var_expr, RangeVarType);
	Expr::setVarBounds(var_expr, lower.getNumerator(), lower.getDenominator(),
			   upper.getNumerator(), upper.getDenominator());
	model.setDefaultInitialValue(var_expr, var.init.get() ? translateExpr(var.init) : lower);
      }
	break;
      }

      if (var.is_parameter) {
	model.setAsParameter(var_expr);
      }
    }
  }

  void translateModel(prismparser_ast::Model& am, Model& model) {
    switch(am.model_type) {
    case prismparser_ast::DTMC:
      model.setModelType(DTMC);
      break;
    case prismparser_ast::MDP:
      model.setModelType(MDP);
      break;
    case prismparser_ast::CTMC:
      model.setModelType(CTMC);
      break;
    case prismparser_ast::CTMDP:
      model.setModelType(CTMDP);
      break;
    case prismparser_ast::Unspecified:
      model.setModelType(MDP);
      break;
    }

    /* 1) Variable table
     *
     * build the variable table by traversing the model
     * collecting variables from each module */

    /* global variables */
    translateVariables(am.globals,model);

    /* local module variables */
    for (prismparser_ast::Modules::const_iterator i(am.modules.begin ());
	 i != am.modules.end (); i++) {
	translateVariables(i->second->locals,model);
      }

    /* 2) translate modules and add them to the model */
    for (prismparser_ast::Modules::const_iterator i(am.modules.begin ()); i != am.modules.end (); i++) {
	model.addModule(translateModule(i->second));
      }

    /* 3) translate the rest */

    // boost::shared_ptr < Expr > initial
    try {
      if (am.initial.get()) {
	Expr e(translateExpr(am.initial));
	model.setInitial(e);
      }
      if (model.getInitial().isNull()) {
	model.computeDefaultInitialValue();
      }
    } catch(prismparser_error& p) {
      throw prismparser_error("Initial condition " + am.initial->toString() + "\n"
			      + " Reason: " +  p.toString() + "\n");
    }

    // Exprs invariants
    for (prismparser_ast::Exprs::const_iterator i=am.invariants.begin();i!=am.invariants.end();++i) {
      Expr e(translateExpr(*i));
      model.addInvariant(e);
    }

    // Actions actions;
    for (prismparser_ast::Actions::const_iterator i=am.actions.begin();i!=am.actions.end();++i) {
      model.addAction(*i);
    }

    // Exprs predicates;
    for (prismparser_ast::Exprs::const_iterator i=am.predicates.begin(); i != am.predicates.end(); i++) {
      Expr e(translateExpr(*i));
      model.addPredicate(e);
    }

    // Exprs invariants
    replaceInit.clear();
    replaceInit.insert(make_pair(Expr::varExpr("init"), model.getInitial()));
    for (prismparser_ast::Exprs::const_iterator i=am.properties.begin(); i != am.properties.end(); i++) {
      Property* p(translateProperty(*i));
      model.addProperty(p);
    }

    // StateRewards state_rewards;
    for (prismparser_ast::StateRewards::const_iterator i=am.state_rewards.begin();i!=am.state_rewards.end();++i) {
      const unsigned structure(i->get<0>());
      Expr guard(translateExpr(i->get<1>()));
      Expr reward(translateExpr(i->get<2>()));
      model.addStateReward(structure, guard, reward);
    }

    // TransitionRewards transition_rewards;
    for (prismparser_ast::TransitionRewards::const_iterator i = am.transition_rewards.begin();
	 i != am.transition_rewards.end(); i++) {
      const unsigned structure(i->get<0>());
      Action action(i->get<1>());
      Expr guard(translateExpr(i->get<2>()));
      Expr reward(translateExpr(i->get<3>()));

      model.addTransReward(structure, action, guard, reward);
    }
  }
}

