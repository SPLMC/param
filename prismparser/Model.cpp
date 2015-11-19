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

#include <boost/foreach.hpp>
#include "Util.h"
#include "Node.h"
#include "Property.h"
#include "Model.h"
#include "Util.h"
#include "Expr.h"

#define WITH_PARAM

namespace prismparser {
  using namespace std;

  Model::Model() : Node (), model_type (MDP), maxNrOfAlt (0) {
    transRewards.resize(1);
    stateRewards.resize(1);
  }

  Model::Model (const Model & m) {
    *this = m;
  }

  /*********************************************************************/
  // Create String from Model.
  /*********************************************************************/
  string Model::toString() const {
    string s;

    // go through the variables

    s += "module M\n";
    for (vector <string>::const_iterator i = var_names.begin();
	 i != var_names.end(); i++) {
	const std::string & v = *i;
	std::tr1::unordered_map<std::string,Expr>::const_iterator vit =
	  variables.find (*i);
	assert(vit != variables.end());
	s += v + " : " + Expr::varTypeToString(Expr::getVarType(vit->second)) + ";\n";
      }
    long long i = 0;
    BOOST_FOREACH (boost::shared_ptr<Command> c, guarded_transitions) {
      assert(c);
      s += "Nr. " + intToString(i) + " : " + c->toString();
      i++;
    }
    s += "endmodule\n";
    s += "init\n" + init.toString() + "\nendinit" + "\n";
    BOOST_FOREACH (boost::shared_ptr<Property> p, properties) {
      assert(p);
      s += p->toString() + "\n";
    }
    return s;
  }

  /*********************************************************************/
  //get a module by name.
  /*********************************************************************/
  Module *Model::getModule(const std::string & module_name) const {
    // go through the modules to find suitable one
    for (Modules::const_iterator i = modules.begin(); i != modules.end();
	 ++i)
      if ((*i)->getName() == module_name) {
	return (*i).get();
      }
    return NULL;
  }

  /*********************************************************************/
  //Instantiate a given module
  /*********************************************************************/
  Module::Module(const std::string & __name,
		 const Module & existing,
		 const ExprHashMap <Expr> &repl,
		 const HashMap <std::string,std::string> &al) : name (__name) {
    // recursively descend into synchronized commands and do replacements
    for (SynchronizedCommands::const_iterator i =
	   existing.sync_guarded_transitions.begin ();
	 i != existing.sync_guarded_transitions.end (); i++) {
	std::string action = (*i).first;
	HashMap < std::string, std::string >::const_iterator ait =
	  al.find (action);
	if (ait != al.end()) {
	  action = (*ait).second;
	}
	const Commands & gts = (*i).second;
	for (Commands::const_iterator i = gts.begin (); i != gts.end (); ++i) {
	  Command *gt = new Command(**i, repl);
	  gt->setAction(action);
	  addCommand(gt);
	}
      }

    // recursively descend into commands and do replacements
    for (Commands::const_iterator i = existing.guarded_transitions.begin();
	 i != existing.guarded_transitions.end (); ++i) {
      addCommand(new Command (**i, repl));
    }

    for (HashMap < std::string, Expr >::const_iterator vit =
	   existing.local_vars.begin (); vit != existing.local_vars.end();
	 ++vit) {
	Expr expr = (*vit).second.substExpr(repl);
	local_vars.insert(std::pair<std::string,Expr>(expr.getName(), expr));
      }
  }

  /*********************************************************************/
  //insert a variable.
  /*********************************************************************/
  void Model::addVariable(const Expr &var, VarType varType) {
    assert(varType != RangeVarType);
    const string id(var.toString());

    if (variables.find(id) != variables.end()) {
      return;
    }
    var_names.push_back(id);
    var_types.insert(pair<Expr,VarType>(var,varType));
    variables.insert(pair<string,Expr>(id, var));
  }

  /*********************************************************************/
  //insert a variable.
  /*********************************************************************/
  void Model::addVariable(const Expr &var,
			  VarType varType,
			  const int lowerNum,
			  const int lowerDen,
			  const int upperNum,
			  const int upperDen) {
    assert(varType == RangeVarType);
    const string id(var.toString());

    if (variables.find(id) != variables.end()) {
      return;
    }
    var_names.push_back(id);
    var_types.insert(pair<Expr,VarType>(var,varType));
    variables.insert(pair<string,Expr>(id, var));
    variable_bounds[var] = make_pair(make_pair(lowerNum, lowerDen),
				     make_pair(upperNum, upperDen));
  }

  void Model::computeDefaultInitialValue() {
    init = Expr::trueExpr();
    std::vector<Expr> init_vec;
    for (ExprHashMap<Expr>::iterator it =
	   default_initial_value.begin(); it != default_initial_value.end();
	 ++it) {
      if (!isParameterVariable(it->first)) {
	if ((it->second).isBoolConst()) {
	  if ((it->second).isTrue()) {
	    init_vec.push_back(it->first);
	  } else {
	    init_vec.push_back(Expr::notExpr(it->first));
	  }
	} else {
	  init_vec.push_back(Expr::eqExpr(it->first, it->second));
	}
      }
    }
    init = Expr::andExpr(init_vec);
  }

  /*********************************************************************/
  //add an action :-)
  /*********************************************************************/
  void Model::addAction (const Action & a) {
    assert (a != "");
    actions.insert (a);
  }

  /*********************************************************************/
  // Model destructor
  /*********************************************************************/
  Model::~Model () {
  }

  void Model::addModule (Module *module) {
    assert (module);
    boost::shared_ptr < Module > mod_ptr (module);
    modules.push_back (mod_ptr);
  }

  void Model::setInitial(const Expr & __init) {
    init = __init;
  }

  /*! \brief add another predicate */
  void Model::addPredicate(const Expr & __expr) {
    user_predicates.push_back (__expr);
  }
  
  /*! \brief add another property */
  void Model::addProperty(Property *p) {
    assert(p);
    boost::shared_ptr < Property > ptr (p);
    properties.push_back (ptr);
  }

  /*! \brief add invariant */
  void Model::addInvariant(const Expr & __expr) {
    invariants.push_back(Expr::simplify(__expr));
  }
  
  /*! \brief add guarded state reward */
  void Model::addStateReward
  (const unsigned structure, const Expr & __guard, const Expr & __reward) {
    if (stateRewards.size() < structure + 1) {
      stateRewards.resize(structure + 1);
    }
    stateRewards[structure].push_back(std::pair<Expr,Expr> (__guard, __reward));
  };

  /*! get guarded state rewards */
  const std::vector < std::pair < Expr, Expr > >&Model::getStateRewards() const {
    return stateRewards[0];
  };

  /*! get guarded state rewards */
  const std::vector < std::pair < Expr, Expr > >&Model::getStateRewards(const unsigned structure) const {
    assert(structure < stateRewards.size());

    return stateRewards[structure];
  };

  void Model::clearStateRewards() {
    stateRewards.clear();
  }

  void Model::clearTransRewards() {
    transRewards.clear();
  }

  /*! \brief add guarded transition reward */
  void Model::addTransReward
  (const unsigned structure, const Expr & __guard, const Expr & __reward) {
    if (transRewards.size() < structure + 1) {
      transRewards.resize(structure + 1);
    }
    transRewards[structure].
      push_back (std::make_pair (std::make_pair ("", __guard), __reward));
  };

  /*! \brief add guarded transition reward */
  void Model::addTransReward
  (const unsigned structure, const Action & __action, const Expr & __guard,
   const Expr & __reward) {
    if (transRewards.size() < structure + 1) {
      transRewards.resize(structure + 1);
    }
    transRewards[structure].
      push_back (std::make_pair (std::make_pair (__action, __guard), __reward));
  };


  /*! get guarded transition rewards */
  std::vector < std::pair < std::pair < Action, Expr >,
			    Expr > >&Model::getTransRewards() {
    return transRewards[0];
  };

  /*! get guarded transition rewards */
  std::vector < std::pair < std::pair < Action, Expr >,
			    Expr > >&Model::getTransRewards(const unsigned structure) {
    if (transRewards.size() < structure + 1) {
      transRewards.resize(structure + 1);
    }

    return transRewards[structure];
  };

  /*********************************************************************/
  //merge all modules into the model
  //   1. collect the guarded transitions
  //   2. multiply out synchronized guarded transitions
  /*********************************************************************/
  /*! \brief Flatten multiplies out all synchronized guarded transitions
    \pre The model is complete, i.e. contains all desired modules
    \post The guarded transitions of the flattened model are in field Model::guarded_transitions
  */
  void Model::flatten() {
    maxNrOfAlt = 0;

    //   1. collect the guarded transitions
    for (Modules::const_iterator mit = modules.begin(); mit != modules.end(); ++mit) {
      const Module & module = **mit;
      const Commands & gts = module.guarded_transitions;
      for (Commands::const_iterator git = gts.begin(); git != gts.end(); ++git) {
	boost::shared_ptr < Command> gt_ptr(*git);
	guarded_transitions.push_back(gt_ptr);
	unsigned NrOfAlt = gt_ptr->getNrOfAlt();
	maxNrOfAlt = maxNrOfAlt > NrOfAlt ? maxNrOfAlt : NrOfAlt;
      }
    }
    //   2. multiply out synchronized guarded transitions
    for (Actions::const_iterator ait = actions.begin(); ait != actions.end(); ++ait) {
      Commands collection;
      const Action & action(*ait);
      for (Modules::const_iterator mit = modules.begin(); mit
	     != modules.end(); ++mit) {

	const Module & module = **mit;
	const SynchronizedCommands & sgts = module.sync_guarded_transitions;
	SynchronizedCommands::const_iterator sit = sgts.find(action);
	if (sit == sgts.end())
	  continue;

	const Commands & gts = (sit->second);
	//initialize if necessary
	if (collection.empty()) {
	  collection = gts;
	  continue;
	}

	Commands new_collection;
	//multiply all guarded transitions
	for (Commands::iterator git1 = collection.begin(); git1
	       != collection.end(); ++git1) {
	  for (Commands::const_iterator git2 = gts.begin(); git2
		 != gts.end(); ++git2) {
	    assert (*git1);
	    assert (*git2);
	    Command *gt = (**git1) * (**git2);
	    //if the guards are disjoint there may be no such guarded transition
	    if (gt) {
	      boost::shared_ptr < Command> gt_ptr(gt);
	      new_collection.push_back(gt_ptr);
	    }
	  }

	}
	collection = new_collection;
      }
      //finally add the collected transitions
      BOOST_FOREACH (boost::shared_ptr < Command>& gt, collection) {
	assert (gt.get ());
	guarded_transitions.push_back(gt);
	unsigned NrOfAlt = gt->getNrOfAlt();
	maxNrOfAlt = maxNrOfAlt > NrOfAlt ? maxNrOfAlt : NrOfAlt;
      }

    }
  }

  // Notice: to be called after flatten().
  void Model::fixDeadlocks() {
    Expr sinkGuard(Expr::trueExpr());
    for (unsigned gt_nr = 0; gt_nr < guarded_transitions.size(); gt_nr++) {
      Command &gt = *((guarded_transitions)[gt_nr]);
      const Expr &guard = gt.getGuard();
      sinkGuard = Expr::andExpr(sinkGuard, Expr::notExpr(guard));
    }

    Command *sinkCommand = new Command();
    sinkCommand->setGuard(sinkGuard);
    sinkCommand->setAction("");
    Alternative *sinkAlternative = new Alternative();
    sinkAlternative->setWeight(Expr::ratExpr(1, 1));
    sinkCommand->addAlternative(sinkAlternative);
    guarded_transitions.push_back(boost::shared_ptr<Command>(sinkCommand));
  }

  Model &Model::operator= (const Model &m) {
    init = m.init;
    guarded_transitions = m.guarded_transitions;
    properties = m.properties;
    modules = m.modules;
    user_predicates = m.user_predicates;
    invariants = m.invariants;
    var_names = m.var_names;
    variables = m.variables;
    actions = m.actions;
    maxNrOfAlt = m.maxNrOfAlt;
    default_initial_value = m.default_initial_value;
    variable_bounds = m.variable_bounds;
    stateRewards = m.stateRewards;
    return *this;
  }

  /*! \brief Getter for initial condition */
  const Expr &Model::getInitial () const {
    return init;
  }

  /*! \brief setter for initial default value */
  const Expr &Model::getDefaultInitialValue (const Expr & var) const {
    ExprHashMap<Expr>::const_iterator it =
      default_initial_value.find(var);
    if (it != default_initial_value.end ()) {
      return it->second;
    } else {
      static Expr dummy;
      return dummy;
    }
  }

  /*! \brief Getter for guarded transitions */
  const Commands &Model::getCommands() const {
    return guarded_transitions;
  }
  
  /*! \brief Getter for user-provided predicates */
  const std::vector <Expr> &Model::getUserPreds() const {
    return user_predicates;
  }
  
  /*! \brief Getter for properties */
  const Properties &Model::getProperties() const {
    return properties;
  }
  
  /*! \brief Getter for invariants */
  const std::vector < Expr > &Model::getInvar() const {
    return invariants;
  }
  
  /*! \brief Maximal number of alternatives in assignment
    \note  For example [a] x>0 -> 0.5 : (x'=1) + 0.5 : (x'=2) has 2 alternatives
    while [a] x>0 -> 0.5 : (x'=1) + 0.25 : (x'=2) + 0.25: (x'=3) has three.
  */
  unsigned Model::getMaxNrOfAlt() const {
    return maxNrOfAlt;
  }
  
  bool Model::isParameterVariable(const Expr & var) const {
    return parameter_variables.find (var) != parameter_variables.end ();
  }
    
  /*! \brief get the kind of model i.e. deterministic
    vs. non-deterministic,  continuous vs. discrete-time */
  ModelType Model::getModelType() const {
    return model_type;
  }

  /*! \brief setter for initial default value */
  void Model::setDefaultInitialValue(const Expr & var,
				     const Expr & val)  {
    default_initial_value[var] = val;
  }
  
  /*! \brief get the kind of model i.e. deterministic vs. non-deterministic, continuous vs. discrete-time */
  void Model::setModelType(ModelType mt) {
    model_type = mt;
  }

  //! \brief create copy of guarded transition with expressions replaced (given by [lhs/rhs])
  Command::Command(const Command & existing,
		   const ExprHashMap<Expr> &repl) {
    action = existing.action;
    g = existing.g.substExpr(repl);

    // do replacements in alternatives
    for (Alternatives::const_iterator i = existing.alternatives.begin();
	 i != existing.alternatives.end(); ++i) {
      addAlternative(new Alternative (**i, repl));
    }
  }

  void Command::setGuard(const Expr & __g) {
    g = __g;
  }

  /*********************************************************************/
  //convert to string
  /*********************************************************************/
  std::string Module::toString() const {
    std::string s;
    for (SynchronizedCommands::const_iterator it =
	   sync_guarded_transitions.begin ();
	 it != sync_guarded_transitions.end (); ++it) {
      const Commands & gts = it->second;
      for (Commands::const_iterator it = gts.begin();
	   it != gts.end (); it++) {
	s += (*it)->toString();
      }
    }
    s += "\n";

    return s;
  }

  /*********************************************************************/
  //add a guarded transition
  /*********************************************************************/
  void Module::addCommand(Command *c) {
    boost::shared_ptr < Command > c_ptr (c);
    std::string a(c->getAction());
    if (a == "") {
      guarded_transitions.push_back (c_ptr);
    } else {
      sync_guarded_transitions[a].push_back (c_ptr);
    }
  }

  Module::Module(const std::string &__name) : name (__name) {
  }
  
  /*! \brief dummy constructor */
  Module::Module() {
  }

  const std::string &Module::getName() const {
    return name;
  }

  Alternative::Alternative(const Alternative& existing, const ExprHashMap<Expr>& repl ) {
    for(Map::const_iterator i = existing.map.begin(); i!=existing.map.end(); ++i) {
      Assign((i->first).substExpr(repl),(i->second).substExpr(repl));
    }
    p = existing.p.substExpr(repl);
  }

  void Alternative::Assign(const Expr& left, const Expr& right) {
    if (left.isNull() || right.isNull()) {
      throw prismparser_error("Assignment::Assign: assignment to Null "
			      + left.toString()+" -> " + right.toString());
    }
    if (map.count(left)>0) {
      throw prismparser_error("Assignment::Assign: Conflicting assignment"
			      + left.toString()+" -> { "+right.toString()
			      + "," + map[left].toString() + "}");
    }
    map.insert(make_pair(left, right));
    support.insert(left);
  }


  std::string Alternative::toString() const {
    std::string s;
    s+= p.toString() + " : ";

    for (Map::const_iterator i = map.begin(); i!=map.end(); ++i) {
      if (i != map.begin()) {
	s+=" & ";
      }
      s += " ( " + (i->first).toString() + "' = "
	+ (i->second).toString() + " ) ";
    }

    return s;
  }

  Alternative* Alternative::Clone() const {
    Alternative* a = new Alternative();
    a->p = p;
    a->map = map;
    a->support = support;
    return a;
  }

  void Alternative::Cleanup() {
  }

  /*********************************************************************/
  //return a pointer to an assignment
  //containing
  /*********************************************************************/
  Alternative* Alternative::operator*(const Alternative& ass) const {
    Alternative* a = Clone();
    a->p = Expr::multExpr(a->p, ass.p);
    a->map = map;
    for (Map::const_iterator i = ass.map.begin(); i != ass.map.end(); i++) {
      a->Assign(i->first,i->second);
    }

    return a;
  }

  void Alternative::CollectLhs(ExprHashSet& collection) const {
    collection = support;
  }

  Expr Alternative::operator()(const Expr& e) const {
    return e.substExpr(map);
  }

  void Alternative::setWeight(const Expr& weight) {
    p = Expr::simplify(weight);
  }

  Alternative::Alternative() {}

  const Expr &Alternative::getWeight() {
    return p;
  }

  const ExprHashMap<Expr> &Alternative::getMap() const {
    return map;
  }
  
  const ExprHashSet &Alternative::getSupport() const {
    return support;
  }  

  /*********************************************************************/
  // Constructor
  /*********************************************************************/
  Command::Command () {
  }

  void Command::factorize
  (const std::vector<int>& prob_choices,
   Alternative::Map& base,
   std::vector<Alternative::Map>& alt) const {
    alt.clear();
    alt.resize(prob_choices.size());
    BOOST_FOREACH(Expr v,support) {
      std::vector<Expr> rhs;
      ExprHashSet s;
      rhs.reserve(prob_choices.size());
      BOOST_FOREACH(int choice, prob_choices) {
	assert(choice < int(alternatives.size()));
	const Alternative& alt(*alternatives[choice]);
	rhs.push_back(alt(v));
	s.insert(rhs.back());
      }
      if(s.size()>1) {
	for(unsigned i=0; i<alt.size();++i)
	  alt[i][v] = rhs[i];
      } else if(s.size()==1) {
	base[v] = rhs[0];
      }
    }
  }

  /** \brief compute a constraint on the present states based on constraint on present and next states
   *  \pre v.size == nr of branches of command + 1
   */
  Expr Command::WP(const Expr& e, unsigned i) const {
    assert(i<alternatives.size());
    const prismparser::Alternative& a = *alternatives[i];
    return a(e);
  }

  /*********************************************************************/
  // check if the probabilities within a guarded transition sum up to 1.
  /*********************************************************************/
  void Command::checkSanity() {
    for (Alternatives::iterator it = alternatives.begin ();
	 it != alternatives.end (); ++it) {
    }
  }

  /*********************************************************************/
  // add an Assignment to the guarded transition.
  /*********************************************************************/
  void Command::addAlternative(Alternative *a) {
    assert(a);
    boost::shared_ptr < Alternative > a_ptr(a);
    alternatives.push_back(a_ptr);
    support.insert(a->getSupport().begin(), a->getSupport().end());
  }

  /*********************************************************************/
  // Write the guarded transition to a string.
  /*********************************************************************/
  std::string Command::toString() const {
    std::string s;
    s += "[" + action + "] ";
    s += g.toString () + " -> ";
    for (Alternatives::const_iterator it = alternatives.begin ();
	 it != alternatives.end (); it++) {
	assert (*it);
	if (it != alternatives.begin())
	  s += "\n + ";
	s += (*it)->toString();
      }
    s += ";\n";
    return s;
  }

  /*********************************************************************/
  //Warning: may also return 0
  /*********************************************************************/
  Command *Command::operator* (const Command & gt) const {
    //go through all alternatives, compose them and multiply their probabilities

    Command *ng = new Command();
    ng->g = Expr::andExpr(g, gt.g);

    ng->action = action;
    for (Alternatives::const_iterator it1 = alternatives.begin ();
	 it1 != alternatives.end (); ++it1) {
      for (Alternatives::const_iterator it2 = gt.alternatives.begin ();
	   it2 != gt.alternatives.end (); ++it2) {
	assert (*it1);
	assert (*it2);
	ng->addAlternative ((**it1) * (**it2));
      }
    }

    return ng;
  }

  /*********************************************************************/
  //Warning: may also return 0
  /*********************************************************************/
  Command *Command::operator+(const Command &gt) const {
    //go through all alternatives, compose them and multiply their probabilities

    Command *ng = new Command ();
    ng->g = Expr::andExpr(g, gt.g);

    ng->action = action;
    for (Alternatives::const_iterator it1 = alternatives.begin ();
	 it1 != alternatives.end (); ++it1) {
	assert (*it1);
	ng->addAlternative ((*it1).get ());
    }
    for (Alternatives::const_iterator it2 = gt.alternatives.begin ();
	 it2 != gt.alternatives.end (); ++it2) {
      assert (*it2);
      ng->addAlternative ((*it2).get ());
    }

    return ng;
  }

  /*! \brief getter for Command::guard */
  const Expr &Command::getGuard() const {
    return g;
  }

  /*! \brief getter for Command::action */
  const std::string &Command::getAction() const {
    return action;
  }

  /*! \brief getter for Alternatives */
  const Alternatives &Command::getAlternatives() const {
    return alternatives;
  }

  unsigned Command::getNrOfAlt() const {
    return alternatives.size();
  }

  const Alternative &Command::operator[](unsigned choice) const {
    assert(choice < alternatives.size());
    return *alternatives[choice];
  }

  const ExprHashSet &Command::getSupport() const {
    return support;
  }
  
  bool Model::hasBounds(const Expr & var) const {
    return variable_bounds.find (var) != variable_bounds.end();
  }

  pair<pair<int, int>, pair<int, int> > Model::getBounds(const Expr & var) {
    return variable_bounds[var];
  }

  void Model::setAsParameter(const Expr &var) {
    parameter_variables.insert(var);
  }

  unsigned Model::getNumVariables() const {
    return var_names.size();
  }

  const Expr &Model::getVariable(unsigned varNr) const {
    assert(varNr < variables.size());
    return variables.find(var_names[varNr])->second;
  }

  VarType Model::getVarType(const Expr &var) const {
    return var_types.find(var)->second;
  }
}
