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

#ifndef Model_H
#define Model_H

#include <tr1/unordered_map>
#include <tr1/unordered_set>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <map>
#include <set>

#include "Expr.h"
#include "Node.h"
#include "Property.h"
#include "VarType.h"

/*! interface to hash_map */
template<class Key, class Entry>
class HashMap : public std::tr1::unordered_map<Key,Entry> {
};

namespace prismparser {
  //  class Property;
  class Module;
  class Command;
  class Alternative;

  /*! container for set of Alternatives */
  class Alternatives : public std::vector < boost::shared_ptr < Alternative > >{};
  /*! container for set of Commands */
  class Commands : public std::vector < boost::shared_ptr < Command > >{};
  /*! container for set of synchronized Commands */
  class SynchronizedCommands : public std::map<std::string,Commands>{};
  /*! container for set of properties */
  class Properties : public std::vector < boost::shared_ptr < Property > > {};
  /*! \typedef action label */
  typedef std::string Action;
  /*! \typedef container for set of action labels */
  typedef std::set<std::string> Actions;
  /*! \typedef container for set of modules */
  typedef std::vector < boost::shared_ptr < Module > >Modules;

  typedef std::tr1::unordered_map<std::string,Expr> Variables;

  enum ModelType {
    DTMC, MDP, CTMC, CTMDP
  };

  /*! \brief a model

    The getter functions are the most interesting part for
    people who want to use the class.
    The other functions are mainly used for model construction
    which is done automatically by the parser.
  */
  struct Model : Node {
    Model();
    Model(const Model &);
    ~Model();

    /*************************/
    /* The getter functions  */
    /*************************/
    /*! \brief Get module by name
      \return pointer to module if found, NULL otherwise
    */
    Module *getModule (const std::string &) const;
    const Expr &getInitial() const;
    const Expr &getDefaultInitialValue (const Expr &) const;
    const Commands &getCommands() const;
    const std::vector<Expr> &getUserPreds() const;
    const Properties &getProperties() const;
    const std::vector<Expr> &getInvar() const;
    unsigned getMaxNrOfAlt() const;
    bool isParameterVariable(const Expr &) const;
    ModelType getModelType() const;
    Model &operator=(const Model &);
    void computeDefaultInitialValue();
    bool hasBounds(const Expr &) const;
    std::pair<std::pair<int,int>,std::pair<int,int> > getBounds(const Expr &);
    virtual std::string toString() const;
    unsigned getNumVariables() const;
    const Expr &getVariable(unsigned) const;
    VarType getVarType(const Expr &) const;

    /*****************************************************************/
    /* The following part is only interesting for model construction */
    /*****************************************************************/
    void setDefaultInitialValue(const Expr &, const Expr &);
    void setModelType (ModelType);
    void flatten();
    void fixDeadlocks();
    void addModule(Module *);
    void addAction(const Action &);
    void setInitial(const Expr &);
    void addVariable(const Expr &, VarType);
    void addVariable(const Expr &, VarType, int, int, int, int);
    void addPredicate(const Expr &);
    void addProperty(Property *);
    void addInvariant(const Expr &);
    void addStateReward(const unsigned, const Expr &, const Expr &);
    const std::vector <std::pair<Expr, Expr> > &getStateRewards() const;
    const std::vector <std::pair<Expr, Expr> > &getStateRewards(const unsigned) const;
    void clearStateRewards();
    void clearTransRewards();
    void addTransReward(const unsigned, const Expr &, const Expr &);
    void addTransReward(const unsigned, const Action &, const Expr &, const Expr &);
    std::vector < std::pair < std::pair < Action, Expr >, Expr > >
      &getTransRewards();
    std::vector < std::pair < std::pair < Action, Expr >, Expr > >
      &getTransRewards(const unsigned);
    void setAsParameter(const Expr &);

  private:
    std::tr1::unordered_map<std::string,Expr> variables;
    ExprHashMap<VarType> var_types;
    ExprHashSet parameter_variables;
    ModelType model_type;	//! DTMC, DTMDP, CTMC, CMTDP,...
    Expr init;		//! initial states
    Commands guarded_transitions;	//! guarded transitions
    Properties properties;	//! properties
    Modules modules;		//! modules
    std::vector<Expr> user_predicates;	//! user-added predicates
    std::vector<Expr> invariants;	//! invariants
    ExprHashMap<std::pair<std::pair<int,int>, std::pair<int,int> > > variable_bounds;	//! bound on a variable
    ExprHashMap<Expr> default_initial_value;	//! default initial value if no explicit initial state given
    std::vector<std::string> var_names;	//! variables of global namespace
    Actions actions;		//! set of actions
    unsigned maxNrOfAlt;	//! maximal number of alternatives in assignment
    std::vector<std::vector<std::pair<Expr, Expr> > > stateRewards;
    std::vector<std::vector<std::pair<std::pair<Action, Expr>,
      Expr> > > transRewards;
  };

  /*! \brief A model is typically the parallel composition of different modules */
  struct Module : Node {
    /*****************************************************************/
    /* The following part is only interesting for model construction */
    /*****************************************************************/
    Module(const std::string &);
    Module();
    /*! \brief Instantiate with existing module
      \par existing module in which to substitute
      \par lhs left-hand sides of substitution
      \par rhs right-hand sides of substitution
      \par al  mapping of action labels
    */
    Module(const std::string &,
	   const Module &,
	   const ExprHashMap <Expr> &,
	   const HashMap <std::string,std::string> &);

    /*! \brief add a command */
    void addCommand(Command *);

    /* Getters */
    const std::string &getName() const;

    /* Node functions */
    virtual std::string toString() const;

  private:
    friend class Model;
    SynchronizedCommands sync_guarded_transitions;	//! synchronized guarded transtions
    Commands guarded_transitions;	//! interleaved guarded transitions
    std::tr1::unordered_map<std::string,Expr> local_vars;	//! local variables
    std::string name;		//! name of the module
  };

  class Alternative : Node {
  public:
    Alternative();

    //! \brief create copy of guarded transition with substitution[lhs/rhs]
    //Assignment ( const Assignment&, const std::vector<Expr>& lhs, const std::vector<Expr>& rhs);
    //! \brief create copy of guarded transition with substitution[lhs/rhs]
    Alternative(const Alternative &, const ExprHashMap<Expr> &);

    void setWeight(const Expr &);
    const Expr &getWeight();

    /* Node functions */
    virtual std::string toString() const;
    virtual Alternative* Clone() const;
    virtual void Cleanup();
    //void Assign(BasicExpr* left, BasicExpr* right);

    void Assign(const Expr &, const Expr &);

    /*! \brief multiply with another assignment */
    Alternative* operator*(const Alternative &) const;

    // Apply an assignment to an expression
    Expr operator()(const Expr &) const;

    void CollectLhs(ExprHashSet &) const;

    typedef ExprHashMap<Expr> Map;

    const Map &getMap() const;
    const ExprHashSet& getSupport() const;
    
  private:
    Expr p; //! weight of the assignment
    Map map;
    Map wp_cache;
    ExprHashSet support;
  };

  /*! \brief A command
    \note A command consists of an action label, a guard, and different weighted alternatives
  */
  struct Command : public Node {
    const Expr &getGuard() const;
    const std::string & getAction() const;
    const Alternatives &getAlternatives() const;

    unsigned getNrOfAlt() const;
    /*****************************************************************/
    /* The following part is only interesting for model construction */
    /*****************************************************************/
    /*! \brief setter for Command::guard */
    void setGuard(const Expr & __g);

    /*! \brief setter for Command::action */
    void setAction(const std::string & __action) {
      action = __action;
    }
    /*! \brief add another Assignment */
    void addAlternative(Alternative *);
    /*! \brief Perform consistency check on guarded transition (probabilities sum up to 1) */
    void checkSanity();

    //! \brief plain constructor
    Command();
    /*! \brief create copy of guarded transition with some variables replaced
      \par lhs left-hand sides of substitution
      \par rhs right-hand sides of substitution
      \par al  mapping of action labels
    */
    Command(const Command & gt,
	    const ExprHashMap < Expr > &repl);

    /* Node functionality */
    virtual std::string toString () const;

    const Alternative& operator[](unsigned) const;

    /*! \brief compute sum of two commands
      \note ([] guard_1 -> D_1) + ([] guard_2 -> D_2) = [] guard_1 & guard_2 -> D_1 + D_2 */
    Command *operator+(const Command &) const;

    /*! \brief compute product for synchronous parallel composition
      \note ([] guard_1 -> D_1) * ([] guard_2 -> D_2) = [] guard_1 & guard_2 -> D_1 * D_2
      @see Model::Flatten()
    */
    Command *operator*(const Command &) const;

    /*!
     * get the updates in which the alternatives
     * of the command differ
     * Example:
     * for command
     * [] x > 2 -> 0.1: (x'=1) & (y'=1) + 0.8: (x'=2) & (y'=1) + 0.1: (x'=3) & (y'=1)
     * prob_choices 0, 2
     * => ( base = [y'=1] , alt = { [x -> 1], [x -> 3] } )
     */
    void factorize(const std::vector<int>& prob_choices,
		   Alternative::Map& base,
		   std::vector<Alternative::Map>& alt
		   ) const;

    const ExprHashSet& getSupport() const;


    Expr WP(const Expr& e, unsigned i) const;
  private:
    Action action;		//! action on which transition is triggered
    Expr g;		//! guard
    Alternatives alternatives;	//! weighted alternatives
    ExprHashSet support;

    friend class Model;
    friend class AbsModel;
  };
}

#endif
