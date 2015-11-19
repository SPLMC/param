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

#ifndef __AST_H
#define __AST_H

#include <boost/tuple/tuple.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <tr1/unordered_map>
#include <tr1/unordered_set>
#include <vector>
#include <map>
#include <set>

namespace prismparser_ast {
  enum Kind {
    Null,

    Var,
    Bool,
    Int,
    Rat,
    Double,
    
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
    Max,
    
    Next,
    Until,
    P,
    Steady,
    ReachabilityReward,
    CumulativeReward,
    InstantaneousReward,
    SteadyStateReward,
    Filter
  };

  enum ModelType {
    Unspecified, DTMC, MDP, CTMC, CTMDP
  };

  class Expr;

  typedef std::tr1::unordered_map < std::string, boost::shared_ptr<Expr> > Substitution;
  typedef std::tr1::unordered_map < std::string, std::string > ActionSubstitution;

  typedef std::vector < boost::shared_ptr < Expr > >Exprs;


  extern Substitution dummy_subst;
  extern ActionSubstitution dummy_action_subst;

  class Expr {
  public:
    Exprs children;

    Expr();
    Expr(Kind, const char *);
    Expr(bool b);
    Expr(int i);
    Expr(Kind);
    Expr(Kind, Expr *);
    Expr(Kind, Expr *, Expr *);
    Expr(Kind, Expr *, Expr *, Expr *);
    Expr(Kind, Expr *, Expr *, Expr *, Expr *);
    Expr(Kind, Expr *, Expr *, Expr *, Expr *, Expr *);
    Expr(const Expr *, const Substitution & = dummy_subst);

    Expr &operator=(const Expr & e);

    Kind getKind() const;
    bool getBool() const;
    int  getInt() const;
    double getDoubleVal() const;
    const std::string &getDoubleAsString() const;
    const std::string &getIdentifier() const;
    bool isNull() const { return kind == Null; }
    bool isVariable() const { return kind==Var; }
    bool isDouble() const {return kind == Double;}
    bool isInt() const { return kind == Int;}
    bool isBool() const { return kind == Bool;}
    std::string toString(const Kind& kind) const;
    std::string toString() const;
    unsigned arity() const;
  private:
    void addChild(Expr *);

    Kind kind;
    std::string identifier;
    bool bool_value;
    std::string double_value;
    int int_value;
  };


  typedef std::vector < std::pair<boost::shared_ptr < Expr >,boost::shared_ptr < Expr > > > Assignment;

  class Update {
  public:
    Assignment assignment;
    void Assign(Expr* lhs, Expr* rhs) {
      assignment.push_back(std::pair<boost::shared_ptr < Expr >,boost::shared_ptr < Expr > > (
											      boost::shared_ptr < Expr >(lhs),boost::shared_ptr < Expr >(rhs)));
    }
    Update (const Update &, const Substitution &);
    Update () {}
    std::string toString() const;
  };


  class Alternative {
  public:
    Update update;
    boost::shared_ptr < Expr > weight;
    
    Alternative() {}

    void setWeight (Expr * e)
    {
      weight = boost::shared_ptr < Expr > (e);
    }
    Alternative (const Alternative &, const Substitution &);

    std::string toString() const;
  };


  typedef std::vector<boost::shared_ptr<Alternative> > Alternatives;

  class Command {
  public:
    std::string label;
    boost::shared_ptr < Expr > guard;
    Alternatives alternatives;

    Command(const std::string& s) : label(s) {}
    Command(const Command &, const Substitution &, const ActionSubstitution& as);

    void setGuard(Expr * e) {
      guard = boost::shared_ptr < Expr > (e);
    }
    void addAlternative(Alternative * alt) {
      alternatives.push_back (boost::shared_ptr < Alternative > (alt));
    }

    std::string toString() const;
  };

  typedef std::vector < boost::shared_ptr < Command > >Commands;

  class Type {
  public:
    enum Kind {
      Boolean,
      Integer,
      Double,
      Range
    } kind;

    struct RangeData {
      RangeData() {
      }
      RangeData (Expr * e1, Expr * e2):lower (e1), upper (e2) {
      }
      boost::shared_ptr<Expr> lower;
      boost::shared_ptr<Expr> upper;
    } range_data;

    Type(bool) : kind (Boolean) {
    }
    Type(int) : kind (Integer) {
    }
    Type(double) : kind (Double) {
    }
    Type (Expr *e1, Expr *e2) : kind(Range), range_data(e1, e2) {
    }
    Type( const Type* t, const Substitution& s);
    Type& operator=(const Type& t);

    bool isRange() const { return kind==Range; }

    std::string toString() const;
  };

  class Variable {
  public:
    boost::shared_ptr <Type> type;
    std::string identifier;
    boost::shared_ptr <Expr> init;

    bool is_parameter;

    Variable ( const Variable*, const Substitution& s = dummy_subst);

    Variable ( const std::string & s, Type * ty, Expr* e = 0 ):
    type (ty), identifier (s), init(e), is_parameter(false) {
    }

    std::string toString() const;
  };

  typedef std::map < std::string, boost::shared_ptr < Variable > >Variables;

  class Module {
  public:

    std::string name;

    Module(const std::string & s) : name (s) {
    }

    Module(const std::string&, const Module*, const Substitution & = dummy_subst, const ActionSubstitution& = dummy_action_subst);

    Commands commands;
    Variables locals;

    void addVariable (Variable* vptr) {
      locals[vptr->identifier] = boost::shared_ptr<Variable>(vptr);
    }
    void addCommand (Command * c) {
      commands.push_back (boost::shared_ptr < Command > (c));
    }

    std::string toString() const;
  };

  typedef std::map<std::string,boost::shared_ptr<Module> > Modules;
  typedef std::set<std::string> Actions;

  typedef boost::tuple<unsigned, boost::shared_ptr<Expr>, boost::shared_ptr<Expr> > StateReward;
  typedef std::vector<StateReward> StateRewards;
  typedef boost::tuple<unsigned, std::string, boost::shared_ptr<Expr>, boost::shared_ptr<Expr> > TransitionReward;
  typedef std::vector<TransitionReward> TransitionRewards;
  typedef std::map<std::string, unsigned> RewNameToNr;

  class Model {
  public:
    Model() {}
    Model(const Model&, const Substitution&);

    ModelType model_type;
    Variables globals;


    boost::shared_ptr < Expr > initial;

    void addInvariant( Expr* e) {
      invariants.push_back( boost::shared_ptr<Expr> (e));
    }

    void addVariable (Variable* vptr);

    void addProperty (Expr* e)
    {
      properties.push_back( boost::shared_ptr<Expr> (e));
    }

    void addPredicate (Expr* e)
    {
      predicates.push_back( boost::shared_ptr<Expr> (e));
    }

    void setInitial (Expr * e)
    {
      initial = boost::shared_ptr < Expr > (e);
    }

    Modules modules;
    void addModule (boost::shared_ptr < Module > ptr) {
      assert(ptr.get());
      modules.insert( std::pair<std::string,boost::shared_ptr<Module> >(ptr->name,ptr)) ;
    }

    Module* getModule(const std::string name) const {
      Modules::const_iterator i(modules.find(name));
      if(i!=modules.end()) return (i->second).get();
      else return 0;
    }

    std::string toString() const;

    Actions actions;

    Exprs predicates;
    Exprs invariants;
    Exprs properties;

    StateRewards state_rewards;
    TransitionRewards transition_rewards;

    void clear() {
      modules.clear();
      actions.clear();
      predicates.clear();
      invariants.clear();
      properties.clear();
      state_rewards.clear();
      transition_rewards.clear();
    }

  };

  /** \brief composition s_1 \circle \s_2 */
  void compose(Substitution& result, const Substitution& s1, const Substitution& s2);
}

#endif
