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

#ifndef __PROPERTY_H
#define __PROPERTY_H

#include <boost/shared_ptr.hpp>
#include "Node.h"
#include "Expr.h"

namespace prismparser {
  enum PropertyKind {
    expr,
    binary,
    neg,
    next,
    until,
    quant,
    steadystate,
    reachability_reward,
    cumulative_reward,
    instantaneous_reward,
    steadystate_reward
  };

  class Bound {
  public:
    enum Kind { GR,  // >  bound ... greater
		GEQ, // >= bound ... greater or equal
		LE,  // <  bound ... strictly less
		LEQ, // <= bound ... less or equal
		EQ,  // =  bound ... equal
		DK   // = ?      ... value to be computed
    } kind;
    Bound(const Bound &);
    Bound(Kind, const Expr &, bool, bool);
    Bound();
    ~Bound();
    std::string toString() const;
    bool isMin() const;
    bool isMinOrMax() const;
    double getBoundAsDouble() const;
    const Expr &getBound() const;
  private:
    Expr *bound;
    bool min;
    bool minOrMax;
  };

  class Filter {
  public:
    Filter();
    Filter(const Filter &);
    Filter(const Expr &, bool, bool);
    ~Filter();
    const Expr &getExpr() const;
    bool isMin() const;
    bool isMinOrMax() const;
  private:
    Expr *expr;
    bool min;
    bool minOrMax;
  };

  class Property : public Node {
  public:
    PropertyKind kind;
    Property(PropertyKind kind);
    virtual std::string toString() const = 0;
    virtual Property* clone() const = 0;
    virtual unsigned arity() const = 0;
    virtual const Property &operator[](unsigned) const = 0;
  };

  class ReachabilityReward : public Property {
  public:
    ReachabilityReward(const Bound &, Property *, const unsigned, const Filter &);
    virtual std::string toString() const;
    virtual ReachabilityReward* clone() const;
    Property *getProp() const;
    const Bound &getBound() const;
    virtual unsigned arity() const;
    virtual const Property &operator[](unsigned) const;
    const unsigned &getRewardStruct() const;
    const Filter &getFilter() const;
  private:
    Bound bound;
    boost::shared_ptr<Property> prop;
    unsigned rewardStruct;
    Filter filter;
  };

  class CumulativeReward : public Property {
  public:
    CumulativeReward(const Bound &, const double, const unsigned, const Filter &);
    virtual std::string toString() const;
    virtual CumulativeReward* clone() const;
    const double getTime() const;
    virtual unsigned arity() const;
    virtual const Property &operator[](unsigned) const;
    const unsigned &getRewardStruct() const;
    const Filter &getFilter() const;
  private:
    Bound bound;
    double time;
    unsigned rewardStruct;
    Filter filter;
  };


  class InstantaneousReward : public Property {
  public:
    InstantaneousReward(const Bound &, double, const unsigned, const Filter &);
    virtual std::string toString() const;
    virtual InstantaneousReward* clone() const;
    const double getTime() const;
    virtual unsigned arity() const;
    virtual const Property &operator[](unsigned) const;
    const unsigned &getRewardStruct() const;
    const Filter &getFilter() const;
  private:
    Bound bound;
    double time;
    unsigned rewardStruct;
    Filter filter;
  };

  class SteadyStateReward : public Property {
  public:
    SteadyStateReward(const Bound &, const unsigned, const Filter &);
    virtual std::string toString() const;
    virtual SteadyStateReward* clone() const;
    virtual unsigned arity() const;
    virtual const Property &operator[](unsigned) const;
    const Bound& getBound() const;
    const unsigned &getRewardStruct() const;
    const Filter &getFilter() const;
  private:
    Bound bound;
    unsigned rewardStruct;
    Filter filter;
  };

  class SteadyState : public Property {
  public: 
    SteadyState(Property *);
    SteadyState(const Bound &, Property *, const Filter &);
    virtual std::string toString() const;
    virtual SteadyState* clone() const;
    Property* getProp() const;
    virtual unsigned arity() const;
    virtual const Property &operator[](unsigned) const;
    const Bound& getBound() const;
    const Filter &getFilter() const;
  private:
    boost::shared_ptr<Property> prop;
    Bound bound;
    Filter filter;
  };

  /*! \brief atomic proposition combination formula

   */
  class PropExpr : public Property {
  protected:
    Expr *e;
  public:
    PropExpr(const Expr &);
    ~PropExpr();
    virtual std::string toString() const;
    virtual PropExpr* clone() const;
    const Expr &getExpr() const;
    virtual unsigned arity() const;
    virtual const Property &operator[](unsigned) const;
  };

  class PropNeg : public Property {
  protected:
    boost::shared_ptr<Property> p;
  public:
    PropNeg(Property*);
    virtual std::string toString() const;
    virtual PropNeg* clone() const;
    Property* getProp() const;
    virtual unsigned arity() const;
    virtual const Property &operator[](unsigned) const;
  };

  class PropBinary : public Property {
  protected:
    boost::shared_ptr<Property> p1;
    boost::shared_ptr<Property> p2;
  public:
    enum Operator { OR,   // p1 |  p2
		    AND,  // p1 &  p2
		    IMPL  // p1 => p2
    } op;
    PropBinary(Operator, Property *, Property *);
    virtual std::string toString() const;
    virtual PropBinary* clone() const;
    Operator getOp() const;
    Property* getProp1() const;
    Property* getProp2() const;
    virtual unsigned arity() const;
    virtual const Property &operator[](unsigned) const;
  };

  class Quant : public Property {
  public:
    Quant(const Bound &, Property *, const Filter &);
    virtual std::string toString() const;
    virtual Quant* clone() const;
    const Bound& getBound() const;
    bool isMin() const;
    Property* getProp() const;
    virtual unsigned arity() const;
    virtual const Property &operator[](unsigned) const;
    const Filter &getFilter() const;
  private:
    Bound bound;
    Filter filter;
    boost::shared_ptr<Property> pathprop;
  };

  struct Time {
    enum Kind {
      GE,        // >=t1
      LE,        // <=t1
      INTERVAL,  // [t1,t2]
      UNBOUNDED  // [0,infty]
    } kind;
    double t1, t2;
    Time(const Time &);
    Time();
    Time(Kind, double, double);
    std::string toString() const;
  };

  class Next : public Property {
  protected:
    Time time;
    boost::shared_ptr<Property> prop;
  public:
    Next(const Time &, Property *);
    Next(Time::Kind, double, double, Property *);
    Next(Property *);
    virtual std::string toString() const;
    virtual Next* clone() const;
    const Time& getTime() const;
    Property* getProp() const;
    virtual unsigned arity() const;
    virtual const Property &operator[](unsigned) const;
  };

  class Until : public Property {
  protected:
    Time time;
    boost::shared_ptr<Property> prop1;
    boost::shared_ptr<Property> prop2;
  public:
    Until(const Time &, Property *, Property *);
    Until(Time::Kind, double, double, Property *, Property *);
    Until(Property*,Property*);
    virtual std::string toString() const;
    virtual Until* clone() const;
    const Time& getTime() const;
    Property* getProp1() const;
    Property* getProp2() const;
    virtual unsigned arity() const;
    virtual const Property &operator[](unsigned) const;
  };
}

#endif
