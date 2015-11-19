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

#include <sstream>
#include <limits>
#include "Util.h"
#include "Node.h"
#include "Property.h"

namespace prismparser {

  std::string Bound::toString() const {
    std::string result;

    if (minOrMax) {
      if (min) {
	result = "min";
      } else {
	result = "max";
      }
    }

    switch (kind) {
    case Bound::GR : result += ">"; break;
    case Bound::GEQ: result += ">="; break;
    case Bound::LE : result += "<"; break;
    case Bound::LEQ: result += "<="; break;
    case Bound::DK : result += "=?"; break;
    default: assert(false); break;
    }

    if (Bound::DK != kind) {
      result += bound->toString();
    }
    return result;
  }

  bool Bound::isMinOrMax() const {
    return minOrMax;
  }
  
  Filter::Filter() {
    expr = NULL;
  }

  Filter::Filter(const Filter &filter) {
    if (NULL != filter.expr) {
      expr = new Expr(*filter.expr);
    } else {
      expr = NULL;
    }
    min = filter.min;
    minOrMax = filter.minOrMax;
  }

  Filter::Filter(const Expr &__expr, bool __min, bool __minOrMax) {
    expr = new Expr(__expr);
    min = __min;
    minOrMax = __minOrMax;
  }

  Filter::~Filter() {
    if (NULL != expr) {
      delete expr;
    }
  }

  const Expr &Filter::getExpr() const {
    return *expr;
  }

  bool Filter::isMin() const {
    return min;
  }

  bool Filter::isMinOrMax() const {
    return minOrMax;
  }

  Property::Property(PropertyKind __kind) : 
    kind(__kind) {
  }

  PropExpr::PropExpr(const Expr& __e) : Property(expr) {
    e = new Expr(__e);
  }

  PropExpr::~PropExpr() {
    delete e;
  }

  std::string PropExpr::toString() const {
    return e->toString();
  }

  ReachabilityReward::ReachabilityReward
  (const Bound &__bound, Property *__prop, unsigned __rewardStruct, const Filter &__filter)
    : Property(reachability_reward), bound(__bound), prop(__prop), filter(__filter) {
    rewardStruct = __rewardStruct;
  }

  std::string ReachabilityReward::toString() const {
    return "R" + bound.toString() + " [ F " + prop->toString() + "]";
  }

  ReachabilityReward* ReachabilityReward::clone() const {
    return new ReachabilityReward(bound, prop.get(), rewardStruct, filter);
  }

  Property *ReachabilityReward::getProp() const {
    return prop.get();
  };

  const Bound &ReachabilityReward::getBound() const {
    return bound;
  }

  unsigned ReachabilityReward::arity() const {
    return 1;
  }

  const Property &ReachabilityReward::operator[]
  (unsigned index) const {
    assert(0 == index);
    return *prop.get();
  }

  const unsigned &ReachabilityReward::getRewardStruct() const {
    return rewardStruct;
  }

  const Filter &ReachabilityReward::getFilter() const {
    return filter;
  }

  CumulativeReward::CumulativeReward
  (const Bound &__bound, const double __time, const unsigned __rewardStruct,
   const Filter &__filter)
    : Property(cumulative_reward), bound(__bound), filter(__filter) {
    time = __time;
    rewardStruct = __rewardStruct;
  }

  std::string CumulativeReward::toString() const {
    std::string time_string;
    std::stringstream time_sstream;
    time_sstream << time;
    time_sstream >> time_string;
    return "R=? [ C<=" + time_string + " ]";
  }

  CumulativeReward* CumulativeReward::clone() const {
    return new CumulativeReward(bound, time, rewardStruct, filter);
  }


  const double CumulativeReward::getTime() const {
    return time;
  };

  unsigned CumulativeReward::arity() const {
    return 0;
  }

  const Property &CumulativeReward::operator[](unsigned index) const {
    assert(false);
    return *this;
  }

  const unsigned &CumulativeReward::getRewardStruct() const {
    return rewardStruct;
  }

  const Filter &CumulativeReward::getFilter() const {
    return filter;
  }

  InstantaneousReward::InstantaneousReward
  (const Bound &__bound, double __time, const unsigned __rewardStruct,
   const Filter &__filter)
    : Property(instantaneous_reward), bound(__bound), filter(__filter) {
    time = __time;
    rewardStruct = __rewardStruct;
  }

  std::string InstantaneousReward::toString() const {
    std::string time_string;
    std::stringstream time_sstream;
    time_sstream << time;
    time_sstream >> time_string;
    return "R=? [ I=" + time_string + "]";
  }

  InstantaneousReward* InstantaneousReward::clone() const {
    return new InstantaneousReward(bound, time, rewardStruct, filter);
  }

  const double InstantaneousReward::getTime() const {
    return time;
  };

  unsigned InstantaneousReward::arity() const {
    return 0;
  }

  const Property &InstantaneousReward::operator[](unsigned index) const {
    assert(false);
    return *this;
  }

  const unsigned &InstantaneousReward::getRewardStruct() const {
    return rewardStruct;
  }

  const Filter &InstantaneousReward::getFilter() const {
    return filter;
  }

  SteadyStateReward::SteadyStateReward
  (const Bound &bound__, const unsigned __rewardStruct, const Filter &__filter)
    : Property(steadystate_reward), bound(bound__), filter(__filter) {
    rewardStruct = __rewardStruct;
  }

  std::string SteadyStateReward::toString() const {
    std::string result("R");
    result += bound.toString();
    result += "[ S ]";

    return result;
  }

  SteadyStateReward* SteadyStateReward::clone() const {
    return new SteadyStateReward(bound, rewardStruct, filter);
  }

  unsigned SteadyStateReward::arity() const {
    return 0;
  }

  const Property &SteadyStateReward::operator[](unsigned index) const {
    assert(false);
    return *this;
  }

  const Bound& SteadyStateReward::getBound() const {
    return bound;
  }

  const unsigned &SteadyStateReward::getRewardStruct() const {
    return rewardStruct;
  }

  const Filter &SteadyStateReward::getFilter() const {
    return filter;
  }

  SteadyState::SteadyState(Property *__prop)
    : Property(steadystate), prop(__prop) {
  }

  SteadyState::SteadyState
  (const Bound& __bound, Property* __prop, const Filter &__filter)
    : Property(steadystate), prop(__prop), bound(__bound), filter(__filter) {
    assert(__prop);
  }


  std::string SteadyState::toString() const {
    std::string result("S");
    result += bound.toString();
    result += "[ ";
    result += prop->toString();
    result += " ]";

    return result;
  }

  SteadyState* SteadyState::clone() const {
    return new SteadyState(prop.get());
  }


  Property* SteadyState::getProp() const {
    return prop.get();
  }

  unsigned SteadyState::arity() const {
    return 1;
  }

  const Property &SteadyState::operator[](unsigned index) const {
    assert(0 == index);
    return *prop.get();
  }

  const Bound &SteadyState::getBound() const {
    return bound;
  }

  const Filter &SteadyState::getFilter() const {
    return filter;
  }

  PropExpr* PropExpr::clone() const {
    return new PropExpr(*e);
  }

  const Expr &PropExpr::getExpr() const {
    return *e;
  }

  unsigned PropExpr::arity() const {
    return 0;
  }

  const Property &PropExpr::operator[](unsigned index) const {
    assert(false);
    return *this;
  }

  PropNeg::PropNeg(Property* __p) : Property(neg), p(__p) {
  }

  std::string PropNeg::toString() const {
    return "!" + p->toString();
  }

  PropNeg* PropNeg::clone() const {
    return new PropNeg(p.get());
  }

  Property *PropNeg::getProp() const {
    return p.get();
  }

  unsigned PropNeg::arity() const {
    return 1;
  }

  const Property &PropNeg::operator[](unsigned index) const {
    assert(0 == index);
    return *p.get();
  }

  PropBinary::PropBinary(Operator __op,Property* __p1,Property* __p2)
  : Property(binary), p1(__p1), p2(__p2) {
    assert(__p1 && __p2);
    assert(__op == OR || __op == AND || __op == IMPL);
    op = __op;
  }

  std::string PropBinary::toString() const {
    std::string s1 (p1->toString()),
      s2 (p2->toString());
    std::string ops;
    switch(op) {
    case OR:  ops = " | "; break;
    case AND: ops = " & "; break;
    case IMPL:ops = " => ";break;
    }
    return s1 + ops + s2;
  }

  PropBinary* PropBinary::clone() const {
    return new PropBinary(op,p1.get(),p2.get());
  }

  PropBinary::Operator PropBinary::getOp() const {
    return op;
  }

  Property *PropBinary::getProp1() const {
    return p1.get();
  }

  Property *PropBinary::getProp2() const {
    return p2.get();
  }

  unsigned PropBinary::arity() const {
    return 2;
  }
  
  const Property &PropBinary::operator[](unsigned index) const {
    assert(index < 2);
    if (0 == index) {
      return *p1.get();
    } else {
      return *p2.get();
    }
  }

  Quant::Quant(const Bound& __bound, Property* __pathprop, const Filter &__filter)
    : Property(quant), bound(__bound), filter(__filter), pathprop(__pathprop)  {
    assert(__pathprop);
  }

  std::string Quant::toString() const {
    std::string result("P");
    result += bound.toString();
    result +=" [ ";
    result += pathprop->toString();
    result += " ]";
    return result;
  }

  Quant* Quant::clone() const {
    return new Quant(bound, pathprop.get(), filter);
  }

  const Bound &Quant::getBound() const {
    return bound;
  }

  bool Quant::isMin() const {
    return bound.isMin();
  }

  Property *Quant::getProp() const {
    return pathprop.get();
  }

  unsigned Quant::arity() const {
    return 1;
  }

  const Filter &Quant::getFilter() const {
    return filter;
  }
  
  const Property &Quant::operator[](unsigned index) const {
    assert(0 == index);
    return *pathprop.get();
  }

  std::string Time::toString() const {
    std::string result;
    switch(kind) {
    case GE:        result = ">=" + floatToString(t1);
      break;
    case LE:        result  = "<=" + floatToString(t2);
      break;
    case INTERVAL:  result = "["+floatToString(t1)+","+floatToString(t2) + "]";
      break;
    case UNBOUNDED: result = "[0,infty]"; 
      break;
    default:
      break;
    } 
    return result;
  }

  Time::Time(const Time& t) : kind(t.kind), t1(t.t1), t2(t.t2) {
  }
  
  Time::Time() : kind(UNBOUNDED) {
  }
  
  Time::Time(Kind __kind, double __t1, double __t2) : 
    kind(__kind), t1(__t1), t2(__t2) {
    assert(t1<=t2);
  }
  
  Next::Next(Property* __prop) : Property(next), prop(__prop) {
    assert(__prop);
    time.t1 = 0.0;
    time.t2 = std::numeric_limits<double>::infinity();
  }

  Next::Next(const Time& __time,Property* __prop) : 
    Property(next), time(__time),	prop (__prop) {
    assert(__prop );
  }

  Next::Next(Time::Kind k,double t1, double t2, Property* __prop) 
    : Property(next), time(k,t1,t2), prop ( __prop ) {
    assert(__prop );
  }
	
  std::string Next::toString() const {
    return "X " + time.toString() + " " + prop->toString();
  }

  Next* Next::clone() const {
    return new Next(prop.get());
  }

  const Time &Next::getTime() const {
    return time;
  }
  
  Property *Next::getProp() const {
    return prop.get();
  }

  unsigned Next::arity() const {
    return 1;
  }

  const Property &Next::operator[](unsigned index) const {
    assert(0 == index);
    return *prop.get();
  }

  Until::Until(const Time& __time,Property* __prop1,Property*__prop2)
  : Property(until), time(__time), prop1 (__prop1), prop2 (__prop2) {
    assert(__prop1 && __prop2);
  }

  Until::Until(Time::Kind k,double t1, double t2,Property* __prop1,Property* __prop2) 
    : Property(until), time(k,t1,t2), prop1 (__prop1), prop2 (__prop2) {
    assert(__prop1 && __prop2);
	
  }

  Until::Until(Property* __prop1,Property* __prop2) :
    Property(until), prop1 (__prop1), prop2 (__prop2) {
    assert(__prop1 && __prop2);
    time.t1 = 0;
    time.t2 = std::numeric_limits<double>::infinity();
  }

  std::string Until::toString() const {
    std::string result;
    if (expr != prop1->kind) {
      result += "(";
    }
    result += prop1->toString()+" U ";
    if (expr != prop1->kind) {
      result += ")";
    }
    result += time.toString() + " ";		

    if (expr != prop2->kind) {
      result += "(";
    }
    result += prop2->toString();
    if (expr != prop2->kind) {
      result += ")";
    }

    return result;
  }

  Until* Until::clone() const {
    return new Until(time,prop1.get(),prop2.get());
  }

  const Time &Until::getTime() const {
    return time;
  }

  Property *Until::getProp1() const {
    return prop1.get();
  }
  
  Property *Until::getProp2() const {
    return prop2.get();
  }
  
  unsigned Until::arity() const {
    return 2;
  }
  
  const Property &Until::operator[](unsigned index) const {
    assert(index < 2);
    if (0 == index) {
      return *prop1.get();
    } else {
      return *prop2.get();
    }
  }

  Bound::Bound(const Bound& b) {
    kind = b.kind;
    bound = new Expr(*b.bound);
    min = b.min;
    minOrMax = b.minOrMax;
  }

  Bound::Bound(Kind __kind, const Expr &__bound, bool __min, bool __minOrMax) {
    kind = __kind;
    bound = new Expr(__bound);
    min = __min;
    minOrMax = __minOrMax;
  }
  
  Bound::Bound() : kind(DK) {
    bound = NULL;
  }

  Bound::~Bound() {
    if (NULL != bound) {
      delete bound;
    }
  }

  bool Bound::isMin() const {
    return min;
  }

#if 0
  double Bound::getBoundAsDouble() const {
    assert(bound->isDouble());
    return bound->getDouble();
  }
#endif

  const Expr &Bound::getBound() const {
    return *bound;
  }

}
