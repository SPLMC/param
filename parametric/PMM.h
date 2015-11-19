/*
 * This file is part of PARAM.
 *
 * PARAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * PARAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with PARAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2010-2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#ifndef PMM_H
#define PMM_H

#include <set>
#include <list>
#include <map>
#include <boost/dynamic_bitset_fwd.hpp>

namespace rational {
  class RationalFunction;
}

namespace parametric {
  class state_iter;

  class PMM {
  public:
    typedef unsigned state;
    enum ModelType {PMC, PMDP};
    enum TimeType {DISC, CONT};

    PMM();
    virtual ~PMM();
    virtual void reserveRowsMem(unsigned) = 0;
    virtual void reserveColsMem(unsigned) = 0;
    virtual void reserveStateRewardsMem(unsigned) = 0;
    virtual void reserveTransRewardsMem(unsigned) = 0;
    virtual void addSucc(state, rational::RationalFunction) = 0;
    virtual void addSuccReward(rational::RationalFunction) = 0;
    virtual void setStateReward(state, rational::RationalFunction) = 0;
    virtual void setStateReward(rational::RationalFunction) = 0;
    virtual void finishState() = 0;
    virtual unsigned getNumStates() const = 0;
    virtual ModelType getModelType() const = 0;
    state getInvalidState() const;
    void setTimeType(TimeType);
    TimeType getTimeType() const;
    void addInit(PMM::state);
    bool isInit(PMM::state) const;
    void reserveAPMem(unsigned, unsigned);
    void setAP(PMM::state, unsigned, bool);
    bool isAP(PMM::state, unsigned) const;
    unsigned getNumAPs() const;
    void setAbsorbing(state, bool);
    virtual bool useRewards() const = 0;
  protected:
    TimeType timeType;
    std::set<PMM::state> *inits;
    unsigned numAPs;
    boost::dynamic_bitset<> *aps;
    boost::dynamic_bitset<> *absorbing;
  };

  typedef std::set<PMM::state> StateSet;
  typedef std::list<PMM::state> StateList;
  typedef std::map<PMM::state, rational::RationalFunction> StateMap;
}

#endif
