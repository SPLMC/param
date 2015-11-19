/*
 * This file is part of Model2X.
 *
 * Model2X is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Model2X is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Model2X. If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2010-2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#ifndef MODEL2X_H
#define MODEL2X_H

#include <vector>
#include <iosfwd>
#include <tr1/unordered_map>
#include "ValueCompute.h"
#include "Model2XPARAM.h"

namespace prismparser {
  class Model;
  class Commands;
  class Alternative;
  class Properties;
  class Property;
  class Expr;
  template<class T> class ExprHashMap;
}

#ifdef WITH_PARAM
namespace rational {
  class RationalFunction;
}
#endif

struct lua_State;
typedef struct lua_State lua_State;

namespace model2x {
#ifndef WITH_PARAM
  typedef double ValueType;
#else
  typedef rational::RationalFunction ValueType;
#endif

  class Model2X {
  public:
    Model2X();
    ~Model2X();
    void setModel(prismparser::Model &);
    void setUseRewards(bool);
    void addStateSet(const prismparser::Expr &);
    void build();
    void addInitStates();
    unsigned addState(const prismparser::ExprHashMap<int> &);
    void setRewardStruct(const unsigned);
    void getStateSuccessors(unsigned);
    unsigned getNumStates() const;
    bool isContTime() const;
    bool isNonDet() const;
    unsigned getNumInitStates() const;
    const unsigned *getInitStates() const;
    unsigned getNumSuccStates() const;
    const unsigned *getSuccStatesList() const;
    const ValueType *getSuccRatesList() const;
    const ValueType *getSuccRewardsList() const;
    unsigned getNumNonDet() const;
    const unsigned *getNonDetBounds() const;
    ValueType getStateReward(unsigned) const;
    bool inStateSet(unsigned, unsigned) const;

  private:
    struct eqstate {
      unsigned size;
      std::vector<unsigned> *stateList;
      bool operator()(const unsigned s1, const unsigned s2) const {
	for (unsigned i = 0; i < size; i++) {
	  if ((*stateList)[s1 + i] != (*stateList)[s2 + i]) {
	    return false;
	  }
	}
	return true;
      }
    };
    struct hashstate {
      unsigned size;
      std::vector<unsigned> *stateList;
      size_t operator()(const unsigned s) const {
	size_t hash = 0;
	for (unsigned i = 0; i < size; i++) {
	  hash = (*stateList)[s + i] + (hash << 6) + (hash << 16) - hash;
	}
	return hash;
      }
    };
    typedef std::tr1::unordered_map<const unsigned, unsigned, hashstate,
      eqstate> StateMap;

    void prepareStateFormulasFromPropertyStructure();
    void prepareStateFormulasFromPropertyStructure
      (const prismparser::Property &);
    void init();
    bool isInfVariable(unsigned) const;
    unsigned numVariableBits(unsigned) const;
    void enumerateVariables();
    void createInterface();
    unsigned encodeState(unsigned);
    unsigned insertState(unsigned);
    void decodeState(unsigned) const;
#ifdef WITH_PARAM
    void decodeStateVals(unsigned);
#endif
    void pushState(unsigned) const;
    void printGetStateSuccessors(std::stringstream &) const;
    void printSuccStateAssignments(std::stringstream &, prismparser::Alternative &) const;
    void printGetStateReward(std::stringstream &) const;
    void printInStateSet(std::stringstream &) const;
#ifndef WITH_PARAM
    void printAdjustNextStateProbs(std::stringstream &) const;
#endif
    void printGetValue(std::stringstream &) const;
    void printEncodeState(std::stringstream &) const;
    void printDecodeState(std::stringstream &) const;
    void printVarParams(std::stringstream &) const;
#ifdef WITH_PARAM
    void buildRateAndRewardValueTable();
    ValueType compRateValue(unsigned) const;
#endif

    prismparser::Model *model;
    const prismparser::Commands *guardedTransitions;
    std::vector<prismparser::Expr> variables;
    std::vector<prismparser::Expr> variablesInC;
    unsigned numStates;
    unsigned numSuccStates;
    unsigned numNonDet;
    unsigned *succStatesList;
    ValueType *succRatesList;
    ValueType *succRewardsList;
    unsigned *nonDetBounds;
    unsigned maxSuccStates;
    std::vector<int> lowerBounds;
    std::vector<int> upperBounds;
    bool useRewards;
    std::vector<prismparser::Expr> stateSets;
    unsigned numInitStates;
    unsigned *initStates;
    prismparser::ExprHashMap<unsigned> *exprNumbers;
    lua_State *luaState;
    unsigned intBits;
    unsigned encStateSize;
    unsigned *encDecSupport;
    StateMap stateMap;
    std::vector<unsigned> stateList;
    unsigned rewardStruct;
#ifdef WITH_PARAM
    std::vector<int> currState;
    std::vector<unsigned> rateValues;
    std::vector<unsigned> succRewardValues;
    std::vector<unsigned> stateRewardValues;
    ValueCompute valueCompute;
#endif
  };
}

#endif
