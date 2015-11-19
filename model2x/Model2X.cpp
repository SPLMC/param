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

#include "Model2X.h"
#include <limits>
#include <iostream>
#include <fstream>
#include <sstream>
extern "C" {
#include <luajit-2.0/lua.h>
#include <luajit-2.0/lauxlib.h>
#include <luajit-2.0/lualib.h>
}
#ifdef WITH_PARAM
#include "rationalFunction/RationalFunction.h"
#include "rationalFunction/ExprConverter.h"
#endif
#include "prismparser/Property.h"
#include "prismparser/Model.h"
#include "prismparser/PRISMParser.h"
#include "expr2lua.h"
#include "Model2XError.h"
#include "InitStatesComp.h"

namespace model2x {
  using namespace std;
  using namespace prismparser;
#ifdef WITH_PARAM
  using namespace rational;
#endif

  /**
   * Construct new Model-to-Lua converter.
   */
  Model2X::Model2X() {
    model = NULL;
    guardedTransitions = NULL;
    succStatesList = NULL;
    succRatesList = NULL;
    succRewardsList = NULL;
    nonDetBounds = NULL;
    useRewards = false;
    initStates = NULL;
    exprNumbers = NULL;
    luaState = NULL;
    encDecSupport = NULL;
    rewardStruct = 0;
  }

  /**
   * Sets model to be explored.
   *
   * @param __model model to be used
   */
  void Model2X::setModel(Model &__model) {
    model = &__model;
  }

  /**
   * Set whether a reward analysis shall be done.
   *
   * @param __useRewards true for reward analysis
   */
  void Model2X::setUseRewards(bool __useRewards) {
    useRewards = __useRewards;
  }

  void Model2X::setRewardStruct(const unsigned __rewardStruct) {
    rewardStruct = __rewardStruct;
  }

  /**
   * Return number of states obtained so far.
   *
   * @return number of model states
   */
  unsigned Model2X::getNumStates() const {
    assert(NULL != luaState);
    return numStates;
  }

  /**
   * Compute successors of a state @a state.
   * Successors will be written to successor list to be read later.
   *
   * @param state state to get successors of
   */
  void Model2X::getStateSuccessors(unsigned state) {
    assert(NULL != luaState);
    lua_getglobal(luaState, "getStateSuccessors");
    decodeStateVals(state);
    lua_call(luaState, variablesInC.size(), LUA_MULTRET);
    numSuccStates = lua_tointeger(luaState, 1);
    const unsigned rewardsAdd(useRewards ? 1 : 0);
    const unsigned valSize(1);
    const unsigned succEntrySize(encStateSize + valSize + rewardsAdd);
#ifdef WITH_PARAM
    RationalFunction succSum(0);
#endif
    for (unsigned succNr = 0; succNr < numSuccStates; succNr++) {
      const unsigned state = insertState
	(1 + 1 + succNr * succEntrySize);

      succStatesList[succNr] = state;
      const double rate(lua_tonumber
			(luaState, 1 + 1 +
			 succNr * succEntrySize + encStateSize));
#ifdef WITH_PARAM
      succRatesList[succNr] = compRateValue(rate);
      if (DTMC == model->getModelType()) {
	succSum += succRatesList[succNr];
      }
#else
      succRatesList[succNr] = rate;
#endif
      if (useRewards) {
	const double reward(lua_tonumber
			    (luaState, 1 + 1 +
			     succNr * succEntrySize + encStateSize + valSize));
#ifdef WITH_PARAM
	int rewNr(reward);
	if (-1 == rewNr) {
	  succRewardsList[succNr] = 0;
	} else {
	  valueCompute.compute(succRewardValues[rewNr], currState, succRewardsList[succNr]);
	}
#else
	succRewardsList[succNr] = reward;
#endif
      }
    }
    if (isNonDet()) {
      const unsigned nonDetBoundsSize
	(lua_tonumber(luaState, 1 + 1 + numSuccStates * succEntrySize));
      for (unsigned boundNr(0); boundNr < nonDetBoundsSize; boundNr++) {
	nonDetBounds[boundNr] =
	  lua_tonumber(luaState, 1 + 1 + numSuccStates * succEntrySize + 1 + boundNr);
      }
      numNonDet = nonDetBoundsSize;
    } else {
      nonDetBounds[1] = getNumSuccStates();
      numNonDet = 1;
    }
#ifdef WITH_PARAM
    if (DTMC == model->getModelType()) {
      for (unsigned succ(0); succ < numSuccStates; succ++) {
	succRatesList[succ] /= succSum;
      }
    }
#endif
    lua_settop(luaState, 0);
    numStates = stateList.size() / encStateSize;
  }

  unsigned Model2X::encodeState(unsigned begin) {
    const unsigned startIndex(stateList.size());
    stateList.resize(stateList.size() + encStateSize, 0);
    unsigned bitindex = 0;
    for (unsigned varNr = 0; varNr < variablesInC.size(); varNr++) {
      unsigned varValue;
      if (encDecSupport[varNr] == numeric_limits<unsigned>::max()) {
	varValue = unsigned(lua_toboolean(luaState, begin + varNr));
	const unsigned resultIndex = startIndex + (bitindex / intBits);
	stateList[resultIndex] |= (varValue << (bitindex % intBits));
	bitindex++;
      } else {
	const unsigned varBits(encDecSupport[varNr]);
	varValue = unsigned(lua_tointeger(luaState, begin + varNr) - lowerBounds[varNr]);
	const unsigned resultIndex = startIndex + (bitindex / intBits);
	stateList[resultIndex] |= (varValue << (bitindex % intBits));
	bitindex += varBits;
      }
    }

    if (0 == stateMap.count(startIndex)) {
      stateMap.insert(make_pair(startIndex, startIndex));
      return startIndex / encStateSize;
    } else {
      const unsigned result(stateMap[startIndex] / encStateSize);
      stateList.resize(startIndex);
      return result;
    }
  }

  unsigned Model2X::insertState(unsigned begin) {
    const unsigned startIndex(stateList.size());
    stateList.resize(stateList.size() + encStateSize, 0);
    for (unsigned placeNr = 0; placeNr < encStateSize; placeNr++) {
      stateList[placeNr + startIndex]
	= unsigned(lua_tointeger(luaState, begin + placeNr));
    }

    if (0 == stateMap.count(startIndex)) {
      stateMap.insert(make_pair(startIndex, startIndex));
      return startIndex / encStateSize;
    } else {
      const unsigned result(stateMap[startIndex] / encStateSize);
      stateList.resize(startIndex);
      return result;
    }
  }

  void Model2X::printEncodeState(stringstream &stream) const {
    for (unsigned placeNr(0); placeNr < encStateSize; placeNr++) {
      stream << "      local place" << placeNr << " = 0\n";
    }
    unsigned bitindex = 0;
    for (unsigned varNr = 0; varNr < variablesInC.size(); varNr++) {
      if (encDecSupport[varNr] == numeric_limits<unsigned>::max()) {
	const unsigned resultIndex = bitindex / intBits;
	stream << "      if (" << variablesInC[varNr] << "P) then\n";
	stream << "        local shiftOne = bit.lshift(1, "
	       << (bitindex % intBits) << ")\n";
	stream << "        place" << resultIndex << " = bit.bor(place"
	       << resultIndex << ", shiftOne)\n";
	stream << "      end\n";
	bitindex++;
      } else {
	const unsigned varBits(encDecSupport[varNr]);
	stream << "      " << variablesInC[varNr] << "P = "
	       << variablesInC[varNr] << "P - " << lowerBounds[varNr]
	       << "\n";
	const unsigned resultIndex = bitindex / intBits;
	stream << "      place" << resultIndex << " = bit.bor(place"
	       << resultIndex << ", bit.lshift(" << variablesInC[varNr]
	       << "P, " << bitindex % intBits << "))\n";
	bitindex += varBits;
      }
    }
    for (unsigned placeNr(0); placeNr < encStateSize; placeNr++) {
      stream << "      table.insert(resultTable, place" << placeNr << ")\n";
    }
    stream << "      table.insert(resultTable, rate__)\n";
    if (useRewards) {
      stream << "      table.insert(resultTable, reward__)\n";
    }
  }

  void Model2X::decodeState(unsigned encoded) const {
    const unsigned slBeginIndex(encoded * encStateSize);
    unsigned bitindex = 0;
    for (unsigned varNr = 0; varNr < variablesInC.size(); varNr++) {
      unsigned varBits(encDecSupport[varNr]);
      if (numeric_limits<unsigned>::max() == varBits) {
	varBits = 1;
      }
      const unsigned encIndex = slBeginIndex + (bitindex / intBits);
      const unsigned mask = numeric_limits<unsigned>::max() >> (intBits - varBits);
      unsigned varValue = (stateList[encIndex] >> (bitindex % intBits))
	& mask;
      int sVarValue = (int) varValue + lowerBounds[varNr];
      if (numeric_limits<unsigned>::max() == encDecSupport[varNr]) {
	lua_pushboolean(luaState, varValue);
	bitindex++;
      } else {
	lua_pushnumber(luaState, sVarValue);
	bitindex += varBits;
      }
    }
  }

  void Model2X::decodeStateVals(unsigned encoded) {
    const unsigned slBeginIndex(encoded * encStateSize);
    unsigned bitindex = 0;
    for (unsigned varNr = 0; varNr < variablesInC.size(); varNr++) {
      unsigned varBits(encDecSupport[varNr]);
      if (numeric_limits<unsigned>::max() == varBits) {
	varBits = 1;
      }
      const unsigned encIndex = slBeginIndex + (bitindex / intBits);
      const unsigned mask = numeric_limits<unsigned>::max() >> (intBits - varBits);
      unsigned varValue = (stateList[encIndex] >> (bitindex % intBits))
	& mask;
      int sVarValue = (int) varValue + lowerBounds[varNr];
      if (numeric_limits<unsigned>::max() == encDecSupport[varNr]) {
	currState[varNr] = varValue;
	lua_pushboolean(luaState, varValue);
	bitindex++;
      } else {
	currState[varNr] = sVarValue;
	lua_pushnumber(luaState, sVarValue);
	bitindex += varBits;
      }
    }
  }

  void Model2X::printDecodeState(stringstream &stream) const {
    stream << "  local maskedValue__\n";
    unsigned bitindex = 0;
    for (unsigned varNr = 0; varNr < variablesInC.size(); varNr++) {
      unsigned varBits(encDecSupport[varNr]);
      if (numeric_limits<unsigned>::max() == varBits) {
	varBits = 1;
      }
      const unsigned encIndex = bitindex / intBits;
      const unsigned mask = numeric_limits<unsigned>::max() >> (intBits - varBits);
      stream << "  " << variablesInC[varNr] << " = bit.band(bit.rshift(place"
	     << encIndex << ", " << (bitindex % intBits) << "), " << mask
	     << ")\n";
      if (numeric_limits<unsigned>::max() == encDecSupport[varNr]) {
	stream << "  " << variablesInC[varNr] << " = (" 
	       << variablesInC[varNr] << " == 1)\n";
	bitindex++;
      } else {
	bitindex += varBits;
      }
      stream << "  " << variablesInC[varNr] << " = "
	     << variablesInC[varNr] << " + " << lowerBounds[varNr] << "\n";
    }
  }

  void Model2X::pushState(unsigned encoded) const {
    const unsigned slBeginIndex(encoded * encStateSize);
    for (unsigned placeNr = 0; placeNr < encStateSize; placeNr++) {
      lua_pushnumber(luaState, stateList[slBeginIndex + placeNr]);      
    }
  }

  /**
   * Return number of successors from last call of @a getStateSuccessors.
   *
   * @return number of successor states
   */
  unsigned Model2X::getNumSuccStates() const {
    assert(NULL != luaState);
    return numSuccStates;
  }

  /**
   * Get array of successor states.
   * This value won't change for calls of @a getStateSuccessors. For this,
   * you need to call the function just once after calling @a build.
   *
   * @return array of successor states
   */
  const unsigned *Model2X::getSuccStatesList() const {
    assert(NULL != luaState);
    return succStatesList;
  }

  /**
   * Get array of rates to successor states.
   * This value won't change for calls of @a getStateSuccessors.  For this,
   * you need to call the function just once after calling @a build.
   *
   * @return array of rates to successor states
   */
  const ValueType *Model2X::getSuccRatesList() const {
    assert(NULL != luaState);
    return succRatesList;
  }
  
  /**
   * Get array of rewards to successor states.
   * This value won't change for calls of @a getStateSuccessors. For this,
   * you need to call the function just once after calling @a build.
   *
   * @return array of rates to successor states
   */
  const ValueType *Model2X::getSuccRewardsList() const {
    assert(NULL != luaState);
    return succRewardsList;
  }
  
  /**
   * Get non-determinism bounds.
   * This value won't change for calls of @a getStateSuccessors. For this,
   * you need to call the function just once after calling @a build.
   * TODO more explanation
   *
   * @return array of bounds of non-determinism
   */
  const unsigned *Model2X::getNonDetBounds() const {
    assert(NULL != luaState);
    return nonDetBounds;
  }
  
  /**
   * Return reward of state @a state.
   *
   * @param state state to obtain reward of
   * @return reward of @a state
   */
  ValueType Model2X::getStateReward(unsigned state) const {
    assert(NULL != luaState);
    assert(useRewards);
    lua_getglobal(luaState, "getStateReward");
    decodeState(state);
    lua_call(luaState, variablesInC.size(), 1);
    const double retVal = lua_tonumber(luaState, -1);
    lua_pop(luaState, 1);
#ifdef WITH_PARAM
    unsigned bitmask(retVal);
    RationalFunction result(0);
    for (unsigned index(0); index < stateRewardValues.size(); index++) {
      if (0 != (bitmask & 1)) {
	RationalFunction res;
	valueCompute.compute(stateRewardValues[index], currState, res);
	result += res;
      }
      bitmask >>= 1;
    }
    return result;
#else
    return retVal;
#endif
  }
  
  /**
   * Check whether state is in given state set.
   *
   * @param state state to check whether it is in set
   * @param setNr number of set to check whether state is in
   * @return true iff given state is in given set
   */
  bool Model2X::inStateSet(unsigned state, unsigned setNr) const {
    lua_getglobal(luaState, "inStateSet");
    lua_pushnumber(luaState, setNr);
    decodeState(state);
    lua_call(luaState, variablesInC.size() + 1, 1);
    const bool result = lua_toboolean(luaState, -1);
    lua_pop(luaState, 1);
    return result;
  }

  /**
   * Get number of initial states.
   *
   * @return number of initial states
   */
  unsigned Model2X::getNumInitStates() const {
    return numInitStates;
  }  

  /**
   * Get list of initial states.
   *
   * @return list of initial states
   */
  const unsigned *Model2X::getInitStates() const {
    return initStates;
  }

  /**
   * Get number of non-deterministic choices.
   * For models without non-determinism, this value will always be 1.
   */  
  unsigned Model2X::getNumNonDet() const {
    return numNonDet;
  }

  void Model2X::prepareStateFormulasFromPropertyStructure
  (const Property &prop) {
    if (expr == prop.kind) {
      const PropExpr &propExpr(dynamic_cast<const PropExpr &>(prop));
      addStateSet(propExpr.getExpr());
      (*exprNumbers)[propExpr.getExpr()] = stateSets.size() - 1;
    } else {
      for (unsigned childNr(0); childNr < prop.arity(); childNr++) {
        prepareStateFormulasFromPropertyStructure(prop[childNr]);
      }
    }
  }

  void Model2X::prepareStateFormulasFromPropertyStructure() {
    const Properties &props(model->getProperties());
    for (Properties::const_iterator iter(props.begin()); iter != props.end();
         iter++) {
      const Property &prop(*((*iter).get()));
      prepareStateFormulasFromPropertyStructure(prop);
    }
  }

  void Model2X::init() {
    numNonDet = 1;
    luaState = luaL_newstate();
    luaL_openlibs(luaState);
    guardedTransitions = &model->getCommands();
    enumerateVariables();
    maxSuccStates = 0;
    initStates = NULL;
    for (unsigned gt_nr = 0; gt_nr < guardedTransitions->size(); gt_nr++) {
      Command &gt = *((*guardedTransitions)[gt_nr]);
      const Alternatives &asss = gt.getAlternatives();
      maxSuccStates += asss.size();
    }
    exprNumbers = NULL;
    intBits = sizeof(int) * 8;
    succStatesList = new unsigned[maxSuccStates];
    succRatesList = new ValueType[maxSuccStates];
    if (useRewards) {
      succRewardsList = new ValueType[maxSuccStates];
    } else {
      succRewardsList = NULL;
    }
    const unsigned maxNumNonDet = guardedTransitions->size();
    nonDetBounds = new unsigned[maxNumNonDet + 1];
    unsigned numBits = 0;
    for (unsigned varNr = 0; varNr < variablesInC.size(); varNr++) {
      const unsigned varBits(numVariableBits(varNr));
      if (((numBits / intBits) < ((numBits + varBits) / intBits))
	  && (((numBits + varBits) % intBits) != 0)) {
	numBits = (numBits / intBits)*intBits + intBits;
      }

      numBits += varBits;
    }
    encStateSize = (numBits / intBits) + ((numBits % intBits == 0) ? 0 : 1);
    encDecSupport = new unsigned[variablesInC.size()];
    numBits = 0;
    for (unsigned varNr(0); varNr < variablesInC.size(); varNr++) {
      Expr &var = variables[varNr];
      const unsigned varBits(numVariableBits(varNr));
      if (((numBits / intBits) < ((numBits + varBits) / intBits))
	  && (((numBits + varBits) % intBits) != 0)) {
	unsigned numFillBits((numBits / intBits) * intBits + intBits - numBits);
	encDecSupport[varNr - 1] += numFillBits;
	numBits += numFillBits;
      }
      numBits += varBits;

      if (BoolVarType == model->getVarType(var)) {
	encDecSupport[varNr] = numeric_limits<unsigned>::max();
      } else {
	encDecSupport[varNr] = varBits;
      }
    }

    struct hashstate hs;
    hs.size = encStateSize;
    hs.stateList = &stateList;
    struct eqstate es;
    es.size = encStateSize;
    es.stateList = &stateList;
    StateMap newStateMap
      (8, hs, es, allocator<pair<const unsigned *, unsigned> >());
    stateMap = newStateMap;
    valueCompute.setStateVariables(variables);
  }

  /**
   * Compute initial initial states.
   * Will compute the set of initial states from initial states preciate
   * and set the list of initial states and the number of initial states.
   */
  void Model2X::addInitStates() {
    if (NULL != initStates) {
      delete initStates;
    }

    Expr initStatesExpr(model->getInitial());

    InitStatesComp initComp;
    initComp.setExpr(initStatesExpr);
    initComp.setVars(variables);
    initComp.setBounds(lowerBounds, upperBounds);
    std::vector<std::vector<int> > initStatesVec;
    initComp.compInits(initStatesVec);

    vector<unsigned> initStatesVecEnc;
    for (unsigned iStateNr = 0; iStateNr < initStatesVec.size(); iStateNr++) {
      for (unsigned varNr = 0; varNr < variables.size(); varNr++) {
	Expr &var = variables[varNr];
	if (BoolVarType == model->getVarType(var)) {
	  lua_pushboolean(luaState, initStatesVec[iStateNr][varNr]);
	} else {
	  lua_pushnumber(luaState, initStatesVec[iStateNr][varNr]);
	}
      }
      const unsigned initState(encodeState(1));
      initStatesVecEnc.push_back(initState);
    }
    lua_settop(luaState, 0);

    initStates = new unsigned[initStatesVecEnc.size()];
    copy(initStatesVecEnc.begin(), initStatesVecEnc.end(), initStates);
    numInitStates = initStatesVec.size();
    numStates = stateList.size() / encStateSize;
  }

  /**
   * Compile model.
   * Call after all relevant options have been set. After this, initial
   * states can be computed, successor states be explored, etc.
   */
  void Model2X::build() {
    assert(NULL != model);
    init();
    createInterface();
    lua_checkstack (luaState, 1 +
		    maxSuccStates * (max(size_t(encStateSize), variablesInC.size())
				     + 1 + (useRewards ? 1 : 0)));
  }

  /**
   * Destroy Model-to-Lua converter and cleanup.
   */
  Model2X::~Model2X() {
    if (NULL != exprNumbers) {
      delete exprNumbers;
    }
    if (NULL != succStatesList) {
      delete[] succStatesList;
    }
    if (NULL != succRatesList) {
      delete[] succRatesList;
    }
    if (NULL != succRewardsList) {
      delete[] succRewardsList;
    }
    if (NULL != encDecSupport) {
      delete[] encDecSupport;
    }
    if (NULL != initStates) {
      delete[] initStates;
    }
    if (NULL != nonDetBounds) {
      delete[] nonDetBounds;
    }
    if (NULL != luaState) {
      lua_close(luaState);
    }
  }

  unsigned Model2X::addState(const ExprHashMap<int> &state) {
    for (unsigned varNr = 0; varNr < variables.size(); varNr++) {
      const Expr &var = variables[varNr];
      if (BoolVarType == model->getVarType(var)) {
	lua_pushboolean(luaState, state.find(var)->second);
      } else {
	lua_pushnumber(luaState, state.find(var)->second);
      }
    }
    const unsigned stateNr(encodeState(1));
    lua_settop(luaState, 0);
    numStates = stateList.size() / encStateSize;
    
    return stateNr;
  }

  /**
   * Add distinguished set of states.
   * To check whether a state is element of a given set of states, the
   * function @a inStateSet() may be used. State sets will be given
   * ascending numbers starting with 0.
   *
   * @param stateSet state set to be specified
   */
  void Model2X::addStateSet(const Expr &stateSet) {
    stateSets.push_back(stateSet);
  }

  /**
   * Creates Lua interface.
   */
  void Model2X::createInterface() {
    stringstream stream;

    stream << "local bit = require(\"bit\")\n";
    printGetStateSuccessors(stream);
    printInStateSet(stream);
    if (useRewards) {
      printGetStateReward(stream);
    }

#ifndef NDEBUG
    ofstream dbgstream("debugout.lua");
    dbgstream << stream.str() << endl;
#endif
    int error = luaL_dostring(luaState, stream.str().c_str());
    if (0 != error) {
      const string errorString(lua_tostring(luaState, -1));
      throw model2x_error("Error compiling Lua script: \"" + errorString + "\"");
    }
#ifdef WITH_PARAM
    currState.resize(variables.size());
    buildRateAndRewardValueTable();
#endif
  }
  
  /**
   * Print out function to check whether a state is in a given state set.
   */
  void Model2X::printInStateSet(stringstream &stream) const {
    stream << "function inStateSet(setNr, ";
    printVarParams(stream);
    stream << ")\n";
    for (unsigned setNr = 0; setNr < stateSets.size(); setNr++) {
      stream << "  if ((setNr == " << setNr << ") and ("
	     << expr2lua(stateSets[setNr].substExpr(variables, variablesInC))
	     << ")) then \n"
	     << "    return true\n"
	     << "  end\n";
    }
    stream << "  return false\n"
	   << "end\n\n";
  }

  /**
   * Enumerate model variables.
   * Also compute position in state vector, number of bits neeed, etc. if
   * this is neccessary.
   */
  void Model2X::enumerateVariables() {
    /* for performance reasons, have "infinite" variables first. */
    
    for (unsigned varNr = 0; varNr < model->getNumVariables(); varNr++) {
      const Expr &var = model->getVariable(varNr);
      if (!model->isParameterVariable(var)) {
        if (IntVarType == model->getVarType(var)) {
	  const string varName(var.toString());
	  variables.push_back(var);
	  Expr varInC(Expr::varExpr("__mv_" + varName + "_"));
	  variablesInC.push_back(varInC);
	  lowerBounds.push_back(0);
	  upperBounds.push_back(0);
	}
      }
    }

    /* then non-boolean */
    for (unsigned varNr = 0; varNr < model->getNumVariables(); varNr++) {
      const Expr &var = model->getVariable(varNr);
      if (!model->isParameterVariable(var)) {
        if (RangeVarType == model->getVarType(var)) {
	  const string varName(var.toString());
	  variables.push_back(var);
	  pair<pair<int,int>,pair<int,int> > bounds(model->getBounds(var));
	  Expr varInC(Expr::varExpr("__mv_" + varName + "_"));
	  variablesInC.push_back(varInC);
	  assert(1 == bounds.first.second);
	  assert(1 == bounds.second.second);
	  lowerBounds.push_back(bounds.first.first);
	  upperBounds.push_back(bounds.second.first);
        }
      }
    }
    
    /* then boolean */
    for (unsigned varNr = 0; varNr < model->getNumVariables(); varNr++) {
      const Expr &var = model->getVariable(varNr);
      if (!model->isParameterVariable(var)) {
	if (BoolVarType == model->getVarType(var)) {
          variables.push_back(var);
	  const string varName(var.toString());
	  Expr varInC(Expr::varExpr("__mv_" + varName + "_"));
	  variablesInC.push_back(varInC);
          lowerBounds.push_back(0);
          upperBounds.push_back(1);
        }
      }
    }
  }

#ifndef WITH_PARAM
  void Model2X::printAdjustNextStateProbs(stringstream &stream) const {
    const unsigned loopBegin(1 + encStateSize);
    const unsigned loopStep(encStateSize + 1 + (useRewards ? 1 : 0));
    stringstream loopEnd;
    loopEnd << "(" << loopBegin << " + (" << loopStep
	    << " * (numSuccStates-1)))";

    stream << "  for succState = " << loopBegin << ", "
	   << loopEnd.str() << ", " << loopStep << " do\n";
    stream << "    resultTable[succState] = resultTable[succState]"
      " / numNonDet\n";
    stream << "  end\n";
  }
#endif

  /**
   * Print Lua function to obtain successors of a state.
   */
  void Model2X::printGetStateSuccessors(stringstream &stream) const {
    vector<pair<pair<string,Expr>,Expr> > *transRew = NULL;

    if (useRewards) {
      transRew = &model->getTransRewards(rewardStruct);
    }

    stream << "function getStateSuccessors(";
    printVarParams(stream);
    stream << ")\n";
    stream << "  local numSuccStates = 0\n"
      "  local resultTable = {}\n";
    if (isNonDet() || (DTMC == model->getModelType())) {
      stream << "  local numNonDet = 0\n";
    }
    if (isNonDet()) {
      stream << "  local nonDetBounds = {}\n";
      stream << "  table.insert(nonDetBounds,0)\n";
    }
    unsigned rateIndex(0);
    for (unsigned gt_nr = 0; gt_nr < guardedTransitions->size(); gt_nr++) {
      Command &gt = *((*guardedTransitions)[gt_nr]);
      const Expr &guard = gt.getGuard();
      stream << "  if (" << expr2lua(guard.substExpr(variables, variablesInC))
	     << ") then\n";
      const Alternatives &asss = gt.getAlternatives();
      if (useRewards) {
#ifdef WITH_PARAM
        stream << "    local reward__ = -1\n";
#else
        stream << "    local reward__ = 0.0\n";
#endif
        string action = gt.getAction();
        for (unsigned i = 0; i < transRew->size(); i++) {
          string &rewardAction = (*transRew)[i].first.first;
          Expr &guard = (*transRew)[i].first.second;
          if (action == rewardAction) {
            stream << "    if (" << expr2lua(guard.substExpr(variables, variablesInC)) << ") then\n";
#ifdef WITH_PARAM
	    stream << "      reward__ = " << i << "\n";
#else
	    Expr &reward = (*transRew)[i].second;
	    stream << "      reward__ =  " << expr2lua(reward) << "\n";
#endif
            stream << "    end\n";
          }
        }
      }
      for (unsigned ass_nr = 0; ass_nr < asss.size(); ass_nr++) {
        Alternative &ass = *asss[ass_nr];
        stream << "    local rate__ = ";
#ifdef WITH_PARAM
	stream << rateIndex;
#else
	stream << expr2lua(ass.getWeight().substExpr(variables, variablesInC));
#endif
        stream << "\n";
#ifdef WITH_PARAM
	stream << "    if (true) then\n";
#else
	stream << "    if (0 ~= rate__) then\n";
#endif
        printSuccStateAssignments(stream, ass);
	printEncodeState(stream);
	stream << "      numSuccStates = numSuccStates + 1\n";
	stream << "    end\n";
	rateIndex++;
      }
      if (isNonDet() || (DTMC == model->getModelType())) {
        stream << "    numNonDet = numNonDet + 1\n";
      }
      if (isNonDet()) {
        stream << "    table.insert(nonDetBounds, numSuccStates)\n";
      }
      stream << "  end\n";
    }
#ifndef WITH_PARAM
    if (DTMC == model->getModelType()) {
      printAdjustNextStateProbs(stream);
    }
#endif
    if (isNonDet()) {
      stream << "  table.insert(resultTable, #nonDetBounds)\n";
      stream << "  for bound=1,#nonDetBounds,1 do\n";
      stream << "    table.insert(resultTable, nonDetBounds[bound])\n";
      stream << "  end\n";
    }
    stream << "  return numSuccStates, unpack(resultTable)\n";
    stream << "end\n\n";
  }

  /**
   * Print variable names as parameters for a Lua function.
   *
   * @param stream stream to print into
   */
  void Model2X::printVarParams(stringstream &stream) const {
    for (unsigned varNr = 0; varNr < variablesInC.size(); varNr++) {
      stream << variablesInC[varNr];
      if (varNr != variablesInC.size() - 1) {
	stream << ", ";
      }
    }
  }

  /**
   * Print C sequence to assign values to variables of successor state.
   *
   * @param ass alternative from which successor results
   */
  void Model2X::printSuccStateAssignments
  (stringstream &stream, Alternative &ass) const {
    const Alternative::Map& map(ass.getMap());
    for (Alternative::Map::const_iterator i = map.begin(); i!=map.end(); ++i) {
      Expr var(i->first.substExpr(variables, variablesInC));
      Expr assExpr(i->second.substExpr(variables, variablesInC));
      stream << "      local " << var.toString() << "P = ";
      stream << expr2lua(assExpr);
      stream << "\n";
    }
    for (unsigned varNr = 0; varNr < variablesInC.size(); varNr++) {
      const Expr &var(variables[varNr]);
      if (0 == map.count(var)) {
	stream << "      local " << variablesInC[varNr]
	       << "P = " << variablesInC[varNr] << "\n";
      }
    }
  }

  /**
   * Checks whether @a var is a variable of unbounded range
   *
   * @param var number of variable to check for whether it is unbounded
   * @return true iff @a var is unbounded
   */
  bool Model2X::isInfVariable(unsigned var) const {
    return (IntVarType == model->getVarType(variables[var]));
  }

  /**
   * Return number of bits needed to store variables.
   * The number of bits needed to store a value assignment to a certain
   * variable is the binary logarithm of this variable, rounded up. For
   * infinite bound variables, we assume that sizeof(int) bits are
   * sufficient.
   *
   * @param var number of variable to compute number of bits for
   * @return number of bits needed to store values of @a var
   */
  unsigned Model2X::numVariableBits(unsigned var) const {
    if (isInfVariable(var)) {
      return sizeof(int) * 8;
    } else {
      const unsigned numPossibleValues(1 + upperBounds[var] - lowerBounds[var]);
      unsigned numBits = 1;
      unsigned roundedSize = 2;
      while (roundedSize < numPossibleValues) {
        numBits++;
        roundedSize *= 2;
      }
      
      return numBits;
    }
  }

  /**
   * Print Lua code for function to obtain state reward.
   */
  void Model2X::printGetStateReward(std::stringstream &stream) const {
    stream << "function getStateReward(";
    printVarParams(stream);
    stream << ")\n";
    stream << "  local reward = 0;\n";
    const vector<pair<Expr,Expr> > &gr = model->getStateRewards(rewardStruct);
    unsigned marker(1);
    for (unsigned i = 0; i < gr.size(); i++) {
      const Expr &guard = gr[i].first;
      stream << "  if (" << expr2lua(guard.substExpr(variables, variablesInC))
	     << ") then\n";
#ifdef WITH_PARAM
      stream << "   reward = bit.bor(reward, " << marker << ")\n";
#else
      stream <<"    reward = reward + " << expr2lua(gr[i].second.substExpr(variables, variablesInC)) << "\n";
#endif
      stream << "  end\n";
      marker <<= 1;
    }
    stream << "  return reward\n"
	   << "end\n\n";
  }

  /**
   * Checks whether underlying model is non-deterministic.
   *
   * @return true if time model is noin-deterministic
   */
  bool Model2X::isNonDet() const {
    return (prismparser::MDP == model->getModelType())
      || (prismparser::CTMDP == model->getModelType());
  }

  /**
   * Checks whether underlying model is continuous time or not.
   *
   * @return true if time model is continuous
   */
  bool Model2X::isContTime() const {
    const ModelType type(model->getModelType());
    if ((CTMC == type) || (CTMDP == type)) {
      return true;
    } else {
      return false;
    }
  }
  
#ifdef WITH_PARAM
  void Model2X::buildRateAndRewardValueTable() {
    vector<pair<pair<string,Expr>,Expr> > *transRew = NULL;

    if (useRewards) {
      transRew = &model->getTransRewards(rewardStruct);
    }

    for (unsigned gt_nr = 0; gt_nr < guardedTransitions->size(); gt_nr++) {
      Command &gt = *((*guardedTransitions)[gt_nr]);
      const Alternatives &asss = gt.getAlternatives();
      for (unsigned ass_nr = 0; ass_nr < asss.size(); ass_nr++) {
        Alternative &ass = *asss[ass_nr];
	rateValues.push_back(valueCompute.addEntry(ass.getWeight()));
      }
    }
    if (useRewards) {
      for (unsigned i = 0; i < transRew->size(); i++) {
	succRewardValues.push_back(valueCompute.addEntry((*transRew)[i].second));
      }
    }

    if (useRewards) {
      const vector<pair<Expr,Expr> > &gr(model->getStateRewards(rewardStruct));
      for (unsigned i = 0; i < gr.size(); i++) {
	stateRewardValues.push_back(valueCompute.addEntry(gr[i].second));
      }
    }
  }

  ValueType Model2X::compRateValue(unsigned rate) const {
    RationalFunction res;

    valueCompute.compute(rateValues[rate], currState, res);
    return res;
  }
#endif
}
