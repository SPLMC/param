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
 *
 * You should have received a copy of the GNU General Public License
 * along with PARAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2010-2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#include <vector>
#include "prismparser/Property.h"
#include "rationalFunction/RationalFunction.h"
#include "RegionResult.h"
#include "Cache.h"
#include "boost/tuple/tuple_comparison.hpp"

namespace parametric {
  using namespace std;
  using namespace std::tr1;
  using namespace prismparser;
  using namespace rational;
  using namespace boost;

  Cache::Cache() {
  }

  Cache::~Cache() {
  }

  void Cache::insertProperty(const Property &prop, const Result &res) {
    pmcCache.insert(make_pair(&prop, res));
  }

  void Cache::lookupProperty(const Property &prop, Result &res) const {
    if (0 != pmcCache.count(&prop)) {
      res = pmcCache.find(&prop)->second;
    } else {
      res.clear();
    }
  }

  void Cache::insertProperty(const Property &prop, const Region &region, const Result &res) {
    pmdpCache.insert(make_pair(make_pair(&prop, region), res));
  }

  void Cache::lookupProperty(const Property &prop, const Region &region, Result &res) const {
    if (0 != pmdpCache.count(make_pair(&prop, region))) {
      res = pmdpCache.find(make_pair(&prop, region))->second;
    } else {
      res.clear();
    }
  }

  void Cache::insert(const Property &prop, const unsigned number, const Scheduler &sched, const Result &res) {
    pmdpIterCache.insert(make_pair(make_tuple(&prop, number, sched), res));
  }

  void Cache::lookup(const Property &prop, const unsigned number, const Scheduler &sched, Result &res) const {
    if (0 != pmdpIterCache.count(make_tuple(&prop, number, sched))) {
      res = pmdpIterCache.find(make_tuple(&prop, number, sched))->second;
    } else {
      res.clear();
    }
  }

  void Cache::insert(const Property &prop, const unsigned number, const Result &res) {
    pmcIterCache.insert(make_pair(make_pair(&prop, number), res));
  }

  void Cache::lookup(const Property &prop, const unsigned number, Result &res) const {
    if (0 != pmcIterCache.count(make_pair(&prop, number))) {
      res = pmcIterCache.find(make_pair(&prop, number))->second;
    } else {
      res.clear();
    }
  }

  void Cache::insertScheduler(const Property &prop, const Scheduler &scheduler) {
    if (0 != schedulerCache.count(&prop)) {
      list<Scheduler> &schedList(*(schedulerCache.find(&prop)->second));
      schedList.push_back(scheduler);
    } else {
      list<Scheduler> *schedList = new list<Scheduler>();
      schedList->push_back(scheduler);
      schedulerCache[&prop] = schedList;
    }
  }
  
  list<Cache::Scheduler> &Cache::lookupScheduler(const Property &prop) {
    if (0 == schedulerCache.count(&prop)) {
      list<Scheduler> *schedList = new list<Scheduler>();
      schedulerCache[&prop] = schedList;
    }
    return *schedulerCache[&prop];
  }

  void Cache::insertCheckSet
  (const Property &prop, const unsigned number, const Scheduler &scheduler,
   const unordered_set<RationalFunction> &checkSet) {
    const vector<RationalFunction> checkVector(checkSet.begin(), checkSet.end());
    checkSetCache.insert(make_pair(make_tuple(&prop, number, scheduler), checkVector));
  }

  void Cache::lookupCheckSet
  (const Property &prop, const unsigned number, const Scheduler &scheduler,
   unordered_set<RationalFunction> &checkSet) const {
    checkSet.clear();
    if (0 != checkSetCache.count(make_tuple(&prop, number, scheduler))) {
      const vector<RationalFunction> &checkVector(checkSetCache.find(make_tuple(&prop, number, scheduler))->second);
      for (unsigned i(0); i < checkVector.size(); i++) {
	checkSet.insert(checkVector[i]);
      }
    } else {
      checkSet.clear();
    }
  }
}
