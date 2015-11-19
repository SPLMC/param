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

#ifndef CACHE_H
#define CACHE_H

#include <utility>
#include <vector>
#include <list>
#include <map>
#include <tr1/unordered_set>
#include <boost/tuple/tuple.hpp>

namespace prismparser {
  class Property;
}

namespace parametric {
  class Result;
  class Region;

  class Cache {
  private:
    typedef std::vector<unsigned> Scheduler;
  public:
    Cache();
    ~Cache();
    void insertProperty(const prismparser::Property &, const Result &);
    void lookupProperty(const prismparser::Property &, Result &) const;
    void insertProperty(const prismparser::Property &, const Region &, const Result &);
    void lookupProperty(const prismparser::Property &, const Region &, Result &) const;
    void insert(const prismparser::Property &, const unsigned, const Result &);
    void lookup(const prismparser::Property &, const unsigned, Result &) const;
    void insert(const prismparser::Property &, const unsigned, const Scheduler &, const Result &);
    void lookup(const prismparser::Property &, const unsigned, const Scheduler &, Result &) const;
    void insertScheduler(const prismparser::Property &, const Scheduler &);
    std::list<Scheduler> &lookupScheduler(const prismparser::Property &);
    void insertCheckSet(const prismparser::Property &, const unsigned, const Scheduler &,
			const std::tr1::unordered_set<rational::RationalFunction> &);
    void lookupCheckSet(const prismparser::Property &, const unsigned, const Scheduler &,
			std::tr1::unordered_set<rational::RationalFunction> &) const;
  private:
    std::map<const prismparser::Property *, const Result> pmcCache;
    std::map<std::pair<const prismparser::Property *, const Region>, const Result> pmdpCache;
    std::map<std::pair<const prismparser::Property *, const unsigned>, const Result> pmcIterCache;
    std::map<boost::tuple<const prismparser::Property *, const unsigned, const Scheduler>, const Result> pmdpIterCache;
    std::map<const prismparser::Property *, std::list<Scheduler> *> schedulerCache;
    std::map<boost::tuple<const prismparser::Property *, const unsigned, const Scheduler>,
      const std::vector<rational::RationalFunction> > checkSetCache;
  };
}

#endif
