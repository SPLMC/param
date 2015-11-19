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

#ifndef MDP_ITERATOR_H
#define MDP_ITERATOR_H

#include <vector>
#include <map>
#include <boost/dynamic_bitset_fwd.hpp>
#include <boost/tuple/tuple.hpp>
#include <gmpxx.h>

namespace rational {
  class RationalFunction;
}

namespace prismparser {
  class Property;
}

namespace parametric {
  class PMC;
  class PMDP;
  class MC;
  class MDP;
  class Result;

  class MDPIterator {
  private:
    typedef std::vector<unsigned> Scheduler;
    typedef std::vector<mpq_class> Point;
  public:
    MDPIterator();
    ~MDPIterator();
    void setPMDP(const PMDP &);
    void setPrecision(const double);
    void setToleranceFactor(const double);
    void setPoint(const Point &);
    void next(const bool, const boost::dynamic_bitset<> &,
	      Scheduler &);
    void bounded(const bool, const unsigned,
		 const Result &, Scheduler &);
    void unbounded(const bool, const boost::dynamic_bitset<> &,
		   Scheduler &);
    bool checkUnbounded(const bool, const boost::dynamic_bitset<> &,
			const Scheduler &);
    void reachReward(const bool, Scheduler &);
    bool checkReachReward(const bool, const Scheduler &);
    void computeInducedPMC(const Scheduler &, const bool, PMC &) const;
    void setProperty(const prismparser::Property &prop, const unsigned);
  private:
    typedef std::map<boost::tuple<const prismparser::Property*, unsigned, Point>,
      std::pair<Scheduler, std::vector<double> > > OptSched;
    typedef std::map<boost::tuple<const prismparser::Property*, unsigned, Point, Scheduler>, bool> SchedValid;

    bool better(const bool, const double, const double) const;
    void instantiateMDP(bool);
    void computeScheduler
      (const boost::dynamic_bitset<> &targets, const std::vector<double> &,
       Scheduler &);
    double diff(const double, const double) const;
    void instantiateVector(const Result &, std::vector<double> &) const;
    void computeInducedMC(const Scheduler &, bool, MC &) const;
    void reachReward(const MC &, std::vector<double> &) const;
    void unbounded(const MC &, const boost::dynamic_bitset<> &,
		   std::vector<double> &) const;
    mpq_class distance(const Point &, const Point &);

    const PMDP *pmdp;
    MDP *mdp;
    Point point;
    double precision;
    double toleranceFactor;
    const prismparser::Property *prop;
    unsigned propNr;
    SchedValid schedValid;
    OptSched optSched;
  };
}

#endif
