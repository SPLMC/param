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

#ifndef MODEL_CHECKER_H
#define MODEL_CHECKER_H

#include <string>
#include <tr1/unordered_set>
#include <gmpxx.h>
#include <boost/dynamic_bitset_fwd.hpp>

typedef enum { l_false=-1, l_undef, l_true } lbool;

namespace prismparser {
  class Property;
  class Properties;
  class Bound;
}

namespace parametric {
  class PMM;
  class PMC;
  class PMDP;
  class Region;
  class RegionResult;
  class ExprToNumber;
  class IneqChecker;
  class RegionsTODO;
  class Cache;
  class MDPIterator;

  class ModelChecker {
  public:
    ModelChecker();
    ~ModelChecker();
    void setModel(PMM &);
    void setProperties(prismparser::Properties &);
    void setResult(RegionResult &);
    void setExprToNumber(ExprToNumber &);
    void setLumpMethod(const std::string &);
    void setMaxUnknown(const mpq_class &);
    void setEliminationOrder(const std::string &);
    void setRefineOrder(const std::string &);
    void setRegionSplitMode(const std::string &);
    void setSolver(const std::string &);
    void setISATBinary(const std::string &);
    void setRSolverBinary(const std::string &);
    void setRAHDBinary(const std::string &);
    void setIteratePrecision(const double);
    void setToleranceFactor(const double);
    void setRandomSchedCheck(const unsigned);
    void execute();
  private:
    typedef std::tr1::unordered_set<rational::RationalFunction> CheckSet;
    typedef CheckSet::iterator CheckSetIter;
    typedef CheckSet::const_iterator ConstCheckSetIter;
    typedef std::vector<unsigned> Scheduler;

    void modelCheckProperty(const prismparser::Property &);
    bool modelCheckRegion
      (const prismparser::Property &, const Region &, bool,
       const boost::dynamic_bitset<> &, Result &);
    bool modelCheckExpr
      (const prismparser::Property &, const Region &, bool,
       const boost::dynamic_bitset<> &, Result &);
    bool modelCheckNeg
      (const prismparser::Property &, const Region &, bool,
       const boost::dynamic_bitset<> &, Result &);
    bool modelCheckBinary
      (const prismparser::Property &, const Region &, bool,
       const boost::dynamic_bitset<> &, Result &);
    bool modelCheckQuant
      (const prismparser::Property &, const Region &, bool,
       const boost::dynamic_bitset<> &, Result &);
    bool modelCheckNext
      (const prismparser::Property &, const Region &, bool,
       const boost::dynamic_bitset<> &, Result &);
    bool modelCheckUntil
      (const prismparser::Property &, const Region &, bool,
       const boost::dynamic_bitset<> &, Result &);
    bool modelCheckReachReward
      (const prismparser::Property &, const Region &, bool,
       const boost::dynamic_bitset<> &, Result &);
    bool modelCheckNextPMC
      (const prismparser::Property &, const Region &, bool,
       const boost::dynamic_bitset<> &, Result &);
    bool modelCheckUntilPMC
      (const prismparser::Property &, const Region &, bool,
       const boost::dynamic_bitset<> &, Result &);
    bool modelCheckReachRewardPMC
      (const prismparser::Property &, const Region &, bool,
       const boost::dynamic_bitset<> &, Result &);
    bool modelCheckNextPMDP
      (const prismparser::Property &, const Region &, bool,
       const boost::dynamic_bitset<> &, Result &);
    bool modelCheckUntilPMDP
      (const prismparser::Property &, const Region &, bool,
       const boost::dynamic_bitset<> &, Result &);
    bool modelCheckReachRewardPMDP
      (const prismparser::Property &, const Region &, bool,
       const boost::dynamic_bitset<> &, Result &);
    void computeIdentityBackMap(std::vector<PMM::state> &);
    void boundedIteratePMC(const prismparser::Property &, const unsigned,
			   const Region &, bool, double,
			   const boost::dynamic_bitset<> &, Result &);
    void unboundedIteratePMC(const prismparser::Property &, const unsigned,
			     const Region &, bool,
			     const boost::dynamic_bitset<> &, Result &);
    bool boundedIteratePMDP(const prismparser::Property &, const unsigned,
			    const Region &, bool, double,
			    const boost::dynamic_bitset<> &, Result &);
    bool unboundedIteratePMDP(const prismparser::Property &, const unsigned,
			      const Region &, bool,
			      const boost::dynamic_bitset<> &, Result &);
    void createInitialPartition(const Result &, std::vector<unsigned> &);
    void split(const Region &, RegionsTODO &);
    void splitAll(const Region &, RegionsTODO &);
    void splitLongest(const Region &, RegionsTODO &);
    bool filterByBound(const prismparser::Bound &,
		       const boost::dynamic_bitset<> &,
		       const std::vector<rational::RationalFunction> &,
		       const Region &,
		       std::vector<rational::RationalFunction> &);
    void computeNeedInnerResultFromBound
      (const prismparser::Bound &, const boost::dynamic_bitset<> &,
       boost::dynamic_bitset<> &);
    void computeLocalOptNextScheduler
      (const Region &, const bool min, const Result &, Scheduler &) const;
    void computeLocalOptUnboundedScheduler
      (const Region &, const bool min, const Result &, Scheduler &) const;
    void computeLocalOptBoundedScheduler
      (const Region &, const bool min, const Result &, const double, Scheduler &) const;
    void findReachable(const boost::dynamic_bitset<> &, boost::dynamic_bitset<> &) const;
    void needInnerResultForNextPMC
      (const prismparser::Property &, const boost::dynamic_bitset<> &,
       boost::dynamic_bitset<> &);
    void needInnerResultForNextPMDP
      (const prismparser::Property &, const boost::dynamic_bitset<> &,
       boost::dynamic_bitset<> &);
    void needInnerResultForUntilPMC
      (const prismparser::Property &, const boost::dynamic_bitset<> &,
       boost::dynamic_bitset<> &);
    void needInnerResultForUntilPMDP
      (const prismparser::Property &, const boost::dynamic_bitset<> &,
       boost::dynamic_bitset<> &);
    void computeNextPMDPResultByScheduler
      (const Scheduler &, const boost::dynamic_bitset<> &,
       const Result &, Result &);
    void computeUnboundedPMDPResultByScheduler
      (const prismparser::Property &, const unsigned, const Scheduler &, Result &);
    void computeBoundedPMDPResultByScheduler
      (const prismparser::Property &, const unsigned, const Scheduler &, const bool,
       Result &, CheckSet &);
    bool verifyNextPMDPResult
      (const prismparser::Property &, const Scheduler &,
       const boost::dynamic_bitset<> &, const Result &, const Result &,
       const Region &, const bool);
    bool verifyUnboundedPMDPResult
      (const prismparser::Property &, const unsigned, const Scheduler &,
       const boost::dynamic_bitset<> &, const Result &,
       const Region &, const bool);
    bool verifyBoundedPMDPResult(const CheckSet &, const Region &);
    void printProgress(const mpq_class &) const;

    ExprToNumber *exprToNumber;
    PMM *pmm;
    PMC *pmc;
    PMDP *pmdp;
    prismparser::Properties *props;
    RegionResult *result;
    std::string lumpMethod;
    std::string eliminationOrder;
    std::string refineOrder;
    std::string regionSplitMode;
    std::string solver;
    std::string iSATBinary;
    std::string RSolverBinary;
    std::string RAHDBinary;
    mpq_class maxUnknown;
    IneqChecker *ineqChecker;
    Cache *cache;
    MDPIterator *mdpIterator;
    double iteratePrecision;
    double toleranceFactor;
    unsigned randomSchedCheck;
  };
}

#endif
