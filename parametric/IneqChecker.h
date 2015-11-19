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

#ifndef INEQ_CHECKER
#define INEQ_CHECKER

#include <gmpxx.h>
#include <string>
#include <vector>
#include <boost/tuple/tuple.hpp>

namespace Rational {
  class rationalFunction;
}

namespace parametric {
  class Region;

  class IneqChecker {
  public:
    IneqChecker();
    ~IneqChecker();
    void setSolver(const std::string &);
    void setISATBinary(const std::string &);
    void setRSolverBinary(const std::string &);
    void setRAHDBinary(const std::string &);
    void setAssumeNoDenomSignChange(bool);
    bool check(const rational::RationalFunction &, bool, const Region &);
  private:
    bool checkRSolverForall
      (const rational::RationalFunction &, bool, const Region &);
    bool checkRSolver(const rational::RationalFunction &, bool, const Region &);
    bool checkISat(const rational::RationalFunction &, bool, const Region &);
    bool checkRAHD(const rational::RationalFunction &, bool, const Region &);
    mpq_class rSolverDoubleToMPQ(const std::string &);
    double convertSafe(const mpq_class &, bool);
    bool preCheck(const rational::RationalFunction &, bool, const Region &);
    bool contains(const Region &, const Region &) const;

    std::string solver;
    std::string iSATBinary;
    std::string RSolverBinary;
    std::string RAHDBinary;
    bool assumeNoDenomSignChange;
    
    std::vector<boost::tuple<rational::RationalFunction, bool, Region> > *known;
  };
}

#endif
