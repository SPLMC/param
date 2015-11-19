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

#include <cstdlib>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <limits>
#include <set>
#include <boost/tuple/tuple.hpp>
#include "rationalFunction/RationalFunction.h"
#include "RegionResult.h"
#include "IneqChecker.h"

// TODO remember what was unknown (needed for bounded reach)

namespace parametric {
  using namespace std;
  using namespace boost;
  using namespace rational;

  IneqChecker::IneqChecker() {
    known = new vector<tuple<RationalFunction, bool, Region> >();
  }

  IneqChecker::~IneqChecker() {
    delete known;
  }

  void IneqChecker::setSolver(const std::string &solver_) {
    solver = solver_;
  }

  void IneqChecker::setISATBinary(const std::string &iSATBinary_) {
    iSATBinary = iSATBinary_;
  }

  void IneqChecker::setRSolverBinary(const string &RSolverBinary_) {
    RSolverBinary = RSolverBinary_;
  }

  void IneqChecker::setRAHDBinary(const string &RAHDBinary_) {
    RAHDBinary = RAHDBinary_;
  }

  void IneqChecker::setAssumeNoDenomSignChange
  (bool __assumeNoDenomSignChange) {
    assumeNoDenomSignChange = __assumeNoDenomSignChange;
  }

  /**
   * Checks whether @a expr >/>= 0 forall x in @a region
   */
  bool IneqChecker::check
  (const RationalFunction &expr, bool strict, const Region &region) {
    if (0 == expr) {
      return !strict;
    }

    if (!preCheck(expr, strict, region)) {
      return false;
    }    

    bool result;
    for (unsigned i(0); i < known->size(); i++) {
      const RationalFunction &cmpExpr((*known)[i].get<0>());
      const bool cmpStrict((*known)[i].get<1>());
      const Region &cmpRegion((*known)[i].get<2>());
      if ((expr == cmpExpr) && (strict == cmpStrict) && (region == cmpRegion)) {
	return true;
      } else if ((expr == cmpExpr) && (strict == cmpStrict)
		 && (contains(cmpRegion, region))) {
	return true;
      }
    }

    if ("isat" == solver) {
      result = checkISat(expr, strict, region);
    } else if ("rsolver-forall" == solver) {
      result = checkRSolverForall(expr, strict, region);      
    } else if ("rsolver" == solver) {
      result = checkRSolver(expr, strict, region);      
    } else if ("rahd" == solver) {
      result = checkRAHD(expr, strict, region);
    } else if ("none" == solver) {
      result = true;
    } else {
      throw runtime_error("Unsupported solver \"" + solver + "\" specified.");
    }

    if (result) {
      known->push_back(make_tuple(expr, strict, region));
    }

    return result;
  }

  bool IneqChecker::contains(const Region &reg1, const Region &reg2) const {
    const unsigned numSymbols(RationalFunction::getNumSymbols());
    for (unsigned symNr(0); symNr < numSymbols; symNr++) {
      const Interval &ival1(reg1[symNr]);
      const Interval &ival2(reg2[symNr]);
      if ((ival2.first < ival1.first) || (ival2.second > ival1.second)) {
	return false;
      }
    }

    return true;
  }

  bool IneqChecker::checkRSolverForall
  (const RationalFunction &expr, bool strict, const Region &region) {
    ofstream rstream("rsolver.rs", ios::out);
    const unsigned numSymbols(RationalFunction::getNumSymbols());
    rstream << "[]\nFORALL [";
    for (unsigned symNr(0); symNr < numSymbols; symNr++) {
      rstream << RationalFunction::getSymbolName(symNr);
      if (symNr < numSymbols - 1) {
	rstream << ", ";
      }
    }
    rstream << "] [";
    for (unsigned symNr(0); symNr < numSymbols; symNr++) {
      double lower(convertSafe(region[symNr].first, false));
      double upper(convertSafe(region[symNr].second, true));
      rstream << "[" << scientific << setprecision(17)
	      << lower
	      << ", "
	      << upper
	      << "]" << setprecision(6)
	      << resetiosflags(ios_base::floatfield);
      if (symNr < numSymbols - 1) {
	rstream << ", ";
      }
    }
    string exprNum(expr.getNum().toString());
    string exprDen(expr.getDen().toString());
    rstream << "]\n[";

    if (assumeNoDenomSignChange) {
      int sign;
      vector<mpq_class> point;
      for (unsigned symNr(0); symNr < numSymbols; symNr++) {
	point.push_back((region[symNr].second + region[symNr].first) / 2);
      }
      sign = (expr.getNum().evaluate(point) < 0) ? -1 : 1;
      if (-1 == sign) {
	rstream << "-";
      }
      rstream << "(" << exprNum << ") >";
      if (!strict) {
	rstream << "=";
      }
      rstream << " 0";
    } else {
      rstream << "[[";
      rstream << exprNum << " > 0] ==> [" << exprDen << " >= 0]] /\\ [[";
      rstream << exprNum << " < 0] ==> [" << exprDen << " <= 0]]";
      if (strict) {
	rstream << " /\\ [[" << exprNum << " < 0] \\/ [";
	rstream << exprNum << " > 0]]";
      }
    }

    rstream << "];\n[]\n";
    rstream.close();

    string callString(RSolverBinary + " < rsolver.rs > rsolver.out");

    if (system(callString.c_str())) {
      throw runtime_error("Error when calling RSolver");
    }

    ifstream istream("rsolver.out", ios::in);
    while (!istream.eof()) {
      string asdf;
      istream >> asdf;
      if ("False," == asdf) {
	istream >> asdf;
	if ("volume" != asdf) {
	  throw runtime_error("Error when interpreting RSolver output");
	}
	istream >> asdf;
	if ("~[" != asdf) {
	  throw runtime_error("Error when interpreting RSolver output");
	}
	istream >> asdf;
	if ("1.," == asdf) {
	  istream >> asdf;
	  if ("1.]" == asdf) {
	    return false;
	  } else {
	    throw runtime_error("Error when interpreting RSolver output");
	  }
	} else if ("0.," == asdf) {
	  istream >> asdf;
	  if ("0.]" == asdf) {
	    return true;
	  } else {
	    throw runtime_error("Error when interpreting RSolver output");
	  }
	}
      }
    }
    throw runtime_error("Error when interpreting RSolver output");
  }

  mpq_class IneqChecker::rSolverDoubleToMPQ(const std::string &str) {
    size_t endFirstNumber = str.find("*2^");
    string numberString(str.substr(0, endFirstNumber));
    mpq_t number;
    mpq_init(number);
    mpq_set_str(number, numberString.c_str(), 10);
    string expoString(str.substr(endFirstNumber + 3));
    mpz_class expo(expoString);
    if (expo >= 0) {
      unsigned long expoUint(expo.get_ui());
      mpq_mul_2exp(number, number, expoUint);
    } else {
      expo *= -1;
      unsigned long expoUint(expo.get_ui());
      mpq_div_2exp(number, number, expoUint);      
    }

    mpq_class result(number);
    mpq_clear(number);
    return result;
  }

  bool IneqChecker::checkRSolver
  (const RationalFunction &expr, bool strict, const Region &region) {
    ofstream rstream("rsolver.rs", ios::out);
    const unsigned numSymbols(RationalFunction::getNumSymbols());
    rstream << "[";
    for (unsigned symNr(0); symNr < numSymbols; symNr++) {
      rstream << RationalFunction::getSymbolName(symNr);
      if (symNr < numSymbols - 1) {
	rstream << ", ";
      }
    }
    string exprNum(expr.getNum().toString());
    string exprDen(expr.getDen().toString());

    rstream << "]\n[";
    if (assumeNoDenomSignChange) {
      int sign;
      vector<mpq_class> point;
      for (unsigned symNr(0); symNr < numSymbols; symNr++) {
	point.push_back((region[symNr].second + region[symNr].first) / 2);
      }
      sign = (expr.getNum().evaluate(point) < 0) ? -1 : 1;
      if (-1 == sign) {
	rstream << "-";
      }
      rstream << "(" << exprNum << ") >";
      if (!strict) {
	rstream << "=";
      }
      rstream << " 0";
    } else {
      rstream << "[[";
      rstream << exprNum << " > 0] ==> [" << exprDen << " >= 0]] /\\ [[";
      rstream << exprNum << " < 0] ==> [" << exprDen << " <= 0]]";
      if (strict) {
	rstream << " /\\ [[" << exprNum << " < 0] \\/ [";
	rstream << exprNum << " > 0]]";
      }
    }
    rstream << "];\n[";
    for (unsigned symNr(0); symNr < numSymbols; symNr++) {
      double lower(convertSafe(region[symNr].first, false));
      double upper(convertSafe(region[symNr].second, true));
      rstream << "[" << scientific << setprecision(17)
	      << lower
	      << ", "
	      << upper
	      << "]" << setprecision(6)
	      << resetiosflags(ios_base::floatfield);
      if (symNr < numSymbols - 1) {
	rstream << ", ";
      }
    }
    rstream << "]\n";
    rstream.close();
    
    string callString(RSolverBinary + " -e 0.05 -f ExactBoundaries -f MeanValueConstraint < rsolver.rs > rsolver.out");
    //    string callString(RSolverBinary + " -ns -f ExactBoundaries -f MeanValueConstraint < rsolver.rs > rsolver.out");

    if (system(callString.c_str())) {
      throw runtime_error("Error when calling RSolver");
    }

    ifstream istream("rsolver.out", ios::in);

    bool allTrue(true);
    enum {none, trueRegions, falseRegions, unknownRegions} readingType;
    readingType = none;
    while (!istream.eof()) {
      string asdf;
      istream >> asdf;
      if (istream.eof()) {
	break;
      }
      if ("True," == asdf) {
	istream >> asdf;
	if ("volume" != asdf) {
	  throw runtime_error("Error when interpreting RSolver output");
	}
	istream >> asdf;
	if ("~[" != asdf) {
	  throw runtime_error("Error when interpreting RSolver output");
	}
	istream >> asdf;
	istream >> asdf;
	readingType = trueRegions;
      } else if ("False," == asdf) {
	istream >> asdf;
	if ("volume" != asdf) {
	  throw runtime_error("Error when interpreting RSolver output");
	}
	istream >> asdf;
	if ("~[" != asdf) {
	  throw runtime_error("Error when interpreting RSolver output");
	}
	istream >> asdf;
	istream >> asdf;
	readingType = falseRegions;
      } else if ("Unknown:" == asdf) {
	readingType = unknownRegions;
      } else if (none != readingType) {
	Region newRegion;
	mpq_class prodUnknown(1);
	for (unsigned symNr(0); symNr < numSymbols; symNr++) {
	  istream >> asdf;
	  istream >> asdf;
	  istream >> asdf;
	  istream >> asdf;
	  if (0 != symNr) {
	    istream >> asdf;
	  }

	  istream >> asdf;
	  mpq_class left(rSolverDoubleToMPQ(asdf));
	  istream >> asdf;
	  istream >> asdf;
	  mpq_class right(rSolverDoubleToMPQ(asdf));
	  Interval newInterval;
	  newInterval.first = left;
	  newInterval.second = right;
	  newRegion.push_back(newInterval);
	  prodUnknown *= right - left;
	}
	if (trueRegions == readingType) {
	  known->push_back(make_tuple(expr, strict, newRegion));
	} else {
	  allTrue = false;
	  if (falseRegions == readingType) {
	    known->push_back(make_tuple(-expr, !strict, newRegion));
	  }
	}
      }
    }

    return allTrue;
  }

  double IneqChecker::convertSafe(const mpq_class &number, bool up) {
    double numberDouble(number.get_d());
    mpq_class compareNumber(numberDouble);
#if 0
    if (!up && (compareNumber > number)) {
      numberDouble = nextafter(numberDouble,
			       -numeric_limits<double>::infinity());
    } else if (up && (compareNumber < number)) {
      numberDouble = nextafter(numberDouble,
			       numeric_limits<double>::infinity());
    }
#endif

    return numberDouble;
  }

  bool IneqChecker::checkISat
  (const RationalFunction &expr, bool strict, const Region &region) {
    ofstream rstream("isat.hys", ios::out);
    rstream << "DECL\n";
    const unsigned numSymbols(RationalFunction::getNumSymbols());
    for (unsigned symNr(0); symNr < numSymbols; symNr++) {
      rstream << "float [" << setiosflags(ios::fixed) << setprecision(17)
	      << convertSafe(region[symNr].first, false)
	      << ", "
	      << convertSafe(region[symNr].second, true)
	      << resetiosflags(ios_base::floatfield) << setprecision(6)
	      << "] " << RationalFunction::getSymbolName(symNr) << ";\n";
    }

    string exprNum(expr.getNum().toString());
    string exprDen(expr.getDen().toString());
    rstream << "\nEXPR\n (";

    if (assumeNoDenomSignChange) {
      int sign;
      vector<mpq_class> point;
      for (unsigned symNr(0); symNr < numSymbols; symNr++) {
	point.push_back((region[symNr].second + region[symNr].first) / 2);
      }
      sign = (expr.getNum().evaluate(point) < 0) ? -1 : 1;
      if (-1 == sign) {
	rstream << "-";
      }
      rstream << "(" << exprNum << ") <";
      if (strict) {
	rstream << "=";
      }
      rstream << " 0";
    } else {
      rstream << "((";
      rstream << exprNum << " > 0) and (" << exprDen << " < 0)) or ((";
      rstream << exprNum << " < 0) and (" << exprDen << " > 0))";
      if (strict) {
	rstream << " or (" << exprNum << " = 0)";
      }
    }

    rstream << ")\n;\n";
    rstream.close();

 //    string callString(iSATBinary + " isat.hys --ch-sat-often --purification --nosol --msw=0.000001 --prabs=0.0000001 > isat.out");
    string callString(iSATBinary + " isat.hys > isat.out");
    // string callString(iSATBinary + " --heu=vsids isat.hys > isat.out");
    if (system(callString.c_str())) {
      throw runtime_error("Error when calling iSAT");
    }

    ifstream istream("isat.out", ios::in);
    while (!istream.eof()) {
      string asdf;
      istream >> asdf;
      if (("satisfiable" == asdf)
	  || ("unknown" == asdf)) {
	return false;
      } else if ("unsatisfiable" == asdf) {
	return true;
      }
    }
    throw runtime_error("Error when interpreting iSAT output");
  }

#if 0
  void MDPHandler::printPolyRAHD
  (const Polynomial &poly, stringstream &formula) {
    const unsigned numTerms(poly.getNumTerms());
    if (0 == numTerms) {
      formula << "0";
      return;
    }

    const mpz_t *coeff(poly.getCoefficients());
    const unsigned *monom(poly.getMonomials());
    const unsigned numSymbols(RationalFunction::getNumSymbols());
    formula << "(+ ";
    for (unsigned termNr(0); termNr < numTerms; termNr++) {
      formula << "(* ";
      formula << mpz_class(coeff[termNr]);
      for (unsigned symNr(0); symNr < numSymbols; symNr++) {
	unsigned mult(monom[termNr * numSymbols + symNr]);
	for (unsigned symCnt(0); symCnt < mult; symCnt++) {
	  formula << " " <<  RationalFunction::getSymbolName(symNr);
	}
      }

      formula << ") ";
    }
    formula << ")";
  }
#endif

  bool IneqChecker::checkRAHD(const RationalFunction &, bool, const Region &) {
    throw runtime_error("RAHD not yet supported.");
#if 0
    stringstream formula;
    formula << "(";
    const unsigned numSymbols(RationalFunction::getNumSymbols());
    for (unsigned symNr(0); symNr < numSymbols; symNr++) {
      formula << "((>= " << RationalFunction::getSymbolName(symNr)
	      << box[symNr].first << ")) "
	      << "((>= " << RationalFunction::getSymbolName(symNr)
	      << box[symNr].second << ")) \n";
    }

    formula << "(";
    set<RationalFunction,RatCmp>::iterator it;
    for (it = checkSet.begin(); it != checkSet.end(); it++) {
      const RationalFunction compareValue(*it);
      formula << "(" << (minimize ? "> " : "< " )
	      << "("<< (minimize ? "- " : "+ " )
	      << " ";
      formula << "(/ ";

      const Polynomial &num(compareValue.getNum().getPolynomial());
      const Polynomial &den(compareValue.getDen().getPolynomial());

      printPolyRAHD(num, formula);
      
      printPolyRAHD(den, formula);

      formula << ")\n ";

      formula << mpq_class(precision) << ")"
	      << " 0)";
    }
    formula << ")";
    
    formula << ")";

    //    cout << "====> \n" << formula.str() << endl;

    string callString(RAHDBinary + " -formula \"" + formula.str()
		      + "\" > rahd.out");
    if (system(callString.c_str())) {
      throw runtime_error("Error when calling RAHD");
    }

    ifstream istream("rahd.out", ios::in);
    while (!istream.eof()) {
      string asdf;
      istream >> asdf;
      if (("satisfiable" == asdf)
	  || ("unknown" == asdf)) {
	return false;
      } else if ("unsatisfiable" == asdf) {
	return true;
      }
    }
    throw runtime_error("Error when interpreting iSAT output");
#endif
  }

  bool IneqChecker::preCheck
  (const RationalFunction &expr, bool strict, const Region &region) {
    RationalFunction exprDen(expr.getDen());
    vector<mpq_class> point;
    region.getMidPoint(point);
    RationalFunction exprNum(expr.getNum());
    mpq_class evaledNum(exprNum.evaluate(point));
    if (strict && (0 == evaledNum)) {
      return false;
    }
    mpq_class evaledDen(exprDen.evaluate(point));
    if (0 != evaledDen) {
      if (((evaledNum < 0) && (evaledDen > 0))
	  || ((evaledNum > 0) && (evaledDen < 0))) {
	return false;
      }
    }

    for (unsigned edgeNr(0); edgeNr < region.getNumEdges(); edgeNr++) {
      region.getEdgePoint(edgeNr, point);
      mpq_class evaledNum(exprNum.evaluate(point));
      if (strict && (0 == evaledNum)) {
	return false;
      }
      mpq_class evaledDen(exprDen.evaluate(point));
      if (0 != evaledDen) {
	if (((evaledNum < 0) && (evaledDen > 0))
	    || ((evaledNum > 0) && (evaledDen < 0))) {
	  return false;
	}
      }
    }

    for (unsigned i(0); i < 10; i++) {
      region.getRandomPoint(point);
      mpq_class evaledNum(exprNum.evaluate(point));
      mpq_class evaledDen(exprDen.evaluate(point));
      if (((evaledNum < 0) && (evaledDen > 0))
	  || ((evaledNum > 0) && (evaledDen < 0))) {
	return false;
      }
    }

    return true;
  }
}
