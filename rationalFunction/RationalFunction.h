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
 * Copyright 2009-2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#ifndef RATIONAL_FUNCTION_H
#define RATIONAL_FUNCTION_H

#include <iostream>
#include <vector>
#include <map>
#include <tr1/unordered_map>
#include <gmpxx.h>
#include "RationalCmp.h"

namespace rational {
  class RationalFunction;
  class Base;

  RationalFunction operator+
    (const RationalFunction &, const RationalFunction &);
  RationalFunction operator-
    (const RationalFunction &, const RationalFunction &);
  RationalFunction operator-
    (const RationalFunction &);
  RationalFunction operator*
    (const RationalFunction &, const RationalFunction &);
  RationalFunction operator/
    (const RationalFunction &, const RationalFunction &);
  bool operator==(const RationalFunction &, const RationalFunction &);
  bool operator!=(const RationalFunction &, const RationalFunction &);
  RationalFunction operator+=
    (RationalFunction &, const RationalFunction &);
  RationalFunction operator-=
    (RationalFunction &, const RationalFunction &);
  RationalFunction operator*=
    (RationalFunction &, const RationalFunction &);
  RationalFunction operator/=
    (RationalFunction &, const RationalFunction &);
  RationalFunction operator+=
    (RationalFunction &, const int);

  class RationalFunction {
    friend std::ostream &operator<<(std::ostream &, const RationalFunction &);
    friend RationalFunction operator+
      (const RationalFunction &, const RationalFunction &);
    friend RationalFunction operator-
      (const RationalFunction &, const RationalFunction &);
    friend RationalFunction operator*
      (const RationalFunction &, const RationalFunction &);
    friend RationalFunction operator/
      (const RationalFunction &, const RationalFunction &);
    friend RationalFunction operator-
      (const RationalFunction &);
    friend RationalFunction operator+=
      (RationalFunction &, const RationalFunction &);
    friend RationalFunction operator-=
      (RationalFunction &, const RationalFunction &);
    friend RationalFunction operator*=
      (RationalFunction &, const RationalFunction &);
    friend RationalFunction operator/=
      (RationalFunction &, const RationalFunction &);
    friend RationalFunction operator+=(RationalFunction &, const int);
    friend RationalFunction operator-=(RationalFunction &, const int);
    friend RationalFunction operator*=(RationalFunction &, const int);
    friend RationalFunction operator/=(RationalFunction &, const int);
    friend bool operator==
      (const RationalFunction &, const RationalFunction &);
    friend bool operator!=
      (const RationalFunction &, const RationalFunction &);
    friend class Base;
  public:
    typedef std::pair<Polynomial *, Polynomial *> PolyPair;
    typedef std::map<PolyPair, unsigned, RationalCmp> PolyPairMap;
    typedef PolyPairMap::iterator PolyPairIter;
    enum CleanupMethod {never, always, nocache};

    RationalFunction();
    RationalFunction(int, std::vector<unsigned>);
    RationalFunction(int);
    //    RationalFunction(double);
    RationalFunction(const RationalFunction &);
    RationalFunction(Polynomial *, Polynomial *);
    ~RationalFunction();
    RationalFunction getNum() const;
    RationalFunction getDen() const;
    RationalFunction &operator=(const RationalFunction &);
    inline PolyPairIter getIndex() const {
      return index;
    }
    mpq_class evaluate(const std::vector<mpq_class> &) const;
    mpq_class toMPQ() const;
    double toDouble() const;
    bool isInt() const;
    int toInt() const;
    int getIntNum() const;
    int getIntDen() const;
    std::string toString() const;
    const Polynomial &getPolynomial() const;
    static unsigned getNumSymbols();
    static void addSymbol(const std::string &);
    static void setBounds(const std::string &, const mpq_class &, const mpq_class &);
    static mpq_class getBoundLeft(const std::string &);
    static mpq_class getBoundRight(const std::string &);
    static mpq_class getBoundLeft(const unsigned);
    static mpq_class getBoundRight(const unsigned);
    static const std::string &getSymbolName(unsigned);
    static void addNewSymbolsWhileRunning(const std::vector<std::string> &);
    static void start();
    static void cleanup();
    static void removeUnusedSymbols();
    static void setCleanupMethod(CleanupMethod);
  private:
    PolyPairIter index;
  };

  inline bool operator==
    (const RationalFunction &r1, const RationalFunction &r2) {
    return r1.index == r2.index;
  }

  inline bool operator!=
    (const RationalFunction &r1, const RationalFunction &r2) {
    return r1.index != r2.index;
  }
}

namespace std {
  namespace tr1 {
    template<>
      inline size_t hash<rational::RationalFunction>::operator()
      (rational::RationalFunction r) const {
      rational::RationalFunction::PolyPairIter index(r.getIndex());
      size_t indexUnsigned(reinterpret_cast<size_t>(&*index));
      return std::tr1::hash<size_t>()(indexUnsigned);
    }

    template<>
      inline size_t hash<std::pair<rational::RationalFunction, rational::RationalFunction> >::operator()
      (std::pair<rational::RationalFunction, rational::RationalFunction> r) const {
      return std::tr1::hash<rational::RationalFunction>()(r.first)
        + std::tr1::hash<rational::RationalFunction>()(r.second);
    }
  }
}

#endif
