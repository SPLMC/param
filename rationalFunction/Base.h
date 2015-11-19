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

#ifndef BASE_H
#define BASE_H

#include <vector>
#include <string>
#include <map>
#include <tr1/unordered_map>
#include <tr1/unordered_set>
#include <iostream>
#include <gmp.h>
#include "RationalCmp.h"
#include "PolyPairIterPairCache.h"
#include "Cancellator.h"
#include <boost/dynamic_bitset_fwd.hpp>
#include "RationalFunction.h"

namespace rational {
  class Polynomial;
  class Base;
  class ExprConverter;
  std::ostream &operator<<(std::ostream &os, const Polynomial &poly);

  int compareMonomials(const Polynomial &, const Polynomial &, unsigned, unsigned);
  
  class Base {
    friend std::ostream &operator<<(std::ostream &, const RationalFunction &);
    friend class Polynomial;
    friend class RationalFunction;
    friend class ExprConverter;
    friend RationalFunction operator+(const RationalFunction &, const RationalFunction &);
    friend RationalFunction operator-(const RationalFunction &, const RationalFunction &);
    friend RationalFunction operator*(const RationalFunction &, const RationalFunction &);
    friend RationalFunction operator/(const RationalFunction &, const RationalFunction &);
    friend RationalFunction operator-(const RationalFunction &);
    friend RationalFunction operator+=(RationalFunction &, const RationalFunction &);
    friend RationalFunction operator-=(RationalFunction &, const RationalFunction &);
    friend RationalFunction operator*=(RationalFunction &, const RationalFunction &);
    friend RationalFunction operator/=(RationalFunction &, const RationalFunction &);
    friend RationalFunction operator+=(RationalFunction &, const int);
    friend RationalFunction operator*=(RationalFunction &, const int);
    friend RationalFunction operator-=(RationalFunction &, const int);
    friend RationalFunction operator/=(RationalFunction &, const int);

  public:
    typedef std::pair<Polynomial *, Polynomial *> PolyPair;
    typedef std::map<PolyPair, unsigned, RationalCmp> PolyPairMap;
    typedef PolyPairMap::iterator PolyPairIter;
    typedef RationalFunction::CleanupMethod CleanupMethod;
    static void addSymbol(const std::string &);
    static void addNewSymbolsWhileRunning(const std::vector<std::string> &);
    static std::vector<std::string> &getSymbols();
    static unsigned getNumSymbols();
    static void setBounds(const std::string &, const mpq_class &, const mpq_class &);
    static mpq_class getBoundLeft(const std::string &);
    static mpq_class getBoundRight(const std::string &);
    static mpq_class getBoundLeft(const unsigned);
    static mpq_class getBoundRight(const unsigned);
    static void incRef(PolyPairIter);
    static void decRef(PolyPairIter);
    static void registerRational(RationalFunction *);
    static void deregisterRational(RationalFunction *);
    static void printInfo();
    static void start();
    static void clearCaches();
    static void cleanup();
    static void setCleanupMethod(CleanupMethod);
    static void evaluate(const Polynomial &,
			 const std::vector<mpq_class> &, mpq_t &);
    static mpq_class evaluate(const RationalFunction *,
			      const std::vector<mpq_class> &);
  private:
    typedef std::tr1::unordered_map<std::pair<PolyPairIter, PolyPairIter>,
      PolyPairIter,
      PolyPairIterPairCache, PolyPairIterPairCache> ValueCache;
    typedef ValueCache::iterator ValueCacheIter;
    typedef std::tr1::unordered_map<PolyPairIter, PolyPairIter,
      PolyPairIterPairCache, PolyPairIterPairCache> NegCache;
    typedef NegCache::iterator NegCacheIter;
    typedef std::tr1::unordered_set<RationalFunction *> RationalFunctions;
    typedef RationalFunctions::iterator RationalFunctionsIter;

    static void normalizeCoefficients(Polynomial *, Polynomial *);
    static void normalizeCoefficientsSign(Polynomial *, Polynomial *);
    static void normalize(Polynomial *, Polynomial *);
    static void normalize(PolyPair &);
    static PolyPairIter insertRational(PolyPair &);
    static PolyPairIter add(const RationalFunction &, const RationalFunction &);
    static PolyPairIter sub(const RationalFunction &, const RationalFunction &);
    static PolyPairIter mul(const RationalFunction &, const RationalFunction &);
    static PolyPairIter div(const RationalFunction &, const RationalFunction &);
    static PolyPairIter neg(const RationalFunction &);
    static Polynomial *mul(const Polynomial *, const Polynomial *);
    static void cleanupValueCache(ValueCache &);
    static void cleanupNegCache(NegCache &);
    static void cleanupRationals();
    static void deleteUnreferencedRationals();
    static void findUsedSymbols(const Polynomial *, boost::dynamic_bitset<> &);
    static void findUsedSymbols(boost::dynamic_bitset<> &);
    static Polynomial *removeUnusedSymbols
      (const boost::dynamic_bitset<> &, const Polynomial *, unsigned);
    static Polynomial *insertAdditionalSymbols(const Polynomial *, unsigned);
    static const std::string &getSymbolName(unsigned);

    static void removeUnusedSymbols();
    
    static std::vector<std::string> symbols;
    static PolyPairMap rationals;
    static unsigned numRatRef;
    static ValueCache addCache;
    static ValueCache subCache;
    static ValueCache mulCache;
    static ValueCache divCache;
    static NegCache negCache;
    static Cancellator *cancellator;
    static RationalFunctions rationalFunctions;
    static CleanupMethod cleanupMethod;

    static std::map<std::string, std::pair<mpq_class,mpq_class> > varBounds;
  };

  bool operator==(const Polynomial &, const Polynomial &);
}

#endif
