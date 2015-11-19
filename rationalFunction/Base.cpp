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
 * Copyright 2009-2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#include <limits>
#include <boost/dynamic_bitset.hpp>
#include "Base.h"
#include "Polynomial.h"
#include "RationalFunction.h"
#include "Geobucket.h"
#include "PolyPairIterPairCache.h"
#include "Cancellator.h"

using namespace std;
using namespace boost;

namespace rational {
  vector<std::string> Base::symbols;
  Base::PolyPairMap Base::rationals;
  unsigned Base::numRatRef(0);
  Base::ValueCache Base::addCache;
  Base::ValueCache Base::subCache;
  Base::ValueCache Base::mulCache;
  Base::ValueCache Base::divCache;
  Base::NegCache Base::negCache;
  Base::RationalFunctions Base::rationalFunctions;
  Base::CleanupMethod Base::cleanupMethod(RationalFunction::never);
  map<string, pair<mpq_class,mpq_class> > Base::varBounds;
  
  static bool operator<
  (const Base::PolyPairIter &a, const Base::PolyPairIter &b) {
    size_t aNumber = reinterpret_cast<size_t>(&*a);
    size_t bNumber = reinterpret_cast<size_t>(&*b);

    return aNumber < bNumber;
  }

  void Base::cleanup() {
    PolyPairIter iter;
    for (iter = rationals.begin(); iter != rationals.end(); iter++) {
      const PolyPair &p(iter->first);
      delete p.first;
      delete p.second;
    }
  }

  void Base::registerRational(RationalFunction *rat) {
    rationalFunctions.insert(rat);
  }

  void Base::deregisterRational(RationalFunction *rat) {
    assert(rationalFunctions.find(rat) != rationalFunctions.end());
    rationalFunctions.erase(rat);
  }

  void Base::incRef(PolyPairIter it) {
    if (0 == it->second) {
      numRatRef++;
    }
    it->second++;
  }

  void Base::decRef(PolyPairIter it) {
    assert(it != rationals.end());
    it->second--;
    if (0 == it->second) {
      numRatRef--;
    }
    //    double factor(double(numRatRef) / double(rationals.size()));
    if (rationals.size() - numRatRef > 0) {
      if (RationalFunction::always == cleanupMethod) {
        deleteUnreferencedRationals();
      } else if (RationalFunction::nocache == cleanupMethod) {
        if (0 == it->second) {
          delete it->first.first;
          delete it->first.second;
          rationals.erase(it);
        }
      }
    }
  }

  void Base::cleanupValueCache(ValueCache &cache) {
    vector<ValueCacheIter> deleteVector;
    for (ValueCacheIter it(cache.begin()); it != cache.end(); it++) {
      const pair<PolyPairIter, PolyPairIter> &operands(it->first);
      const PolyPairIter &op1(operands.first);
      const PolyPairIter &op2(operands.second);
      const PolyPairIter &result(it->second);
      if ((0 == op1->second)
          || (0 == op2->second)
          || (0 == result->second)) {
        deleteVector.push_back(it);
      }
    }
    for (unsigned itNr(0); itNr < deleteVector.size(); itNr++) {
      cache.erase(deleteVector[itNr]);
    }
  }

  void Base::cleanupNegCache(NegCache &cache) {
    vector<NegCacheIter> deleteVector;
    for (NegCacheIter it(cache.begin()); it != cache.end(); it++) {
      const PolyPairIter &op(it->first);
      const PolyPairIter &result(it->second);
      if ((0 == op->second) || (0 == result->second)) {
        deleteVector.push_back(it);
      }
    }
    for (unsigned itNr(0); itNr < deleteVector.size(); itNr++) {
      cache.erase(deleteVector[itNr]);
    }
  }

  void Base::cleanupRationals() {
    vector<PolyPairIter> deleteVector;
    for (PolyPairIter it(rationals.begin()); it != rationals.end(); it++) {
      if (0 == it->second) {
        deleteVector.push_back(it);
      }
    }
    for (unsigned itNr(0); itNr < deleteVector.size(); itNr++) {
      const PolyPairIter &it(deleteVector[itNr]);
      delete it->first.first;
      delete it->first.second;
      rationals.erase(it);
    }
  }

  void Base::deleteUnreferencedRationals() {
    cleanupValueCache(addCache);
    cleanupValueCache(subCache);
    cleanupValueCache(mulCache);
    cleanupValueCache(divCache);
    cleanupNegCache(negCache);
    cleanupRationals();
  }

  void Base::findUsedSymbols
  (const Polynomial *poly, dynamic_bitset<> &usedSymbols) {
    const unsigned numSymbols(symbols.size());
    const unsigned numTerms(poly->numTerms);
    const unsigned *monomials(poly->getMonomials());
    for (unsigned termNr(0); termNr < numTerms; termNr++) {
      for (unsigned symbolNr(0); symbolNr < numSymbols; symbolNr++) {
        if (monomials[numSymbols * termNr + symbolNr] > 0) {
          usedSymbols[symbolNr] = true;
        }
      }
    }
  }

  void Base::findUsedSymbols(dynamic_bitset<> &usedSymbols) {
    usedSymbols.clear();
    const unsigned numSymbols(symbols.size());
    usedSymbols.resize(numSymbols);
    for (PolyPairIter it(rationals.begin()); it != rationals.end(); it++) {
      const PolyPair &polyPair(it->first);
      const Polynomial *poly1(polyPair.first);
      const Polynomial *poly2(polyPair.second);
      findUsedSymbols(poly1, usedSymbols);
      findUsedSymbols(poly2, usedSymbols);
    }
  }

  Polynomial *Base::removeUnusedSymbols
  (const boost::dynamic_bitset<> & usedSymbols, const Polynomial *poly,
   unsigned newNumSymbols) {
    const unsigned numSymbols(symbols.size());
    const unsigned numTerms(poly->numTerms);
    Polynomial *result = new Polynomial();
    result->numTerms = numTerms;
    result->monomials = new unsigned[newNumSymbols * numTerms];
    result->coefficients = new mpz_t[numTerms];
    unsigned index(0);
    for (unsigned termNr(0); termNr < numTerms; termNr++) {
      mpz_init_set(result->coefficients[termNr], poly->coefficients[termNr]);
      for (unsigned symbolNr(0); symbolNr < numSymbols; symbolNr++) {
        if (usedSymbols[symbolNr]) {
          result->monomials[index]
            = poly->monomials[numSymbols * termNr + symbolNr];
          index++;
        }
      }
    }
    return result;
  }

  Polynomial *Base::insertAdditionalSymbols
  (const Polynomial *poly, unsigned newNumSymbols) {
    const unsigned numSymbols(symbols.size());
    const unsigned numTerms(poly->numTerms);
    Polynomial *result = new Polynomial();
    result->numTerms = numTerms;
    result->monomials = new unsigned[newNumSymbols * numTerms];
    result->coefficients = new mpz_t[numTerms];
    for (unsigned termNr(0); termNr < numTerms; termNr++) {
      mpz_init_set(result->coefficients[termNr], poly->coefficients[termNr]);
      for (unsigned symbolNr(0); symbolNr < numSymbols; symbolNr++) {
        result->monomials[newNumSymbols * termNr + symbolNr]
          = poly->monomials[numSymbols * termNr + symbolNr];
      }
    }
    for (unsigned termNr(0); termNr < numTerms; termNr++) {
      for (unsigned symbolNr(numSymbols); symbolNr < newNumSymbols; symbolNr++) {
        result->monomials[newNumSymbols * termNr + symbolNr] = 0;
      }
    }
    return result;
  }
  
  void Base::clearCaches() {
    addCache.clear();
    subCache.clear();
    mulCache.clear();
    divCache.clear();
    negCache.clear();
  }

  void Base::removeUnusedSymbols() {
    clearCaches();
    const unsigned numSymbols(symbols.size());
    dynamic_bitset<> usedSymbols;
    findUsedSymbols(usedSymbols);
    unsigned newNumSymbols(0);
    for (unsigned symbolNr(0); symbolNr < numSymbols; symbolNr++) {
      if (usedSymbols[symbolNr]) {
        newNumSymbols++;
      }
    }

    Cancellator::clear();
    vector<string> newSymbols;
    for (unsigned symbolNr(0); symbolNr < numSymbols; symbolNr++) {
      if (usedSymbols[symbolNr]) {
        newSymbols.push_back(symbols[symbolNr]);
        Cancellator::addSymbol(symbols[symbolNr]);
      }
    }

    std::tr1::unordered_map<PolyPairIter, PolyPairIter,
      PolyPairIterPairCache, PolyPairIterPairCache> oldToNewIter;
    
    PolyPairMap newRationals;
    for (PolyPairIter it(rationals.begin()); it != rationals.end(); it++) {
      PolyPair polyPair(it->first);
      Polynomial *poly1(polyPair.first);
      Polynomial *poly2(polyPair.second);
      Polynomial *newPoly1
        (removeUnusedSymbols(usedSymbols, poly1, newNumSymbols));
      Polynomial *newPoly2
        (removeUnusedSymbols(usedSymbols, poly2, newNumSymbols));
      delete poly1;
      delete poly2;
      polyPair.first = newPoly1;
      polyPair.second = newPoly2;
      symbols.swap(newSymbols);
      if (newRationals.find(polyPair) == newRationals.end()) {
	newRationals.insert(make_pair(polyPair, it->second));
	PolyPairIter itTo(newRationals.find(polyPair));
	oldToNewIter.insert(make_pair(it, itTo));
      } else {
	delete newPoly1;
	delete newPoly2;
      }
      symbols.swap(newSymbols);
    }

    for (RationalFunctionsIter it(rationalFunctions.begin());
         it != rationalFunctions.end(); it++) {
      (*it)->index = oldToNewIter[(*it)->index];
    }
    rationals.swap(newRationals);
    symbols.swap(newSymbols);
    Cancellator::start();
  }

  void Base::addNewSymbolsWhileRunning
  (const vector<string> &addSymbols) {
    clearCaches();
    const unsigned numSymbols(symbols.size());
    const unsigned newNumSymbols(numSymbols + addSymbols.size());

    Cancellator::clear();
    vector<string> newSymbols;
    for (unsigned symbolNr(0); symbolNr < numSymbols; symbolNr++) {
      newSymbols.push_back(symbols[symbolNr]);
      Cancellator::addSymbol(symbols[symbolNr]);
    }
    for (unsigned symbolNr(0); symbolNr < addSymbols.size(); symbolNr++) {
      newSymbols.push_back(addSymbols[symbolNr]);
      Cancellator::addSymbol(addSymbols[symbolNr]);      
    }

    std::tr1::unordered_map<PolyPairIter, PolyPairIter,
      PolyPairIterPairCache, PolyPairIterPairCache> oldToNewIter;
    
    PolyPairMap newRationals;
    for (PolyPairIter it(rationals.begin()); it != rationals.end(); it++) {
      PolyPair polyPair(it->first);
      Polynomial *poly1(polyPair.first);
      Polynomial *poly2(polyPair.second);
      Polynomial *newPoly1
        (insertAdditionalSymbols(poly1, newNumSymbols));
      Polynomial *newPoly2
        (insertAdditionalSymbols(poly2, newNumSymbols));
      delete poly1;
      delete poly2;
      polyPair.first = newPoly1;
      polyPair.second = newPoly2;
      symbols.swap(newSymbols);
      newRationals.insert(make_pair(polyPair, it->second));
      PolyPairIter itTo(newRationals.find(polyPair));
      oldToNewIter.insert(make_pair(it, itTo));
      symbols.swap(newSymbols);
    }

    for (RationalFunctionsIter it(rationalFunctions.begin());
         it != rationalFunctions.end(); it++) {
      if ((*it)->index == rationals.end()) {
        (*it)->index = newRationals.end();
      } else {
        (*it)->index = oldToNewIter[(*it)->index];
      }
    }
    rationals.swap(newRationals);
    symbols.swap(newSymbols);
    Cancellator::start();
  }

  void Base::normalize(PolyPair &value) {
    normalize(value.first, value.second);
  }

  void Base::normalize(Polynomial *nom_, Polynomial *den_) {
    normalizeCoefficients(nom_, den_);
    Cancellator::cancel(nom_, den_);
    normalizeCoefficientsSign(nom_, den_);
  }

  void Base::normalizeCoefficients(Polynomial *nom_, Polynomial *den_) {
    Polynomial &nom(*nom_);
    Polynomial &den(*den_);
    unsigned numSymbols(symbols.size());
    if (0 == nom.getNumTerms()) {
      for (unsigned termNr(0); termNr < den.numTerms; termNr++) {
	mpz_clear(den.coefficients[termNr]);
      }
      den.numTerms = 1;
      mpz_init_set_si(den.coefficients[0], 1);
      for (unsigned symbolNr(0); symbolNr < numSymbols; symbolNr++) {
        den.monomials[symbolNr] = 0;
      }
    } else {
      /* find greatest common divisor of all coefficients occuring 
       *  and find sign */
      mpz_t gcd;
      mpz_init_set(gcd, den.coefficients[0]);
      for (unsigned termNr(0); termNr < nom.getNumTerms(); termNr++) {
        mpz_gcd(gcd, gcd, nom.coefficients[termNr]);
      }
      for (unsigned termNr(0); termNr < den.getNumTerms(); termNr++) {
        mpz_gcd(gcd, gcd, den.coefficients[termNr]);
      }
      if (-1 == mpz_sgn(den.coefficients[0])) {
        mpz_neg(gcd, gcd);
      }
      
      /* divide coefficients by greatest common divisor 
       * and normalize sign */
      for (unsigned termNr(0); termNr < nom.getNumTerms(); termNr++) {
        mpz_cdiv_q(nom.coefficients[termNr], nom.coefficients[termNr], gcd);
      }
      for (unsigned termNr(0); termNr < den.getNumTerms(); termNr++) {
        mpz_cdiv_q(den.coefficients[termNr], den.coefficients[termNr], gcd);
      }
      mpz_clear(gcd);
    }
  }

  void Base::normalizeCoefficientsSign(Polynomial *nom_, Polynomial *den_) {
    Polynomial &nom(*nom_);
    Polynomial &den(*den_);
    unsigned numSymbols(symbols.size());
    if (0 == nom.getNumTerms()) {
      den.numTerms = 1;
      mpz_set_si(den.coefficients[0], 1);
      for (unsigned symbolNr(0); symbolNr < numSymbols; symbolNr++) {
        den.monomials[symbolNr] = 0;
      }
    } else {
      if (-1 == mpz_sgn(den.coefficients[0])) {
        /* normalize sign */
        for (unsigned termNr(0); termNr < nom.getNumTerms(); termNr++) {
          mpz_neg(nom.coefficients[termNr], nom.coefficients[termNr]);
        }
        for (unsigned termNr(0); termNr < den.getNumTerms(); termNr++) {
          mpz_neg(den.coefficients[termNr], den.coefficients[termNr]);
        }
      }
    }
  }

  Base::PolyPairIter Base::insertRational(PolyPair &value) {
    PolyPairIter iter(rationals.find(value));
    if (iter != rationals.end()) {
      delete value.first;
      delete value.second;
      return iter;      
    } else {
      rationals.insert(make_pair(value, 0));
      //      assert(rationals.find(value) != rationals.end());
      return rationals.find(value);
    }
  }

  Base::PolyPairIter Base::add(const RationalFunction &r1, const RationalFunction &r2) {
    PolyPairIter index1(max(r1.index, r2.index));
    PolyPairIter index2(min(r1.index, r2.index));
    pair<PolyPairIter, PolyPairIter> indexPair(make_pair(index1, index2));
    ValueCacheIter iter(addCache.find(indexPair));
    if (iter != addCache.end()) {
      return iter->second;
    } else {
      const PolyPair &p1(r1.index->first);
      const PolyPair &p2(r2.index->first);
      if (p1.second->isOne() && p2.second->isOne()) {
        Polynomial *nom(Polynomial::addPolys(p1.first, p2.first));
        Polynomial *den(p1.second->copy());
        PolyPair result(make_pair(nom, den));
        PolyPairIter resultIndex(insertRational(result));
        if (RationalFunction::nocache != cleanupMethod) {
          addCache.insert(make_pair(indexPair, resultIndex));
        }
        return resultIndex;
      } else {
        Polynomial *nom1(mul(p1.first, p2.second));
        Polynomial *nom2(mul(p2.first, p1.second));
        Polynomial *nom(Polynomial::addPolys(nom1, nom2));
        delete nom1;
        delete nom2;
        Polynomial *den(mul(p1.second, p2.second));
        PolyPair result(make_pair(nom, den));
        normalize(result);
        PolyPairIter resultIndex(insertRational(result));
        if (RationalFunction::nocache != cleanupMethod) {
          addCache.insert(make_pair(indexPair, resultIndex));
        }
        return resultIndex;
      }
    }
  }

  Base::PolyPairIter Base::sub
  (const RationalFunction &r1, const RationalFunction &r2) {
    PolyPairIter index1(r1.index);
    PolyPairIter index2(r2.index);
    pair<PolyPairIter, PolyPairIter> indexPair(make_pair(index1, index2));
    ValueCacheIter iter(subCache.find(indexPair));
    if (iter != subCache.end()) {
      return iter->second;
    } else {
      const PolyPair &p1(r1.index->first);
      const PolyPair &p2(r2.index->first);
      if (p1.second->isOne() && p2.second->isOne()) {
        Polynomial *nom(Polynomial::subPolys(p1.first, p2.first));
        Polynomial *den(p1.second->copy());
        PolyPair result(make_pair(nom, den));
        PolyPairIter resultIndex(insertRational(result));
        if (RationalFunction::nocache != cleanupMethod) {
          subCache.insert(make_pair(indexPair, resultIndex));
        }

        return resultIndex;
      } else {
        Polynomial *nom1(mul(p1.first, p2.second));
        Polynomial *nom2(mul(p2.first, p1.second));
        Polynomial *nom(Polynomial::subPolys(nom1, nom2));
        delete nom1;
        delete nom2;
        Polynomial *den(mul(p1.second, p2.second));
        PolyPair result(make_pair(nom, den));
        normalize(result);
        PolyPairIter resultIndex(insertRational(result));
        if (RationalFunction::nocache != cleanupMethod) {
          subCache.insert(make_pair(indexPair, resultIndex));
        }
        
        return resultIndex;
      }
    }
  }

  Polynomial *Base::mul(const Polynomial *p1, const Polynomial *p2) {
    Geobucket bucket;
    unsigned numTerms(p2->getNumTerms());
    for (unsigned termNr(0); termNr < numTerms; termNr++) {
      Polynomial *multiplied(Polynomial::multByTerm(p1, p2, termNr));
      bucket.add(multiplied);
    }
    Polynomial *result(bucket.canonicalize());
    
    return result;
  }

  Base::PolyPairIter Base::mul
  (const RationalFunction &r1, const RationalFunction &r2) {
    PolyPairIter index1(max(r1.index, r2.index));
    PolyPairIter index2(min(r1.index, r2.index));
    pair<PolyPairIter, PolyPairIter> indexPair(make_pair(index1, index2));
    ValueCacheIter iter(mulCache.find(indexPair));
    if (iter != mulCache.end()) {
      return iter->second;
    } else {
      const PolyPair &p1(r1.index->first);
      const PolyPair &p2(r2.index->first);
      if (p1.second->isOne() && p2.second->isOne()) {
        Polynomial *nom(mul(p1.first, p2.first));
        Polynomial *den(p1.second->copy());
        PolyPair result(make_pair(nom, den));
        PolyPairIter resultIndex(insertRational(result));
        if (RationalFunction::nocache != cleanupMethod) {
          mulCache.insert(make_pair(indexPair, resultIndex));
        }
        return resultIndex;
      } else {
        Polynomial *p1nom(p1.first->copy());
        Polynomial *p1den(p1.second->copy());
        Polynomial *p2nom(p2.first->copy());
        Polynomial *p2den(p2.second->copy());
        normalize(p1nom, p2den);
        normalize(p2nom, p1den);

        Polynomial *nom(mul(p1nom, p2nom));        
        Polynomial *den(mul(p1den, p2den));
        delete p1nom;
        delete p1den;
        delete p2nom;
        delete p2den;
        normalizeCoefficientsSign(nom, den);
        PolyPair result(make_pair(nom, den));

        PolyPairIter resultIndex(insertRational(result));
        if (RationalFunction::nocache != cleanupMethod) {
          mulCache.insert(make_pair(indexPair, resultIndex));
        }
        return resultIndex;
      }
    }
  }

  Base::PolyPairIter Base::div
  (const RationalFunction &r1, const RationalFunction &r2) {
    PolyPairIter index1(r1.index);
    PolyPairIter index2(r2.index);
    pair<PolyPairIter, PolyPairIter> indexPair(make_pair(index1, index2));
    ValueCacheIter iter(divCache.find(indexPair));
    if (iter != divCache.end()) {
      return iter->second;
    } else {
      const PolyPair &p1(r1.index->first);
      const PolyPair &p2(r2.index->first);
      if (p2.first->isOne() && p2.second->isOne()) {
        if (RationalFunction::nocache != cleanupMethod) {
          divCache.insert(make_pair(indexPair, index1));
        }
        return index1;
      } else {
        Polynomial *p1nom(new Polynomial(*p1.first));
        Polynomial *p1den = p1.second->copy();
        Polynomial *p2nom = p2.first->copy();
        Polynomial *p2den = p2.second->copy();
        normalize(p1nom, p2nom);
        normalize(p1den, p2den);

        Polynomial *nom(mul(p1nom, p2den));
        Polynomial *den(mul(p1den, p2nom));
        delete p1nom;
        delete p1den;
        delete p2nom;
        delete p2den;
	normalizeCoefficientsSign(nom, den);
        PolyPair result(make_pair(nom, den));

        PolyPairIter resultIndex(insertRational(result));
        if (RationalFunction::nocache != cleanupMethod) {
          divCache.insert(make_pair(indexPair, resultIndex));
        }
        return resultIndex;
      }
    }
  }

  Base::PolyPairIter Base::neg(const RationalFunction &r) {
    PolyPairIter index(r.index);
    NegCache::iterator iter(negCache.find(index));
    if (iter != negCache.end()) {
      return iter->second;
    } else {
      const PolyPair &p(r.index->first);
      Polynomial *nom(p.first->negPoly());
      Polynomial *den(p.second->copy());
      PolyPair result(make_pair(nom, den));
      PolyPairIter resultIndex(insertRational(result));
      if (RationalFunction::nocache != cleanupMethod) {
        negCache.insert(make_pair(index, resultIndex));
      }
      
      return resultIndex;
    }
  }

  void Base::printInfo() {
    cout << "Symbols:\n";
    for (unsigned symbolNr(0); symbolNr < symbols.size(); symbolNr++) {
      cout << "  " << symbols[symbolNr] << "\n";
    }
    cout << "Polynomials:\n";
#if 0
    for (unsigned polyNr(0); polyNr < rationals.size(); polyNr++) {
      //      cout << "  " << *rationals[polyNr] << "\n";
    }
#endif
    cout << endl;
  }

  void Base::start() {
    Cancellator::start();
  }

  void Base::setCleanupMethod(CleanupMethod method) {
    cleanupMethod = method;
    if (RationalFunction::always == method) {
      deleteUnreferencedRationals();      
    } else if (RationalFunction::nocache == method) {
      deleteUnreferencedRationals();
      clearCaches();
    }
  }

  void Base::evaluate(const Polynomial &poly,
		      const vector<mpq_class> &values,
		      mpq_t &result) {
    mpq_init(result);
    mpq_t termVal;
    mpq_init(termVal);
    mpq_t symVal;
    mpq_init(symVal);
    const unsigned numSymbols(Base::getNumSymbols());
    const mpz_t *coeff(poly.coefficients);
    const unsigned *monom(poly.monomials);
    const unsigned numTerms(poly.getNumTerms());
    for (unsigned termNr(0); termNr < numTerms; termNr++) {
      mpq_set_num(termVal, coeff[termNr]);
      mpz_set_ui(mpq_denref(termVal), 1);
      for (unsigned symbolNr(0); symbolNr < numSymbols; symbolNr++) {
	mpq_set(symVal, values[symbolNr].get_mpq_t());
	const unsigned power(monom[numSymbols * termNr + symbolNr]);
	mpz_pow_ui(mpq_numref(symVal), mpq_numref(symVal), power);
	mpz_pow_ui(mpq_denref(symVal), mpq_denref(symVal), power);
	mpq_mul(termVal, termVal, symVal);
      }
      mpq_add(result, result, termVal);
    }
    mpq_clear(termVal);
    mpq_clear(symVal);
  }

  mpq_class Base::evaluate(const RationalFunction *rat,
			   const vector<mpq_class> &values) {
    const PolyPair &ppair(rat->index->first);
    mpq_t numEval;
    evaluate(*ppair.first, values, numEval);
    mpq_t denEval;
    evaluate(*ppair.second, values, denEval);
    mpq_t resultFrac;
    mpq_init(resultFrac);
    mpq_div(resultFrac, numEval, denEval);
    mpq_clear(numEval);
    mpq_clear(denEval);
    mpq_class result(resultFrac);
    mpq_clear(resultFrac);

    return result;
  }

  const std::string &Base::getSymbolName(unsigned symNr) {
    return symbols[symNr];
  }

  void Base::addSymbol(const string &symbol) {
    symbols.push_back(symbol);
    cancellator->addSymbol(symbol);
  }

  vector<std::string> &Base::getSymbols() {
    return symbols;
  }

  unsigned Base::getNumSymbols() {
    return symbols.size();
  }

  void Base::setBounds
  (const string &var, const mpq_class &left, const mpq_class &right) {
    assert(0 == varBounds.count(var));
    varBounds.insert(make_pair(var, make_pair(left, right)));
  }


  mpq_class Base::getBoundLeft(const string &var) {
    assert(0 != varBounds.count(var));
    return varBounds[var].first;
  }

  mpq_class Base::getBoundRight(const string &var) {
    assert(0 != varBounds.count(var));
    return varBounds[var].second;
  }

  mpq_class Base::getBoundLeft(const unsigned var) {
    return varBounds[symbols[var]].first;
  }

  mpq_class Base::getBoundRight(const unsigned var) {
    return varBounds[symbols[var]].second;
  }
}
