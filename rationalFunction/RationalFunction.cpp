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
 * along with PARAM.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2009-2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#include <sstream>
#include <assert.h>
#include "RationalFunction.h"
#include "Base.h"
#include "Polynomial.h"
#include "Cancellator.h"

using namespace std;

namespace rational {
  void RationalFunction::start() {
    Base::start();
  }

  void RationalFunction::cleanup() {
    Base::cleanup();
  }

  void RationalFunction::removeUnusedSymbols() {
    Base::removeUnusedSymbols();
  }

  void RationalFunction::addSymbol(const string &symbol) {
    Base::addSymbol(symbol);
  }

  void RationalFunction::setBounds
  (const string &var, const mpq_class &left, const mpq_class &right) {
    Base::setBounds(var, left, right);
  }

  mpq_class RationalFunction::getBoundLeft(const string &var) {
    return Base::getBoundLeft(var);
  }

  mpq_class RationalFunction::getBoundRight(const string &var) {
    return Base::getBoundRight(var);
  }

  mpq_class RationalFunction::getBoundLeft(const unsigned var) {
    return Base::getBoundLeft(var);
  }

  mpq_class RationalFunction::getBoundRight(const unsigned var) {
    return Base::getBoundRight(var);
  }

  void RationalFunction::addNewSymbolsWhileRunning
  (const vector<string> &symbols) {
    Base::addNewSymbolsWhileRunning(symbols);
  }

  unsigned RationalFunction::getNumSymbols() {
    return Base::getNumSymbols();
  }

  RationalFunction::RationalFunction() {
    Polynomial *nom(new Polynomial(0));
    Polynomial *den(new Polynomial(1));
    mpz_set_si(den->coefficients[0], 1);
    for (unsigned i = 0; i < Base::getNumSymbols(); i++) {
      den->monomials[i] = 0;
    }
    Base::PolyPair rational(make_pair(nom, den));
    index = Base::insertRational(rational);
    Base::registerRational(this);
    Base::incRef(index);
  }

  RationalFunction::RationalFunction(Polynomial *nom, Polynomial *den) {
    Cancellator::cancel(nom, den);
    Base::PolyPair rational(make_pair(nom, den));
    index = Base::insertRational(rational);
    Base::registerRational(this);
    Base::incRef(index);
  }

  RationalFunction::RationalFunction(const RationalFunction &rat) {
    index = rat.index;
    Base::registerRational(this);
    Base::incRef(index);
  }

  RationalFunction &RationalFunction::operator=
  (const RationalFunction &rat) {
    Base::incRef(rat.index);
    Base::decRef(index);
    index = rat.index;

    return *this;
  }

  RationalFunction::RationalFunction
  (int coefficient, vector<unsigned> monomial) {
    Polynomial *nom;
    if (0 == coefficient) {
      nom = new Polynomial(0);
    } else {
      nom = new Polynomial(1);
      mpz_set_si(nom->coefficients[0], coefficient);
      for (unsigned i = 0; i < monomial.size(); i++) {
        nom->monomials[i] = monomial[i];
      }
    }
    Polynomial *den = new Polynomial(1);
    mpz_set_si(den->coefficients[0], 1);
    for (unsigned i = 0; i < monomial.size(); i++) {
      den->monomials[i] = 0;
    }
    Base::PolyPair rational(make_pair(nom, den));
    index = Base::insertRational(rational);
    Base::registerRational(this);
    Base::incRef(index);
  }

  RationalFunction::RationalFunction(const int coefficient) {
    Polynomial *nom;
    if (0 == coefficient) {
      nom = new Polynomial(0);
    } else {
      nom = new Polynomial(1);
      mpz_set_si(nom->coefficients[0], coefficient);
      for (unsigned i = 0; i < Base::getNumSymbols(); i++) {
        nom->monomials[i] = 0;
      }
    }
    Polynomial *den = new Polynomial(1);
    mpz_set_si(den->coefficients[0], 1);
    for (unsigned i = 0; i < Base::getNumSymbols(); i++) {
      den->monomials[i] = 0;
    }
    Base::PolyPair rational(make_pair(nom, den));
    index = Base::insertRational(rational);
    Base::registerRational(this);
    Base::incRef(index);
  }

  string RationalFunction::toString() const {
    stringstream stream;
    stream << *this;
    return stream.str();
  }

#if 0
  RationalFunction::RationalFunction(const double val) {
    mpq_t valQ;
    mpq_init(valQ);
    mpq_set_d(valQ, val);

    Polynomial *nom;
    if (0.0 == val) {
      nom = new Polynomial(0);
    } else {
      nom = new Polynomial(1);
      mpz_set(nom->coefficients[0], mpq_numref(valQ));
      for (unsigned i = 0; i < Base::getNumSymbols(); i++) {
        nom->monomials[i] = 0;
      }
    }
    Polynomial *den = new Polynomial(1);
    mpz_set(den->coefficients[0], mpq_denref(valQ));
    for (unsigned i = 0; i < Base::getNumSymbols(); i++) {
      den->monomials[i] = 0;
    }
    Base::PolyPair rational(make_pair(nom, den));
    index = Base::insertRational(rational);
    Base::registerRational(this);
    Base::incRef(index);
    mpq_clear(valQ);
  }
#endif

  mpq_class RationalFunction::evaluate(const vector<mpq_class> &values) const {
    return Base::evaluate(this, values);
  }

  mpq_class RationalFunction::toMPQ() const {
    vector<mpq_class> values(Base::getNumSymbols());
    return Base::evaluate(this, values);
  }

  bool RationalFunction::isInt() const {
    const unsigned numSymbols(Base::getNumSymbols());
    const PolyPair &ppair(this->index->first);
    const Polynomial &num(*ppair.first);
    const Polynomial &den(*ppair.second);
    if (1 != den.getNumTerms()) {
      return false;
    } else if (1 != mpz_get_d(den.coefficients[0])) {
      return false;
    } else if ((0 != num.getNumTerms()) && (1 != num.getNumTerms())) {
      return false;
    } else {
      for (unsigned sym = 0; sym < numSymbols; sym++) {
	if (0 != den.monomials[sym]) {
	  return false;
	}
      }
      if (1 == num.getNumTerms()) {
	for (unsigned sym = 0; sym < numSymbols; sym++) {
	  if (0 != num.monomials[sym]) {
	    return false;
	  }
	}
      }
    }

    return true;
  }

  int RationalFunction::toInt() const {
    return getIntNum();
  }

  int RationalFunction::getIntNum() const {
    const PolyPair &ppair(this->index->first);
    const Polynomial &num(*ppair.first);
    if (0 == num.getNumTerms()) {
      return 0;
    } else {
      return mpz_get_d(num.coefficients[0]);
    }
  }

  int RationalFunction::getIntDen() const {
    const PolyPair &ppair(this->index->first);
    const Polynomial &den(*ppair.second);
    if (0 == den.getNumTerms()) {
      return 0;
    } else {
      return mpz_get_d(den.coefficients[0]);
    }
  }

  double RationalFunction::toDouble() const {
    const PolyPair &ppair(this->index->first);
    const Polynomial &num(*ppair.first);
    const Polynomial &den(*ppair.second);
    if (0 == num.getNumTerms()) {
      return 0;
    } else {
      return mpz_get_d(num.coefficients[0]) / mpz_get_d(den.coefficients[0]);
    }
  }

  RationalFunction RationalFunction::getNum() const {
    Polynomial *den(new Polynomial(1));
    mpz_set_si(den->coefficients[0], 1);
    for (unsigned i = 0; i < Base::getNumSymbols(); i++) {
      den->monomials[i] = 0;
    }
    Polynomial *nom((index->first.first)->copy());

    Base::PolyPair rational(make_pair(nom, den));
    RationalFunction result;
    result.index = Base::insertRational(rational);    
    Base::registerRational(&result);
    Base::incRef(result.index);
    return result;
  }

  RationalFunction RationalFunction::getDen() const {
    Polynomial *den(new Polynomial(1));
    mpz_set_si(den->coefficients[0], 1);
    for (unsigned i = 0; i < Base::getNumSymbols(); i++) {
      den->monomials[i] = 0;
    }
    Polynomial *nom((index->first.second)->copy());

    Base::PolyPair rational(make_pair(nom, den));
    RationalFunction result;
    result.index = Base::insertRational(rational);    
    Base::registerRational(&result);
    Base::incRef(result.index);
    return result;
  }

  RationalFunction::~RationalFunction() {
    Base::decRef(index);
    Base::deregisterRational(this);
  }

  RationalFunction operator+
  (const RationalFunction &r1, const RationalFunction &r2) {
    RationalFunction::PolyPairIter number(Base::add(r1, r2));
    RationalFunction result;
    Base::incRef(number);
    result.index = number;
    return result;
  }
  
  RationalFunction operator-
  (const RationalFunction &r1, const RationalFunction &r2) {
    RationalFunction::PolyPairIter number(Base::sub(r1, r2));
    RationalFunction result;
    Base::incRef(number);
    result.index = number;
    return result;
  }
  
  RationalFunction operator*
  (const RationalFunction &r1, const RationalFunction &r2) {
    RationalFunction::PolyPairIter number(Base::mul(r1, r2));
    RationalFunction result;
    Base::incRef(number);
    result.index = number;
    return result;
  }

  RationalFunction operator/
  (const RationalFunction &r1, const RationalFunction &r2) {
    RationalFunction::PolyPairIter number(Base::div(r1, r2));
    RationalFunction result;
    Base::incRef(number);
    result.index = number;
    return result;
  }

  RationalFunction operator-
  (const RationalFunction &r1) {
    RationalFunction::PolyPairIter number(Base::neg(r1));
    RationalFunction result;
    Base::incRef(number);
    result.index = number;
    return result;
  }

  RationalFunction operator+=
  (RationalFunction &r1, const RationalFunction &r2) {
    RationalFunction::PolyPairIter number(Base::add(r1, r2));
    Base::incRef(number);
    Base::decRef(r1.index);
    r1.index = number;

    return r1;
  }

  RationalFunction operator-=
  (RationalFunction &r1, const RationalFunction &r2) {
    RationalFunction::PolyPairIter number(Base::sub(r1, r2));
    Base::incRef(number);
    Base::decRef(r1.index);
    r1.index = number;

    return r1;
  }

  RationalFunction operator*=
  (RationalFunction &r1, const RationalFunction &r2) {
    RationalFunction::PolyPairIter number(Base::mul(r1, r2));
    Base::incRef(number);
    Base::decRef(r1.index);
    r1.index = number;

    return r1;
  }

  RationalFunction operator/=
  (RationalFunction &r1, const RationalFunction &r2) {
    RationalFunction::PolyPairIter number(Base::div(r1, r2));
    Base::incRef(number);
    Base::decRef(r1.index);
    r1.index = number;

    return r1;
  }

  RationalFunction operator+=(RationalFunction &r1, const int i2) {
    RationalFunction r2(i2);
    RationalFunction::PolyPairIter number(Base::add(r1, r2));
    Base::incRef(number);
    Base::decRef(r1.index);
    r1.index = number;

    return r1;
  }

  RationalFunction operator-=(RationalFunction &r1, const int i2) {
    RationalFunction r2(i2);
    RationalFunction::PolyPairIter number(Base::sub(r1, r2));
    Base::incRef(number);
    Base::decRef(r1.index);
    r1.index = number;

    return r1;
  }

  RationalFunction operator*=(RationalFunction &r1, const int i2) {
    RationalFunction r2(i2);
    RationalFunction::PolyPairIter number(Base::mul(r1, r2));
    Base::incRef(number);
    Base::decRef(r1.index);
    r1.index = number;

    return r1;
  }

  RationalFunction operator/=(RationalFunction &r1, const int i2) {
    RationalFunction r2(i2);
    RationalFunction::PolyPairIter number(Base::div(r1, r2));
    Base::incRef(number);
    Base::decRef(r1.index);
    r1.index = number;

    return r1;
  }

  void RationalFunction::setCleanupMethod(CleanupMethod method) {
    Base::setCleanupMethod(method);
  }

  const std::string &RationalFunction::getSymbolName
  (unsigned symNr) {
    return Base::getSymbolName(symNr);
  }

  const Polynomial &RationalFunction::getPolynomial() const {
    return *index->first.first;
  }

  ostream &operator<<(ostream &os, const RationalFunction &rat) {
    Polynomial &den(*(rat.index->first.second));
    const mpz_t *coefficients(den.getCoefficients());
    const unsigned *monomials(den.getMonomials());

    bool isPoly(true);
    if (den.getNumTerms() > 1) {
      isPoly = false;
    }
    if (0 != mpz_cmp_si(coefficients[0], 1)) {
      isPoly = false;
    } else {
      for (unsigned symbolNr(0); symbolNr < Base::getNumSymbols(); symbolNr++) {
        if (0 != monomials[symbolNr]) {
          isPoly = false;
          break;
        }
      }
    }
    
    if (isPoly) {
      os << *(rat.index->first.first);
    } else {
      os << "(" << *(rat.index->first.first)
         << ") / (" << *(rat.index->first.second)
         << ")";
    }

    return os;
  }
}
