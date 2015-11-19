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

#include <string>
#include <vector>
#include <ostream>
#include "Polynomial.h"
#include "Base.h"

using namespace std;

namespace rational {
  std::ostream &operator<<(std::ostream &os, const Polynomial &poly) {
    const mpz_t *coefficients(poly.getCoefficients());
    const unsigned *monomials(poly.getMonomials());
    const std::vector<std::string> &symbols(Base::getSymbols());
    for (unsigned termNr = 0; termNr < poly.getNumTerms(); termNr++) {
      char *numberStr =  mpz_get_str(NULL, 10, coefficients[termNr]);
      if ((1 == mpz_sgn(coefficients[termNr]))
	  && (0 != termNr)) {
        os << "+";
      }
      os << std::string(numberStr);
      free(numberStr);
      for (unsigned symbolNr = 0; symbolNr < symbols.size(); symbolNr++) {
        const unsigned monomVal(monomials[termNr * symbols.size() + symbolNr]);
        if (0 != monomVal) {
	  os << "*";
          os << symbols[symbolNr];
          if (1 != monomVal) {
            os << "^" << monomVal;
          }
        }
      }
      
      if (termNr + 1 < poly.getNumTerms()) {
        os << " ";
      }
    }
    if (0 == poly.getNumTerms()) {
      os << "0";
    }
    return os;
  }

  Polynomial::Polynomial(unsigned size) {
    monomials = new unsigned[size * Base::getNumSymbols()];
    coefficients = new mpz_t[size];
    numTerms = size;
    for (unsigned termNr = 0; termNr < numTerms; termNr++) {
      mpz_init(coefficients[termNr]);
    }
  }

  Polynomial::Polynomial() {
    monomials = NULL;
    coefficients = NULL;
    numTerms = 0;
  }

  Polynomial::Polynomial(const Polynomial &copy) {
    numTerms = copy.numTerms;
    monomials = new unsigned[numTerms * Base::getNumSymbols()];
    coefficients = new mpz_t[numTerms];
    for (unsigned termNr = 0; termNr < numTerms; termNr++) {
      mpz_init_set(coefficients[termNr], copy.coefficients[termNr]);
      for (unsigned symbolNr = 0; symbolNr < Base::getNumSymbols(); symbolNr++) {
        monomials[termNr * Base::getNumSymbols() + symbolNr] =
          copy.monomials[termNr * Base::getNumSymbols() + symbolNr];
      }
    }
  }

  Polynomial::~Polynomial() {
    for (unsigned termNr = 0; termNr < numTerms; termNr++) {
      mpz_clear(coefficients[termNr]);
    }
    if (NULL != monomials) {
      delete[] monomials;
    }
    if (NULL != coefficients) {
      delete[] coefficients;
    }
  }

  unsigned Polynomial::computeResultSizeAdd
  (const Polynomial *p1, const Polynomial *p2) {
    unsigned term1Nr(0);
    unsigned term2Nr(0);
    unsigned resultSize(0);
    const mpz_t *p1Coeff(p1->coefficients);
    const mpz_t *p2Coeff(p2->coefficients);
    while ((term1Nr < p1->getNumTerms())
           && (term2Nr < p2->getNumTerms())) {
      int monomCmp(compareMonomials(p1, p2, term1Nr, term2Nr));
      if (0 == monomCmp) {
        if ((0 == mpz_cmp(p1Coeff[term1Nr], p2Coeff[term2Nr]))
            || (0 != mpz_cmpabs(p1Coeff[term1Nr], p2Coeff[term2Nr]))) {
          resultSize++;
        }
        term1Nr++;
        term2Nr++;
      } else if (1 == monomCmp) {
        resultSize++;
        term1Nr++;
      } else if (-1 == monomCmp) {
        resultSize++;
        term2Nr++;
      }
    }

    resultSize += p2->getNumTerms() - term2Nr;
    resultSize += p1->getNumTerms() - term1Nr;

    return resultSize;
  }

  unsigned Polynomial::computeResultSizeSub
  (const Polynomial *p1, const Polynomial *p2) {
    unsigned term1Nr(0);
    unsigned term2Nr(0);
    unsigned resultSize(0);
    const mpz_t *p1Coeff(p1->coefficients);
    const mpz_t *p2Coeff(p2->coefficients);
    while ((term1Nr < p1->getNumTerms())
           && (term2Nr < p2->getNumTerms())) {
      int monomCmp(compareMonomials(p1, p2, term1Nr, term2Nr));
      if (0 == monomCmp) {
        if (0 != mpz_cmp(p1Coeff[term1Nr], p2Coeff[term2Nr])) {
          resultSize++;
        }
        term1Nr++;
        term2Nr++;
      } else if (1 == monomCmp) {
        resultSize++;
        term1Nr++;
      } else if (-1 == monomCmp) {
        resultSize++;
        term2Nr++;
      }
    }

    resultSize += p2->getNumTerms() - term2Nr;
    resultSize += p1->getNumTerms() - term1Nr;

    return resultSize;
  }

  Polynomial *Polynomial::addPolys(const Polynomial *p1, const Polynomial *p2) {
    unsigned resultSize(computeResultSizeAdd(p1, p2));
    unsigned term1Nr(0);
    unsigned term2Nr(0);
    const unsigned numSymbols(Base::getNumSymbols());

    /* allocate result and compute it */
    Polynomial *result = new Polynomial(resultSize+1);
    mpz_t *resCoeff(result->coefficients);
    const mpz_t *p1Coeff(p1->coefficients);
    const mpz_t *p2Coeff(p2->coefficients);
    unsigned *resMonom(result->monomials);
    const unsigned *p1Monom(p1->monomials);
    const unsigned *p2Monom(p2->monomials);
    unsigned resultIndex(0);
    while ((term1Nr < p1->getNumTerms()) && (term2Nr < p2->getNumTerms())) {
      int monomCmp(compareMonomials(p1, p2, term1Nr, term2Nr));
      if (0 == monomCmp) {
        mpz_add(resCoeff[resultIndex], p1Coeff[term1Nr], p2Coeff[term2Nr]);
        if (0 != mpz_sgn(resCoeff[resultIndex])) {
          for (unsigned symbolNr = 0; symbolNr < numSymbols; symbolNr++) {
            resMonom[numSymbols * resultIndex + symbolNr] =
              p1Monom[numSymbols * term1Nr + symbolNr];
          }
          resultIndex++;
        }
        term1Nr++;
        term2Nr++;
      } else if (1 == monomCmp) {
        mpz_set(resCoeff[resultIndex], p1Coeff[term1Nr]);
        for (unsigned symbolNr = 0; symbolNr < numSymbols; symbolNr++) {
          resMonom[numSymbols * resultIndex + symbolNr] =
            p1Monom[numSymbols * term1Nr + symbolNr];
        }
        resultIndex++;
        term1Nr++;
      } else if (-1 == monomCmp) {
        mpz_set(resCoeff[resultIndex], p2Coeff[term2Nr]);
        for (unsigned symbolNr = 0; symbolNr < numSymbols; symbolNr++) {
          resMonom[numSymbols * resultIndex + symbolNr] =
            p2Monom[numSymbols * term2Nr + symbolNr];
        }
        resultIndex++;
        term2Nr++;
      }
    }
    
    while (term1Nr < p1->getNumTerms()) {
      mpz_set(resCoeff[resultIndex], p1Coeff[term1Nr]);
      for (unsigned symbolNr = 0; symbolNr < numSymbols; symbolNr++) {
        resMonom[numSymbols * resultIndex + symbolNr] =
          p1Monom[numSymbols * term1Nr + symbolNr];
      }
      resultIndex++;
      term1Nr++;    
    }
    
    while (term2Nr < p2->getNumTerms()) {
      mpz_set(resCoeff[resultIndex], p2Coeff[term2Nr]);
      for (unsigned symbolNr = 0; symbolNr < numSymbols; symbolNr++) {
        resMonom[numSymbols * resultIndex + symbolNr] =
          p2Monom[numSymbols * term2Nr + symbolNr];
      }
      resultIndex++;
      term2Nr++;
    }
    mpz_clear(resCoeff[resultSize]);
    result->numTerms = resultIndex;

    return result;
  }

  Polynomial *Polynomial::subPolys(const Polynomial *p1, const Polynomial *p2) {
    unsigned resultSize(computeResultSizeSub(p1, p2));
    unsigned term1Nr(0);
    unsigned term2Nr(0);
    const unsigned numSymbols(Base::getNumSymbols());

    /* allocate result and compute it */
    Polynomial *result = new Polynomial(resultSize+1);
    mpz_t *resCoeff(result->coefficients);
    const mpz_t *p1Coeff(p1->coefficients);
    const mpz_t *p2Coeff(p2->coefficients);
    unsigned *resMonom(result->monomials);
    const unsigned *p1Monom(p1->monomials);
    const unsigned *p2Monom(p2->monomials);
    unsigned resultIndex(0);
    while ((term1Nr < p1->getNumTerms()) && (term2Nr < p2->getNumTerms())) {
      int monomCmp(compareMonomials(p1, p2, term1Nr, term2Nr));
      if (0 == monomCmp) {
        mpz_sub(resCoeff[resultIndex], p1Coeff[term1Nr], p2Coeff[term2Nr]);
        if (0 != mpz_sgn(resCoeff[resultIndex])) {
          for (unsigned symbolNr = 0; symbolNr < numSymbols; symbolNr++) {
            resMonom[numSymbols * resultIndex + symbolNr] =
              p1Monom[numSymbols * term1Nr + symbolNr];
          }
          resultIndex++;
        }
        term1Nr++;
        term2Nr++;
      } else if (1 == monomCmp) {
        mpz_set(resCoeff[resultIndex], p1Coeff[term1Nr]);
        for (unsigned symbolNr = 0; symbolNr < numSymbols; symbolNr++) {
          resMonom[numSymbols * resultIndex + symbolNr] =
            p1Monom[numSymbols * term1Nr + symbolNr];
        }
        resultIndex++;
        term1Nr++;
      } else if (-1 == monomCmp) {
        mpz_neg(resCoeff[resultIndex], p2Coeff[term2Nr]);
        for (unsigned symbolNr = 0; symbolNr < numSymbols; symbolNr++) {
          resMonom[numSymbols * resultIndex + symbolNr] =
            p2Monom[numSymbols * term2Nr + symbolNr];
        }
        resultIndex++;
        term2Nr++;
      }
    }
    
    while (term1Nr < p1->getNumTerms()) {
      mpz_set(resCoeff[resultIndex], p1Coeff[term1Nr]);
      for (unsigned symbolNr = 0; symbolNr < numSymbols; symbolNr++) {
        resMonom[numSymbols * resultIndex + symbolNr] =
          p1Monom[numSymbols * term1Nr + symbolNr];
      }
      resultIndex++;
      term1Nr++;    
    }
    
    while (term2Nr < p2->getNumTerms()) {
      mpz_neg(resCoeff[resultIndex], p2Coeff[term2Nr]);
      for (unsigned symbolNr = 0; symbolNr < numSymbols; symbolNr++) {
        resMonom[numSymbols * resultIndex + symbolNr] =
          p2Monom[numSymbols * term2Nr + symbolNr];
      }
      resultIndex++;
      term2Nr++;
    }
    mpz_clear(resCoeff[resultSize]);
    result->numTerms = resultIndex;

    return result;
  }

  Polynomial *Polynomial::multByTerm
  (const Polynomial *p1, const Polynomial *p2, unsigned whichTerm) {
    Polynomial *result(new Polynomial(p1->getNumTerms()));
    unsigned *resMonom(result->monomials);
    const unsigned *p1Monom(p1->monomials);
    const unsigned *p2Monom(p2->monomials);
    const unsigned numSymbols(Base::getNumSymbols());
    for (unsigned termNr = 0; termNr < p1->getNumTerms(); termNr++) {
      mpz_mul(result->coefficients[termNr], p1->coefficients[termNr],
              p2->coefficients[whichTerm]);
      for (unsigned symbolNr = 0; symbolNr < numSymbols; symbolNr++) {
        resMonom[numSymbols * termNr + symbolNr]
          = p1Monom[numSymbols * termNr + symbolNr]
          + p2Monom[numSymbols * whichTerm + symbolNr];
      }
    }
    
    return result;
  }
      
  bool operator==(const Polynomial &p1, const Polynomial &p2) {
    if (p1.getNumTerms() != p2.getNumTerms()) {
      return false;
    }
    
    const mpz_t *p1Coeff(p1.getCoefficients());
    const mpz_t *p2Coeff(p2.getCoefficients());
    const unsigned *p1Monom(p1.getMonomials());
    const unsigned *p2Monom(p2.getMonomials());
    unsigned numSymbols(Base::getNumSymbols());
    for (unsigned termNr = 0; termNr < p1.getNumTerms(); termNr++) {
      if (0 != mpz_cmp(p1Coeff[termNr], p2Coeff[termNr])) {
        return false;
      }
      for (unsigned symbolNr = 0; symbolNr < numSymbols; symbolNr++) {
        if (p1Monom[numSymbols * termNr + symbolNr] !=
            p2Monom[numSymbols * termNr + symbolNr]) {
          return false;
        }
      }
    }
    return true;
  }

  void Polynomial::setMonom
    (unsigned termNr, unsigned symNr, unsigned value) {
    monomials[Base::getNumSymbols() * termNr + symNr] = value;
  }

  int Polynomial::compareMonomials
    (const Polynomial *p1, const Polynomial *p2,
     unsigned term1Nr, unsigned term2Nr) {
    int result(0);
    
    for (unsigned symbolNr = 0; symbolNr < Base::getNumSymbols(); symbolNr++) {
      unsigned index1(term1Nr * Base::getNumSymbols() + symbolNr);
      unsigned index2(term2Nr * Base::getNumSymbols() + symbolNr);
      if (p1->monomials[index1] > p2->monomials[index2]) {
        result = 1;
        break;
      } else if (p1->monomials[index1] < p2->monomials[index2]) {
        result = -1;
        break;
      }
    }
    
    return result;
  }

  Polynomial *Polynomial::copy() {
    Polynomial *result = new Polynomial(numTerms);
    const unsigned numSymbols(Base::getNumSymbols());
    for (unsigned termNr(0); termNr < numTerms; termNr++) {
      mpz_set(result->coefficients[termNr], coefficients[termNr]);
      for (unsigned symbolNr(0); symbolNr < numSymbols; symbolNr++) {
        result->monomials[numSymbols * termNr + symbolNr] =
          monomials[numSymbols * termNr + symbolNr];
      }
    }
    
    return result;
  }

  Polynomial *Polynomial::negPoly() {
    Polynomial *result = new Polynomial(numTerms);
    const unsigned numSymbols(Base::getNumSymbols());
    for (unsigned termNr(0); termNr < numTerms; termNr++) {
      mpz_neg(result->coefficients[termNr], coefficients[termNr]);
      for (unsigned symbolNr = 0; symbolNr < numSymbols; symbolNr++) {
        result->monomials[numSymbols * termNr + symbolNr] =
          monomials[numSymbols * termNr + symbolNr];
      }
    }
    
    return result;
  }

  bool Polynomial::isOne() const {
    if (1 == numTerms) {
      const std::vector<std::string> &symbols(Base::getSymbols());
      if (0 != mpz_cmp_si(coefficients[0], 1)) {
        return false;
      }
      for (unsigned symbolNr(0); symbolNr < symbols.size(); symbolNr++) {
        if (0 != monomials[symbolNr]) {
          return false;
        }
      }
      return true;
    } else {
      return false;
    }
  }

  bool Polynomial::isConstant() const {
    if (0 == numTerms) {
      return true;
    }
    if (numTerms > 1) {
      return false;
    }
    for (unsigned symbolNr(0); symbolNr < Base::getNumSymbols(); symbolNr++) {
      if (monomials[symbolNr] > 0) {
        return false;
      }
    }
    return true;
  }

  void Polynomial::setToConstant(int constant) {
    delete[] monomials;
    for (unsigned termNr(0); termNr < numTerms; termNr++) {
      mpz_clear(coefficients[termNr]);
    }
    delete[] coefficients;
    if (0 == constant) {
      numTerms = 0;
      monomials = NULL;
      coefficients = NULL;
    } else {
      numTerms = 1;
      const unsigned numSymbols(Base::getNumSymbols());
      monomials = new unsigned[numSymbols];
      for (unsigned symbolNr(0); symbolNr < numSymbols; symbolNr++) {
        monomials[symbolNr] = 0;
      }
      coefficients = new mpz_t[1];
      mpz_init_set_si(coefficients[0], constant);
    }
  }
}
