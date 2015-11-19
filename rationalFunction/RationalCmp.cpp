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

#include <iostream>
#include "RationalCmp.h"
#include "Polynomial.h"
#include "Base.h"

using namespace std;

namespace rational {
  int RationalCmp::operator()(const Polynomial *p1, const Polynomial *p2) {
    if (p1->getNumTerms() < p2->getNumTerms()) {
      return 1;
    }
    if (p1->getNumTerms() > p2->getNumTerms()) {
      return -1;
    }
    
    const mpz_t *p1Coeff(p1->getCoefficients());
    const mpz_t *p2Coeff(p2->getCoefficients());
    const unsigned *p1Monom(p1->getMonomials());
    const unsigned *p2Monom(p2->getMonomials());
    unsigned numSymbols(Base::getNumSymbols());
    for (unsigned termNr = 0; termNr < p1->getNumTerms(); termNr++) {
      int cmpVal(mpz_cmp(p1Coeff[termNr], p2Coeff[termNr]));
      if (cmpVal < 0) {
        return -1;
      } else if (cmpVal > 0) {
        return 1;
      }
      for (unsigned symbolNr = 0; symbolNr < numSymbols; symbolNr++) {
        unsigned p1Val(p1Monom[numSymbols * termNr + symbolNr]);
        unsigned p2Val(p2Monom[numSymbols * termNr + symbolNr]);
        if (p1Val < p2Val) {
          return -1;
        } else if (p1Val > p2Val) {
          return 1;
        }
      }
    }
    return 0;
  }

  bool RationalCmp::operator()(const std::pair<Polynomial *, Polynomial *> &r1,
                  const std::pair<Polynomial *, Polynomial *> &r2) {
    int valFirst((*this)(r1.first, r2.first));
    if (0 == valFirst) {
      int valSecond((*this)(r1.second, r2.second));
      return (-1 == valSecond);
    } else {
      return (-1 == valFirst);
    }
  }
}
