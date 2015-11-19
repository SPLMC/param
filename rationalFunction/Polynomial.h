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

#ifndef POLYNOMIAL_H
#define POLYNOMIAL_H

#include <cstdlib>
#include <gmp.h>

namespace rational {
  class Base;
  class Geobucket;
  class RationalFunction;
  class Cancellator;

  class Polynomial {
    friend class Geobucket;
    friend class Base;
    friend class RationalFunction;
    friend class Cancellator;
  public:
    Polynomial();
    Polynomial(unsigned);
    Polynomial(const Polynomial &);
    Polynomial *copy();
    Polynomial *negPoly();
    inline unsigned getNumTerms() const {
      return numTerms;
    }
    inline void setCoefficient(unsigned termNr, mpz_t &coefficient) {
      mpz_set(coefficients[termNr], coefficient);
    }
    void setMonom(unsigned, unsigned, unsigned);
    inline const mpz_t *getCoefficients() const {
      return coefficients;
    }
    inline const unsigned *getMonomials() const {
      return monomials;
    }
    inline bool isZero() const {
      return 0 == numTerms;
    }
    bool isConstant() const;
    bool isOne() const;
    void setToConstant(int);
    static Polynomial *addPolys(const Polynomial *, const Polynomial *);
    static Polynomial *multByTerm(const Polynomial *, const Polynomial *, unsigned);
    static Polynomial *subPolys(const Polynomial *, const Polynomial *);
    static unsigned computeResultSizeAdd(const Polynomial *, const Polynomial *);
    static unsigned computeResultSizeSub(const Polynomial *, const Polynomial *);
    static int compareMonomials
      (const Polynomial *, const Polynomial *, unsigned, unsigned);

    ~Polynomial();
    
  private:
    unsigned numTerms;
    unsigned *monomials;
    mpz_t *coefficients;
  };

  bool operator==(const Polynomial &p1, const Polynomial &p2);
}

#endif
