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
 * Copyright 2010-2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

// TODO converting to string first to convert between mpz_t and GiNaC
// should be avoided

#include <assert.h>
#include <string>
#include <sstream>
#include <ginac/ginac.h>
#include "Cancellator.h"
#include "Base.h"
#include "Polynomial.h"

using namespace GiNaC;
using namespace std;

namespace rational {
  std::vector<std::string> Cancellator::symbolStrings;
  int Cancellator::cleanupInt(atexit(Cancellator::clear));
  std::vector<GiNaC::symbol> *Cancellator::ginacSymbols;
  
  vector<symbol> &Cancellator::getGiNaCSymbols() {
    return *ginacSymbols;
  }

  vector<string> &Cancellator::getSymbolStrings() {
    return symbolStrings;
  }

  string Cancellator::int2string(unsigned number) {
    stringstream numberStringStream;
    numberStringStream << number;
    string numberString(numberStringStream.str());
    for (unsigned index(0); index < numberString.length(); index++) {
      numberString[index] -= '0';
      numberString[index] += 'A';
    }

    return numberString;
  }

  void Cancellator::addSymbol(const string &symbol) {
    //    symbolStrings.push_back(int2string(symbolStrings.size()));
    symbolStrings.push_back(symbol);
  }

  void Cancellator::clear() {
    delete ginacSymbols;
    symbolStrings.clear();
  }

  void Cancellator::start() {
    if (0 == symbolStrings.size()) {
      return;
    }
    ginacSymbols = new vector<symbol>();
    const unsigned numSymbols(symbolStrings.size());
    for (unsigned symNr(0); symNr < numSymbols; symNr++) {
      ginacSymbols->push_back(symbol(symbolStrings[symNr]));
    }
  }

  void Cancellator::convert(const Polynomial *poly, ex &result) {
    const unsigned *monomials(poly->getMonomials());
    const mpz_t *coefficients(poly->getCoefficients());
    const unsigned numSymbols(Base::getNumSymbols());
    const unsigned numTerms(poly->getNumTerms());
    result = 0;
    for (unsigned termNr(0); termNr < numTerms; termNr++) {
      mpz_class coeff(coefficients[termNr]);
      lst asdf;
      ex monom(coeff.get_str(), asdf);

      for (unsigned symNr(0); symNr < numSymbols; symNr++) {
        monom *= pow((*ginacSymbols)[symNr],
		     monomials[termNr * numSymbols + symNr]);
      }
      result += monom;
    }
  }

  struct CompCf {
    vector<symbol> *symbols;
    bool operator()(const ex &poly1, const ex &poly2) {
      for (unsigned i(0); i < symbols->size(); i++) {
	const int degree1(degree(poly1, (*symbols)[i]));
	const int degree2(degree(poly2, (*symbols)[i]));
	if (degree1 < degree2) {
	  return false;
	} else if (degree1 > degree2) {
	  return true;
	}
      }

      return true;
    }
  };

  void Cancellator::convert(const ex &poly, Polynomial *result) {
    const unsigned numSymbols(ginacSymbols->size());
    vector<ex> toSort;
    if (!is_a<add>(poly)) {
      toSort.push_back(poly);
    } else {
      for (const_iterator i = poly.begin(); i != poly.end(); i++) {
	toSort.push_back(*i);
      }
    }
    struct CompCf compCf;
    compCf.symbols = ginacSymbols;
    sort(toSort.begin(), toSort.end(), compCf);
    delete[] result->monomials;
    for (unsigned termNr(0); termNr < result->numTerms; termNr++) {
      mpz_clear(result->coefficients[termNr]);
    }
    delete[] result->coefficients;
    result->numTerms = toSort.size();
    result->monomials = new unsigned[result->numTerms * numSymbols];
    result->coefficients = new mpz_t[result->numTerms];
    
    for (unsigned termNr(0); termNr < result->numTerms; termNr++) {
      for (unsigned symNr(0); symNr < numSymbols; symNr++) {
	const int degr(degree(toSort[termNr], (*ginacSymbols)[symNr]));
        result->monomials[termNr * numSymbols + symNr] = degr;
      }
      ex coefff(1);
      if (is_a<numeric>(toSort[termNr])) {
	coefff = toSort[termNr];
      } else if (is_a<mul>(toSort[termNr])) {
	for (const_iterator i = toSort[termNr].begin(); i != toSort[termNr].end(); i++) {
	  if (is_a<numeric>(*i)) {
	    coefff = *i;
	  }
	}
      }
      stringstream sstream;
      sstream << coefff;
      std::string coeffStr = sstream.str();
      mpz_class coeffMpz(coeffStr);
      mpz_init(result->coefficients[termNr]);      
      mpz_set(result->coefficients[termNr], coeffMpz.get_mpz_t());
    }
  }

  void Cancellator::cancel(ex &poly1, ex &poly2) {
#if 1
    ex result(numer_denom(poly1 / poly2));
    poly1 = expand(result[0]);
    poly2 = expand(result[1]);
#endif
#if 0
    ex gc(gcd(poly1, poly2));
    divide(poly1, gc, poly1);
    divide(poly2, gc, poly2);
    poly1 = expand(poly1);
    poly2 = expand(poly2);
#endif
  }

  void Cancellator::cancel(Polynomial *poly1, Polynomial *poly2) {
    if (0 == symbolStrings.size()) {
      return;
    }
    if (poly1->isZero()) {
      poly2->setToConstant(1);
      return;
    }
    if (poly1->isConstant()) {
      return;
    }
    if (poly2->isConstant()) {
      return;
    }
    if (*poly1 == *poly2) {
      poly1->setToConstant(1);
      poly2->setToConstant(1);
      return;
    }

    ex ginac1;
    ex ginac2;
    convert(poly1, ginac1);
    convert(poly2, ginac2);
    cancel(ginac1, ginac2);
    convert(ginac1, poly1);
    convert(ginac2, poly2);
  }

}
