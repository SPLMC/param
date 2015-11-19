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

#include "Geobucket.h"
#include "Polynomial.h"

using namespace std;

namespace rational {
  Geobucket::Geobucket() {
    d = 4;
  }

  Geobucket::~Geobucket() {
    for (unsigned bucketNr(0); bucketNr < buckets.size(); bucketNr++) {
      delete buckets[bucketNr];
    }
  }
  
  unsigned Geobucket::logd(unsigned value) {
    unsigned result(0);
    unsigned dtoi(1);
    while (dtoi < value) {
      dtoi *= d;
      result++;
    }

    return result;
  }
  
  void Geobucket::add(Polynomial *poly) {
    Polynomial *f(poly);
    unsigned i(std::max(1u, logd(f->getNumTerms())));
    unsigned dtoi(1);
    for (unsigned icnt = 0; icnt < i; icnt++) {
      dtoi *= d;
    }
    
    if (i <= buckets.size()) {
      Polynomial *fp(Polynomial::addPolys(f, buckets[i - 1]));
      delete f;
      f = fp;
      while ((i <= buckets.size()) && (f->getNumTerms() > dtoi)) {
        if ((i < buckets.size()) && (0 != buckets[i]->getNumTerms())) {
          fp = Polynomial::addPolys(f, buckets[i]);
          delete f;
          f = fp;
        }
        delete buckets[i-1];
        buckets[i-1] = new Polynomial(0);
        i++;
        dtoi *= d;
      }
    }
    while (buckets.size() < i) {
      Polynomial *fill = new Polynomial(0);
      buckets.push_back(fill);
    }
    delete buckets[i-1];
    buckets[i-1] = f;
  }
  
  Polynomial *Geobucket::canonicalize() {
    Polynomial *result = new Polynomial(0);
    for (unsigned i = 0; i < buckets.size(); i++) {
      Polynomial *resultP(Polynomial::addPolys(result, buckets[i]));
      delete result;
      result = resultP;
    }

    return result;
  } 
}
