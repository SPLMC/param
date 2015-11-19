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

#ifndef BOUNDED_ITERATOR_H
#define BOUNDED_ITERATOR_H

#include <vector>
#include <set>
#include "rationalFunction/RationalFunction.h"

namespace parametric {
  template<class Key, class Entry>
    class HashMap : public std::tr1::unordered_map<Key,Entry> {
  };

  class PMC;

  class BoundedIterator {
  public:
    void setPMC(PMC &);
    void setPresValues(const std::vector<rational::RationalFunction> &);
    void setNextValues(std::vector<rational::RationalFunction> &);
    void multiply();
  private:
    PMC *pmc;
    const std::vector<rational::RationalFunction> *presValues;
    std::vector<rational::RationalFunction> *nextValues;
  };
}

#endif
