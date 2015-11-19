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

#ifndef POLY_PAIR_ITER_PAIR_CACHE_H
#define POLY_PAIR_ITER_PAIR_CACHE_H

#include <map>
#include <utility>

namespace rational {
  struct PolyPairIterPairCache {
    typedef std::pair<Polynomial *, Polynomial *> PolyPair;
    typedef std::map<PolyPair, unsigned, RationalCmp> PolyPairMap;
    typedef PolyPairMap::iterator PolyPairIter;

    inline std::size_t operator()(const std::pair<PolyPairIter, PolyPairIter> &value) const {
      std::tr1::hash<size_t> stHash;
      size_t firstNumber(reinterpret_cast<size_t>(&*value.first));
      size_t secondNumber(reinterpret_cast<size_t>(&*value.second));
      
      size_t hash(stHash(firstNumber));
      hash = stHash(secondNumber) + (hash << 6) + (hash << 16) - hash;
      
      return hash;
    }

    inline bool operator()(const std::pair<PolyPairIter, PolyPairIter> &v1,
                    const std::pair<PolyPairIter, PolyPairIter> &v2) const {
      return (v1.first == v2.first) && (v1.second == v2.second);
    }

    inline std::size_t operator()(const PolyPairIter &value) const {
      std::tr1::hash<unsigned> uiHash;
      size_t asNumber(reinterpret_cast<size_t>(&*value));
      
      size_t hash(uiHash(asNumber));
      
      return hash;
    }

    inline bool operator()
      (const PolyPairIter &v1, const PolyPairIter &v2) const {
      return v1 == v2;
    }
  };
}

#endif
