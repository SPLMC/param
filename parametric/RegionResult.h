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
 * Copyright 2010-2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#include <utility>
#include <vector>
#include <list>
#include <gmpxx.h>

#ifndef REGION_RESULT_H
#define REGION_RESULT_H

namespace rational {
  class RationalFunction;
}

namespace parametric {
  class Result : public std::vector<rational::RationalFunction> {
  };
  class Interval : public std::pair<mpq_class,mpq_class> {};
  class Region : public std::vector<Interval> {
  public:
    void getMidPoint(std::vector<mpq_class> &) const;
    void getRandomPoint(std::vector<mpq_class> &) const;
    unsigned getNumEdges() const;
    void getEdgePoint(const unsigned, std::vector<mpq_class> &) const;
    bool contains(const std::vector<mpq_class> &) const;
  };
  class RegionResult : public std::list<std::pair<Region,Result> > {};
}

#endif
