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

#ifndef REGIONS_TODO_H
#define REGIONS_TODO_H

#include <queue>
#include "RegionResult.h"

namespace parametric {
  mpq_class measure(const Region &);
  class RegionCmp {
  public:
    bool operator() (const Region &, const Region &) const;
  };

  class RegionsTODO {
  public:
    RegionsTODO();
    ~RegionsTODO();
    void createInitialRegion();
    void popLargestRegion(Region &);
    void insertRegion(const Region &);
    mpq_class measure() const;
    const unsigned size() const;
  private:
    std::priority_queue<Region,std::vector<Region>,RegionCmp> regions;
    mpq_class volume;
  };
}

#endif
