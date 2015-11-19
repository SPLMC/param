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

#ifndef GEOBUCKET_H
#define GEOBUCKET_H

#include <vector>

namespace rational {
  class Polynomial;
  class Geobucket {
  public:
    Geobucket();
    ~Geobucket();
    void add(Polynomial *);
    Polynomial *canonicalize();
    unsigned logd(unsigned);
  private:
    unsigned d;
    std::vector<Polynomial *> buckets;
  };
}

#endif
