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

#ifndef WEAK_REFINER_H
#define WEAK_REFINER_H

#include <boost/dynamic_bitset_fwd.hpp>
#include "Refiner.h"

namespace rational {
  class RationalFunction;
}

namespace parametric {
  
  class WeakRefiner : public Refiner {
  public:
    WeakRefiner();
    void createInitialPartition(Partition &);
    void refineClass(EqClass &);
    void createQuotient(Partition &);
    
  private:
    std::map<unsigned, std::vector<unsigned> > refined;
    bool weakIsSilent(unsigned, EqClass &);
    void silentSplit
      (EqClass &, std::list<unsigned> &,
       std::map<unsigned,std::vector<bool> > &,
       std::map<unsigned,unsigned> &);
    rational::RationalFunction weakP_Chi(unsigned, EqClass &, EqClass &);
    void backSearch(boost::dynamic_bitset<> &, unsigned);
  };
}

#endif
