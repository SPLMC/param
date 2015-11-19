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

#ifndef PARTITION_H
#define PARTITION_H

#include <string>
#include <set>
#include <list>
#include <map>
#include "MayChange.h"

namespace parametric {
  class PMC;
  class Quotient;
  class Refiner;
  class StrongRefiner;
  class WeakRefiner;
  
  typedef std::set<unsigned> EqClass;
  typedef std::list<EqClass> PartitionList;
  typedef std::map<unsigned,PartitionList::iterator> PartitionMap;
  typedef std::map<EqClass *, unsigned> QuotMap;
  /**
   * State partition.
   */
  class Partition {
    friend class Quotient;
    friend class Refiner;
    friend class StrongRefiner;
    friend class WeakRefiner;
    
  public:
    Partition(const std::string &);
    inline PartitionList::iterator operator[](const unsigned v) {
      return P_map[v];
    }
    
    inline PartitionList::iterator begin() {
      return P.begin();
    }

    inline PartitionList::iterator end() {
      return P.end();
    }
  private:

    /* list of equivalence classes */
    PartitionList P;
    /* maps each state to its equivalence class */
    PartitionMap P_map;
    /* if in this set, partition was changed by algorithm */
    std::set<EqClass *> partitionChanged;
    /* if in this set, partition may need to be changed by algorithm */
    std::auto_ptr<MayChange> mayChange;
    /* */
    std::list<EqClass*> newPartitions;
    
    PMC *pmc;
    
    /**
     * Gives the number of equivalence classes of partition.
     *
     * @return number of equivalence classes
     */
    inline unsigned size() {
      return P.size();
    }
    
    /**
     * Clears a partition.
     */
    inline void clear() {
      P.clear();
      partitionChanged.clear();
      //    mayChange.clear();
    }
    
    /**
     * Inserts an equivalence class into a partition.
     * Will create a copy, you should use the version in the partition
     * afterwards.
     *
     * @param eqClass equivalence class to be inserted
     */
    inline PartitionList::iterator insert(EqClass &eqClass, bool calcChanged) {
      /* insert class into list of equivalence classes */
      PartitionList::iterator partIt =
        P.insert(P.begin(), eqClass);
      
      /* remap states */
      EqClass &newClass = *partIt;
      EqClass::iterator it;
      for (it = newClass.begin(); it != newClass.end(); it++) {
	unsigned u = *it;
        P_map[u] = partIt;
      }
      
      if (calcChanged) {
        newPartitions.push_back(&newClass);
      }
      
      return partIt;
    }
    
    inline void afterRefine() {
      for (std::list<EqClass *>::iterator it = newPartitions.begin();
           it != newPartitions.end(); it++) {
        calcMayChange(*it);
      }
      newPartitions.clear();
    }
    
    inline void remapAll() {
      for (PartitionList::iterator it = P.begin(); it != P.end(); it++) {
        EqClass &eqClass = *it;
        for (EqClass::iterator it2 = eqClass.begin(); it2 != eqClass.end();
             it2++) {
	  unsigned u = *it2;
          P_map[u] = it;
        }
      }
    }
    
    inline void erase(PartitionList::iterator P_it) {
      P.erase(P_it);
    }
    
    void calcMayChange(EqClass *);
  };
}

#endif
