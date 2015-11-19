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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with PARAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2009-2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#ifndef MAY_CHANGE_H
#define MAY_CHANGE_H

#include <memory>
#include <queue>
#include <set>
#include <list>

namespace parametric {
  typedef std::set<unsigned> EqClass;
  typedef std::list<EqClass> PartitionList;
}

/*
 * Needed to allow defining sets of partition list iterators.
 */
namespace std {
  inline bool operator<
    (const parametric::PartitionList::iterator &p1,
     const parametric::PartitionList::iterator &p2) {
    return &*p1 < &*p2;
  }
}

namespace parametric {
  class PListIt_less {
  public:
    bool operator()(const PartitionList::iterator x,
                    const PartitionList::iterator y) const {
      return x->size() < y->size();
    }
  };
  
  class PListIt_greater {
  public:
    bool operator()(const PartitionList::iterator x,
                    const PartitionList::iterator y) const {
      return x->size() > y->size();
    }
  };
  
  typedef std::priority_queue<PartitionList::iterator,
    std::vector<PartitionList::iterator>,
    PListIt_less> MayChangePQBigFirst;
  
  typedef std::priority_queue<PartitionList::iterator,
    std::vector<PartitionList::iterator>,
    PListIt_greater> MayChangePQSmallFirst;
  
  typedef std::list<PartitionList::iterator> MayChangeList;		  
  
  /**
   * Container to hold candidates to be split next.
   */
  class MayChange {
  public:    
    MayChange(const std::string &);
    void push(PartitionList::iterator);
    PartitionList::iterator top() const;
    void pop();
    bool empty();
    
  private:
    enum PartRefOrder {
      bigFirst,
      smallFirst,
      firstFirst,
      lastFirst
    };

    std::set<PartitionList::iterator> alreadyContained;
    std::auto_ptr<MayChangePQBigFirst> bigFirstQueue;
    std::auto_ptr<MayChangePQSmallFirst> smallFirstQueue;
    std::auto_ptr<MayChangeList> mchList;
    PartRefOrder partRefOrder;
  };
}

#endif
