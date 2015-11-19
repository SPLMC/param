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
 * Copyright 2009-2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#include <stdexcept>
#include "MayChange.h"

namespace parametric {
  
  using namespace std;
  
  MayChange::MayChange(const std::string & partRefOrder_) {
    if ("small-first" == partRefOrder_) {
      partRefOrder = smallFirst;
    } else if ("big-first" == partRefOrder_) {
      partRefOrder = bigFirst;
    } else if ("first-first" == partRefOrder_) {
      partRefOrder = firstFirst;
    } else if ("last-first" == partRefOrder_) {
      partRefOrder = lastFirst;
    } else {
      throw runtime_error("Unsupported refine order \""
			  + partRefOrder_ + "\"");
    }

    if (smallFirst == partRefOrder) {
      smallFirstQueue.reset(new MayChangePQSmallFirst());
    } else if (bigFirst == partRefOrder) {
      bigFirstQueue.reset(new MayChangePQBigFirst());
    } else if (firstFirst == partRefOrder) {
      mchList.reset(new MayChangeList());
    } else if (lastFirst == partRefOrder) {
      mchList.reset(new MayChangeList());
    }  
  }
  
  
  void MayChange::push(PartitionList::iterator it) {
    if (alreadyContained.find(it) == alreadyContained.end()) {
      if (smallFirst == partRefOrder) {
        smallFirstQueue->push(it);
      } else if (bigFirst == partRefOrder) {
        bigFirstQueue->push(it);
      } else if (firstFirst == partRefOrder) {
        mchList->push_back(it);
      } else if (lastFirst == partRefOrder) {
        mchList->push_back(it);
      }
      alreadyContained.insert(it);
    }
  }
  
  PartitionList::iterator MayChange::top() const {
    if (smallFirst == partRefOrder) {
      return smallFirstQueue->top();
    } else if (bigFirst == partRefOrder) {
      return bigFirstQueue->top();
    } else if (firstFirst == partRefOrder) {
      return mchList->front();
    } else if (lastFirst == partRefOrder) {
      return mchList->back();
    } else {
      throw runtime_error("Unsupported order type.");
    }
  }
  
  void MayChange::pop() {
    alreadyContained.erase(top());
    if (smallFirst == partRefOrder) {
      smallFirstQueue->pop();
    } else if (bigFirst == partRefOrder) {
      bigFirstQueue->pop();
    } else if (firstFirst == partRefOrder) {
      mchList->pop_front();
    } else if (lastFirst == partRefOrder) {
      mchList->pop_back();
    }  
  }
  
  bool MayChange::empty() {
    if (smallFirst == partRefOrder) {
      return smallFirstQueue->empty();
    } else if (bigFirst == partRefOrder) {
      return bigFirstQueue->empty();
    } else if (firstFirst == partRefOrder) {
      return mchList->empty();
    } else if (lastFirst == partRefOrder) {
      return mchList->empty();
    }

    throw runtime_error("Unsupported partition refine order");
  }
}

