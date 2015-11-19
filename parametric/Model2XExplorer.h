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

#ifndef MODEL2X_EXPLORER_H
#define MODEL2X_EXPLORER_H

#include "ModelExplorer.h"
#include <string>

namespace prismparser {
  class Model;
  class Properties;
  class Property;
}

namespace model2x {
  class Model2X;
}

namespace parametric {
  
  class PMM;
  class PMC;
  class ExprToNumber;
  
  class Model2XExplorer : public ModelExplorer {
  public:
    Model2XExplorer();
    virtual ~Model2XExplorer();
    void explore();
  private:
    void loadModel(const std::string &, const std::string &);
    void constructMC(model2x::Model2X &);
    void exploreAllStates(model2x::Model2X &);
    void checkUseReward(const prismparser::Property &);
    void checkUseReward();
    void embed();
    
    prismparser::Model *model;
    bool useReward;
    unsigned rewardStruct;
  };
}

#endif
