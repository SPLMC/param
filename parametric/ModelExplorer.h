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

#ifndef MODEL_EXPLORER_H
#define MODEL_EXPLORER_H

#include <vector>
#include <string>
#include <iosfwd>
#include "rationalFunction/RationalFunction.h"

namespace prismparser {
  class Properties;
}

namespace parametric {
  class ExprToNumber;
  class PMM;
  class PMC;
  class PMDP;
  class RationalParser;
  
  class ModelExplorer {
  public:
    ModelExplorer();
    virtual ~ModelExplorer();
    void setProperties(prismparser::Properties &);
    void setModelFilename(const std::string &);
    void setPropertyFilename(const std::string &);
    void setExprToNumber(ExprToNumber &);
    PMM *getPMMModel();
    virtual void explore() = 0;
  protected:
    std::string modelFilename;
    std::string propertyFilename;
    ExprToNumber *exprToNumber;
    prismparser::Properties *props;
    PMM *pmm;
    PMC *pmc;
    PMDP *pmdp;
  };
}

#endif
