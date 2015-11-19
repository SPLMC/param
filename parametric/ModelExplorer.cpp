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
 * Copyright 2010-2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#include <cassert>
#include "PMM.h"
#include "ModelExplorer.h"

namespace parametric {
  using namespace std;
  using namespace rational;

  ModelExplorer::ModelExplorer() {
    props = NULL;
    exprToNumber = NULL;
    pmm = NULL;
    pmc = NULL;
    pmdp = NULL;
  }

  ModelExplorer::~ModelExplorer() {
  }

  PMM *ModelExplorer::getPMMModel() {
    return pmm;
  }

  void ModelExplorer::setModelFilename(const string &modelFilename_) {
    modelFilename = modelFilename_;
  }

  void ModelExplorer::setPropertyFilename(const string &propertyFilename_) {
    propertyFilename = propertyFilename_;
  }

  void ModelExplorer::setProperties(prismparser::Properties &props_) {
    assert(NULL == props);
    props = &props_;
  }

  void ModelExplorer::setExprToNumber(ExprToNumber &exprToNumber_) {
    assert(NULL == exprToNumber);
    exprToNumber = &exprToNumber_;
  }
}
