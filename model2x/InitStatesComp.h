/*
 * This file is part of Model2X.
 *
 * Model2X is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Model2X is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Model2X. If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#ifndef INIT_STATES_COMP_H
#define INIT_STATES_COMP_H

#include <vector>

namespace prismparser {
  class Expr;
}

namespace model2x {

  class InitStatesComp {
  public:
    InitStatesComp();
    ~InitStatesComp();
    void setExpr(const prismparser::Expr &);
    void setVars(const std::vector<prismparser::Expr> &);
    void setBounds(const std::vector<int> &, const std::vector<int> &);
    void compInits(std::vector<std::vector<int> > &);

  private:
    prismparser::Expr negToInner(const prismparser::Expr &, const bool);
    void compInits(std::vector<std::vector<int> > &, unsigned, std::vector<int> &, const prismparser::Expr &);
    int fromTree(const prismparser::Expr &, unsigned, const std::vector<int> &);

    const prismparser::Expr *initExpr;
    const std::vector<prismparser::Expr> *vars;
    const std::vector<int> *lowerBounds;
    const std::vector<int> *upperBounds;
  };
}

#endif
