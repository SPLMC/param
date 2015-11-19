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

 * You should have received a copy of the GNU General Public License
 * along with Model2X. If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2010-2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#ifndef EXPR2LUA_H
#define EXPR2LUA_H

#include <tr1/unordered_map>

namespace prismparser {
  class Expr;
}

namespace model2x {
  class expr2lua {
    friend std::ostream &operator<<(std::ostream &, const expr2lua &);
  public:
    expr2lua(const prismparser::Expr &);
    ~expr2lua();
  private:
    static std::tr1::unordered_map<unsigned,std::string>
      prepareKindToString();
    void printExprC(std::ostream &, const prismparser::Expr &) const;
    static std::tr1::unordered_map<unsigned,std::string> kindToString;
    const prismparser::Expr *expr;
  };

  inline std::ostream &operator<<(std::ostream &os, const expr2lua &expr2lua) {
    expr2lua.printExprC(os, *expr2lua.expr);
    return os;
  }
}

#endif
