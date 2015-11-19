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

#ifndef CANCELLATOR_H
#define CANCELLATOR_H

#include <vector>
#include <string>

namespace GiNaC {
  class ex;
  class symbol;
}

namespace rational {
  class Polynomial;

  class Cancellator {
  public:
    static void addSymbol(const std::string &);
    static void start();
    static void clear();
    static void convert(const Polynomial *, GiNaC::ex &);
    static void convert(const GiNaC::ex &, Polynomial *);
    static void cancel(Polynomial *, Polynomial *);
    static void cancel(GiNaC::ex &, GiNaC::ex &);
    static std::vector<GiNaC::symbol> &getGiNaCSymbols();
    static std::vector<std::string> &getSymbolStrings();
    
  private:
    static std::string int2string(unsigned);
    static std::vector<std::string> symbolStrings;
    static std::vector<GiNaC::symbol> *ginacSymbols;
    static int cleanupInt;
  };
}

#endif
