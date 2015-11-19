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
 * along with PARAM.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2010-2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#include <iostream>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <ginac/ginac.h>
#include "RationalParser.h"
#include "rationalFunction/RationalFunction.h"
#include "rationalFunction/Cancellator.h"
#include "rationalFunction/Polynomial.h"

namespace parametric {
  using namespace std;
  using namespace rational;
  using namespace GiNaC;

  RationalParser::RationalParser() {
    ginacTab = new symtab();
    reader = NULL;
  }

  RationalParser::~RationalParser() {
    delete ginacTab;
    if (NULL != reader) {
      delete reader;
    }
  }

  void RationalParser::parseSymbols(const string &line) {
    istringstream iss(line);

    copy(std::istream_iterator<string>(iss),
         std::istream_iterator<string>(),
         back_inserter<vector<string> >(symbols));

    vector<unsigned> coeff(symbols.size());
    for (unsigned symNr(0); symNr < symbols.size(); symNr++) {
      if (!isalpha(symbols[symNr][0])) {
	throw runtime_error("First character of symbol \""
			    + symbols[symNr] + "\" is not an alphabetic letter." );
      }
      RationalFunction::addSymbol(symbols[symNr]);
    }
    RationalFunction::start();

    vector<symbol> &ginacSymbols = Cancellator::getGiNaCSymbols();
    vector<string> &symbolStrings = Cancellator::getSymbolStrings(); 
    for (unsigned symNr(0); symNr < symbolStrings.size(); symNr++) {
      (*ginacTab)[symbolStrings[symNr]] = ginacSymbols[symNr];
    }
    reader = new parser(*ginacTab);
  }

  RationalFunction RationalParser::parseRational(const string &line) {    
    ex expr = numer_denom(expand((*reader)(line)));

    Polynomial *numer = new Polynomial();
    Polynomial *denom = new Polynomial();
    Cancellator::convert(expr[0], numer);
    Cancellator::convert(expr[1], denom);

    return RationalFunction(numer, denom);
  }
}
