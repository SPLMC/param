/*
 * This file is part of a parser for an extension of the PRISM language.
 *
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The parser is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the program this parser part of.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2007-2010 Bjoern Wachter (Bjoern.Wachter@comlab.ox.ac.uk)
 * Copyright 2009-2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#include "Util.h"
#include <sstream>

namespace prismparser {
  using namespace std;

  std::string intToString(int i) {
    stringstream str;
    
    str << i;
    
    return str.str();
  }

  std::string intToString(long long i) {
    stringstream str;
    
    str << i;
    
    return str.str();
  }
  
  std::string floatToString(double f) {
    stringstream str;
    
    str << f;
    
    return str.str();
  }

  prismparser_error::~prismparser_error() throw() {
  }

  prismparser_error::prismparser_error(const std::string& s)
  : runtime_error(s), msg(s) {
  }

  std::ostream& prismparser_error::message(std::ostream& o) const {
    return o << msg << std::endl;
  }
  
  const std::string& prismparser_error::toString() const {
    return msg;
  }

  std::ostream& operator<<(std::ostream &o, const prismparser_error& e) {
    return e.message(o);
  }
}
