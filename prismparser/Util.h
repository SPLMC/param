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

#ifndef __UTIL_H__
#define __UTIL_H__

#include <stdexcept>

namespace prismparser {
  std::string intToString(int i);
  std::string intToString(long long i);
  std::string floatToString(double f);

  class prismparser_error : public std::runtime_error {
  public:
    ~prismparser_error() throw();
    explicit prismparser_error(const std::string& s);
    std::ostream& message(std::ostream &) const;
    const std::string& toString() const;
  private:
    const std::string msg;
  };

  std::ostream& operator<<(std::ostream &, const class Error& e);
}

#endif
