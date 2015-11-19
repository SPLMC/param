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

#ifndef PARAM_PROGRAM_OPTIONS_H
#define PARAM_PROGRAM_OPTIONS_H

#include <utility>
#include <boost/program_options.hpp>

namespace parametric {
  
  void showHelp
    (const char *, const boost::program_options::options_description &);
  
  std::pair<boost::program_options::variables_map,boost::program_options::options_description *>
    parseCommandLine(int argc, char *argv[]);
}

#endif
