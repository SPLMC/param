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

#include <cstdlib>
#include <stdexcept>
#include <iostream>
#include "Controller.h"

using namespace std;

int main(int argc, char *argv[]) {
  try {
    parametric::Controller(argc, argv);
    exit(EXIT_SUCCESS);
  } catch (std::exception &e) {
    cerr << "\nThe following problem occured:\n  " << e.what() << endl;
    exit(EXIT_FAILURE);
  }
}
