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

#include <utility>
#include <string>
#include <iostream>
#include "ProgramOptions.h"

namespace parametric {
  
  using namespace std;
  namespace po = boost::program_options;
  
  /**
   * Create options map from command line.
   *
   * @param argc number of command line parameters
   * @param argv command line parameters
   * @return options read from command line
   */
  pair<po::variables_map,po::options_description *>
  parseCommandLine(int argc, char *argv[]) {
    /* named options */
    po::options_description bisimulation("Bisimulation");
    bisimulation.add_options()
      ("lump-method", po::value<string>()->default_value("auto"),
       "lumping method ([auto], weak, strong, none)")
      ("refine-order", po::value<string>()->default_value("small-first"),
       "Order of equivalence class refinement "
       "([small-first], big-first, first-first, last-first")
      ;
    
    po::options_description stateElimination("State-elimination");
    stateElimination.add_options()
      ("elimination-order", po::value<string>()->default_value("forward-reversed"),
       "order of state elimination (forward, [forward-reversed], random, backward-opt)")
      ;
    
    po::options_description output("Output");
    output.add_options()
      ("no-startup-message", "Suppress startup message")
      ("result-file", po::value<string>()->default_value("result"),
       "Filename prefix to output result to")
      ("result-format", po::value<string>()->default_value("regions"),
       "Output format ([regions], gnuplot, dat, pgf-regions, pgfplots)")
      ("plot-step", po::value<string>()->default_value("1/100"),
       "Relative plot step width")
      ("model-dot", po::value<string>(),
       "Filename to output GraphViz graph of model")
      ("lumped-dot", po::value<string>(),
       "Filename to print lumped model as GraphViz")
      ("statistics-file", po::value<string>(),
       "Filename to output statistics")
      ("regions-plot-lines", po::value<bool>()->default_value(true),
       "Plot lines to separate regions in PGF regoins output")
      ("regions-true-color",
       po::value<string>()->default_value("white"),
       "Color for regions in which property holds")
      ("regions-false-color",
       po::value<string>()->default_value("black"),
       "Color for regions in which property does not hold")
      ("regions-dontknow-color",
       po::value<string>()->default_value("gray"),
       "Color for regions in which truth value is unknown")
      ;

    po::options_description mdp("MDP");
    mdp.add_options()
      ("region-max-unknown", po::value<double>()->default_value(0.05),
       "maximal volume of remaining volume of remaining regions")
      ("rsolver-binary", po::value<string>()->default_value("rsolver"),
       "Location of RSolver binary")
      ("isat-binary", po::value<string>()->default_value("isat"),
       "Location of iSAT binary")
      ("rahd-binary", po::value<string>()->default_value("rahd"),
       "Location of RAHD binary")
      ("solver", po::value<string>()->default_value("isat"),
       "Solver to use ([isat], rsolver, rsolver-forall, none)")
      ("split-mode", po::value<string>()->default_value("all"),
       "Split mode ([all], longest)")
      ("iterate-precision", po::value<double>()->default_value(1E-6),
       "Stop precision for unbounded iteration")
      ("tolerance-factor", po::value<double>()->default_value(2),
       "Tolerance factor for optimality in policy iteration")
      ("random-sched-check", po::value<unsigned>()->default_value(10),
       "Number of random instantiations to check optimality of a "
       "scheduler in a region")
      ;
    
    po::options_description *visible = new po::options_description("Options");
    visible->add_options()
      ("help", "produce help message")
      ("low-level-input", "use low-level input format")
      ;
    
    visible->add(bisimulation);
    visible->add(stateElimination);
    visible->add(output);
    visible->add(mdp);
    
    /* the two positional values model and property file */
    po::positional_options_description pos;
    po::options_description hidden("Hidden options");
    hidden.add_options()
      ("model-file", po::value<string>(), "model to use")
      ("property-file", po::value<string>(), "property file to use")
      ("called-name", po::value<string>()->default_value(argv[0]), "program name")
      ;
    
    po::options_description cmdline_options;
    cmdline_options.add(*visible).add(hidden);
    
    pos.add("model-file", 1);
    pos.add("property-file", 2);;
    
    /* parse options */
    po::variables_map vm;
    try {
      po::store(po::command_line_parser(argc, argv).
                options(cmdline_options).positional(pos).run(), vm);
      po::notify(vm);
    } catch (...) {
      throw runtime_error("Error on parsing command line");
    }
    
    return make_pair(vm,visible);
  }
}
