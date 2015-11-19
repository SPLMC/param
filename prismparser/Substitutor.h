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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the program this parser part of.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2007-2010 Bjoern Wachter (Bjoern.Wachter@comlab.ox.ac.uk)
 * Copyright 2009-2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#ifndef SUBSTITUTOR_H
#define SUBSTITUTOR_H

#include <string>
#include <set>
#include <vector>
#include <tr1/unordered_map>
#include <boost/dynamic_bitset_fwd.hpp>
#include "AST.h"

namespace prismparser_ast {
  class Substitutor {
  public:
    Substitutor(Substitution &, Substitution &);
  private:
    struct substNode {
      std::string name;
      boost::shared_ptr<Expr> value;
      std::set<unsigned> out;
      std::set<unsigned> in;
    };
    typedef std::vector<struct substNode> Graph;
    typedef std::tr1::unordered_map<std::string,unsigned> NameToNumber;
    void insertConstantsAndFormulasToGraph
      (const Substitution &, const Substitution &);
    void computeGraphConnections();
    void getSuccessors(Expr &, std::set<unsigned> &);
    void substitute();
    void substitute(const boost::shared_ptr<Expr>&, const std::string &,
		    boost::shared_ptr<Expr>&);
    void mapBack(Substitution &, Substitution &);
    void findCyclicDependency
      (unsigned state, std::string &);
    void findCyclicDependency
      (std::vector<unsigned> &, boost::dynamic_bitset<> &, unsigned state, bool &);

    Graph graph;
    NameToNumber nameToNumber;
  };
}

#endif
