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

#include <set>
#include <sstream>
#include <boost/dynamic_bitset.hpp>
#include "Util.h"
#include "Substitutor.h"

namespace prismparser_ast {
  using namespace std;
  using namespace boost;

  Substitutor::Substitutor(Substitution &constants, Substitution &formulas) {
    insertConstantsAndFormulasToGraph(constants, formulas);
    computeGraphConnections();
    substitute();
    mapBack(constants, formulas);
  }

  void Substitutor::mapBack(Substitution &constants, Substitution &formulas) {
    for (Substitution::iterator i(constants.begin());
	 i != constants.end(); i++) {
      i->second = graph[nameToNumber[i->first]].value;
    }
    for (Substitution::iterator i(formulas.begin());
	 i != constants.end(); i++) {
      i->second = graph[nameToNumber[i->first]].value;
    }
  }

  void Substitutor::substitute
  (const shared_ptr<Expr> &insert, const string &name,
   shared_ptr<Expr> &into) {
    if (into->isVariable()) {
      if (into->getIdentifier() == name) {
	into = insert;
      }
    } else {
      for (Exprs::iterator i(into->children.begin());
	   i != into->children.end(); i++) {
	substitute(insert, name, *i);
      }
    }
  }

  void Substitutor::substitute() {
    set<unsigned> zerodepend;
    set<unsigned> todo;
    for (unsigned i(0); i < graph.size(); i++) {
      todo.insert(i);
      if (0 == graph[i].out.size()) {
	zerodepend.insert(i);
      }
    }

    while (0 != todo.size()) {
      if (0 != zerodepend.size()) {
	unsigned state(*zerodepend.begin());
	set<unsigned> &dep(graph[state].in);
	for (set<unsigned>::iterator i(dep.begin()); i != dep.end(); i++) {
	  substitute(graph[state].value, graph[state].name, graph[*i].value);
	  graph[*i].out.erase(state);
	  if (0 == graph[*i].out.size()) {
	    zerodepend.insert(*i);
	  }
	}
	zerodepend.erase(state);
	todo.erase(state);
      } else {
	string cyclic;

	findCyclicDependency(*todo.begin(), cyclic);
	throw prismparser::prismparser_error
	  ("Cyclic dependency found:\n" + cyclic);
      }
    }
  }

  void Substitutor::findCyclicDependency(unsigned state, string &cyclic) {
    vector<unsigned> stack;
    dynamic_bitset<> lookup(graph.size(), false);
    bool done = false;

    findCyclicDependency(stack, lookup, state, done);
    assert(done);

    unsigned cycleState(stack[stack.size() - 1]);
    unsigned beginCycle;
    for (beginCycle = stack.size() - 2; stack[beginCycle] != cycleState;
	 beginCycle--)
      ;

    stringstream sstream;
    for (unsigned cState(beginCycle); cState < stack.size(); cState++) {
      sstream << graph[stack[cState]].name << " = \""
	      << graph[stack[cState]].value->toString() << "\"\n";
    }
    cyclic = sstream.str();
  }

  void Substitutor::findCyclicDependency
  (vector<unsigned> &stack, dynamic_bitset<> &lookup, unsigned state, bool &done) {
    stack.push_back(state);

    if (lookup[state]) {
      done = true;
      return;
    }
    lookup[state] = true;
    set<unsigned> &out = graph[state].out;
    for (set<unsigned>::iterator i(out.begin()); i != out.end(); i++) {
      if (!done) {
	findCyclicDependency(stack, lookup, *i, done);
      }
    }
    if (!done) {
      lookup[state] = false;
      stack.pop_back();
    }
  }

  void Substitutor::insertConstantsAndFormulasToGraph
  (const Substitution &constants, const Substitution &formulas) {
    for (Substitution::const_iterator i(constants.begin());
	 i != constants.end(); i++) {
      struct substNode node;
      node.name = i->first;
      node.value = i->second;
      graph.push_back(node);
    }
    for (Substitution::const_iterator i(formulas.begin());
	 i != formulas.end(); i++) {
      struct substNode node;
      node.name = i->first;
      node.value = i->second;
      graph.push_back(node);
    }
  }

  void Substitutor::computeGraphConnections() {
    unsigned number(0);
    for (Graph::iterator i(graph.begin()); i != graph.end(); i++) {
      struct substNode &node(*i);
      nameToNumber[node.name] = number;
      number++;
    }

    for (unsigned i(0); i < graph.size(); i++) {
      struct substNode &node(graph[i]);
      getSuccessors(*node.value.get(), node.out);      
      for (set<unsigned>::iterator j(node.out.begin()); j != node.out.end(); j++) {
	graph[*j].in.insert(i);
      }
    }    
  }

  void Substitutor::getSuccessors(Expr &value, set<unsigned> &out) {
    if (value.isVariable()) {
      const string &name(value.getIdentifier());
      if (0 != nameToNumber.count(name)) {
	out.insert(nameToNumber[name]);
      }
    } else {
      for (Exprs::iterator i(value.children.begin()); i != value.children.end(); i++) {
	getSuccessors(*(i->get()), out);
      }
    }
  }
}
