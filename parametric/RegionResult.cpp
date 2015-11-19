#include <cstdlib>
#include <cmath>
#include "rationalFunction/RationalFunction.h"
#include "RegionResult.h"

namespace parametric {
  using namespace std;
  using namespace rational;

  void Region::getMidPoint(vector<mpq_class> &point) const {
    point.clear();
    const unsigned numSymbols(RationalFunction::getNumSymbols());
    point.reserve(numSymbols);
    for (unsigned symNr(0); symNr < numSymbols; symNr++) {
      const Interval &interval((*this)[symNr]);
      mpq_class mid((interval.second + interval.first) / 2);
      point.push_back(mid);
    }
  }

  void Region::getRandomPoint(vector<mpq_class> &point) const {
    point.clear();
    const unsigned numSymbols(RationalFunction::getNumSymbols());
    point.reserve(numSymbols);
    for (unsigned symNr(0); symNr < numSymbols; symNr++) {
      const Interval &interval((*this)[symNr]);
      mpq_class random(rand() / (RAND_MAX + 1.0));
      random = interval.first + (interval.second - interval.first) * random;
      point.push_back(random);
    }
  }

  unsigned Region::getNumEdges() const {
    return pow(2, RationalFunction::getNumSymbols());
  }

  void Region::getEdgePoint(const unsigned edgeNr, vector<mpq_class> &point) const {
    point.clear();
    const unsigned numSymbols(RationalFunction::getNumSymbols());
    point.reserve(numSymbols);
    unsigned regionRest(edgeNr);
    for (unsigned symNr(0); symNr < numSymbols; symNr++) {
      bool lower(0 == (regionRest % 2));
      regionRest /= 2;
      const Interval &interval((*this)[symNr]);
      if (lower) {
	point.push_back(interval.first);
      } else {
	point.push_back(interval.second);
      }
    }
  }

  bool Region::contains(const vector<mpq_class> &point) const {
    const unsigned numSymbols(RationalFunction::getNumSymbols());
    for (unsigned sym(0); sym < numSymbols; sym++) {
      if ((point[sym] < (*this)[sym].first)
	  || (point[sym] > (*this)[sym].second)) {
	return false;
      }
    }

    return true;
  }
}
