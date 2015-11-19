#include <cassert>
#include "rationalFunction/RationalFunction.h"
#include "RegionsTODO.h"

namespace parametric {
  using namespace std;
  using namespace rational;
  
  mpq_class measure(const Region &region) {
    mpq_class result(1);
    for (unsigned i(0); i < region.size(); i++) {
      mpq_class width(RationalFunction::getBoundRight(i)
		      - RationalFunction::getBoundLeft(i));
      result *= (region[i].second - region[i].first) / width;
    }
    return result;
  }

  bool RegionCmp::operator() (const Region &lhs, const Region &rhs) const {
    mpq_class leftMeasure(measure(lhs));
    mpq_class rightMeasure(measure(rhs));
    if (leftMeasure < rightMeasure) {
      return true;
    } else if (leftMeasure > rightMeasure) {
      return false;
    }

    const unsigned numSymbols(RationalFunction::getNumSymbols());
    for (unsigned symNr(0); symNr < numSymbols; symNr++) {
      if (lhs[symNr].first < rhs[symNr].first) {
	return true;
      } else if (lhs[symNr].second < rhs[symNr].second) {
	return true;
      }
    }

    return false;
  }

  RegionsTODO::RegionsTODO() {
    volume = -1.0;
  }

  RegionsTODO::~RegionsTODO() {
  }

  void RegionsTODO::createInitialRegion() {
    volume = 1;
    Region region;
    for (unsigned symNr(0);
	 symNr < RationalFunction::getNumSymbols(); symNr++) {
      Interval ival;
      ival.first = RationalFunction::getBoundLeft(symNr);
      ival.second = RationalFunction::getBoundRight(symNr);
      region.push_back(ival);
    }
    regions.push(region);
  }
  
  void RegionsTODO::popLargestRegion(Region &region) {
    assert(0 != regions.size());
    region = regions.top();
    volume -= parametric::measure(region);
    regions.pop();
  }

  void RegionsTODO::insertRegion(const Region &region) {
    volume += parametric::measure(region);
    regions.push(region);
  }

  mpq_class RegionsTODO::measure() const {
    return volume;
  }

  const unsigned RegionsTODO::size() const {
    return regions.size();
  }
}
