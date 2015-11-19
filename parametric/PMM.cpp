#include <set>
#include <boost/dynamic_bitset.hpp>
#include "PMM.h"

namespace parametric {
  using namespace std;

  PMM::PMM() {
    inits = new set<PMM::state>();
    aps = NULL;
    absorbing = NULL;
  }
  
  PMM::~PMM() {
    delete inits;
    if (NULL != aps) {
      delete aps;
    }
    if (NULL != absorbing) {
      delete absorbing;
    }
  }

  void PMM::addInit(PMM::state state) {
    inits->insert(state);
  }

  bool PMM::isInit(PMM::state state) const {
    return (inits->find(state) != inits->end());
  }

  void PMM::setTimeType(TimeType timeType_) {
    timeType = timeType_;
  }

  PMM::TimeType PMM::getTimeType() const {
    return timeType;
  }

  void PMM::reserveAPMem(unsigned numStates, unsigned numAPs_) {
    numAPs = numAPs_;
    aps = new boost::dynamic_bitset<>(numStates * numAPs);
  }

  void PMM::setAP(PMM::state state, unsigned ap, bool value) {
    (*aps)[state * numAPs + ap] = value;
  }

  bool PMM::isAP(PMM::state state, unsigned ap) const {
    return (*aps)[state * numAPs + ap];
  }

  unsigned PMM::getNumAPs() const {
    return numAPs;
  }

  void PMM::setAbsorbing(PMM::state state, bool absorb) {
    (*absorbing)[state] = absorb;
  }

  PMM::state PMM::getInvalidState() const {
    return numeric_limits<unsigned>::max();
  }
}
