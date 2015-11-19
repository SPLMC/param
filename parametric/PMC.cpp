#include <limits>
#include "rationalFunction/RationalFunction.h"
#include "PMC.h"

namespace parametric {
  using namespace std;
  using namespace rational;

  PMM::ModelType PMC::getModelType() const {
    return PMM::PMC;
  }

  void copy(const PMC &input, PMC &output) {
    output.reserveRowsMem(input.getNumStates());
    output.reserveColsMem(input.getNumTrans());
    if (input.useRewards()) {
      output.reserveTransRewardsMem(input.getNumTrans());
      //      output.reserveStateRewardsMem(input.getNumStates());
    }
    for (PMM::state state(0); state < input.getNumStates(); state++) {
      if (input.isInit(state)) {
	output.addInit(state);
      }
      for (unsigned succ(0); succ < input.getNumSuccStates(state); succ++) {
	const unsigned succState(input.getSuccState(state, succ));
	RationalFunction succProb(input.getSuccProb(state, succ));
	output.addSucc(succState, succProb);
	if (input.useRewards()) {
	  RationalFunction succReward(input.getSuccReward(state, succ));
	  output.addSuccReward(succReward);
	}
      }
      if (input.useRewards()) {
	//	const rational::RationalFunction stateRew(input.getStateReward(state));
	//	output.setStateReward(stateRew);
      }
      output.finishState();
    }
  }

}
