#include "ModelExporter.h"

#include <fstream>
#include "rationalFunction/RationalFunction.h"
#include "ExprToNumber.h"
#include "PMC.h"
#include "prismparser/Expr.h"

namespace parametric {
  using namespace std;
  using namespace rational;

  ModelExporter::ModelExporter() {
    stream = NULL;
  }

  ModelExporter::~ModelExporter() {
    if (NULL != stream) {
      delete stream;
    }
  }

  void ModelExporter::setModel(PMM &pmm_) {
    pmm = &pmm_;
    if (PMM::PMC == pmm->getModelType()) {
      pmc = (PMC *) pmm;
    }
  }
  
  void ModelExporter::setOutput(const std::string &fn) {
    outputFilename = fn;
    stream = new ofstream(outputFilename.c_str());
  }

  void ModelExporter::setFormat(const std::string &format_) {
    format = format_;
  }

  void ModelExporter::setUseRewards(bool useRewards_) {
    useRewards = useRewards_;
  }

  void ModelExporter::setExprToNumber(const ExprToNumber &exprToNumber_) {
    exprToNumber = &exprToNumber_;
  }

  void ModelExporter::write() {
    if ("dot" == format) {
      exportDOT();
    }
  }

  /**
   * Export model to output stream @a stream in GraphViz format.
   *
   * @param stream stream to print model to
   */
  void ModelExporter::exportDOT() {
    (*stream) << "digraph G {" << endl;
    
    /* print states include target/initial markings */
    for (PMM::state state(0); state < pmc->getNumStates(); state++) {
      *stream << "  " << state << " [label=\"" << state;
      bool firstAPWritten(false);
      for (unsigned ap(0); ap < pmc->getNumAPs(); ap++) {
	if (pmc->isAP(state, ap) && !exprToNumber->getExprByNumber(ap).isTrue()) {
	  if (firstAPWritten) {
	    *stream << " & ";
	  } else {
	    *stream << ":";
	  }
	  *stream << exprToNumber->getExprByNumber(ap);
	  firstAPWritten = true;
	}
      }
      *stream << "\"]";
      *stream << ";" << endl;
      if (pmc->isInit(state)) {
        *stream << "init_" << state <<
          " [label=\"\", fillcolor=\"black\", width=\"0.0\""
          " shape=\"circle\", style=\"filled\", root];" << endl;
        *stream << "  init_" << state << " -> "
               << state << endl;
      }
    }

    /* print transitions between states */
    for (PMM::state state(0); state < pmc->getNumStates(); state++) {
      for (unsigned succ(0); succ < pmc->getNumSuccStates(state); succ++) {
	PMM::state succState = pmc->getSuccState(state, succ);
	RationalFunction succProb = pmc->getSuccProb(state, succ);
        *stream << "  " << state << " -> "      
               << succState << " [label=\"" << succProb;
        if (useRewards) {
	  RationalFunction succReward = pmc->getSuccReward(state, succ);
          (*stream) << " // " << succReward;
        }
        *stream << "\",labelfontsize=10];" << endl;
      }    
    }
    *stream << "}" << endl;
  }
}
