#include <iostream>
#include <fstream>
#include "SparseNonDet.h"

using namespace std;

namespace parametric {
  SparseNonDet::SparseNonDet() {
    clear();
  }

  /**
   * Reset sparse matrix to initial state.
   */
  void SparseNonDet::clear() {
    numStates = 0;
    vertices_added = 0;
    nonZeros.clear();
    transRewards.clear();
    cols.clear();
    rows.clear();
    rows.push_back(0);
    init.clear();
    transReward.clear();
    choices.clear();
    choices.push_back(0);
  }

  /**
   * Print sparse matrix to cout.
   */
  void SparseNonDet::print() {
    print(cout);
  }

  /**
   * Print sparse matrix to @a stream.
   *
   * @param stream output stream to print to
   */
  void SparseNonDet::print(ostream &stream) {
    cout << "rows.size() = " << rows.size() << "\n"
         << "choices.size() = " << choices.size() << "\n"
         << "cols.size() = " << cols.size() << "\n"
         << "nonZeros.size() = " << nonZeros.size() << endl;
    for (unsigned i = 0; i < rows.size(); i++) {
      stream << "rows[" << i << "] = " << rows[i] << endl;
    }
    for (unsigned i = 0; i < choices.size(); i++) {
      stream << "choices[" << i << "] = " << choices[i] << endl;
    }
    for (unsigned i = 0; i < cols.size(); i++) {
      stream << "cols,nz[" << i << "] = " << cols[i] << " "
	   << nonZeros[i] << endl;
    }
    for (unsigned i = 0; i < init.size(); i++) {
      stream << "init[" << i << "] = " << init[i] << endl;
    }
  }

  /**
   * Print sparse matrix in MRMC format to cout
   */
  void SparseNonDet::printMRMCTra() {
#if 0
    printMRMCTra(cout);
#endif
  }

  /**
   * Print sparse matrix in MRMC format to @a filename.
   *
   * @param filename file to print to
   */
  void SparseNonDet::printMRMCTra(const string &filename) {
#if 0
    ofstream file(filename.c_str (), ios::out);
    printMRMCTra(file);
    file.close();
#endif
  }

  /**
   * Print sparse matrix in MRMC format to @a stream
   *
   * @param stream output stream to print to
   */
  void SparseNonDet::printMRMCTra(ostream &stream) {
#if 0
    stream << "STATES " << numStates << endl;
    stream << "TRANSITIONS " << nonZeros.size() << endl;
    for (unsigned stateNr = 0; stateNr < rows.size() - 1; stateNr++) {
      unsigned stateBegin = rows[stateNr];
      unsigned stateEnd = rows[stateNr + 1];
      for (unsigned traNr = stateBegin; traNr < stateEnd; traNr++) {
	stream << (stateNr+1) << " " << (cols[traNr]+1) << " "
	       << nonZeros[traNr] << endl;
      }
    }
#endif
  }

  /**
   * Print MRMC state labelling to cout.
   */
  void SparseNonDet::printMRMCLab() {
#if 0
    printMRMCLab(cout);
#endif
  }

  /**
   * Print MRMC state labelling to @a filename.
   *
   * @param filename file to print to
   */
  void SparseNonDet::printMRMCLab(const string &filename) {
#if 0
    ofstream file (filename.c_str (), ios::out);
    printMRMCLab(file);
    file.close ();
#endif
  }

  /**
   * Print MRMC state labelling to @a sream.
   *
   * @param stream output stream to print to
   */
  void SparseNonDet::printMRMCLab(ostream &stream) {
#if 0
    stream << "#DECLARATION" << endl;
    stream << "target sink border" << endl;
    stream << "#END" << endl;
    stream << "1 sink" << endl;
    stream << "2 target" << endl;
    for (unsigned state = rows.size(); state < numStates; state++) {
      cout << (state + 1) << " border" << endl;
    }
#endif
  }
}
