#ifndef SPARSE_NON_DET_H
#define SPARSE_NON_DET_H

#include <vector>
#include <iosfwd>
#include <string>

namespace parametric {
  class Model2C;

  class SparseNonDet {
  public:
    typedef std::vector<double> NonZeros;
    typedef std::vector<double> TransRewards;

    SparseNonDet();

    inline void addState() {
      numStates++;
    }

    inline void finishState() {
      rows.push_back(choices.size() - 1u);
    }

    inline void finishChoice() {
      choices.push_back(cols.size());
    }

    inline const std::vector<unsigned> &getInit() const {
      return init;
    }

    inline unsigned getNumStates() const {
      return numStates;
    }

    inline unsigned getNumFinishedStates() const {
      return rows.size() - 1u;
    }

    inline unsigned getNumTransitions() const {
      return nonZeros.size();
    }

    inline const std::vector<double> &getNonZeros() const {
      return nonZeros;
    }

    inline const std::vector<unsigned> &getCols() const {
      return cols;
    }

    inline std::pair<bool,unsigned>
      getTarget(bool isChoice, unsigned number) const {
      if (!isChoice) {
        return std::make_pair(true, choices[number]);
      } else {
        return std::make_pair(false, cols[number]);
      }
    }

    inline unsigned getStateBegin(unsigned state) const {
      return rows[state];
    }

    inline unsigned getStateEnd(unsigned state) const {
      return rows[state + 1];
    }

    inline unsigned getChoiceBegin(unsigned choice) const {
      return choices[choice];
    }

    inline unsigned getChoiceEnd(unsigned choice) const {
      return choices[choice + 1];
    }

    inline const std::vector<double> &getTransRewards() const {
      return transRewards;
    }

    inline void addTransition(unsigned from, unsigned to, double rate) {
      cols.push_back(to);
      nonZeros.push_back(rate);
    }

    inline void addTransition
      (unsigned from, unsigned to, double rate, double reward) {
      cols.push_back(to);
      nonZeros.push_back(rate);
      transRewards.push_back(reward);
    }

    inline void close() {
      while (rows.size() < numStates + 1) {
        rows.push_back(choices.size() - 1u);
        vertices_added++;
      }
    }

    inline void open() {
      for (unsigned i = 0; i < vertices_added; i++) {
        rows.pop_back();
      }
      vertices_added = 0;
    }

    inline void setInit(const std::vector<unsigned> &__init) {
      init = __init;
    }

    inline void addInit(const unsigned init__) {
      init.push_back(init__);
    }

    inline void setModel2c(Model2C *model2c__) {
      model2c = model2c__;
    }

    void print();
    void print(std::ostream &);
    void printMRMCTra();
    void printMRMCTra(const std::string &);
    void printMRMCTra(std::ostream &);
    void printMRMCLab();
    void printMRMCLab(const std::string &);
    void printMRMCLab(std::ostream &);
    void clear();

    //  private:
    unsigned numStates;
    unsigned vertices_added;
    std::vector<double> nonZeros;
    std::vector<double> transRewards;
    std::vector<unsigned> cols;
    std::vector<unsigned> rows;
    std::vector<unsigned> choices;
    std::vector<unsigned> init;
    std::vector<double> transReward;
    Model2C *model2c;
  };
}

#endif
