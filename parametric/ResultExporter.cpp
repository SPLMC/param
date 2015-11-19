#include <vector>
#include <list>
#include <algorithm>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cmath>
#include <set>
#include <tr1/unordered_set>
#include <gmpxx.h>
#include "rationalFunction/RationalFunction.h"
#include "RegionResult.h"
#include "ResultExporter.h"

namespace parametric {
  using namespace std;
  using namespace std::tr1;
  using namespace rational;

  ResultExporter::ResultExporter() {
    regionsTrueColor = "white";
    regionsFalseColor = "black";
    regionsDontknowColor = "gray";
  }

  ResultExporter::~ResultExporter() {
  }

  void ResultExporter::setOutputPrefix(const string &outputPrefix__) {
    outputPrefix = outputPrefix__;
  }

  void ResultExporter::setOutputFormat(const string &outputFormat__) {
    outputFormat = outputFormat__;
  }

  void ResultExporter::setResult(const RegionResult &result__) {
    result = &result__;
  }

  void ResultExporter::setPlotStep(const mpq_class &plotStep__) {
    plotStep = plotStep__;
  }

  void ResultExporter::setPlotStep(const string &plotStep__) {
    plotStep = plotStep__;
  }

  void ResultExporter::setMinimize(bool minimize__) {
    minimize = minimize__;
  }

  void ResultExporter::setRegionsPlotLines(bool __plotLines) {
    regionsPlotLines = __plotLines;
  }

  void ResultExporter::setRegionsTrueColor
  (const string &__regionsTrueColor) {
    regionsTrueColor = __regionsTrueColor;
  }

  void ResultExporter::setRegionsFalseColor
  (const string &__regionsFalseColor) {
    regionsFalseColor = __regionsFalseColor;
  }

  void ResultExporter::setRegionsDontknowColor
  (const string &__regionsDontknowColor) {
    regionsDontknowColor = __regionsDontknowColor;
  }

  void ResultExporter::write() {
    if ("regions" == outputFormat) {
      exportRegionMap();
    } else if ("gnuplot" == outputFormat) {
      exportGnuplotFile();
      exportDATFile();
    } else if ("dat" == outputFormat) {
      exportDATFile();
    } else if ("pgf-regions" == outputFormat) {
      exportPGFRegionsFile();
    } else if ("pgfplots" == outputFormat) {
      exportPGFPlotsFile();
      exportDATFile();
    } else {
      throw runtime_error("Unknown output format \"" + outputFormat + "\"");
    }
  }

  void ResultExporter::exportRegion
  (const Region &region, ostream &out) {
    out << "[";
    for (unsigned symNr(0); symNr < region.size(); symNr++) {
      out << "[" << region[symNr].first << ", "
	  << region[symNr].second << "]";
      if (symNr != region.size() - 1) {
	out << " ";
      }
    }
    out << "]";
  }

  void ResultExporter::exportPGFRegionsFile() {
    const string outFilename(outputPrefix + ".tex");
    ofstream out(outFilename.c_str(), ios::out);
    //    const unsigned numSymbols(RationalFunction::getNumSymbols());
    mpq_class width0(RationalFunction::getBoundRight(0)
		     - RationalFunction::getBoundLeft(0));
    mpq_class width1(RationalFunction::getBoundRight(1)
		     - RationalFunction::getBoundLeft(1));
    mpq_class width(10 / max(width0, width1));

    out << "\\begin{tikzpicture}[scale=" << width << "]\n";
    out << "\\draw[rectangle,fill,color=" + regionsDontknowColor + "] ";
    out << "(" << RationalFunction::getBoundLeft(0) << ",";
    out << RationalFunction::getBoundLeft(1) << ") rectangle (";
    out << RationalFunction::getBoundRight(0) << ",";
    out << RationalFunction::getBoundRight(1) << ");\n";

    RegionResult::const_iterator rit;
    for (rit = result->begin(); rit != result->end(); rit++) {
      const Region &region(rit->first);
      const Result &rresult(rit->second);
      string color((1 == rresult[0]) ? regionsTrueColor : regionsFalseColor);
      out << "\\draw[rectangle,fill,color=" + color;
      if (regionsPlotLines) {
	string lineColor((1 == rresult[0]) ? "black" : "white");
	out << ",draw=" << lineColor;
      }
      out  << "] ";
      out << "(" << region[0].first << ","
	  << region[1].first << ")";
      out << " rectangle ";
      out << "(" << region[0].second << ","
	  << region[1].second << ");\n";
    }
    out << "\\end{tikzpicture}\n";
  }

  void ResultExporter::exportRegionMap() {
    const string outFilename(outputPrefix + ".out");
    ofstream out(outFilename.c_str(), ios::out);

    const unsigned numSymbols(RationalFunction::getNumSymbols());
    out << (minimize ? "MIN" : "MAX") << "\n";
    out << "[";
    for (unsigned symNr(0); symNr < numSymbols; symNr++) {
      out << RationalFunction::getSymbolName(symNr);
      if (symNr != numSymbols - 1) {
	out << ", ";
      }
    }
    out << "]\n";
 
    RegionResult::const_iterator rit;
    for (rit = result->begin(); rit != result->end(); rit++) {
      exportRegion(rit->first, out);
      out << "\n";
      const Result &r(rit->second);
      Result::const_iterator it2;
      for (it2 = r.begin(); it2 != r.end(); it2++) {
	out << "  " << *it2 << "\n";
      }
    }
  }

  void ResultExporter::exportGnuplotFile() {
    const unsigned numSymbols(RationalFunction::getNumSymbols());
    if (2 < numSymbols) {
      throw runtime_error("gnuplot output can only be generated for one or "
			  "two parameters");
    }
    const string gpiFilename(outputPrefix + ".gpi");
    ofstream out(gpiFilename.c_str(), ios::out);
    if (1 == numSymbols) {
      out << "set xrange[" << RationalFunction::getBoundLeft(0).get_d() << ":"
	  << RationalFunction::getBoundRight(0).get_d() << "]\n"
	  << "set term postscript eps font \"Helvetica, 25\"\n"
	  << "set output \"" << outputPrefix << ".eps\"\n"
	  << "set xlabel \"" << RationalFunction::getSymbolName(0)
	  << "\" font \"Helvetica Italic, 25\"\n"
	  << "plot \"" << outputPrefix << ".dat\" with lines title \""
	  << outputPrefix << "\" lt rgb \"black\"\n";
    } else {
      out << "set data style lines\n"
	  << "set dgrid3d 20,20,3\n"
	  << "set xrange[" << RationalFunction::getBoundLeft(0).get_d() << ":"
	  << RationalFunction::getBoundRight(0).get_d() << "]\n"
	  << "set yrange[" << RationalFunction::getBoundLeft(1).get_d() << ":"
	  << RationalFunction::getBoundRight(1).get_d() << "]\n"
	  << "set term postscript eps font \"Helvetica, 25\"\n"
	  << "set output \"" << outputPrefix << ".eps\"\n"
	  << "set xlabel \"" << RationalFunction::getSymbolName(0)
	  << "\" font \"Helvetica Italic, 25\"\n"
	  << "set ylabel \"" << RationalFunction::getSymbolName(1)
	  << "\" font \"Helvetica Italic, 25\"\n"
	  << "set ztics 0.4\n"
	  << "splot \"" << outputPrefix << ".dat\" title \""
	  << outputPrefix << "\" lt rgb \"black\"\n";
    }
  }

  void ResultExporter::exportPGFPlotsFile() {
    const unsigned numSymbols(RationalFunction::getNumSymbols());
    if (2 < numSymbols) {
      throw runtime_error("pgfplots output can only be generated for one or "
			  "two parameters");
    }
    const string pgfFilename(outputPrefix + ".tex");
    ofstream out(pgfFilename.c_str(), ios::out);
    out << "\\begin{tikzpicture}\n\\begin{axis}\n";

    if (1 == numSymbols) {
#if 0
      out << "set xrange[" << RationalFunction::getBoundLeft(0).get_d() << ":"
	  << RationalFunction::getBoundRight(0).get_d() << "]\n"
	  << "set term postscript eps font \"Helvetica, 25\"\n"
	  << "set output \"" << outputPrefix << ".eps\"\n"
	  << "set xlabel \"" << RationalFunction::getSymbolName(0)
	  << "\" font \"Helvetica Italic, 25\"\n"
	  << "plot \"" << outputPrefix << ".dat\" with lines title \""
	  << outputPrefix << "\" lt rgb \"black\"\n";
#endif
    } else {
      out << "\\addplot3[mesh,color=black] file {";
      out << outputPrefix << ".dat};\n";
    }
    out << "\\end{axis}\n\\end{tikzpicture}\n";
 
  }

  void ResultExporter::exportDATFile() {
    unordered_set<RationalFunction> values;
    RegionResult::const_iterator rit;
    for (rit = result->begin(); rit != result->end(); rit++) {
      values.insert((rit->second)[0]);
    }

    ResultPoints points;
    map<vector<mpq_class>, RationalFunction > pointsKnown;
    unsigned pointsPerAxis(18);
    mpq_class xWidth((RationalFunction::getBoundRight(0)
		      - RationalFunction::getBoundLeft(0))
		     / (pointsPerAxis - 1));
    mpq_class yWidth((RationalFunction::getBoundRight(1)
		      - RationalFunction::getBoundLeft(1))
		     / (pointsPerAxis - 1));
    for (unsigned xPointNr(0); xPointNr < pointsPerAxis; xPointNr++) {
      mpq_class xCoord(RationalFunction::getBoundLeft(0)
		       + xPointNr * xWidth);
      for (unsigned yPointNr(0); yPointNr < pointsPerAxis; yPointNr++) {
	mpq_class yCoord(RationalFunction::getBoundLeft(1)
			 + yPointNr * yWidth);
	ResultPoint point;
	point.first.push_back(xCoord);
	point.first.push_back(yCoord);
	unordered_set<RationalFunction>::iterator it;
	point.second = (*values.begin()).evaluate(point.first);
	for (it = values.begin(); it != values.end(); it++) {
	  if (minimize) {
	    point.second = min(point.second, it->evaluate(point.first));
	  } else {
	    point.second = max(point.second, it->evaluate(point.first));
	  }
	}
	points.push_back(point);
      }
    }

    const string datFilename(outputPrefix + ".dat");
    ofstream out(datFilename.c_str(), ios::out);
    mpq_class lastx(points[0].first[0]);
    for (unsigned i(0); i < points.size(); i++) {
      const ResultPoint &point(points[i]);
      if (point.first[0] != lastx) {
	lastx = point.first[0];
	out << "\n";
      }
      for (unsigned symNr(0); symNr < 2; symNr++) {
	out << point.first[symNr].get_d() << "  ";
      }
      out << point.second.get_d() << "\n";
    }
  }
}
