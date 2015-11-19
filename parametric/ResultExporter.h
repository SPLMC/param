/*
 * This file is part of PARAM.
 *
 * PARAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * PARAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with PARAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2010-2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#ifndef RESULT_EXPORTER_H
#define RESULT_EXPORTER_H

#include <string>

namespace parametric {
  class Region;
  class Result;
  class RegionResult;

  class ResultExporter {
  public:
    ResultExporter();
    ~ResultExporter();
    void setOutputPrefix(const std::string &);
    void setOutputFormat(const std::string &);
    void setResult(const RegionResult &);
    void setPlotStep(const mpq_class &);
    void setPlotStep(const std::string &);
    void setMinimize(bool);
    void setRegionsPlotLines(bool);
    void setRegionsTrueColor(const std::string &);
    void setRegionsFalseColor(const std::string &);
    void setRegionsDontknowColor(const std::string &);
    void write();

  private:
    typedef std::pair<std::vector<mpq_class>, mpq_class> ResultPoint;
    typedef std::vector<ResultPoint> ResultPoints;

    void exportRegionMap();
    void exportGnuplotFile();
    void exportPGFPlotsFile();
    void exportDATFile();
    void exportPGFRegionsFile();
    void exportRegion(const Region &, std::ostream &);

    std::string outputPrefix;
    std::string outputFormat;
    mpq_class plotStep;
    bool minimize;
    const RegionResult *result;
    bool regionsPlotLines;
    std::string regionsTrueColor;
    std::string regionsFalseColor;
    std::string regionsDontknowColor;
  };
}

#endif
