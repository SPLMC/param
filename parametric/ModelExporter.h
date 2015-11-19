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

 * You should have received a copy of the GNU General Public License
 * along with PARAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2009-2011 Ernst Moritz Hahn (emh@cs.uni-saarland.de)
 */

#ifndef MODEL_EXPORTER_H
#define MODEL_EXPORTER_H

#include <iosfwd>
#include <string>

namespace parametric {
  class PMM;
  class PMC;
  class ExprToNumber;
  
  class ModelExporter {
  private:
    enum Format {dot, plain};
  public:
    ModelExporter();
    ~ModelExporter();
    void setModel(PMM &);
    void setOutput(const std::string &);
    void setFormat(const std::string &);
    void setExprToNumber(const ExprToNumber &);
    void setUseRewards(bool);
    void write();
  private:
    void exportDOT();

    PMM *pmm;
    PMC *pmc;
    std::ofstream *stream;
    std::string outputFilename;
    std::string format;
    bool useRewards;
    const ExprToNumber *exprToNumber;
  };
}

#endif
