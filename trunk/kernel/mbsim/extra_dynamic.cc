/* Copyright (C) 2004-2009 MBSim Development Team
 *
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 *  
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * Lesser General Public License for more details. 
 *  
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: mbachmayer@gmx.de
 */

#include <config.h>
#include "mbsim/extra_dynamic.h"
#include "mbsim/object.h"
#include "mbsim/link.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/utils/utils.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  ExtraDynamic::ExtraDynamic(const string &name) : Element(name), y(0) {
  }

  void ExtraDynamic::updatexRef(const Vec& xParent) {
    x >> xParent(xInd,xInd+xSize-1);
  }

  void ExtraDynamic::updatexdRef(const Vec& xdParent) {
    xd >> xdParent(xInd,xInd+xSize-1);
  }

  void ExtraDynamic::initz() {
    x = x0;
  }

  void ExtraDynamic::closePlot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      Element::closePlot();
    }
  }

  void ExtraDynamic::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      if(getPlotFeature(state)==enabled)
        for(int i=0; i<xSize; ++i)
          plotVector.push_back(x(i));
      if(getPlotFeature(stateDerivative)==enabled)
        for(int i=0; i<xSize; ++i)
          plotVector.push_back(xd(i)/dt);

      Element::plot(t,dt);
    }
  }

  void ExtraDynamic::init(InitStage stage) {
    if(stage==MBSim::plot) {
      updatePlotFeatures(parent);
  
      if(getPlotFeature(plotRecursive)==enabled) {
        if(getPlotFeature(state)==enabled)
          for(int i=0; i<xSize; ++i)
            plotColumns.push_back("x("+numtostr(i)+")");
        if(getPlotFeature(stateDerivative)==enabled)
          for(int i=0; i<xSize; ++i)
            plotColumns.push_back("xd("+numtostr(i)+")");
  
        Element::init(stage, parent);
      }
    }
    else
      Element::init(stage, parent);
  }

  Element * ExtraDynamic::getByPathSearch(string path) {
    if (path.substr(0, 3)=="../") // relative path
      return parent->getByPathSearch(path.substr(3));
    else // absolut path
      return ds->getByPathSearch(path.substr(3));
  }


}

