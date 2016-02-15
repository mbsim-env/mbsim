/* Copyright (C) 2004-2010 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "mbsim/constraint.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/frame.h"
#include "mbsim/utils/nonlinear_algebra.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/joint.h"
#include "mbsim/gear.h"
#include "mbsim/kinematic_excitation.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/objectfactory.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/frame.h>
#endif

#include <hdf5serie/simpledataset.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  Constraint::Constraint(const std::string &name) : Element(name), updGC(true), updGJ(true) {
    setPlotFeature(state, enabled);
  }

  void Constraint::updatexRef(const Vec &xParent) {
    x >> xParent(xInd,xInd+xSize-1);
  } 

  void Constraint::updatexdRef(const Vec &xdParent) {
    xd >> xdParent(xInd,xInd+xSize-1);
  } 

  void Constraint::init(InitStage stage) {
    if(stage==plotting) {
      updatePlotFeatures();

      if(getPlotFeature(plotRecursive)==enabled) {
        if(getPlotFeature(state)==enabled)
          for(int i=0; i<xSize; ++i)
            plotColumns.push_back("x("+numtostr(i)+")");
        if(getPlotFeature(stateDerivative)==enabled)
          for(int i=0; i<xSize; ++i)
            plotColumns.push_back("xd("+numtostr(i)+")");

        Element::init(stage);
      }
    }
    else
      Element::init(stage);
  }

  void Constraint::initz() {
    x = (x0.size()==0)? Vec(xSize, INIT, 0) : x0;
  }

  void Constraint::writez(H5::GroupBase *group) {
    group->createChildObject<H5::SimpleDataset<vector<double> > >("x0")(x.size())->write(x);
  }

  void Constraint::readz0(H5::GroupBase *group) {
    x0.resize() = group->openChildObject<H5::SimpleDataset<vector<double> > >("x0")->read();
  }

  void Constraint::plot(double t, double dt) {
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

  void Constraint::closePlot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      Element::closePlot();
    }
  }

}
