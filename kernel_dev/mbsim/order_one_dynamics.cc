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
#include "mbsim/order_one_dynamics.h"
#include "mbsim/object.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/utils/function.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  OrderOneDynamics::OrderOneDynamics(const string &name) : Element(name) {
    xSize = 0;
    y = Vec(1,INIT,0);
  }

  OrderOneDynamics::OrderOneDynamics(const string &name, int xSize_) : Element(name) {
    xSize = xSize_;
    y = Vec(1,INIT,0);
  }

  void OrderOneDynamics::updatexRef(const Vec& xParent) {
    x >> xParent(xInd,xInd+xSize-1);
  }

  void OrderOneDynamics::updatexdRef(const Vec& xdParent) {
    xd >> xdParent(xInd,xInd+xSize-1);
  }

  void OrderOneDynamics::initz() {
    x = x0;
  }

  void OrderOneDynamics::closePlot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      Element::closePlot();
    }
  }

  void OrderOneDynamics::plot(double t, double dt) {
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

  void OrderOneDynamics::initPlot() {
    updatePlotFeatures(parent);

    if(getPlotFeature(plotRecursive)==enabled) {
      if(getPlotFeature(state)==enabled)
        for(int i=0; i<xSize; ++i)
          plotColumns.push_back("x("+numtostr(i)+")");
      if(getPlotFeature(stateDerivative)==enabled)
        for(int i=0; i<xSize; ++i)
          plotColumns.push_back("xd("+numtostr(i)+")");

      Element::initPlot(parent);
    }
  }

  void OrderOneDynamics::writex(){
//    string fname="PREINTEG/"+getPath()+".x0.asc";  
//    ofstream osx(fname.c_str(), ios::out);
//    osx << x;
//    osx.close();
  }

  void OrderOneDynamics::readx0(){
//    string fname="PREINTEG/"+getPath()+".x0.asc";  
//    ifstream isx(fname.c_str());
//    if(isx) isx >> x0;
//    else {cout << "EDI " << name << ": No Preintegration Data x0 available. Run Preintegration first." << endl; throw 50;} 
//    isx.close();
  }


}

