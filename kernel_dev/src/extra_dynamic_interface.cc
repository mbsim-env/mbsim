/* Copyright (C) 2006 Mathias Bachmayer
 
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
 * Institute of Applied Mechanics
 * Technical University of Munich
 *
 * 
 * Contact:
 *    rzander@users.berlios.de
 *    
 */

#include <config.h>
#include "extra_dynamic_interface.h"
#include "object.h"
#include "multi_body_system.h"
#include "function.h"

namespace MBSim {

  ExtraDynamicInterface::ExtraDynamicInterface(const string &name) : Element(name){
    xSize=0;
    y=Vec(1,INIT,0);
  }
  ExtraDynamicInterface::ExtraDynamicInterface(const string &name, int xSize_) : Element(name){
    xSize=xSize_;
    y=Vec(1,INIT,0);
  }


  void ExtraDynamicInterface::init() {
  }
  void ExtraDynamicInterface::initz() {
    x=x0;
  }
  void ExtraDynamicInterface::writex(){
    string fname="PREINTEG/"+getFullName()+".x0.asc";  
    ofstream osx(fname.c_str(), ios::out);
    osx << x;
    osx.close();
  }
  void ExtraDynamicInterface::readx0(){
    string fname="PREINTEG/"+getFullName()+".x0.asc";  
    ifstream isx(fname.c_str());
    if(isx) isx >> x0;
    else {cout << "EDI " << name << ": No Preintegration Data x0 available. Run Preintegration first." << endl; throw 50;} 
    isx.close();
  }

  void ExtraDynamicInterface::updatexRef(const Vec& xParent) {
    x >> xParent(xInd,xInd+xSize-1);
  }

  void ExtraDynamicInterface::updatexdRef(const Vec& xdParent) {
    xd >> xdParent(xInd,xInd+xSize-1);
  }

  void ExtraDynamicInterface::plot(double t, double dt, bool top) {
    if(getPlotFeature(plotRecursive)==enabled) {
      Element::plot(t,dt,false);

      if(getPlotFeature(state)==enabled)
        for(int i=0; i<xSize; ++i)
          plotVector.push_back(x(i));
      if(getPlotFeature(stateDerivative)==enabled)
        for(int i=0; i<xSize; ++i)
          plotVector.push_back(xd(i)/dt);

      if(top && plotColumns.size()>1)
        plotVectorSerie->append(plotVector);
    }
  }

  void ExtraDynamicInterface::initPlot(bool top) {
    Element::initPlot(parent, true, false);

    if(getPlotFeature(plotRecursive)==enabled) {
      if(getPlotFeature(state)==enabled)
        for(int i=0; i<xSize; ++i)
          plotColumns.push_back("x("+numtostr(i)+")");
      if(getPlotFeature(stateDerivative)==enabled)
        for(int i=0; i<xSize; ++i)
          plotColumns.push_back("xd("+numtostr(i)+")");

      if(top) createDefaultPlot();
    }
  }

  void ExtraDynamicInterface::closePlot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      Element::closePlot();
    }
  }

}
