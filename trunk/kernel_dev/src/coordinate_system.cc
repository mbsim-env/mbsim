/* Copyright (C) 2004-2006  Martin Förg

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
 * Contact:
 *   mfoerg@users.berlios.de
 *
 */
#include <config.h>
#include "coordinate_system.h"
#include "object.h"
#include "subsystem.h"
#include "function.h"
#ifdef HAVE_AMVIS
#include "kos.h"
#include "data_interface_base.h"
#include "rotarymatrices.h"
using namespace AMVis;
int MBSim::CoordinateSystem::kosAMVisCounter=0;
#endif

namespace MBSim {

  CoordinateSystem::CoordinateSystem(const string &name) : Element(name), parent(0), adress(0), WrOP(3), WvP(3), WomegaP(3), AWP(3), WjP(3), WjR(3) {

#ifdef HAVE_AMVIS
    kosAMVis = NULL;
#endif

    hSize[0] = 0;
    hSize[1] = 0;
    hInd[0] = 0;
    hInd[1] = 0;
    AWP(0,0) = 1;
    AWP(1,1) = 1;
    AWP(2,2) = 1;
    WJP.resize(3,0);
    WJR.resize(3,0);
  }

  // string CoordinateSystem::getFullName() const {
  //   return parent->getFullName() + "." + name;
  // }

  void CoordinateSystem::init() {

    getJacobianOfTranslation().resize(3,hSize[0]);
    getJacobianOfRotation().resize(3,hSize[0]);
  }

  void CoordinateSystem::resizeJacobians() {

    getJacobianOfTranslation().resize();
    getJacobianOfRotation().resize();
  }

  void CoordinateSystem::resizeJacobians(int j) {

    getJacobianOfTranslation().resize(3,hSize[j]);
    getJacobianOfRotation().resize(3,hSize[j]);
  }

  //int CoordinateSystem::gethInd(Subsystem* sys) {
  //  return parent->gethInd(sys);
  // }


#ifdef HAVE_AMVIS
  void CoordinateSystem::setAMVisKosSize(double size) {
    if(size>0) {
      kosAMVis=new Kos("XXX"+numtostr(kosAMVisCounter)+"."+name,1,false);
      kosAMVisCounter++;
      kosAMVis->setSize(size);
    }
  }
#endif

  void CoordinateSystem::plot(double t, double dt, bool top) {
    if(getPlotFeature(plotRecursive)==enabled) {
      Element::plot(t,dt,false);

      if(getPlotFeature(globalPosition)==enabled) {
        for(int i=0; i<3; i++)
          plotVector.push_back(WrOP(i));
        Vec cardan=AIK2Cardan(AWP);
        for(int i=0; i<3; i++)
          plotVector.push_back(cardan(i));
      }

      if(top && plotColumns.size()>1)
        plotVectorSerie->append(plotVector);

#ifdef HAVE_AMVIS
      if(kosAMVis && getPlotFeature(amvis)==enabled) {
        Vec cardan=AIK2Cardan(AWP);
        kosAMVis->setTime(t);
        kosAMVis->setTranslation(WrOP(0),WrOP(1),WrOP(2));
        kosAMVis->setRotation(cardan(0),cardan(1),cardan(2));
        kosAMVis->appendDataset(0);
      }
#endif
    }
  }

  void CoordinateSystem::initPlot(bool top) {
    Element::initPlot(parent, true, false);

    if(getPlotFeature(plotRecursive)==enabled) {
      if(getPlotFeature(globalPosition)==enabled) {
        for(int i=0; i<3; i++)
          plotColumns.push_back("WrOP("+numtostr(i)+")");
        plotColumns.push_back("alpha");
        plotColumns.push_back("beta");
        plotColumns.push_back("gamma");
      }

      if(top) createDefaultPlot();

#ifdef HAVE_AMVIS
      if(kosAMVis && getPlotFeature(amvis)==enabled) {
        kosAMVis->writeBodyFile();  
        kosAMVis->setColor(0);
      }
#endif
    }
  }

  void CoordinateSystem::closePlot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      Element::closePlot();
    }
  }

}
