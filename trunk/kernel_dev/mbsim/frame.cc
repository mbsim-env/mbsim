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
 * Contact: mfoerg@users.berlios.de
 */

#include <config.h>
#include <mbsim/frame.h>
#include <mbsim/object.h>
#include <mbsim/dynamic_system.h>
#include <mbsim/utils/function.h>
#include <mbsim/utils/rotarymatrices.h>
#include <mbsim/rigid_body.h>
#ifdef HAVE_AMVIS
#include "kos.h"
#include <mbsim/data_interface_base.h>
using namespace AMVis;
int MBSim::Frame::kosAMVisCounter=0;
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  Frame::Frame(const string &name) : Element(name), parent(0), adress(0), WrOP(3), WvP(3), WomegaP(3), AWP(3), WjP(3), WjR(3) {

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
#ifdef HAVE_AMVISCPPINTERFACE
    amvisFrame=0;
#endif
  }
  
  void Frame::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      if(getPlotFeature(globalPosition)==enabled) {
        for(int i=0; i<3; i++)
          plotVector.push_back(WrOP(i));
        Vec cardan=AIK2Cardan(AWP);
        for(int i=0; i<3; i++)
          plotVector.push_back(cardan(i));
      }
#ifdef HAVE_AMVIS
      if(kosAMVis && getPlotFeature(amvis)==enabled) {
        Vec cardan=AIK2Cardan(AWP);
        kosAMVis->setTime(t);
        kosAMVis->setTranslation(WrOP(0),WrOP(1),WrOP(2));
        kosAMVis->setRotation(cardan(0),cardan(1),cardan(2));
        kosAMVis->appendDataset(0);
      }
#endif
#ifdef HAVE_AMVISCPPINTERFACE
      if(getPlotFeature(amvis)==enabled && amvisFrame && !amvisFrame->isHDF5Link()) {
        vector<double> data;
        data.push_back(t);
        data.push_back(WrOP(0));
        data.push_back(WrOP(1));
        data.push_back(WrOP(2));
        Vec cardan=AIK2Cardan(AWP);
        data.push_back(cardan(0));
        data.push_back(cardan(1));
        data.push_back(cardan(2));
        data.push_back(0);
        amvisFrame->append(data);
      }
#endif
      Element::plot(t,dt);
    }
  }

  void Frame::closePlot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      Element::closePlot();
    }
  }

  void Frame::initPlot() {
    updatePlotFeatures(parent);

    if(getPlotFeature(plotRecursive)==enabled) {
      if(getPlotFeature(globalPosition)==enabled) {
        for(int i=0; i<3; i++)
          plotColumns.push_back("WrOP("+numtostr(i)+")");
        plotColumns.push_back("alpha");
        plotColumns.push_back("beta");
        plotColumns.push_back("gamma");
      }

#ifdef HAVE_AMVIS
      if(kosAMVis && getPlotFeature(amvis)==enabled) {
        kosAMVis->writeBodyFile();  
        kosAMVis->setColor(0);
      }
#endif
#ifdef HAVE_AMVISCPPINTERFACE
      if(getPlotFeature(amvis)==enabled && amvisFrame) {
        amvisFrame->setName(name);
        RigidBody *rigidBody;
        parent->getAMVisGrp()->addObject(amvisFrame);
        if((rigidBody=dynamic_cast<RigidBody*>(parent))!=0) {
          if(rigidBody->amvisBody==0) {
            cout<<"To visualize a frame on a rigid body, the body must at least have a AMVis::InvisibleBody!"<<endl;
            _exit(1);
          }
          amvisFrame->setHDF5LinkTarget(rigidBody->amvisBody);
          amvisFrame->setInitialTranslation(rigidBody->SrSK[rigidBody->frameIndex(this)]);
          amvisFrame->setInitialRotation(AIK2Cardan(rigidBody->ASK[rigidBody->frameIndex(this)]));
        }
      }
#endif
      Element::initPlot(parent);
    }
  }

  void Frame::resizeJacobians() {
    getJacobianOfTranslation().resize();
    getJacobianOfRotation().resize();
  }

  void Frame::resizeJacobians(int j) {
    getJacobianOfTranslation().resize(3,hSize[j]);
    getJacobianOfRotation().resize(3,hSize[j]);
  }

  void Frame::init() {
    getJacobianOfTranslation().resize(3,hSize[0]);
    getJacobianOfRotation().resize(3,hSize[0]);
  }

#ifdef HAVE_AMVIS
  void Frame::setAMVisKosSize(double size) {
    if(size>0) {
      kosAMVis=new Kos("XXX"+numtostr(kosAMVisCounter)+"."+name,1,false);
      kosAMVisCounter++;
      kosAMVis->setSize(size);
    }
  }
#endif

#ifdef HAVE_AMVISCPPINTERFACE
  void Frame::enableAMVis(double size, double offset) {
    if(size>=0) {
      amvisFrame=new AMVis::Frame;
      amvisFrame->setSize(size);
      amvisFrame->setOffset(offset);
    }
    else amvisFrame=0;
  }
#endif
  // string Frame::getFullName() const {
  //   return parent->getFullName() + "." + name;
  // }

  //int Frame::gethInd(Subsystem* sys) {
  //  return parent->gethInd(sys);
  // }
}

