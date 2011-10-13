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
#include "mbsim/frame.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/rotarymatrices.h"
//#include "mbsim/object_interface.h"
#include "mbsim/mbsim_event.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/frame.h>
#include "openmbvcppinterface/group.h"
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  Frame::Frame(const string &name) : Element(name), WrOP(3,INIT,0.), AWP(3,INIT,0.), WvP(3,INIT,0.), WomegaP(3,INIT,0.), WaP(3,INIT,0), WpsiP(3,INIT,0) {
    AWP(0,0) = 1;
    AWP(1,1) = 1;
    AWP(2,2) = 1;

    hSize[0] = 0;
    hSize[1] = 0;
    hInd[0] = 0;
    hInd[1] = 0;
    WJP[0].resize(3,0);
    WJR[0].resize(3,0);
    WJP[1].resize(3,0);
    WJR[1].resize(3,0);
    WjP[0].resize(3,INIT,0.);
    WjR[0].resize(3,INIT,0.);
    WjP[1].resize(3,INIT,0.);
    WjR[1].resize(3,INIT,0.);

#ifdef HAVE_OPENMBVCPPINTERFACE
    openMBVFrame=0;
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
      if(getPlotFeature(globalVelocity)==enabled) {
        for(int i=0; i<3; i++)
          plotVector.push_back(WvP(i));
        for(int i=0; i<3; i++)
          plotVector.push_back(WomegaP(i));
      }
      if(getPlotFeature(globalAcceleration)==enabled) {
        for(int i=0; i<3; i++)
          plotVector.push_back(WaP(i));
        for(int i=0; i<3; i++)
          plotVector.push_back(WpsiP(i));
      }
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled && openMBVFrame && !openMBVFrame->isHDF5Link()) {
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
        openMBVFrame->append(data);
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

  void Frame::init(InitStage stage) {
    if(stage==unknownStage) {
      getJacobianOfTranslation(0).resize(3,hSize[0]);
      getJacobianOfRotation(0).resize(3,hSize[0]);
      getJacobianOfTranslation(1).resize(3,hSize[1]);
      getJacobianOfRotation(1).resize(3,hSize[1]);
    }
    else if(stage==MBSim::plot) {
      updatePlotFeatures();
  
      if(getPlotFeature(plotRecursive)==enabled) {
        if(getPlotFeature(globalPosition)==enabled) {
          for(int i=0; i<3; i++)
            plotColumns.push_back("WrOP("+numtostr(i)+")");
          plotColumns.push_back("alpha");
          plotColumns.push_back("beta");
          plotColumns.push_back("gamma");
        }
        if(getPlotFeature(globalVelocity)==enabled) {
          for(int i=0; i<3; i++)
            plotColumns.push_back("WvP("+numtostr(i)+")");
          for(int i=0; i<3; i++)
            plotColumns.push_back("WomegaP("+numtostr(i)+")");
        }
        if(getPlotFeature(globalAcceleration)==enabled) {
          for(int i=0; i<3; i++)
            plotColumns.push_back("WaP("+numtostr(i)+")");
          for(int i=0; i<3; i++)
            plotColumns.push_back("WpsiP("+numtostr(i)+")");
        }
  
  #ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled && openMBVFrame) {
          openMBVFrame->setName(name);
          // The next line adds redundant data to the h5 file (if the parent is a RigidBody or a Group)
          // but this line of code work for always!
          parent->getOpenMBVGrp()->addObject(openMBVFrame);
          // if the parent is a Group; TODO
          // BEGIN
          //RigidBody *rigidBody;
          //if((rigidBody=dynamic_cast<RigidBody*>(parent))!=0) {
          //  if(rigidBody->getOpenMBVBody()==0) {
          //    cout<<"To visualize a frame on a rigid body, the body must at least have a OpenMBV::InvisibleBody!"<<endl;
          //    _exit(1);
          //  }
          //  parent->getOpenMBVGrp()->addObject(openMBVFrame);
          //  openMBVFrame->setHDF5LinkTarget(rigidBody->getOpenMBVBody());
          //  openMBVFrame->setInitialTranslation((rigidBody->getContainerForFramePositions())[rigidBody->frameIndex(this)]);
          //  openMBVFrame->setInitialRotation(AIK2Cardan((rigidBody->getContainerForFrameOrientations())[rigidBody->frameIndex(this)]));
          //}
          // END
        }
  #endif
        Element::init(stage);
      }
    }
    else
      Element::init(stage);
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void Frame::enableOpenMBV(double size, double offset) {
    if(size>=0) {
      openMBVFrame=new OpenMBV::Frame;
      openMBVFrame->setSize(size);
      openMBVFrame->setOffset(offset);
    }
    else openMBVFrame=0;
  }
#endif

}

