/* Copyright (C) 2004-2009 MBSim Development Team
 * 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 * 
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. 
 *
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "mbsim/link_mechanics.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/contour.h"
#include "mbsim/utils/eps.h"
#include "mbsim/utils/utils.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/group.h>
#include <openmbvcppinterface/arrow.h>
#endif

using namespace fmatvec;
using namespace std;

namespace MBSim {

  LinkMechanics::LinkMechanics(const std::string &name) : Link(name) {
#ifdef HAVE_OPENMBVCPPINTERFACE
    openMBVForceGrp=0;
#endif
  }

  LinkMechanics::~LinkMechanics() {}

  void LinkMechanics::updatedhdz(double t) {
    throw;
  }

  void LinkMechanics::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      for(unsigned int i=0; i<openMBVArrowF.size(); i++) {
        if(openMBVArrowF[i]) {
          vector<double> data;
          data.push_back(t); 
          FVec toPoint=frame[i]->getPosition();
          data.push_back(toPoint(0));
          data.push_back(toPoint(1));
          data.push_back(toPoint(2));
          data.push_back(WF[i](0));
          data.push_back(WF[i](1));
          data.push_back(WF[i](2));
          data.push_back(nrm2(WF[i]));
          openMBVArrowF[i]->append(data);
        }
      }
      for(unsigned int i=0; i<openMBVArrowM.size(); i++) {
        if(openMBVArrowM[i]) {
          vector<double> data;
          data.push_back(t); 
          FVec toPoint=frame[i]->getPosition();
          data.push_back(toPoint(0));
          data.push_back(toPoint(1));
          data.push_back(toPoint(2));
          data.push_back(WM[i](0));
          data.push_back(WM[i](1));
          data.push_back(WM[i](2));
          data.push_back(nrm2(WM[i]));
          openMBVArrowM[i]->append(data);
        }
      }
#endif
      Link::plot(t,dt);
    }
  }

  void LinkMechanics::closePlot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      Link::closePlot();
    }
  }

  void LinkMechanics::init(InitStage stage) {
    if(stage==unknownStage) {
      Link::init(stage);

      for(unsigned int i=0; i<frame.size(); i++) {
        WF.push_back(FVec());
        WM.push_back(FVec());
        fF.push_back(FVMat(laSize));
        fM.push_back(FVMat(laSize));
      }
#ifdef HAVE_OPENMBVCPPINTERFACE
      assert(openMBVArrowF.size()==0 || openMBVArrowF.size()==frame.size());
      assert(openMBVArrowM.size()==0 || openMBVArrowM.size()==frame.size());
#endif

      for(unsigned int i=0; i<contour.size(); i++) {
        WF.push_back(FVec());
        WM.push_back(FVec());
        fF.push_back(FVMat(laSize));
        fM.push_back(FVMat(laSize));
      }
    }
    else if(stage==MBSim::plot) {
      updatePlotFeatures();

      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        openMBVForceGrp=new OpenMBV::Group;
        openMBVForceGrp->setExpand(false);
        openMBVForceGrp->setName(name+"_ArrowGroup");
        parent->getOpenMBVGrp()->addObject(openMBVForceGrp);
        for(unsigned int i=0; i<openMBVArrowF.size(); i++) {
          if(openMBVArrowF[i]) {
            openMBVArrowF[i]->setName("Force_"+numtostr((int)i));
            openMBVForceGrp->addObject(openMBVArrowF[i]);
          }
        }
        for(unsigned int i=0; i<openMBVArrowM.size(); i++) {
          if(openMBVArrowM[i]) {
            openMBVArrowM[i]->setName("Moment_"+numtostr((int)i));
            openMBVForceGrp->addObject(openMBVArrowM[i]);
          }
        }
#endif
        Link::init(stage);
      }
    }
    else
      Link::init(stage);
  }

  void LinkMechanics::connect(Frame *frame_) {
    frame.push_back(frame_);
  }

  void LinkMechanics::connect(Contour *contour_) {
    contour.push_back(contour_);
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void LinkMechanics::setOpenMBVForceArrow(OpenMBV::Arrow *arrow, const vector<bool>& which) {
    for(unsigned int i=0; i<which.size(); i++) {
      if(which[i]==true)
        openMBVArrowF.push_back(new OpenMBV::Arrow(*arrow));
      else
        openMBVArrowF.push_back(NULL);
    }
  }

  void LinkMechanics::setOpenMBVMomentArrow(OpenMBV::Arrow *arrow, const vector<bool>& which) {
    for(unsigned int i=0; i<which.size(); i++) {
      if(which[i]==true)
        openMBVArrowM.push_back(new OpenMBV::Arrow(*arrow));
      else
        openMBVArrowM.push_back(NULL);
    }
  }
#endif

}

