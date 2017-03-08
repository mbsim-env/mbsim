/* Copyright (C) 2004-2014 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "mbsim/frames/frame.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/mbsim_event.h"
#include <openmbvcppinterface/frame.h>
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  Frame::Frame(const string &name) : Element(name), AWP(EYE), updGA(true), updPos(true), updVel(true), updAcc(true) {

    hSize[0] = 0;
    hSize[1] = 0;
    hInd[0] = 0;
    hInd[1] = 0;
    updJac[0] = true;
    updJac[1] = true;
    updJac[2] = true;
  }

  void Frame::plot() {
    if(plotFeature[11334901831169464975ULL]==enabled) {
      if(plotFeature[12307443656510933270ULL]==enabled) {
        if(updPos) updatePositions();
        for(int i=0; i<3; i++)
          plotVector.push_back(WrOP(i));
      }
      if(plotFeature[14158292694042650861ULL]==enabled) {
        if(updPos) updatePositions();
        Vec3 cardan=AIK2Cardan(AWP);
        for(int i=0; i<3; i++)
          plotVector.push_back(cardan(i));
      }
      if(plotFeature[9333706489830170160ULL]==enabled) {
        if(updVel) updateVelocities();
        for(int i=0; i<3; i++)
          plotVector.push_back(WvP(i));
      }
      if(plotFeature[10919411498317597175ULL]==enabled) {
        if(updVel) updateVelocities();
        for(int i=0; i<3; i++)
          plotVector.push_back(WomegaP(i));
      }
      if(plotFeature[1640323793842503125ULL]==enabled) {
        if(updAcc) updateAccelerations();
        for(int i=0; i<3; i++)
          plotVector.push_back(WaP(i));
      }
      if(plotFeature[1907010450089173335ULL]==enabled) {
        if(updAcc) updateAccelerations();
        for(int i=0; i<3; i++)
          plotVector.push_back(WpsiP(i));
      }
      if(plotFeature[13464197197848110344ULL]==enabled) {
        if(openMBVFrame && !openMBVFrame->isHDF5Link()) {
          if(updPos) updatePositions();
          vector<double> data;
          data.push_back(getTime());
          data.push_back(WrOP(0));
          data.push_back(WrOP(1));
          data.push_back(WrOP(2));
          Vec3 cardan=AIK2Cardan(AWP);
          data.push_back(cardan(0));
          data.push_back(cardan(1));
          data.push_back(cardan(2));
          data.push_back(0);
          openMBVFrame->append(data);
        }
      }
      Element::plot();
    }
  }

  void Frame::init(InitStage stage) {
    if(stage==unknownStage) {
      WJP[0].resize(hSize[0]);
      WJR[0].resize(hSize[0]);
      WJP[1].resize(hSize[1]);
      WJR[1].resize(hSize[1]);
    }
    else if(stage==plotting) {
      updatePlotFeatures();
  
      if(plotFeature[11334901831169464975ULL]==enabled) {
        if(plotFeature[12307443656510933270ULL]==enabled) {
          for(int i=0; i<3; i++)
            plotColumns.push_back("WrOP("+numtostr(i)+")");
        }
        if(plotFeature[14158292694042650861ULL]==enabled) {
          plotColumns.push_back("alpha");
          plotColumns.push_back("beta");
          plotColumns.push_back("gamma");
        }
        if(plotFeature[9333706489830170160ULL]==enabled) {
          for(int i=0; i<3; i++)
            plotColumns.push_back("WvP("+numtostr(i)+")");
        }
        if(plotFeature[10919411498317597175ULL]==enabled) {
          for(int i=0; i<3; i++)
            plotColumns.push_back("WomegaP("+numtostr(i)+")");
        }
        if(plotFeature[1640323793842503125ULL]==enabled) {
          for(int i=0; i<3; i++)
            plotColumns.push_back("WaP("+numtostr(i)+")");
        }
        if(plotFeature[1907010450089173335ULL]==enabled) {
          for(int i=0; i<3; i++)
            plotColumns.push_back("WpsiP("+numtostr(i)+")");
        }
  
        if(plotFeature[13464197197848110344ULL]==enabled) {
          if(openMBVFrame) {
            openMBVFrame->setName(name);
            parent->getOpenMBVGrp()->addObject(openMBVFrame);
          }
        }
        Element::init(stage);
      }
    }
    else
      Element::init(stage);
  }

  void Frame::initializeUsingXML(DOMElement *element) {
    Element::initializeUsingXML(element);

    DOMElement *ee=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(ee) {
      OpenMBVFrame ombv;
      openMBVFrame=ombv.createOpenMBV(ee); 
    }
  }

  void Frame::resetUpToDate() { 
    updJac[0] = true;
    updJac[1] = true;
    updJac[2] = true;
    updGA = true;
    updPos = true;
    updVel = true;
    updAcc = true;
  }

  void Frame::resetPositionsUpToDate() {
    updPos = true;
  }

  void Frame::resetVelocitiesUpToDate() {
    updVel = true;
  }

  void Frame::resetJacobiansUpToDate() {
    updJac[0] = true;
    updJac[1] = true;
    updJac[2] = true;
  }

  void Frame::resetGyroscopicAccelerationsUpToDate() {
    updGA = true;
  }

}
