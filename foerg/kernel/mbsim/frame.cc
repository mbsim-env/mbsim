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
#include "mbsim/frame.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/rigid_body.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/mbsim_event.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/frame.h>
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/group.h>
#endif

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  Frame::Frame(const string &name) : Element(name), AWP(EYE) {

    hSize[0] = 0;
    hSize[1] = 0;
    hInd[0] = 0;
    hInd[1] = 0;
  }

  void Frame::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      if(getPlotFeature(globalPosition)==enabled) {
        for(int i=0; i<3; i++)
          plotVector.push_back(WrOP(i));
        Vec3 cardan=AIK2Cardan(AWP);
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
      if(getPlotFeature(openMBV)==enabled) {
        if(openMBVFrame && !openMBVFrame->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
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
    if(stage==resize) {
      WJP[0].resize(hSize[0]);
      WJR[0].resize(hSize[0]);
      WJP[1].resize(hSize[1]);
      WJR[1].resize(hSize[1]);
    }
    else if(stage==plotting) {
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
        if(getPlotFeature(openMBV)==enabled) {
          if(openMBVFrame) {
            openMBVFrame->setName(name);
            parent->getOpenMBVGrp()->addObject(openMBVFrame);
          }
        }
  #endif
        Element::init(stage);
      }
    }
    else
      Element::init(stage);
  }

  void Frame::initializeUsingXML(DOMElement *element) {
    Element::initializeUsingXML(element);

#ifdef HAVE_OPENMBVCPPINTERFACE
    DOMElement *ee=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(ee) {
      OpenMBVFrame ombv;
      openMBVFrame=ombv.createOpenMBV(ee); 
    }
#endif
  }

  DOMElement* Frame::writeXMLFile(DOMNode *parent) {

    DOMElement *ele0 = Element::writeXMLFile(parent);

//#ifdef HAVE_OPENMBVCPPINTERFACE
//    if(openMBVFrame) {
//      DOMElement *ele1 = new DOMElement( MBSIM%"enableOpenMBV" );
//      addElementText(ele1,MBSIM%"size",openMBVFrame->getSize());
//      addElementText(ele1,MBSIM%"offset",openMBVFrame->getOffset());
//      ele0->LinkEndChild(ele1);
//    }
//#endif
    return ele0;
  }

  void Frame::resetUpToDate() { 
    updateJac[0] = true; 
    updateJac[1] = true; 
    updatePos = true;
    updateVel = true;
    updateAcc = true;
  }

  const Vec3& Frame::getPosition(double t) {
    if(updatePos) {
      updatePos = false;
      updatePositions(t); 
    }
    return getPosition(); 
  }
  const SqrMat3& Frame::getOrientation(double t) {
    if(updatePos) {
      updatePos = false;
      updatePositions(t); 
    }
    return getOrientation(); 
  }
  const Vec3& Frame::getVelocity(double t) {
    if(updateVel) {
      updateVel = false;
      updateVelocities(t); 
    }
    return getVelocity(); 
  }
  const Vec3& Frame::getAngularVelocity(double t) {
    if(updateVel) {
      updateVel = false;
      updateVelocities(t); 
    }
    return getAngularVelocity(); 
  }
  const Vec3& Frame::getAcceleration(double t) {
    if(updateAcc) {
//      updateAcc = false;
//      updateAcceleration(t); 
//      updateAngularVelocity(t); 
    }
    return getAcceleration(); 
  }
  const Vec3& Frame::getAngularAcceleration(double t) {
    if(updateAcc) {
//      updateAcc = false;
//      updateAcceleration(t); 
//      updateAngularVelocity(t); 
    }
    return getAngularAcceleration(); 
  }


  const Mat3xV& Frame::getJacobianOfTranslation(double t, int j) {
//    cout << "getJacobianOfTranslation of " << getPath() << endl;
    if(updateJac[j]) {
      updateJac[j] = false;
      updateJacobians(t,j); 
    }
    return getJacobianOfTranslation(j); 
  }

  const Mat3xV& Frame::getJacobianOfRotation(double t, int j) {
    if(updateJac[j]) {
      updateJac[j] = false;
      updateJacobians(t,j); 
    }
    return getJacobianOfRotation(j); 
  }

  const Vec3& Frame::getGyroscopicAccelerationOfTranslation(double t, int j) {
    if(updateJac[j]) {
      updateJac[j] = false;
      updateJacobians(t,j); 
    }
    return getGyroscopicAccelerationOfTranslation(j); 
  }

  const Vec3& Frame::getGyroscopicAccelerationOfRotation(double t, int j) {
    if(updateJac[j]) {
      updateJac[j] = false;
      updateJacobians(t,j); 
    }
    return getGyroscopicAccelerationOfRotation(j); 
  }

  void FixedRelativeFrame::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_frameOfReference!="")
        setFrameOfReference(getByPath<Frame>(saved_frameOfReference));
      Frame::init(stage);
    }
    else
      Frame::init(stage);
  }

  void FixedRelativeFrame::initializeUsingXML(DOMElement *element) {
    Frame::initializeUsingXML(element);
    DOMElement *ec=element->getFirstElementChild();
    ec=E(element)->getFirstElementChildNamed(MBSIM%"frameOfReference");
    if(ec) setFrameOfReference(E(ec)->getAttribute("ref"));
    ec=E(element)->getFirstElementChildNamed(MBSIM%"relativePosition");
    if(ec) setRelativePosition(getVec3(ec));
    ec=E(element)->getFirstElementChildNamed(MBSIM%"relativeOrientation");
    if(ec) setRelativeOrientation(getSqrMat3(ec));
  }

  DOMElement* FixedRelativeFrame::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Frame::writeXMLFile(parent);
//     if(getFrameOfReference()) {
//        DOMElement *ele1 = new DOMElement( MBSIM%"frameOfReference" );
//        string str = string("../Frame[") + getFrameOfReference()->getName() + "]";
//        ele1->SetAttribute("ref", str);
//        ele0->LinkEndChild(ele1);
//      }
//     addElementText(ele0,MBSIM%"relativePosition",getRelativePosition());
//     addElementText(ele0,MBSIM%"relativeOrientation",getRelativeOrientation());
   return ele0;
  }

  void FixedRelativeFrame::updatePositions(double t) { 
    if(updateByParent[0])
      parent->updatePositions(t);
    else {
      setOrientation(R->getOrientation(t)*ARP);
      WrRP = R->getOrientation(t)*RrRP; 
      setPosition(R->getPosition(t) + WrRP); 
    }
  }

  void FixedRelativeFrame::updateVelocities(double t) { 
    if(updateByParent[0])
      parent->updateVelocities(t);
    else {
      setAngularVelocity(R->getAngularVelocity(t));
      setVelocity(R->getVelocity(t) + crossProduct(R->getAngularVelocity(t), WrRP)); 
    }
  }

  void FixedRelativeFrame::updateJacobians(double t, int j) {
    if(updateByParent[j])
      parent->updateJacobians(t,j);
    else {
      fmatvec::SqrMat3 tWrRP = tilde(WrRP);
      setJacobianOfTranslation(R->getJacobianOfTranslation(t,j) - tWrRP*R->getJacobianOfRotation(t,j),j);
      setJacobianOfRotation(R->getJacobianOfRotation(t,j),j);
      setGyroscopicAccelerationOfTranslation(R->getGyroscopicAccelerationOfTranslation(t,j) - tWrRP*R->getGyroscopicAccelerationOfRotation(t,j) + crossProduct(R->getAngularVelocity(),crossProduct(R->getAngularVelocity(),WrRP)),j);
      setGyroscopicAccelerationOfRotation(R->getGyroscopicAccelerationOfRotation(t,j),j);
    }
  }

}

