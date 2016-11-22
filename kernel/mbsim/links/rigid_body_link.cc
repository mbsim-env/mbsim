/* Copyright (C) 2004-2015 MBSim Development Team
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
 * Contact: martin.o.foerg@gmail.com
 */

#include <config.h>
#include "mbsim/links/rigid_body_link.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/objects/rigid_body.h"
#include <openmbvcppinterface/group.h>
#include "mbsim/utils/utils.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  RigidBodyLink::RigidBodyLink(const string &name) : Link(name), updPos(true), updVel(true), updFD(true), updF(true), updM(true), updRMV(true), support(NULL) {
    FArrow.resize(1);
    MArrow.resize(1);
  }

  void RigidBodyLink::updatehRef(const Vec &hParent, int j) {
    for(unsigned i=0; i<body.size(); i++) {
      Index I = Index(body[i]->gethInd(j),body[i]->gethInd(j)+body[i]->gethSize(j)-1);
      h[j][i]>>hParent(I);
      I = Index(support->gethInd(j),support->gethInd(j)+support->gethSize(j)-1);
      h[j][body.size()+i]>>hParent(I);
    }
  } 

  void RigidBodyLink::updaterRef(const Vec &rParent, int j) {
    for(unsigned i=0; i<body.size(); i++) {
      Index I = Index(body[i]->gethInd(j),body[i]->gethInd(j)+body[i]->gethSize(j)-1);
      r[j][i]>>rParent(I);
      I = Index(support->gethInd(j),support->gethInd(j)+support->gethSize(j)-1);
      r[j][body.size()+i]>>rParent(I);
    }
  } 

  void RigidBodyLink::updateWRef(const Mat &WParent, int j) {
    for(unsigned i=0; i<body.size(); i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(body[i]->gethInd(j),body[i]->gethInd(j)+body[i]->gethSize(j)-1);

      W[j][i]>>WParent(I,J);
      I = Index(support->gethInd(j),support->gethInd(j)+support->gethSize(j)-1);
      W[j][body.size()+i]>>WParent(I,J);
    }
  } 

  void RigidBodyLink::updateVRef(const Mat &VParent, int j) {
    for(unsigned i=0; i<body.size(); i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(body[i]->gethInd(j),body[i]->gethInd(j)+body[i]->gethSize(j)-1);

      V[j][i]>>VParent(I,J);
      I = Index(support->gethInd(j),support->gethInd(j)+support->gethSize(j)-1);
      V[j][body.size()+i]>>VParent(I,J);
    }
  } 

  void RigidBodyLink::updatePositions() {
    for(unsigned i=0; i<body.size(); i++) {
      C[i].setPosition(body[i]->getFrameForKinematics()->evalPosition());
      C[i].setOrientation(support->getOrientation());
    }
    updPos = false;
  }

  void RigidBodyLink::updateGeneralizedPositions() {
    rrel.init(0);
    for(unsigned i=0; i<body.size(); i++)
      rrel+=body[i]->evalqRel()*ratio[i];
    updrrel = false;
  }

  void RigidBodyLink::updateGeneralizedVelocities() {
    vrel.init(0);
    for(unsigned i=0; i<body.size(); i++)
      vrel+=body[i]->evaluRel()*ratio[i];
    updvrel = false;
  }

  void RigidBodyLink::updateForceDirections() {
    for(unsigned i=0; i<body.size(); i++) {
      DF[i] = support->evalOrientation()*body[i]->evalPJT();
      DM[i] = support->evalOrientation()*body[i]->evalPJR();
    }
    updFD = false;
  }

  void RigidBodyLink::updateForce() {
    for(unsigned i=0; i<body.size(); i++)
      F[i] = evalGlobalForceDirection(i)*evalGeneralizedForce()(iF);
    updF = false;
  }

  void RigidBodyLink::updateMoment() {
    for(unsigned i=0; i<body.size(); i++)
      M[i] = evalGlobalMomentDirection(i)*evalGeneralizedForce()(iM);
    updM = false;
  }

  void RigidBodyLink::updateR() {
    for(unsigned i=0; i<body.size(); i++) {
      RF[i].set(Index(0,2), Index(iF), evalGlobalForceDirection(i));
      RM[i].set(Index(0,2), Index(iM), evalGlobalMomentDirection(i));
    }
    updRMV = false;
  }

  void RigidBodyLink::updateh(int j) {
    if(j==0) {
      for(unsigned i=0; i<body.size(); i++)
        h[j][i]+=body[i]->evalJRel(j).T()*evalGeneralizedForce()*ratio[i];
    } else {
      for(unsigned i=0; i<body.size(); i++) {
        h[j][i]+=(body[i]->getFrameForKinematics()->evalJacobianOfTranslation(j).T()*evalForce(i)  + body[i]->getFrameForKinematics()->evalJacobianOfRotation(j).T()*evalMoment(i))*ratio[i];
        h[j][body.size()+i]-=(C[i].evalJacobianOfTranslation(j).T()*evalForce(i) + C[i].evalJacobianOfRotation(j).T()*evalMoment(i))*ratio[i];
      }
    }
  }

  void RigidBodyLink::updateW(int j) {
    if(j==0) {
      for(unsigned i=0; i<body.size(); i++)  {
        W[j][i]+=body[i]->evalJRel(j).T()*ratio[i];
      }
    } else {
      for(unsigned i=0; i<body.size(); i++) {
        W[j][i]+=(body[i]->getFrameForKinematics()->evalJacobianOfTranslation(j).T()*evalRF(i) + body[i]->getFrameForKinematics()->evalJacobianOfRotation(j).T()*evalRM(i))*ratio[i];
        W[j][body.size()+i]-=(C[i].evalJacobianOfTranslation(j).T()*evalRF(i) + C[i].evalJacobianOfRotation(j).T()*evalRM(i))*ratio[i];
      }
    }
  }

  void RigidBodyLink::updateg() {
    g = evalGeneralizedRelativePosition();
  }

  void RigidBodyLink::updategd() {
    gd = evalGeneralizedRelativeVelocity();
  }

  void RigidBodyLink::updatewb() {
    for(unsigned i=0; i<body.size(); i++)
      wb += body[i]->evaljRel()*ratio[i];
  }

  void RigidBodyLink::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_supportFrame!="")
        setSupportFrame(getByPath<Frame>(saved_supportFrame));
      Link::init(stage);
    }
    else if(stage==resize) {
      Link::init(stage);

      if(!support)
        support = body[0]->getFrameOfReference();

      F.resize(body.size());
      M.resize(body.size());
      iF = Index(0, body[0]->getPJT(0,false).cols()-1);
      iM = Index(0, body[0]->getPJR(0,false).cols()-1);
      rrel.resize(body[0]->getqRelSize());
      vrel.resize(body[0]->getuRelSize());
      if(isSetValued()) {
        g.resize(rrel.size());
        gd.resize(vrel.size());
        la.resize(vrel.size());
      }
      lambda.resize(vrel.size());
      for(unsigned int i=0; i<body.size(); i++) {
        DF.push_back(Mat3xV(iF.size()));
        DM.push_back(Mat3xV(iM.size()));
        RF.push_back(Mat3xV(vrel.size()));
        RM.push_back(Mat3xV(vrel.size()));
        h[0].push_back(Vec(body[i]->getFrameForKinematics()->gethSize()));
        h[1].push_back(Vec(6));
        W[0].push_back(Mat(body[i]->getFrameForKinematics()->gethSize(),laSize));
        W[1].push_back(Mat(6,laSize));
        stringstream s;
        s << "F" << i;
        C.push_back(FloatingRelativeFrame(s.str()));
        C[i].getJacobianOfTranslation(0,false).resize(support->gethSize());
        C[i].getJacobianOfRotation(0,false).resize(support->gethSize());
        C[i].getJacobianOfTranslation(1,false).resize(support->gethSize(1));
        C[i].getJacobianOfRotation(1,false).resize(support->gethSize(1));
        C[i].setParent(this);
        C[i].setFrameOfReference(support);
      }
      for(unsigned int i=0; i<body.size(); i++) {
        h[0].push_back(Vec(support->gethSize()));
        h[1].push_back(Vec(6));
        W[0].push_back(Mat(support->gethSize(),laSize));
        W[1].push_back(Mat(6,laSize));
        V[0].push_back(Mat(support->gethSize(),laSize));
        V[1].push_back(Mat(6,laSize));
      }
    }
    else if(stage==plotting) {
      updatePlotFeatures();
      if(getPlotFeature(plotRecursive)==enabled) {
        for(int i=0; i<(int)body.size(); i++)
          plotColumns.push_back("la("+numtostr(i)+")*ratio("+numtostr(i)+")");
        if(getPlotFeature(openMBV)==enabled) {
          openMBVForceGrp=OpenMBV::ObjectFactory::create<OpenMBV::Group>();
          openMBVForceGrp->setExpand(false);
          openMBVForceGrp->setName(name+"_ArrowGroup");
          parent->getOpenMBVGrp()->addObject(openMBVForceGrp);
          if(FArrow[0]) {
            FArrow[0]->setName("Force0");
            openMBVForceGrp->addObject(FArrow[0]);
            for(unsigned int i=1; i<body.size(); i++) {
              stringstream s;
              s << i;
              FArrow.push_back(OpenMBV::ObjectFactory::create(FArrow[0]));
              FArrow[i]->setName("Force"+s.str());
              openMBVForceGrp->addObject(FArrow[i]);
            }
          }
          if(MArrow[0]) {
            MArrow[0]->setName("Moment0");
            openMBVForceGrp->addObject(MArrow[0]);
            for(unsigned int i=1; i<body.size(); i++) {
              stringstream s;
              s << i;
              MArrow.push_back(OpenMBV::ObjectFactory::create(MArrow[0]));
              MArrow[i]->setName("Moment"+s.str());
              openMBVForceGrp->addObject(MArrow[i]);
            }
          }
        }
        Link::init(stage);
      }
    }
    else
      Link::init(stage);
  }

  void RigidBodyLink::plot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      for(unsigned int i=0; i<body.size(); i++) {
        plotVector.push_back(ratio[i]*evalGeneralizedForce()(0));
      }
      if(getPlotFeature(openMBV)==enabled) {
        if(FArrow[0]) {
          for(unsigned i=0; i<body.size(); i++) {
            vector<double> data;
            data.push_back(getTime());
            Vec3 WF = evalForce(i)*ratio[i];
            Vec3 WrOS=body[i]->getFrameForKinematics()->evalPosition();
            data.push_back(WrOS(0));
            data.push_back(WrOS(1));
            data.push_back(WrOS(2));
            data.push_back(WF(0));
            data.push_back(WF(1));
            data.push_back(WF(2));
            data.push_back(1.0);
            FArrow[i]->append(data);
          }
        }
        if(MArrow[0]) {
          for(unsigned i=0; i<body.size(); i++) {
            vector<double> data;
            data.push_back(getTime());
            Vec3 WM = evalMoment(i)*ratio[i];
            Vec3 WrOS=body[i]->getFrameForKinematics()->evalPosition();
            data.push_back(WrOS(0));
            data.push_back(WrOS(1));
            data.push_back(WrOS(2));
            data.push_back(WM(0));
            data.push_back(WM(1));
            data.push_back(WM(2));
            data.push_back(1.0);
            MArrow[i]->append(data);
          }
        }
      }
      Link::plot();
    }
  }

  void RigidBodyLink::initializeUsingXML(DOMElement* element) {
    Link::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"supportFrame");
    if(e) saved_supportFrame=E(e)->getAttribute("ref");
    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVForce");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
      RigidBodyLink::setOpenMBVForce(ombv.createOpenMBV(e));
    }
    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMoment");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toDoubleHead,OpenMBV::Arrow::toPoint,1,1);
      RigidBodyLink::setOpenMBVMoment(ombv.createOpenMBV(e));
    }
  }

  void RigidBodyLink::resetUpToDate() {
    Link::resetUpToDate();
    updPos = true;
    updVel = true;
    updFD = true;
    updF = true;
    updM = true;
    updRMV = true;
    for(unsigned int i=0; i<C.size(); i++)
      C[i].resetUpToDate();
  }

}
