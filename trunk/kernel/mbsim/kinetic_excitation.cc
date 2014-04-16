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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "mbsim/kinetic_excitation.h"
#include "mbsim/objectfactory.h"
#include <fmatvec/function.h>
#include <mbsim/dynamic_system_solver.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, KineticExcitation, MBSIM%"KineticExcitation")

  KineticExcitation::KineticExcitation(const string &name) : LinkMechanics(name), refFrame(NULL), refFrameID(1), F(NULL), M(NULL) {}

  KineticExcitation::~KineticExcitation() {}

  void KineticExcitation::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_ref!="")
        connect(getByPath<Frame>(saved_ref));
      else if(saved_ref1!="" && saved_ref2!="")
        connect(getByPath<Frame>(saved_ref1), getByPath<Frame>(saved_ref2));
      if(frame.size()==1) {
        Frame *buf = frame[0];
        connect(buf);
        frame[0]=ds->getFrame("I");
      }
      LinkMechanics::init(stage);
    }
    else if(stage==unknownStage) {
      LinkMechanics::init(stage);
      if(F) assert((*F)(0).size()==forceDir.cols());
      if(M) assert((*M)(0).size()==momentDir.cols());
      refFrame=refFrameID?frame[1]:frame[0];
      C.getJacobianOfTranslation(0).resize(frame[0]->getJacobianOfTranslation(0).cols());
      C.getJacobianOfRotation(0).resize(frame[0]->getJacobianOfRotation(0).cols());
      C.getJacobianOfTranslation(1).resize(frame[0]->getJacobianOfTranslation(1).cols());
      C.getJacobianOfRotation(1).resize(frame[0]->getJacobianOfRotation(1).cols());
    }
    else if(stage==MBSim::plot) {
      updatePlotFeatures();
      if(getPlotFeature(plotRecursive)==enabled) {
        if(getPlotFeature(generalizedLinkForce)==enabled)
          for(int j=0; j<forceDir.cols()+momentDir.cols(); ++j) 
            plotColumns.push_back("la("+numtostr(j)+")");
      }
      LinkMechanics::init(stage);
    }
    else
      LinkMechanics::init(stage);
  }

  void KineticExcitation::connect(Frame *frame0, Frame* frame1) {
    LinkMechanics::connect(frame0);
    LinkMechanics::connect(frame1);
  }

  void KineticExcitation::updateh(double t, int j) {
    Vec3 WrP0P1 = frame[1]->getPosition()-frame[0]->getPosition();
    Mat3x3 tWrP0P1 = tilde(WrP0P1);

    C.setOrientation(frame[0]->getOrientation());
    C.setPosition(frame[0]->getPosition() + WrP0P1);
    C.setAngularVelocity(frame[0]->getAngularVelocity());
    C.setVelocity(frame[0]->getVelocity() + crossProduct(frame[0]->getAngularVelocity(),WrP0P1));
    C.setJacobianOfTranslation(frame[0]->getJacobianOfTranslation(j) - tWrP0P1*frame[0]->getJacobianOfRotation(j),j);
    C.setJacobianOfRotation(frame[0]->getJacobianOfRotation(j),j);
    C.setGyroscopicAccelerationOfTranslation(frame[0]->getGyroscopicAccelerationOfTranslation(j) - tWrP0P1*frame[0]->getGyroscopicAccelerationOfRotation(j) + crossProduct(frame[0]->getAngularVelocity(),crossProduct(frame[0]->getAngularVelocity(),WrP0P1)),j);
    C.setGyroscopicAccelerationOfRotation(frame[0]->getGyroscopicAccelerationOfRotation(j),j);

    if(F) {
      WF[1]=refFrame->getOrientation()*forceDir * (*F)(t);
      WF[0] = -WF[1];
    }
    if(M) {
      WM[1]=refFrame->getOrientation()*momentDir * (*M)(t);
      WM[0] = -WM[1];
    }
    h[j][0]+=C.getJacobianOfTranslation(j).T()*WF[0] + C.getJacobianOfRotation(j).T()*WM[0];
    h[j][1]+=frame[1]->getJacobianOfTranslation(j).T()*WF[1] + frame[1]->getJacobianOfRotation(j).T()*WM[1];
  }

  void KineticExcitation::calclaSize(int j) {
    LinkMechanics::calclaSize(j);
    laSize=forceDir.cols()+momentDir.cols();
  }

  void KineticExcitation::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      if(getPlotFeature(generalizedLinkForce)==enabled) {
        if(F) {
          Vec f = (*F)(t);
          for(int i=0; i<forceDir.cols(); i++) plotVector.push_back(f(i));
        }
        if(M) {
          Vec m = (*M)(t);
          for(int i=0; i<momentDir.cols(); i++) plotVector.push_back(m(i));
        }
      }
      LinkMechanics::plot(t,dt);
    }
  }

  void KineticExcitation::setForceDirection(const Mat3xV &fd) {

    forceDir = fd;

    for(int i=0; i<fd.cols(); i++)
      forceDir.set(i, forceDir.col(i)/nrm2(fd.col(i)));
  }

  void KineticExcitation::setMomentDirection(const Mat3xV &md) {

    momentDir = md;

    for(int i=0; i<md.cols(); i++)
      momentDir.set(i, momentDir.col(i)/nrm2(md.col(i)));
  }

  void KineticExcitation::setForceFunction(Function<VecV(double)> *func) {
    F=func;
  }

  void KineticExcitation::setMomentFunction(Function<VecV(double)> *func) {
    M=func;
  }

  void KineticExcitation::initializeUsingXML(DOMElement *element) {
    LinkMechanics::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"frameOfReferenceID");
    if(e) refFrameID=getDouble(e);
    e=E(element)->getFirstElementChildNamed(MBSIM%"forceDirection");
    if(e) setForceDirection(getMat(e,3,0));
    e=E(element)->getFirstElementChildNamed(MBSIM%"forceFunction");
    if(e) setForceFunction(ObjectFactory<FunctionBase>::createAndInit<Function<VecV(double)> >(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"momentDirection");
    if(e) setMomentDirection(getMat(e,3,0));
    e=E(element)->getFirstElementChildNamed(MBSIM%"momentFunction");
    if(e) setMomentFunction(ObjectFactory<FunctionBase>::createAndInit<Function<VecV(double)> >(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"connect");
    if(E(e)->hasAttribute("ref"))
      saved_ref=E(e)->getAttribute("ref");
    else {
      saved_ref1=E(e)->getAttribute("ref1");
      saved_ref2=E(e)->getAttribute("ref2");
    }
#ifdef HAVE_OPENMBVCPPINTERFACE
    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVForce");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
      std::vector<bool> which; which.resize(2, false);
      which[1]=true;
      LinkMechanics::setOpenMBVForceArrow(ombv.createOpenMBV(e), which);
    }

    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMoment");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toDoubleHead,OpenMBV::Arrow::toPoint,1,1);
      std::vector<bool> which; which.resize(2, false);
      which[1]=true;
      LinkMechanics::setOpenMBVMomentArrow(ombv.createOpenMBV(e), which);
    }
#endif
  }

  DOMElement* KineticExcitation::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = LinkMechanics::writeXMLFile(parent);
//    if(refFrame) {
//      DOMElement *ele1 = new DOMElement( MBSIM%"frameOfReference" );
//      ele1->SetAttribute("ref", refFrame->getXMLPath(this,true)); // relative path
//      ele0->LinkEndChild(ele1);
//    }
//    if(forceDir.cols()) {
//      addElementText(ele0,MBSIM%"forceDirection",forceDir);
//      DOMElement *ele1 = new DOMElement(MBSIM%"forceFunction");
//      F->writeXMLFile(ele1);
//      ele0->LinkEndChild(ele1);
//    }
//    if(momentDir.cols()) {
//      addElementText(ele0,MBSIM%"momentDirection",momentDir);
//      DOMElement *ele1 = new DOMElement(MBSIM%"momentFunction");
//      M->writeXMLFile(ele1);
//      ele0->LinkEndChild(ele1);
//    }
//    DOMElement *ele1 = new DOMElement(MBSIM%"connect");
//    ele1->SetAttribute("ref1", frame[0]->getXMLPath(this,true)); // relative path
//    ele1->SetAttribute("ref2", frame[1]->getXMLPath(this,true)); // relative path
//    ele0->LinkEndChild(ele1);
    return ele0;
  }

}

