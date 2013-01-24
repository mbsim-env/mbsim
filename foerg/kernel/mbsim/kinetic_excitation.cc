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
#include <mbsim/utils/utils.h>
#include <mbsim/dynamic_system_solver.h>
#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/objectfactory.h"
#include "openmbvcppinterface/arrow.h"
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  KineticExcitation::KineticExcitation(const string &name) : LinkMechanics(name), refFrame(NULL), F(NULL), M(NULL) {}

  KineticExcitation::~KineticExcitation() {}

  void KineticExcitation::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_frameOfReference!="")
        setFrameOfReference(getByPath<Frame>(saved_frameOfReference));
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
      if(!refFrame) refFrame=frame[1];
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
    Mat33 tWrP0P1 = tilde(WrP0P1);

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

  void KineticExcitation::setForce(fmatvec::Mat dir, Function1<fmatvec::Vec,double> *func) {
    forceDir << dir;

    for(int i=0; i<dir.cols(); i++)
      forceDir.set(i, forceDir.col(i)/nrm2(dir.col(i)));

    F=func;
    assert((*F)(0).size()==forceDir.cols());
  }

  void KineticExcitation::setMoment(fmatvec::Mat dir, Function1<fmatvec::Vec,double> *func) {
    momentDir << dir;

    for(int i=0; i<dir.cols(); i++)
      momentDir.set(i, momentDir.col(i)/nrm2(dir.col(i)));

    M=func;
    assert((*M)(0).size()==momentDir.cols());
  }

  void KineticExcitation::initializeUsingXML(TiXmlElement *element) {
    LinkMechanics::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMNS"frameOfReference");
    if(e)
      saved_frameOfReference=e->Attribute("ref");
    e=element->FirstChildElement(MBSIMNS"force");
    if(e) {
      TiXmlElement *ee=e->FirstChildElement();
      Mat dir=getMat(ee,3,0);
      ee=ee->NextSiblingElement();
      Function1<Vec,double> *func=ObjectFactory::getInstance()->createFunction1_VS(ee->FirstChildElement());
      func->initializeUsingXML(ee->FirstChildElement());
      setForce(dir, func);
      ee=ee->NextSiblingElement();
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Arrow *arrow=dynamic_cast<OpenMBV::Arrow*>(OpenMBV::ObjectFactory::createObject(ee));
      if(arrow) {
        arrow->initializeUsingXML(ee); // first initialize, because setOpenMBVForceArrow calls the copy constructor on arrow
        setOpenMBVForceArrow(arrow);
        ee=ee->NextSiblingElement();
      }
#endif
    }
    e=element->FirstChildElement(MBSIMNS"moment");
    if(e) {
      TiXmlElement *ee=e->FirstChildElement();
      Mat dir=getMat(ee,3,0);
      ee=ee->NextSiblingElement();
      Function1<Vec,double> *func=ObjectFactory::getInstance()->createFunction1_VS(ee->FirstChildElement());
      func->initializeUsingXML(ee->FirstChildElement());
      setMoment(dir, func);
      ee=ee->NextSiblingElement();
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Arrow *arrow=dynamic_cast<OpenMBV::Arrow*>(OpenMBV::ObjectFactory::createObject(ee));
      if(arrow) {
        arrow->initializeUsingXML(ee); // first initialize, because setOpenMBVMomentArrow calls the copy constructor on arrow
        setOpenMBVMomentArrow(arrow);
        ee=ee->NextSiblingElement();
      }
#endif
    }
    e=element->FirstChildElement(MBSIMNS"connect");
    if(e->Attribute("ref"))
      saved_ref=e->Attribute("ref");
    else {
      saved_ref1=e->Attribute("ref1");
      saved_ref2=e->Attribute("ref2");
    }
    e=e->NextSiblingElement();
  }

  TiXmlElement* KineticExcitation::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = LinkMechanics::writeXMLFile(parent);
    if(refFrame) {
      TiXmlElement *ele1 = new TiXmlElement( MBSIMNS"frameOfReference" );
      ele1->SetAttribute("ref", refFrame->getXMLPath(this,true)); // relative path
      ele0->LinkEndChild(ele1);
    }
    if(forceDir.cols()) {
      TiXmlElement *ele1 = new TiXmlElement(MBSIMNS"force");
      addElementText(ele1,MBSIMNS"directionVectors",forceDir);
      TiXmlElement *ele2 = new TiXmlElement(MBSIMNS"function");
      F->writeXMLFile(ele2);
      ele1->LinkEndChild(ele2);
      ele0->LinkEndChild(ele1);
    }
    if(momentDir.cols()) {
      TiXmlElement *ele1 = new TiXmlElement(MBSIMNS"moment");
      addElementText(ele1,MBSIMNS"directionVectors",momentDir);
      TiXmlElement *ele2 = new TiXmlElement(MBSIMNS"function");
      M->writeXMLFile(ele2);
      ele1->LinkEndChild(ele2);
      ele0->LinkEndChild(ele1);
    }
    TiXmlElement *ele1 = new TiXmlElement(MBSIMNS"connect");
    ele1->SetAttribute("ref1", frame[0]->getXMLPath(this,true)); // relative path
    ele1->SetAttribute("ref2", frame[1]->getXMLPath(this,true)); // relative path
    ele0->LinkEndChild(ele1);
    return ele0;
  }

}

