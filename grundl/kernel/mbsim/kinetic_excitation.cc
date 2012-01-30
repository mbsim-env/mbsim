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
#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/objectfactory.h"
#include "openmbvcppinterface/arrow.h"
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  KineticExcitation::KineticExcitation(const string &name) : LinkMechanics(name), refFrame(NULL),
  forceDir(3,0), momentDir(3,0), F(NULL), M(NULL) {}

  KineticExcitation::~KineticExcitation() {}

  void KineticExcitation::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_frameOfReference!="")
        setFrameOfReference(getByPath<Frame>(saved_frameOfReference));
      if(saved_ref!="")
        connect(getByPath<Frame>(saved_ref));
      LinkMechanics::init(stage);
    }
    else if(stage==unknownStage) {
      LinkMechanics::init(stage);
      if(!refFrame) refFrame=frame[0];
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

  void KineticExcitation::updateh(double t, int j) {
    if(F) WF[0]=refFrame->getOrientation()*forceDir * (*F)(t);
    if(M) WM[0]=refFrame->getOrientation()*momentDir * (*M)(t);
    h[j][0]+=frame[0]->getJacobianOfTranslation(j).T()*WF[0] + frame[0]->getJacobianOfRotation(j).T()*WM[0];
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
    forceDir.resize(3,dir.cols());
    for (int i=0; i<dir.cols(); i++)
      forceDir.col(i)=dir.col(i)/nrm2(dir.col(i));
    F=func;
    assert((*F)(0).size()==forceDir.cols());
  }

  void KineticExcitation::setMoment(fmatvec::Mat dir, Function1<fmatvec::Vec,double> *func) {
    momentDir.resize(3,dir.cols());
    for (int i=0; i<dir.cols(); i++)
      momentDir.col(i)=dir.col(i)/nrm2(dir.col(i));
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
    saved_ref=e->Attribute("ref");
    e=e->NextSiblingElement();
  }

}

