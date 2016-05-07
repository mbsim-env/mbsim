/* Copyright (C) 2004-2016 MBSim Development Team
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
#include "mbsim/observers/relative_kinematics_observer.h"
#include "mbsim/frames/frame.h"
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace xercesc;
using namespace boost;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(RelativeKinematicsObserver, MBSIM%"RelativeKinematicsObserver")

  RelativeKinematicsObserver::RelativeKinematicsObserver(const std::string &name) : KinematicsObserver(name) {
    refFrame = 0;
  }

  void RelativeKinematicsObserver::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_frameOfReference!="")
        setFrameOfReference(getByPath<Frame>(saved_frameOfReference));
      KinematicsObserver::init(stage);
    }
    else if(stage==plotting) {
      updatePlotFeatures();

      if(openMBVPosition) {
        plotColumns.push_back("TranslationalPosition");
        plotColumns.push_back("RelativePosition");
      }
      if(openMBVVelocity) {
        plotColumns.push_back("TranslationalVelocity");
        plotColumns.push_back("RotationalVelocity");
        plotColumns.push_back("RelativeVelocity");
        plotColumns.push_back("GuidingVelocity");
      }
      if(openMBVAngularVelocity) {
        plotColumns.push_back("TranslationalAngularVelocity");
        plotColumns.push_back("RelativeAngularVelocity");
      }
      if(openMBVAcceleration) {
        plotColumns.push_back("TranslationalAcceleration");
        plotColumns.push_back("RotationalAcceleration");
        plotColumns.push_back("CentripetalAcceleration");
        plotColumns.push_back("CoriolisAcceleration");
        plotColumns.push_back("RelativeAcceleration");
        plotColumns.push_back("GuidingAcceleration");
      }
      if(openMBVAngularAcceleration) {
        plotColumns.push_back("TranslationalAngularAcceleration");
        plotColumns.push_back("RotationalAngularAcceleration");
        plotColumns.push_back("RelativeAngularAcceleration");
      }
      KinematicsObserver::init(stage);
      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled) {
          if(openMBVPosition) {
            openMBVrTrans = OpenMBV::ObjectFactory::create(openMBVPosition);
            openMBVrRel = OpenMBV::ObjectFactory::create(openMBVPosition);
            openMBVrTrans->setName("TranslationalPosition");
            openMBVPosGrp->addObject(openMBVrTrans);
            openMBVrRel->setName("RelativePosition");
            openMBVPosGrp->addObject(openMBVrRel);
          }
          if(openMBVVelocity) {
            openMBVvTrans = OpenMBV::ObjectFactory::create(openMBVVelocity);
            openMBVvRot = OpenMBV::ObjectFactory::create(openMBVVelocity);
            openMBVvRel = OpenMBV::ObjectFactory::create(openMBVVelocity);
            openMBVvF = OpenMBV::ObjectFactory::create(openMBVVelocity);
            openMBVvTrans->setName("TranslationalVelocity");
            openMBVVelGrp->addObject(openMBVvTrans);
            openMBVvRot->setName("RotationalVelocity");
            openMBVVelGrp->addObject(openMBVvRot);
            openMBVvRel->setName("RelativeVelocity");
            openMBVVelGrp->addObject(openMBVvRel);
            openMBVvF->setName("GuidingVelocity");
            openMBVVelGrp->addObject(openMBVvF);
          }
          if(openMBVAngularVelocity) {
            openMBVomTrans = OpenMBV::ObjectFactory::create(openMBVAngularVelocity);
            openMBVomRel = OpenMBV::ObjectFactory::create(openMBVAngularVelocity);
            openMBVomTrans->setName("TranslationalAngularVelocity");
            openMBVAngVelGrp->addObject(openMBVomTrans);
            openMBVomRel->setName("RelativeAngularVelocity");
            openMBVAngVelGrp->addObject(openMBVomRel);
          }
          if(openMBVAcceleration) {
            openMBVaTrans = OpenMBV::ObjectFactory::create(openMBVAcceleration);
            openMBVaRot = OpenMBV::ObjectFactory::create(openMBVAcceleration);
            openMBVaZp = OpenMBV::ObjectFactory::create(openMBVAcceleration);
            openMBVaCor = OpenMBV::ObjectFactory::create(openMBVAcceleration);
            openMBVaRel = OpenMBV::ObjectFactory::create(openMBVAcceleration);
            openMBVaF = OpenMBV::ObjectFactory::create(openMBVAcceleration);
            openMBVaTrans->setName("TranslationalAcceleration");
            openMBVAccGrp->addObject(openMBVaTrans);
            openMBVaRot->setName("RotationalAcceleration");
            openMBVAccGrp->addObject(openMBVaRot);
            openMBVaZp->setName("CentripetalAcceleration");
            openMBVAccGrp->addObject(openMBVaZp);
            openMBVaCor->setName("CoriolisAcceleration");
            openMBVAccGrp->addObject(openMBVaCor);
            openMBVaRel->setName("RelativeAcceleration");
            openMBVAccGrp->addObject(openMBVaRel);
            openMBVaF->setName("GuidingAcceleration");
            openMBVAccGrp->addObject(openMBVaF);
          }
          if(openMBVAngularAcceleration) {
            openMBVpsiTrans = OpenMBV::ObjectFactory::create(openMBVAngularAcceleration);
            openMBVpsiRot = OpenMBV::ObjectFactory::create(openMBVAngularAcceleration);
            openMBVpsiRel = OpenMBV::ObjectFactory::create(openMBVAngularAcceleration);
            openMBVpsiTrans->setName("TranslationalAngularAcceleration");
            openMBVAngAccGrp->addObject(openMBVpsiTrans);
            openMBVpsiRot->setName("RotationalAngularAcceleration");
            openMBVAngAccGrp->addObject(openMBVpsiRot);
            openMBVpsiRel->setName("RelativeAngularAcceleration");
            openMBVAngAccGrp->addObject(openMBVpsiRel);
          }
        }
#endif
      }
    }
    else
      KinematicsObserver::init(stage);
  }

  void RelativeKinematicsObserver::plot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      Vec3 vP = frame->evalVelocity();
      Vec3 vOs = refFrame->evalVelocity();
      Vec3 rOP = frame->evalPosition();
      Vec3 rOOs = refFrame->evalPosition();
      Vec3 rOsP = rOP - rOOs;
      Vec3 omB = refFrame->evalAngularVelocity();
      Vec3 vOsP = vP - vOs;
      Vec3 vRot = crossProduct(omB,rOsP);
      Vec3 vRel = vOsP - vRot;
      Vec3 vF = vOs + vRot;
      Vec3 aP = frame->evalAcceleration();
      Vec3 aOs = refFrame->evalAcceleration();
      Vec3 aOsP = aP - aOs;
      Vec3 psiB = refFrame->evalAngularAcceleration();
      Vec3 aRot = crossProduct(psiB,rOsP);
      Vec3 aZp = crossProduct(omB,crossProduct(omB,rOsP));
      Vec3 aCor = 2.*crossProduct(omB,vRel);
      Vec3 aRel = aOsP - aRot - aZp - aCor;
      Vec3 aF = aOs + aRot + aZp;
      Vec3 omK = frame->evalAngularVelocity();
      Vec3 omBK = omK - omB;
      Vec3 psiK = frame->evalAngularAcceleration();
      Vec3 psiBK = psiK - psiB;
      Vec3 psiRot = crossProduct(omB, omBK);
      Vec3 psiRel = psiBK - psiRot;

#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled) {
        if(openMBVPosition&& !openMBVPosition->isHDF5Link()) {
          vector<double> data;
          data.push_back(getTime());
          data.push_back(0);
          data.push_back(0);
          data.push_back(0);
          data.push_back(rOOs(0));
          data.push_back(rOOs(1));
          data.push_back(rOOs(2));
          data.push_back(0.5);
          openMBVrTrans->append(data);
          data.clear();
          data.push_back(getTime());
          data.push_back(rOOs(0));
          data.push_back(rOOs(1));
          data.push_back(rOOs(2));
          data.push_back(rOsP(0));
          data.push_back(rOsP(1));
          data.push_back(rOsP(2));
          data.push_back(0.5);
          openMBVrRel->append(data);
          plotVector.push_back(nrm2(rOOs));
          plotVector.push_back(nrm2(rOsP));
        }
        if(openMBVVelocity&& !openMBVVelocity->isHDF5Link()) {
          vector<double> data;
          data.push_back(getTime());
          data.push_back(rOOs(0));
          data.push_back(rOOs(1));
          data.push_back(rOOs(2));
          data.push_back(vOs(0));
          data.push_back(vOs(1));
          data.push_back(vOs(2));
          data.push_back(0.5);
          openMBVvTrans->append(data);
          data.clear();
          data.push_back(getTime());
          data.push_back(rOP(0));
          data.push_back(rOP(1));
          data.push_back(rOP(2));
          data.push_back(vRot(0));
          data.push_back(vRot(1));
          data.push_back(vRot(2));
          data.push_back(0.5);
          openMBVvRot->append(data);
          data.clear();
          data.push_back(getTime());
          data.push_back(rOP(0));
          data.push_back(rOP(1));
          data.push_back(rOP(2));
          data.push_back(vRel(0));
          data.push_back(vRel(1));
          data.push_back(vRel(2));
          data.push_back(0.5);
          openMBVvRel->append(data);
          data.clear();
          data.push_back(getTime());
          data.push_back(rOP(0));
          data.push_back(rOP(1));
          data.push_back(rOP(2));
          data.push_back(vF(0));
          data.push_back(vF(1));
          data.push_back(vF(2));
          data.push_back(0.5);
          openMBVvF->append(data);
          plotVector.push_back(nrm2(vOs));
          plotVector.push_back(nrm2(vRot));
          plotVector.push_back(nrm2(vRel));
          plotVector.push_back(nrm2(vF));
        }
        if(openMBVAcceleration&& !openMBVAcceleration->isHDF5Link()) {
          vector<double> data;
          data.push_back(getTime());
          data.push_back(rOOs(0));
          data.push_back(rOOs(1));
          data.push_back(rOOs(2));
          data.push_back(aOs(0));
          data.push_back(aOs(1));
          data.push_back(aOs(2));
          data.push_back(0.5);
          openMBVaTrans->append(data);
          data.clear();
          data.push_back(getTime());
          data.push_back(rOP(0));
          data.push_back(rOP(1));
          data.push_back(rOP(2));
          data.push_back(aRot(0));
          data.push_back(aRot(1));
          data.push_back(aRot(2));
          data.push_back(0.5);
          openMBVaRot->append(data);
          data.clear();
          data.push_back(getTime());
          data.push_back(rOP(0));
          data.push_back(rOP(1));
          data.push_back(rOP(2));
          data.push_back(aZp(0));
          data.push_back(aZp(1));
          data.push_back(aZp(2));
          data.push_back(0.5);
          openMBVaZp->append(data);
          data.clear();
          data.push_back(getTime());
          data.push_back(rOP(0));
          data.push_back(rOP(1));
          data.push_back(rOP(2));
          data.push_back(aCor(0));
          data.push_back(aCor(1));
          data.push_back(aCor(2));
          data.push_back(0.5);
          openMBVaCor->append(data);
          data.clear();
          data.push_back(getTime());
          data.push_back(rOP(0));
          data.push_back(rOP(1));
          data.push_back(rOP(2));
          data.push_back(aRel(0));
          data.push_back(aRel(1));
          data.push_back(aRel(2));
          data.push_back(0.5);
          openMBVaRel->append(data);
          data.clear();
          data.push_back(getTime());
          data.push_back(rOP(0));
          data.push_back(rOP(1));
          data.push_back(rOP(2));
          data.push_back(aF(0));
          data.push_back(aF(1));
          data.push_back(aF(2));
          data.push_back(0.5);
          openMBVaF->append(data);
          plotVector.push_back(nrm2(aOs));
          plotVector.push_back(nrm2(aRot));
          plotVector.push_back(nrm2(aZp));
          plotVector.push_back(nrm2(aCor));
          plotVector.push_back(nrm2(aRel));
          plotVector.push_back(nrm2(aF));
        }
        if(openMBVAngularVelocity&& !openMBVAngularVelocity->isHDF5Link()) {
          vector<double> data;
          data.push_back(getTime());
          data.push_back(0);
          data.push_back(0);
          data.push_back(0);
          data.push_back(omB(0));
          data.push_back(omB(1));
          data.push_back(omB(2));
          data.push_back(0.5);
          openMBVomTrans->append(data);
          data.clear();
          data.push_back(getTime());
          data.push_back(rOOs(0));
          data.push_back(rOOs(1));
          data.push_back(rOOs(2));
          data.push_back(omBK(0));
          data.push_back(omBK(1));
          data.push_back(omBK(2));
          data.push_back(0.5);
          openMBVomRel->append(data);
          plotVector.push_back(nrm2(omB));
          plotVector.push_back(nrm2(omBK));
        }
        if(openMBVAngularAcceleration&& !openMBVAngularAcceleration->isHDF5Link()) {
          vector<double> data;
          data.push_back(getTime());
          data.push_back(0);
          data.push_back(0);
          data.push_back(0);
          data.push_back(psiB(0));
          data.push_back(psiB(1));
          data.push_back(psiB(2));
          data.push_back(0.5);
          openMBVpsiTrans->append(data);
          data.clear();
          data.push_back(getTime());
          data.push_back(rOOs(0));
          data.push_back(rOOs(1));
          data.push_back(rOOs(2));
          data.push_back(psiRot(0));
          data.push_back(psiRot(1));
          data.push_back(psiRot(2));
          data.push_back(0.5);
          openMBVpsiRot->append(data);
          data.clear();
          data.push_back(getTime());
          data.push_back(rOOs(0));
          data.push_back(rOOs(1));
          data.push_back(rOOs(2));
          data.push_back(psiRel(0));
          data.push_back(psiRel(1));
          data.push_back(psiRel(2));
          data.push_back(0.5);
          openMBVpsiRel->append(data);
          plotVector.push_back(nrm2(psiB));
          plotVector.push_back(nrm2(psiRot));
          plotVector.push_back(nrm2(psiRel));
        }
      }
#endif

      KinematicsObserver::plot();
    }
  }

  void RelativeKinematicsObserver::initializeUsingXML(DOMElement *element) {
    KinematicsObserver::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"frameOfReference");
    if(e) saved_frameOfReference=E(e)->getAttribute("ref");
  }

}
