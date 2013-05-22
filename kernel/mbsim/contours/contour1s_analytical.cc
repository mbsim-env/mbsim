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
 *          rzander@users.berlios.de
 */

#include <config.h>
#include "mbsim/contours/contour1s_analytical.h"
#include "mbsim/mbsim_event.h"
#include "mbsim/objectfactory.h"
#include "mbsim/utils/contour_functions.h"
#include "mbsim/utils/contact_utils.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include "mbsim/object.h"
#include "mbsim/utils/rotarymatrices.h"
#include <openmbvcppinterface/group.h>
#include "openmbvcppinterface/polygonpoint.h"
#include "openmbvcppinterface/extrusion.h"
#include "mbsim/utils/nonlinear_algebra.h"
#include "mbsim/utils/eps.h"
#endif

using namespace fmatvec;
using namespace MBXMLUtils;
using namespace std;

namespace MBSim {

  Contour1sAnalytical::~Contour1sAnalytical() {
     if (funcCrPC) 
       delete funcCrPC;
     funcCrPC=NULL;
  }

  void Contour1sAnalytical::updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff) {
    if(ff==firstTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
      cp.getFrameOfReference().getOrientation().set(0, funcCrPC->computeN(cp.getLagrangeParameterPosition()(0)));
      cp.getFrameOfReference().getOrientation().set(1, funcCrPC->computeT(cp.getLagrangeParameterPosition()(0)));
      cp.getFrameOfReference().getOrientation().set(2, funcCrPC->computeB(cp.getLagrangeParameterPosition()(0)));
      cp.getFrameOfReference().getOrientation() = R->getOrientation() * cp.getFrameOfReference().getOrientation();
    }
    if(ff==position || ff==position_cosy || ff==all) 
      cp.getFrameOfReference().getPosition() = R->getPosition() + R->getOrientation()*(*funcCrPC)(cp.getLagrangeParameterPosition()(0));
    if(ff==velocity || ff==velocity_cosy || ff==velocities || ff==velocities_cosy || ff==all) 
      cp.getFrameOfReference().getVelocity() = R->getVelocity() + crossProduct(R->getAngularVelocity(),R->getOrientation()*(*funcCrPC)(cp.getLagrangeParameterPosition()(0)));
    if(ff==angularVelocity || ff==velocities || ff==velocities_cosy || ff==all) 
      cp.getFrameOfReference().setAngularVelocity(R->getAngularVelocity());
  }

  void Contour1sAnalytical::updateJacobiansForFrame(ContourPointData &cp, int j) {
    Vec3 WrPC = cp.getFrameOfReference().getPosition() - R->getPosition();
    Mat3x3 tWrPC = tilde(WrPC);

    cp.getFrameOfReference().setJacobianOfTranslation(
        R->getJacobianOfTranslation(j) - tWrPC*R->getJacobianOfRotation(j),j);
    cp.getFrameOfReference().setJacobianOfRotation(
        R->getJacobianOfRotation(j),j);
    cp.getFrameOfReference().setGyroscopicAccelerationOfTranslation(
        R->getGyroscopicAccelerationOfTranslation() - tWrPC*R->getGyroscopicAccelerationOfRotation() + 
        crossProduct(R->getAngularVelocity(),crossProduct(R->getAngularVelocity(),WrPC)));
    cp.getFrameOfReference().setGyroscopicAccelerationOfRotation(
        R->getGyroscopicAccelerationOfRotation());

    // adapt dimensions if necessary
    if(cp.getFrameOfReference().getJacobianOfTranslation(j).rows() == 0)
      cp.getFrameOfReference().getJacobianOfTranslation(j).resize(R->getJacobianOfTranslation(j).cols());
    if(cp.getFrameOfReference().getJacobianOfRotation(j).rows() == 0)
      cp.getFrameOfReference().getJacobianOfRotation(j).resize(R->getJacobianOfRotation(j).cols());
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void Contour1sAnalytical::enableOpenMBV(bool enable) {
    if (enable) {
      double rMax=0;
      for (double a=as; a<=ae; a+=1e-3*(ae-as)) {
        const double r=nrm2((*funcCrPC)(a));
        if (r>rMax)
          rMax=r;
      }
      vector<double> alpha;
      alpha.push_back(as);
      while(alpha.back()<ae) {
        class PointDistance : public Function1<double, double> {
          public:
            PointDistance(Vec3 p1_, ContourFunction1s * f_, double d_) : p1(p1_), f(f_), d(d_) {}
            double operator()(const double &alpha, const void * = NULL) {
              return nrm2((*f)(alpha)-p1)-d;
            }
          private:
            Vec3 p1;
            ContourFunction1s * f;
            double d;
        };
        PointDistance g((*funcCrPC)(alpha.back()), funcCrPC, .1*rMax);
        RegulaFalsi solver(&g);
        solver.setTolerance(epsroot());
        double aeTmp;
        if (alpha.back()<as+.25*(ae-as))
          aeTmp=as+.25*(ae-as);
        else if (alpha.back()<as+.5*(ae-as))
          aeTmp=as+.5*(ae-as);
        else if (alpha.back()<as+.75*(ae-as))
          aeTmp=as+.75*(ae-as);
        else
          aeTmp=ae;
        alpha.push_back(solver.solve(alpha.back(), aeTmp));
      }
      if (alpha.back()>ae)
        alpha.back()=ae;
      else
        alpha.push_back(ae);

      vector<OpenMBV::PolygonPoint*> * vpp = new vector<OpenMBV::PolygonPoint*>();
      for (unsigned int i=0; i<alpha.size(); i++) {
        const Vec3 CrPC=(*funcCrPC)(alpha[i]);
        vpp->push_back(new OpenMBV::PolygonPoint(CrPC(1), CrPC(2), 0));
      }
      openMBVRigidBody=new OpenMBV::Extrusion;
      ((OpenMBV::Extrusion*)openMBVRigidBody)->setHeight(0);
      ((OpenMBV::Extrusion*)openMBVRigidBody)->addContour(vpp);
      ((OpenMBV::Extrusion*)openMBVRigidBody)->setInitialRotation(0, .5*M_PI, .5*M_PI);
    }
    else
      openMBVRigidBody=NULL;
  }
#endif

  void Contour1sAnalytical::init(InitStage stage) {
    if(stage==MBSim::plot) {
      updatePlotFeatures();
  
      if(getPlotFeature(plotRecursive)==enabled) {
  #ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled && openMBVRigidBody) {
          openMBVRigidBody->setName(name);
          parent->getOpenMBVGrp()->addObject(openMBVRigidBody);
        }
  #endif
        Contour1s::init(stage);
      }
    }
    else
      Contour1s::init(stage);
  }

  void Contour1sAnalytical::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled && openMBVRigidBody) {
        vector<double> data;
        data.push_back(t);
        data.push_back(R->getPosition()(0));
        data.push_back(R->getPosition()(1));
        data.push_back(R->getPosition()(2));
        Vec3 cardan=AIK2Cardan(R->getOrientation());
        data.push_back(cardan(0));
        data.push_back(cardan(1));
        data.push_back(cardan(2));
        data.push_back(0);
        openMBVRigidBody->append(data);
      }
#endif
      Contour1s::plot(t,dt);
    }
  }
      
  double Contour1sAnalytical::computeCurvature(ContourPointData &cp) {
    return funcCrPC->computeCurvature(cp);
  }

  double Contour1sAnalytical::computeDistance(const double s, const int order) {
    if (order==0)
      return funcCrPC->computeR(s);
    else if (order==1)
      return funcCrPC->computedRdAlpha(s);
    else if (order==2)
      return funcCrPC->computed2RdAlpha2(s);
    else {
      throw MBSimError("ERROR (Contour1sAnalytical::computeDistance): Not implemented.");
      return 0.;
    }
  }

  void Contour1sAnalytical::initializeUsingXML(TiXmlElement * element) {
    Contour::initializeUsingXML(element);
    TiXmlElement * e;
    //ContourContinuum
    e=element->FirstChildElement(MBSIMNS"alphaStart");
    as=atof(e->GetText());
    e=element->FirstChildElement(MBSIMNS"alphaEnd");
    ae=atof(e->GetText());
    e=element->FirstChildElement(MBSIMNS"nodes");
    nodes=Vec(e->GetText());
    //Contour1s
    e=element->FirstChildElement(MBSIMNS"diameter");
    diameter=atof(e->GetText());
    //Contour1sAnalytical
    e=element->FirstChildElement(MBSIMNS"contourFunction");
    funcCrPC=ObjectFactory::getInstance()->createContourFunction1s(e->FirstChildElement());
    funcCrPC->initializeUsingXML(e->FirstChildElement());
#ifdef HAVE_OPENMBVCPPINTERFACE
    e=element->FirstChildElement(MBSIMNS"enableOpenMBV");
    if (e)
      enableOpenMBV(true);
#endif
  }

  TiXmlElement* Contour1sAnalytical::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = Contour::writeXMLFile(parent);
    addElementText(ele0,MBSIMNS"alphaStart",as);
    addElementText(ele0,MBSIMNS"alphaEnd",ae);
    addElementText(ele0,MBSIMNS"nodes",nodes);
    addElementText(ele0,MBSIMNS"diameter",diameter);
    TiXmlElement *ele1 = new TiXmlElement(MBSIMNS"contourFunction");
    funcCrPC->writeXMLFile(ele1);
    ele0->LinkEndChild(ele1);
#ifdef HAVE_OPENMBVCPPINTERFACE
    if(openMBVRigidBody)
      ele0->LinkEndChild(new TiXmlElement(MBSIMNS"enableOpenMBV"));
#endif
    return ele0;
  }

  ContactKinematics * Contour1sAnalytical::findContactPairingWith(std::string type0, std::string type1) {
    return findContactPairingRigidRigid(type0.c_str(), type1.c_str());
  }

}

