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
 *          rzander@users.berlios.de
 */

#include <config.h>
#include "mbsim/contours/contour1s_analytical.h"
#include "mbsim/mbsim_event.h"
#include "mbsim/objectfactory.h"
#include "mbsim/utils/contour_functions.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include "mbsim/object.h"
#include "mbsim/utils/rotarymatrices.h"
#include <openmbvcppinterface/group.h>
#include "openmbvcppinterface/polygonpoint.h"
#include "openmbvcppinterface/extrusion.h"
#endif

using namespace fmatvec;
using namespace std;

namespace MBSim {

  Contour1sAnalytical::~Contour1sAnalytical() {
     if (funcCrPC) 
       delete funcCrPC;
     funcCrPC=NULL;
  }

  void Contour1sAnalytical::updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff) {
    if(ff==firstTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
      cp.getFrameOfReference().getOrientation().col(0)= funcCrPC->computeN(cp.getLagrangeParameterPosition()(0));
      cp.getFrameOfReference().getOrientation().col(1)= funcCrPC->computeT(cp.getLagrangeParameterPosition()(0));
      cp.getFrameOfReference().getOrientation().col(2)= funcCrPC->computeB(cp.getLagrangeParameterPosition()(0));
      cp.getFrameOfReference().getOrientation() = R.getOrientation() * cp.getFrameOfReference().getOrientation();
    }
    if(ff==position || ff==position_cosy || ff==all) 
      cp.getFrameOfReference().getPosition() = R.getPosition() + R.getOrientation()*(*funcCrPC)(cp.getLagrangeParameterPosition()(0));
    if(ff==velocity || ff==velocity_cosy || ff==velocities || ff==velocities_cosy || ff==all) 
      cp.getFrameOfReference().getVelocity() = R.getVelocity() + crossProduct(R.getAngularVelocity(),R.getOrientation()*(*funcCrPC)(cp.getLagrangeParameterPosition()(0)));
    if(ff==angularVelocity || ff==velocities || ff==velocities_cosy || ff==all) 
      cp.getFrameOfReference().setAngularVelocity(R.getAngularVelocity());
  }

  void Contour1sAnalytical::updateJacobiansForFrame(ContourPointData &cp) {
    Vec WrPC = cp.getFrameOfReference().getPosition() - R.getPosition();
    Mat tWrPC = tilde(WrPC);

    cp.getFrameOfReference().setJacobianOfTranslation(
        R.getJacobianOfTranslation() - tWrPC*R.getJacobianOfRotation());
    cp.getFrameOfReference().setJacobianOfRotation(
        R.getJacobianOfRotation());
    cp.getFrameOfReference().setGyroscopicAccelerationOfTranslation(
        R.getGyroscopicAccelerationOfTranslation() - tWrPC*R.getGyroscopicAccelerationOfRotation() + 
        crossProduct(R.getAngularVelocity(),crossProduct(R.getAngularVelocity(),WrPC)));
    cp.getFrameOfReference().setGyroscopicAccelerationOfRotation(
        R.getGyroscopicAccelerationOfRotation());

    // adapt dimensions if necessary
    if(cp.getFrameOfReference().getJacobianOfTranslation().rows() == 0)
      cp.getFrameOfReference().getJacobianOfTranslation().resize(3,R.getJacobianOfTranslation().cols());
    if(cp.getFrameOfReference().getJacobianOfRotation().rows() == 0)
      cp.getFrameOfReference().getJacobianOfRotation().resize(3,R.getJacobianOfRotation().cols());
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void Contour1sAnalytical::enableOpenMBV(bool enable) {
    if (enable) {
      vector<OpenMBV::PolygonPoint*> * vpp = new vector<OpenMBV::PolygonPoint*>();
      for (double alpha=as; alpha<ae; alpha+=(ae-as)/720.) { 
        Vec CrPC=(*funcCrPC)(alpha);
        vpp->push_back(new OpenMBV::PolygonPoint(CrPC(1), CrPC(2), 0));
      }
      vpp->push_back((*vpp)[0]);
      openMBVRigidBody=new OpenMBV::Extrusion;
      ((OpenMBV::Extrusion*)openMBVRigidBody)->setHeight(0);
      ((OpenMBV::Extrusion*)openMBVRigidBody)->addContour(vpp);
      ((OpenMBV::Extrusion*)openMBVRigidBody)->setInitialRotation(0, .5*M_PI, .5*M_PI);
    }
    else
      openMBVRigidBody=0;
      //throw 1;
  }
#endif

  void Contour1sAnalytical::init(InitStage stage) {
    if(stage==MBSim::plot) {
      updatePlotFeatures(parent);
  
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
        data.push_back(R.getPosition()(0));
        data.push_back(R.getPosition()(1));
        data.push_back(R.getPosition()(2));
        Vec cardan=AIK2Cardan(R.getOrientation());
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
    e=element->FirstChildElement(MBSIMNS"enableOpenMBV");
    if (e)
      enableOpenMBV(true);
  }

}

