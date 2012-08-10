/*
 * ContourPairing.cc
 *
 *  Created on: 25.09.2011
 *      Author: grundl
 */
#include <config.h>

#include "contour_pairing.h"

#include <mbsim/dynamic_system.h>
#include <mbsim/contour_pdata.h>
#include <mbsim/utils/eps.h>
#include <mbsim/utils/rotarymatrices.h>
#include <mbsim/contour.h>
#include <mbsim/utils/utils.h>
#include <mbsim/contact_kinematics/contact_kinematics.h>
#include <fmatvec.h>

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/group.h>
#include <openmbvcppinterface/frame.h>
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/objectfactory.h>
#include <mbsim/utils/eps.h>
#include <mbsim/utils/rotarymatrices.h>
#endif

using namespace fmatvec;
using namespace std;

namespace MBSim {

  ContourPairing::ContourPairing(const std::string & name_, Contour* contour1_, Contour* contour2_, bool plotContact /*= true*/, ContactKinematics * contactKinematics_ /*= 0*/) :
      Object(name_), contour1(contour1_), contour2(contour2_), contactKinematics(contactKinematics_), fdf(0), cpData(0), gTol(0.), gActive(true), gActive0(0), gk(0), gdk(0), lak(0), WF(0)
#ifdef HAVE_OPENMBVCPPINTERFACE
          , openMBVGroup(0), openMBVContactFrameEnabled(true), openMBVContactFrames(0), openMBVContactFrameSize(0), openMBVNormalForceArrow(0), openMBVFrictionForceArrow(0)
#endif
  {
    setPlotFeature(state, enabled);
    setPlotFeature(generalizedLinkForce, enabled);
    setPlotFeature(linkKinematics, enabled);

    if (contactKinematics == 0)
      contactKinematics = contour1->findContactPairingWith(contour1->getType(), contour2->getType());
    if (contactKinematics == 0)
      contactKinematics = contour1->findContactPairingWith(contour2->getType(), contour1->getType());
    if (contactKinematics == 0)
      contactKinematics = contour2->findContactPairingWith(contour2->getType(), contour1->getType());
    if (contactKinematics == 0)
      contactKinematics = contour2->findContactPairingWith(contour1->getType(), contour2->getType());
    if (contactKinematics == 0)
      throw MBSimError("Unknown contact pairing between Contour \"" + contour1->getType() + "\" and Contour\"" + contour2->getType() + "\"!");

    contactKinematics->assignContours(contour1, contour2);

  }

  ContourPairing::~ContourPairing() {
    if (contactKinematics) {
      delete contactKinematics;
      contactKinematics = 0;
    }

    for(vector<ContourPointData*>::iterator i = cpData.begin(); i != cpData.end(); ++i)
      delete[] *i;
    for (vector<Vec*>::iterator i = WF.begin(); i != WF.end(); ++i)
      delete[] *i;
  }

  void ContourPairing::updategd(double t) {
    if (gActive) {
      //update velocities on contour-points

      contour1->updateKinematicsForFrame(cpData[0][0], velocities); // angular velocity necessary e.g. see ContactKinematicsSpherePlane::updatewb
      contour2->updateKinematicsForFrame(cpData[0][1], velocities); // angular velocity necessary e.g. see ContactKinematicsSpherePlane::updatewb

      Vec3 Wn = cpData[0][0].getFrameOfReference().getOrientation().col(0);

      Vec3 WvD = cpData[0][1].getFrameOfReference().getVelocity() - cpData[0][0].getFrameOfReference().getVelocity();

      //write normal-velocity in vector
      gdk(0) = Wn.T() * WvD;

      if (gdk.size() > 1) { //are there more velocity-directions needed?
        Mat3V Wt(gdk.size() - 1);
        Wt.set(0,cpData[0][0].getFrameOfReference().getOrientation().col(1));
        if (gdk.size() > 2)
          Wt.set(1,cpData[0][0].getFrameOfReference().getOrientation().col(2));

        //write second (and third for 3D-contact) velocity into vector
        gdk(1, gdk.size() - 1) = Wt.T() * WvD;
      }
    }
  }

  void ContourPairing::checkActiveg() {
    gActive0 = gActive;
    gActive = gk(0) < gTol ? true : false;
    if(not gActive and gActive0 != gActive)
      for (int j = 0; j < 1 + getFrictionDirections(); j++)
        lak(j) = 0;
  }

  void ContourPairing::updateJacobians(double t, int j) {
    //TODO: checkActiveg shouldn't be here!
    checkActiveg();
    if (isActive()) {
      contour1->updateJacobiansForFrame(cpData[0][0],j);
      contour2->updateJacobiansForFrame(cpData[0][1],j);
    }
  }

  void ContourPairing::init(InitStage stage) {
    if (stage == MBSim::preInit) {
      gk = fmatvec::Vec(1);
      gdk = fmatvec::Vec(1);
      lak = fmatvec::Vec(3);
    }
    else if (stage == MBSim::resize) {
      for (vector<ContourPointData*>::iterator i = cpData.begin(); i != cpData.end(); ++i)
        delete[] *i;
      cpData.clear(); // clear container first, because InitStage resize is called twice (before and after the reorganization)
      cpData.push_back(new ContourPointData[2]);
      cpData[0][0].getFrameOfReference().setName("0");
      cpData[0][1].getFrameOfReference().setName("1");

      cpData[0][0].getFrameOfReference().sethSize(contour1->gethSize(0), 0);
      cpData[0][0].getFrameOfReference().sethSize(contour1->gethSize(1), 1);
      cpData[0][1].getFrameOfReference().sethSize(contour2->gethSize(0), 0);
      cpData[0][1].getFrameOfReference().sethSize(contour2->gethSize(1), 1);

      WF.push_back(new Vec());
      WF.push_back(new Vec());
      WF[0]->resize(3);
      WF[1]->resize(3);
    }

    else if (stage == MBSim::plot) {
      updatePlotFeatures();
      if (getPlotFeature(plotRecursive) == enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE

        if (getPlotFeature(openMBV) == enabled && (openMBVContactFrameSize > macheps() or openMBVNormalForceArrow or openMBVNormalForceArrow)) {
          openMBVGroup = new OpenMBV::Group();
          openMBVGroup->setName(name + "_ContactGroup");
          openMBVGroup->setExpand(false);
          parent->getOpenMBVGrp()->addObject(openMBVGroup);

          // frames
          if (openMBVContactFrameSize > macheps()) {
            openMBVContactFrames.push_back(new OpenMBV::Frame);
            openMBVContactFrames.push_back(new OpenMBV::Frame);
            for (unsigned int k = 0; k < 2; k++) {
              openMBVContactFrames[k]->setOffset(1.);
              openMBVContactFrames[k]->setSize(openMBVContactFrameSize);
              openMBVContactFrames[k]->setName(name + (k == 0 ? "A" : "B"));
              openMBVContactFrames[k]->setEnable(openMBVContactFrameEnabled);
              openMBVGroup->addObject(openMBVContactFrames[k]);
            }
          }

          // arrows
          if (openMBVNormalForceArrow) {
            openMBVNormalForceArrow=new OpenMBV::Arrow(*openMBVNormalForceArrow);
            openMBVNormalForceArrow->setName("NormalForce_" + name);
            openMBVGroup->addObject(openMBVNormalForceArrow);
          }

          if (openMBVFrictionForceArrow && getFrictionDirections()>0) {
            openMBVFrictionForceArrow=new OpenMBV::Arrow(*openMBVFrictionForceArrow);
            openMBVFrictionForceArrow->setName("FrictionForce_" + name);
            openMBVGroup->addObject(openMBVFrictionForceArrow);
          }
        }
#endif

        if (getPlotFeature(linkKinematics) == enabled) {
          plotColumns.push_back("g(" + numtostr(0) + ")");
          for (int j = 0; j < 1 + getFrictionDirections(); ++j)
            plotColumns.push_back("gd(" + numtostr(j) + ")");
        }

        if (getPlotFeature(generalizedLinkForce) == enabled) {
          for (int j = 0; j < 1 + getFrictionDirections(); ++j)
            plotColumns.push_back("la(" + numtostr(j) + ")");
        }

      }
    }

    Object::init(stage);
  }

  void ContourPairing::plot(double t, double dt) {
    if (getPlotFeature(plotRecursive) == enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if (getPlotFeature(openMBV) == enabled && (openMBVContactFrameSize > epsroot() || openMBVNormalForceArrow || openMBVFrictionForceArrow)) {
        vector<double> data;
        // frames
        if (openMBVContactFrameSize > epsroot()) {
          for (int k = 0; k < 2; k++) {
            data.clear();
            data.push_back(t);
            data.push_back(cpData[0][k].getFrameOfReference().getPosition()(0));
            data.push_back(cpData[0][k].getFrameOfReference().getPosition()(1));
            data.push_back(cpData[0][k].getFrameOfReference().getPosition()(2));
            Vec3 cardan = AIK2Cardan(cpData[0][k].getFrameOfReference().getOrientation());
            data.push_back(cardan(0));
            data.push_back(cardan(1));
            data.push_back(cardan(2));
            data.push_back(0);
            openMBVContactFrames[k]->append(data);
          }
        }

        // arrows
        // normal force

        if (openMBVNormalForceArrow) {
          data.clear();
          data.push_back(t);
          data.push_back(cpData[0][1].getFrameOfReference().getPosition()(0));
          data.push_back(cpData[0][1].getFrameOfReference().getPosition()(1));
          data.push_back(cpData[0][1].getFrameOfReference().getPosition()(2));
          Vec3 F;
          if(gActive)
            F = cpData[0][0].getFrameOfReference().getOrientation().col(0) * lak(0);
          data.push_back(F(0));
          data.push_back(F(1));
          data.push_back(F(2));
          data.push_back(nrm2(F));
          openMBVNormalForceArrow->append(data);
        }

        // friction force
        if (openMBVFrictionForceArrow && getFrictionDirections() > 0) {
          data.clear();
          data.push_back(t);
          data.push_back(cpData[0][1].getFrameOfReference().getPosition()(0));
          data.push_back(cpData[0][1].getFrameOfReference().getPosition()(1));
          data.push_back(cpData[0][1].getFrameOfReference().getPosition()(2));
          Vec3 F;
          if(gActive) {
            F = cpData[0][0].getFrameOfReference().getOrientation().col(1) * lak(1);
            if (getFrictionDirections() > 1)
              F += cpData[0][0].getFrameOfReference().getOrientation().col(2) * lak(2);
          }
          data.push_back(F(0));
          data.push_back(F(1));
          data.push_back(F(2));
          data.push_back((1 && lak.size() > 1) ? 1 : 0.5); // draw in green if slipping and draw in red if sticking
          openMBVFrictionForceArrow->append(data);
        }
      }
#endif
      if (getPlotFeature(linkKinematics) == enabled) {
        plotVector.push_back(gk(0)); //gN
        for (int j = 0; j < 1 + getFrictionDirections(); j++)
          plotVector.push_back(gdk(j)); //gd
      }

      if (getPlotFeature(generalizedLinkForce) == enabled) {
        if (gActive) {
          plotVector.push_back(lak(0));
          if (fdf) {
            for (int j = 1; j < 1 + getFrictionDirections(); j++)
              plotVector.push_back(lak(j));
          }
        }
        else {
          for (int j = 0; j < 1 + getFrictionDirections(); j++)
            plotVector.push_back(0);
        }
      }
      Object::plot(t, dt);
    }
  }

  void ContourPairing::closePlot() {
    if (getPlotFeature(plotRecursive) == enabled) {
      Object::closePlot();
    }
  }

  int ContourPairing::getFrictionDirections() {
    if (fdf)
      return fdf->getFrictionDirections();
    else
      return 0;
  }
}
