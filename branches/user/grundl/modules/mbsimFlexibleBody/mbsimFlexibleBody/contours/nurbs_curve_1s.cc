/* Copyright (C) 2004-2012 MBSim Development Team
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
 * Contact: thorsten.schindler@mytum.de
 */

#include<config.h>

#include "mbsimFlexibleBody/contours/nurbs_curve_1s.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_cosserat.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  NurbsCurve1s::NurbsCurve1s(const std::string &name) :
      MBSim::Contour1s(name), Elements(0), openStructure(false), L(0.), degU(0), curveTranslations(), curveVelocities(), curveAngularVelocities(), normalRotationGrid() {
  }

  NurbsCurve1s::~NurbsCurve1s() {
  }

  void NurbsCurve1s::updateKinematicsForFrame(ContourPointData &cp, Frame::Feature ff) {
    if (ff == Frame::position || ff == Frame::position_cosy || ff == Frame::all) {
      Vec3 Tmppt = curveTranslations.pointAt(cp.getLagrangeParameterPosition()(0));
      cp.getFrameOfReference().setPosition(Tmppt);
    }

    if (ff == Frame::velocity || ff == Frame::velocity_cosy || ff == Frame::velocities || ff == Frame::velocities_cosy || ff == Frame::all) {
      Vec3 Tmpv = curveVelocities.pointAt(cp.getLagrangeParameterPosition()(0));
      cp.getFrameOfReference().setVelocity(Tmpv);
    }

    if (ff == Frame::angularVelocity || ff == Frame::velocity_cosy || ff == Frame::velocities || ff == Frame::velocities_cosy || ff == Frame::all) {
      double uStaggered = cp.getLagrangeParameterPosition()(0); // interpolation of angular velocity starts from 0 --> \phi_{1/2} but contour starts from 0 --> r_0 therefore this difference of l0/2
      double l0 = L / Elements;
      if (uStaggered < l0 / 2.)
        uStaggered = L - l0 / 2. + uStaggered;
      else
        uStaggered -= l0 / 2.;
      Vec3 Tmpav = curveAngularVelocities.pointAt(uStaggered);
      cp.getFrameOfReference().setAngularAcceleration(Tmpav);
    }
  }

  void NurbsCurve1s::updateJacobiansForFrame(ContourPointData &cp, int j /*=0*/) {
    cp.getFrameOfReference().getJacobianOfTranslation().resize(qSize);
    cp.getFrameOfReference().getJacobianOfRotation().resize(qSize); // TODO open structure

    double uStaggered = cp.getLagrangeParameterPosition()(0); // interpolation of Jacobian of Rotation starts from 0 --> \phi_{1/2} but Jacobian of Translation and contour starts from 0 --> r_0 therefore this difference of l0/2
    double l0 = L / Elements;
    if (uStaggered < l0 / 2.)
      uStaggered = L - l0 / 2. + uStaggered;
    else
      uStaggered -= l0 / 2.;

    for (int k = 0; k < qSize; k++) {
      Vec3 TmpPtTrans = CurveJacobiansOfTranslation[k].pointAt(cp.getLagrangeParameterPosition()(0));
      Vec3 TmpPtRot = CurveJacobiansOfRotation[k].pointAt(uStaggered);

      cp.getFrameOfReference().getJacobianOfTranslation().set(k, TmpPtTrans);

      cp.getFrameOfReference().getJacobianOfRotation().set(k, TmpPtRot);
    }
  }

  void NurbsCurve1s::initContourFromBody(InitStage stage) {
    if (stage == resize) {
      degU = 3;
      Elements = (static_cast<FlexibleBody1sCosserat*>(parent))->getNumberElements();
      qSize = (static_cast<FlexibleBody1sCosserat*>(parent))->getqSize();
      openStructure = (static_cast<FlexibleBody1sCosserat*>(parent))->isOpenStructure();
      L = (static_cast<FlexibleBody1sCosserat*>(parent))->getLength();

      for (int i = 0; i < Elements; i++) { // TODO openstructure: jacobians of Rotation different
        jacobiansTrans.push_back(ContourPointData(i));
        jacobiansRot.push_back(ContourPointData(i, ContourPointData::staggeredNode)); // jacobians of rotation are on staggered grid
      }
      for (int i = 0; i < Elements; i++) {
        jacobiansTrans[i].getFrameOfReference().getJacobianOfTranslation().resize();
        jacobiansRot[i].getFrameOfReference().getJacobianOfRotation().resize();
      }

      for (int k = 0; k < qSize; k++) { // TODO openstructure: jacobians of Rotation different
        CurveJacobiansOfTranslation.push_back(NurbsCurve());
        CurveJacobiansOfRotation.push_back(NurbsCurve());
      }

      computeCurveTranslations();
      computeCurveVelocities();
      computeCurveAngularVelocities();
    }
    else if (stage == worldFrameContourLocation) {
      R->getOrientation() = (static_cast<FlexibleBody1sCosserat*>(parent))->getFrameOfReference()->getOrientation();
      R->getPosition() = (static_cast<FlexibleBody1sCosserat*>(parent))->getFrameOfReference()->getPosition();
    }
  }

  void NurbsCurve1s::computeCurveTranslations(bool update) {

    int nodes = Elements;
    if (openStructure)
      nodes++;

    MatVx3 Nodelist(nodes, NONINIT);
    for (int i = 0; i < nodes; i++) {
      ContourPointData cp(i);
      static_cast<FlexibleBody1sCosserat*>(parent)->updateKinematicsForFrame(cp, Frame::position);
      Nodelist.set(i, trans(cp.getFrameOfReference().getPosition()));
    }

    if (update)
      curveTranslations.update(Nodelist);
    else {
      if (openStructure) {
        curveTranslations.globalInterp(Nodelist, 0, L, degU, true);
      }
      else {
        curveTranslations.globalInterpClosed(Nodelist, 0, L, degU, true);
      }
    }
  }

  void NurbsCurve1s::computeCurveVelocities(bool update) {
    int nodes = Elements;
    if (openStructure)
      nodes++;

    MatVx3 Nodelist(nodes, NONINIT);
    for (int i = 0; i < nodes; i++) {
      ContourPointData cp(i);
      static_cast<FlexibleBody1sCosserat*>(parent)->updateKinematicsForFrame(cp, Frame::velocity);
      Nodelist.set(i, trans(cp.getFrameOfReference().getVelocity()));
    }

    if (update)
      curveVelocities.update(Nodelist);
    else {
      if (openStructure) {
        curveVelocities.globalInterp(Nodelist, 0, L, degU, true);
      }
      else {
        curveVelocities.globalInterpClosed(Nodelist, 0, L, degU, true);
      }
    }
  }

  void NurbsCurve1s::computeCurveAngularVelocities(bool update) {
    int nodes = Elements;
    if (openStructure)
      nodes++;

    MatVx3 Nodelist(nodes, NONINIT);
    for (int i = 0; i < nodes; i++) {
      ContourPointData cp(i);
      static_cast<FlexibleBody1sCosserat*>(parent)->updateKinematicsForFrame(cp, Frame::angularVelocity);
      Nodelist.set(i, trans(cp.getFrameOfReference().getAngularVelocity()));
    }

    if (update)
      curveAngularVelocities.update(Nodelist);
    else {
      if (openStructure) {
        curveAngularVelocities.globalInterp(Nodelist, 0, L, degU, true);
      }
      else {
        curveAngularVelocities.globalInterpClosed(Nodelist, 0, L, degU, true);
      }
    }
  }

  void NurbsCurve1s::computeCurveJacobians(bool translational, bool rot, bool update) {
//TODO: All the if's should be unnecessary if every interpolation would be capsulated in two routines
    int nodes = Elements;
    if (openStructure)
      nodes++;

    MatVx3 NodelistTrans(nodes, NONINIT);
    MatVx3 NodelistRot(nodes, NONINIT);

    for (int i = 0; i < nodes; i++) {
      if (translational)
        static_cast<FlexibleBody1sCosserat*>(parent)->updateJacobiansForFrame(jacobiansTrans[i]);
      if (rot)
        static_cast<FlexibleBody1sCosserat*>(parent)->updateJacobiansForFrame(jacobiansRot[i]); // jacobians of rotation are on staggered grid
    }

    for (int k = 0; k < qSize; k++) {
      for (int i = 0; i < nodes; i++) {
        if (translational)
          NodelistTrans.set(i, trans(jacobiansTrans[i].getFrameOfReference().getJacobianOfTranslation(0).col(k)));

        if (rot)
          NodelistRot.set(i, trans(jacobiansRot[i].getFrameOfReference().getJacobianOfRotation(0).col(k)));

      }

      if (update) {
        if (translational)
          CurveJacobiansOfTranslation[k].update(NodelistTrans);
        if (rot)
          CurveJacobiansOfRotation[k].update(NodelistRot);
      }
      else {
        if (openStructure) {
        } // TODO
        else {
          if (translational)
            CurveJacobiansOfTranslation[k].globalInterpClosed(NodelistTrans, 0, L, degU, true);
          if (rot)
            CurveJacobiansOfRotation[k].globalInterpClosed(NodelistRot, 0, L, degU, true);
        }
      }
    }
  }

}

