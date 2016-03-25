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
#include "mbsim/mbsim_event.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

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

      computeCurveTranslations(0);
      computeCurveVelocities(0);
      computeCurveAngularVelocities(0);
    }
    else if (stage == worldFrameContourLocation) {
      R->getOrientation() = (static_cast<FlexibleBody1sCosserat*>(parent))->getFrameOfReference()->getOrientation();
      R->getPosition() = (static_cast<FlexibleBody1sCosserat*>(parent))->getFrameOfReference()->getPosition();
    }
  }

  void NurbsCurve1s::computeCurveTranslations(double t, bool update) {

    int nodes = Elements;
    if (openStructure)
      nodes++;

    MatVx3 Nodelist(nodes, NONINIT);
    for (int i = 0; i < nodes; i++) {
      ContourPointData cp(i);
      Nodelist.set(i, trans(cp.getFrameOfReference().getPosition(t)));
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

  void NurbsCurve1s::computeCurveVelocities(double t, bool update) {
    int nodes = Elements;
    if (openStructure)
      nodes++;

    MatVx3 Nodelist(nodes, NONINIT);
    for (int i = 0; i < nodes; i++) {
      ContourPointData cp(i);
      Nodelist.set(i, trans(cp.getFrameOfReference().getVelocity(t)));
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

  void NurbsCurve1s::computeCurveAngularVelocities(double t, bool update) {
    int nodes = Elements;
    if (openStructure)
      nodes++;

    MatVx3 Nodelist(nodes, NONINIT);
    for (int i = 0; i < nodes; i++) {
      ContourPointData cp(i);
      Nodelist.set(i, trans(cp.getFrameOfReference().getAngularVelocity(t)));
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

  void NurbsCurve1s::computeCurveJacobians(double t, bool translational, bool rot, bool update) {
//TODO: All the if's should be unnecessary if every interpolation would be capsulated in two routines
    int nodes = Elements;
    if (openStructure)
      nodes++;

    MatVx3 NodelistTrans(nodes, NONINIT);
    MatVx3 NodelistRot(nodes, NONINIT);

    for (int k = 0; k < qSize; k++) {
      for (int i = 0; i < nodes; i++) {
        if (translational)
          NodelistTrans.set(i, trans(jacobiansTrans[i].getFrameOfReference().getJacobianOfTranslation(t,0).col(k)));

        if (rot)
          NodelistRot.set(i, trans(jacobiansRot[i].getFrameOfReference().getJacobianOfRotation(t,0).col(k)));
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

  void NurbsCurve1s::computeRootFunctionPosition(double t, MBSim::ContourFrame *frame) {
    THROW_MBSIMERROR("(NurbsCurve1s::computeRootFunctionPosition): Not implemented!");
  }

  void NurbsCurve1s::computeRootFunctionFirstTangent(double t, MBSim::ContourFrame *frame) {
    THROW_MBSIMERROR("(NurbsCurve1s::computeRootFunctionFirstTangent): Not implemented!");
  }

  void NurbsCurve1s::computeRootFunctionNormal(double t, MBSim::ContourFrame *frame) {
    THROW_MBSIMERROR("(NurbsCurve1s::computeRootFunctionNormal): Not implemented!");
  }

  void NurbsCurve1s::computeRootFunctionSecondTangent(double t, MBSim::ContourFrame *frame) {
    THROW_MBSIMERROR("(NurbsCurve1s::computeRootFunctionSecondTangent): Not implemented!");
  }

  ContactKinematics *NurbsCurve1s::findContactPairingWith(std::string type0, std::string type1) {
    THROW_MBSIMERROR("(NurbsCurve1s::findContactPairingWith): Not implemented!");
  }

}
