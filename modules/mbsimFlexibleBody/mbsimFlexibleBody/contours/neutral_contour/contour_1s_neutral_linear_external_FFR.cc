/*
 * contour_1s_neutral_linear_external_FFR.cc
 *
 *  Created on: 25.10.2013
 *      Author: zwang
 */
#include <config.h>
#include "contour_1s_neutral_linear_external_FFR.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsimFlexibleBody/utils/cardan.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_linear_external_ffr.h"
#include "mbsim/utils/rotarymatrices.h"

using namespace MBSim;
using namespace fmatvec;
using namespace std;

namespace MBSimFlexibleBody {
  
  Contour1sNeutralLinearExternalFFR::~Contour1sNeutralLinearExternalFFR() {
    delete NP;
    delete NLP;
    delete NV;

    NP = nullptr;
    NLP = nullptr;
    NV = nullptr;
  }

  void Contour1sNeutralLinearExternalFFR::readTransNodes(const string& file) {
    ifstream contourfile((file).c_str());
    if (!contourfile.is_open()) {
      throwError("Can not open file " + file);
    }
    string s;
    getline(contourfile, s);

    transNodes.resize() = VecInt(s.c_str());
  }

  NeutralNurbsVelocity1s* Contour1sNeutralLinearExternalFFR::createNeutralVelocity() {
    return new NeutralNurbsVelocity1s(parent, transNodes, 0, uMin, uMax, degU, openStructure);
  }

  NeutralNurbsPosition1s* Contour1sNeutralLinearExternalFFR::createNeutralPosition() {
    return new NeutralNurbsPosition1s(parent, transNodes, 0, uMin, uMax, degU, openStructure);
  }

  NeutralNurbsLocalPosition1s* Contour1sNeutralLinearExternalFFR::createNeutralLocalPosition() {
    return new NeutralNurbsLocalPosition1s(parent, transNodes, 0, uMin, uMax, degU, openStructure);
  }

  void Contour1sNeutralLinearExternalFFR::createNeutralModeShape() {
    for (int k = 0; k < qSize - 6; k++) {
      curveModeShape.emplace_back();
    }

    MatVx3 Nodelist(transNodes.size(), NONINIT);
    for (int k = 0; k < qSize - 6; k++) {
      for (int i = 0; i < transNodes.size(); i++) {
        Nodelist.set(i, trans((static_cast<FlexibleBodyLinearExternalFFR*>(parent))->getModeShapeVector(transNodes(i), k)));
      }
      if (openStructure) {
        curveModeShape.at(k).globalInterp(Nodelist, uMin, uMax, degU, false); // calculate once, as the mode shape is constant.
      }
      else {
        curveModeShape.at(k).globalInterpClosed(Nodelist, uMin, uMax, degU, false); // calculate once, as the mode shape is constant.
      }
    }
  }

  void Contour1sNeutralLinearExternalFFR::init(InitStage stage, const InitConfigSet &config) {
    if (stage == unknownStage) { //TODO: Actually for the calculate Initial values in the contact search it is necessary to call the following functions before (even though they also just compute initial values)

      qSize = (static_cast<FlexibleBodyLinearExternalFFR*>(parent))->getqSize();

      NP = createNeutralPosition();
      NP->setBinormalDir(-R->evalOrientation().col(2));
      NLP = createNeutralLocalPosition();
      NV = createNeutralVelocity();
      createNeutralModeShape();

      NP->computeCurve(false); // the first time call the computeCurveVelocity, the flag should be false
      NLP->computeCurve(false);
      NV->computeCurve(false);

      resetUpToDate();

      Contour1sNeutralFactory::init(stage, config);
    }

    Contour1sNeutralFactory::init(stage, config);
  }

  Vec3 Contour1sNeutralLinearExternalFFR::evalPosition(const Vec2 &zeta) {
    return NP->evalPosition(zeta(0));
  }

  Vec3 Contour1sNeutralLinearExternalFFR::evalWs(const Vec2 &zeta) {
    return NP->evalWs(zeta(0));
  }

  Vec3 Contour1sNeutralLinearExternalFFR::evalWt(const Vec2 &zeta) {
    return NP->evalWt(zeta(0));
  }

  void Contour1sNeutralLinearExternalFFR::updatePositions(ContourFrame *frame) {
    NP->update(frame);
    NP->updatePositionNormal(frame);
    NP->updatePositionFirstTangent(frame);
    NP->updatePositionSecondTangent(frame);
  }

  void Contour1sNeutralLinearExternalFFR::updateVelocities(ContourFrame *frame) {
    NV->update(frame);
    frame->setAngularVelocity(static_cast<FlexibleBodyLinearExternalFFR*>(parent)->getFloatingFrameOfReference()->evalAngularVelocity());
  }

  void Contour1sNeutralLinearExternalFFR::updateJacobians(ContourFrame *frame, int j) {
    /******************************************************************  Jacobian of Translation  *******************************************************************************/
    Mat3xV Jacobian_trans(qSize, INIT, 0.);
    // translational DOF
    Jacobian_trans.set(RangeV(0, 2), RangeV(0, 2), SqrMat(3, EYE));

    // rotational DOF
    SqrMat3 A = static_cast<FlexibleBodyLinearExternalFFR*>(parent)->evalA();
    SqrMat3 G_bar = static_cast<FlexibleBodyLinearExternalFFR*>(parent)->evalG_bar();
    Vec3 u_bar = NLP->evalLocalPosition(frame->evalEta());
    Jacobian_trans.set(RangeV(0, 2), RangeV(3, 5), -A * tilde(u_bar) * G_bar);

    // elastic DOF
    Mat3xV modeShapeMatrix(qSize - 6, NONINIT);
    double position = frame->getEta();
    for (int k = 0; k < qSize - 6; k++) {
      Vec3 temp = curveModeShape.at(k).pointAt(position);
      modeShapeMatrix.set(k, temp);
    }

    Jacobian_trans.set(RangeV(0, 2), RangeV(6, qSize - 1), A * modeShapeMatrix);

    SqrMat3 wRA = (static_cast<FlexibleBodyLinearExternalFFR*>(parent))->getFrameOfReference()->evalOrientation();
    frame->setJacobianOfTranslation(wRA * Jacobian_trans,j);

    /******************************************************************  Jacobian of Rotation  *******************************************************************************/
    Mat3xV Jacobian_rot(qSize, INIT, 0.);
    Jacobian_rot.set(RangeV(0, 2), RangeV(3, 5), A * G_bar);
    frame->setJacobianOfRotation(wRA * Jacobian_rot,j);
  }

  VecInt Contour1sNeutralLinearExternalFFR::getTransNodes() {
    return transNodes;
  }

  void Contour1sNeutralLinearExternalFFR::resetUpToDate() {
    Contour1sNeutralFactory::resetUpToDate();
    NP->resetUpToDate();
    NLP->resetUpToDate();
    NV->resetUpToDate();
  }

} /* namespace MBSimFlexibleBody */
