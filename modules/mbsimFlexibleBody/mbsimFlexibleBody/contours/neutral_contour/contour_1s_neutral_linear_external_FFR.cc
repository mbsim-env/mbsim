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

namespace MBSimFlexibleBody {
  
  Contour1sNeutralLinearExternalFFR::Contour1sNeutralLinearExternalFFR(const std::string &name_) :
      Contour1sNeutralFactory(name_), transNodes(0), NP(NULL), NLP(NULL), NV(NULL), qSize(0) {
  }
  
//  Contour1sNeutralLinearExternalFFR::Contour1sNeutralLinearExternalFFR(const std::string &name_, FlexibleBodyLinearExternalFFR* parent_, std::vector<int> transNodes_, double nodeOffset_, double uMin_, double uMax_, int degU_, bool openStructure_) :
//      Contour1sNeutralFactory(name_), transNodes(transNodes_), nodeOffset(nodeOffset_), degU(degU_), NP(NULL), NLP(NULL), NV(NULL), qSize(0) {
//
//
//    parent_->addContour(this);
//  }

  Contour1sNeutralLinearExternalFFR::~Contour1sNeutralLinearExternalFFR() {
    delete NP;
    delete NLP;
    delete NV;

    NP = NULL;
    NLP = NULL;
    NV = NULL;
  }

  void Contour1sNeutralLinearExternalFFR::readTransNodes(string file) {
    ifstream contourfile((file).c_str());
    if (!contourfile.is_open()) {
      THROW_MBSIMERROR("Can not open file " + file);
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
      curveModeShape.push_back(NurbsCurve());
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

  void Contour1sNeutralLinearExternalFFR::init(InitStage stage) {

    if (stage == resize) {
      // construct contourPoint for translation nodes
      etaNodes.reserve(transNodes.size());
//      transContourPoints.reserve(transNodes.size());
//      rotContourPoints.reserve(numOfRotNodes);

//        nodeOffset = (static_cast<FlexibleBodyContinuum<double>*>(parent))->getNodeOffset();  // TODO change to be user set value

    }
    else if (stage == unknownStage) { //TODO: Actually for the calculate Initial values in the contact search it is necessary to call the following functions before (even though they also just compute initial values)

      qSize = (static_cast<FlexibleBodyLinearExternalFFR*>(parent))->getqSize();

//      for (int i = 0; i < transNodes.size(); i++)
//        transContourPoints.push_back(ContourPointData(transNodes(i)));

      NP = createNeutralPosition();
      NLP = createNeutralLocalPosition();
      NV = createNeutralVelocity();
      createNeutralModeShape();

      NP->computeCurve(false); // the first time call the computeCurveVelocity, the flag should be false
      NLP->computeCurve(false);
      NV->computeCurve(false);

      // TODO: check this!!!
      Vec u(NV->getuVec());
      for (int i = 0; i < u.size() - degU; i++)
        etaNodes.push_back(u(i));
    }

    Contour1sNeutralFactory::init(stage);
  }

  Vec3 Contour1sNeutralLinearExternalFFR::getPosition(double t, const Vec2 &zeta) {
    return NP->getPosition(t,zeta(0));
  }

  Vec3 Contour1sNeutralLinearExternalFFR::getWs(double t, const Vec2 &zeta) {
    return NP->getWs(t,zeta(0));
  }

  Vec3 Contour1sNeutralLinearExternalFFR::getWt(double t, const Vec2 &zeta) {
    return NP->getWt(t,zeta(0));
  }

  void Contour1sNeutralLinearExternalFFR::updatePositions(double t, ContourFrame *frame) {
    NP->update(t,frame);
    NP->updatePositionNormal(t,frame);
    NP->updatePositionFirstTangent(t,frame);
    NP->updatePositionSecondTangent(t,frame);
  }

  void Contour1sNeutralLinearExternalFFR::updateVelocities(double t, ContourFrame *frame) {
    NV->update(t,frame);
    frame->setAngularVelocity(static_cast<FlexibleBodyLinearExternalFFR*>(parent)->getFloatingFrameOfReference()->getAngularVelocity());
  }

  void Contour1sNeutralLinearExternalFFR::updateJacobians(double t, ContourFrame *frame, int j) {
    /******************************************************************  Jacobian of Translation  *******************************************************************************/
    Mat3xV Jacobian_trans(qSize, INIT, 0.);
    // translational DOF
    Jacobian_trans.set(Index(0, 2), Index(0, 2), SqrMat(3, EYE));

    // rotational DOF
    SqrMat3 A = static_cast<FlexibleBodyLinearExternalFFR*>(parent)->getA(t);
    SqrMat3 G_bar = static_cast<FlexibleBodyLinearExternalFFR*>(parent)->getG_bar(t);
    Vec3 u_bar = NLP->getLocalPosition(t,frame->getEta());
    Jacobian_trans.set(Index(0, 2), Index(3, 5), -A * tilde(u_bar) * G_bar);

    // elastic DOF
    Mat3xV modeShapeMatrix(qSize - 6, NONINIT);
    double position = frame->getEta();
    for (int k = 0; k < qSize - 6; k++) {
      Vec3 temp = curveModeShape.at(k).pointAt(position);
      modeShapeMatrix.set(k, temp);
    }

    Jacobian_trans.set(Index(0, 2), Index(6, qSize - 1), A * modeShapeMatrix);

    SqrMat3 wRA = (static_cast<FlexibleBodyLinearExternalFFR*>(parent))->getFrameOfReference()->getOrientation();
    frame->setJacobianOfTranslation(wRA * Jacobian_trans,j);

    /******************************************************************  Jacobian of Rotation  *******************************************************************************/
    Mat3xV Jacobian_rot(qSize, INIT, 0.);
    Jacobian_rot.set(Index(0, 2), Index(3, 5), A * G_bar);
    frame->setJacobianOfRotation(wRA * Jacobian_rot,j);
  }

//    if (ff == Frame::angle) { // only for opmbvBody visualization
//      SqrMat3 ALocal(INIT, 0);
//      NP->updatePositionNormal(cp);
//      NP->updatePositionFirstTangent(cp);
//      NP->updatePositionSecondTangent(cp);
//      ALocal = cp.getFrameOfReference().getOrientation();
//      cp.getFrameOfReference().setAnglesOfOrientation(AIK2Cardan(ALocal));
//    }
//  }

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
