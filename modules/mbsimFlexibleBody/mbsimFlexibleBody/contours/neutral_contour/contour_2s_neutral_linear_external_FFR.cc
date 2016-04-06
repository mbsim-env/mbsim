/*
 * contour_2s_neutral_linear_external_FFR.cc
 *
 *  Created on: 25.10.2013
 *      Author: zwang
 */
#include <config.h>
#include "contour_2s_neutral_linear_external_FFR.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsimFlexibleBody/utils/cardan.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_linear_external_ffr.h"
#include "mbsim/utils/rotarymatrices.h"

using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  Contour2sNeutralLinearExternalFFR::~Contour2sNeutralLinearExternalFFR() {
    delete NP;
    delete NLP;
    delete NV;

    NP = NULL;
    NLP = NULL;
    NV = NULL;
  }

  void Contour2sNeutralLinearExternalFFR::createNeutralModeShape() {
    for (int k = 0; k < qSize - 6; k++) {
      surfaceModeShape.push_back(NurbsSurface());
    }

    GeneralMatrix<Vec3> Nodelist(getNumberOfTransNodesU(), getNumberOfTransNodesV());
    for (int modeNumber = 0; modeNumber < qSize - 6; modeNumber++) {
      for (int i = 0; i < getNumberOfTransNodesU(); i++) {
        for (int j = 0; j < getNumberOfTransNodesV(); j++) {
          Nodelist(i, j) = (static_cast<FlexibleBodyLinearExternalFFR*>(parent))->getModeShapeVector(transNodes(i, j), modeNumber);
        }
      }
      if (openStructure) {
        surfaceModeShape.at(modeNumber).globalInterp(Nodelist, uk, vl, degU, degV); // calculate once, as the mode shape is constant.
      }
      else {
        surfaceModeShape.at(modeNumber).globalInterpClosedU(Nodelist, uk, vl, degU, degV); // calculate once, as the mode shape is constant.
      }
    }
  }

  void Contour2sNeutralLinearExternalFFR::init(InitStage stage) {

    if (stage == unknownStage) { //TODO: Actually for the calculate Initial values in the contact search it is necessary to call the following functions before (even though they also just compute initial values)

      qSize = (static_cast<FlexibleBodyLinearExternalFFR*>(parent))->getqSize();

      NP = createNeutralPosition();
      NLP = createNeutralLocalPosition();
      NV = createNeutralVelocity();

      // use the local position to generate the lagrange parameters for nurbs interpolation in the undeformed state.
      if (openStructure)
        NLP->surfMeshParams(uk, vl);
      else
        NLP->surfMeshParamsClosedU(uk, vl);

      NP->setuk(uk);
      NP->setvl(vl);
      NLP->setuk(uk);
      NLP->setvl(vl);
      NV->setuk(uk);
      NV->setvl(vl);

      createNeutralModeShape();

      NP->computeCurve(false); // the first time call the computeCurveVelocity, the flag should be false
      NLP->computeCurve(false);
      NV->computeCurve(false);

      Contour2sNeutralFactory::init(stage);
    }

    Contour2sNeutralFactory::init(stage);
  }

  Vec3 Contour2sNeutralLinearExternalFFR::evalPosition(const Vec2 &zeta) {
    return NP->evalPosition(zeta);
  }

  Vec3 Contour2sNeutralLinearExternalFFR::evalWs(const Vec2 &zeta) {
    return NP->evalWs(zeta);
  }

  Vec3 Contour2sNeutralLinearExternalFFR::evalWt(const Vec2 &zeta) {
    return NP->evalWt(zeta);
  }

  Vec3 Contour2sNeutralLinearExternalFFR::evalWn(const Vec2 &zeta) {
    return NP->evalWn(zeta);
  }

  void Contour2sNeutralLinearExternalFFR::updatePositions(ContourFrame *frame) {
    NP->update(frame);
    NP->updatePositionNormal(frame);
    NP->updatePositionFirstTangent(frame);
    NP->updatePositionSecondTangent(frame);
  }

  void Contour2sNeutralLinearExternalFFR::updateVelocities(ContourFrame *frame) {
    NV->update(frame);
    frame->setAngularVelocity(static_cast<FlexibleBodyLinearExternalFFR*>(parent)->getFloatingFrameOfReference()->evalAngularVelocity());
  }

  void Contour2sNeutralLinearExternalFFR::updateJacobians(ContourFrame *frame, int j) {
    /******************************************************************  Jacobian of Translation  *******************************************************************************/
    Mat3xV Jacobian_trans(qSize, INIT, 0.);
    // translational DOF
    Jacobian_trans.set(Index(0, 2), Index(0, 2), SqrMat(3, EYE));

    // rotational DOF
    SqrMat3 A = static_cast<FlexibleBodyLinearExternalFFR*>(parent)->evalA();
    SqrMat3 G_bar = static_cast<FlexibleBodyLinearExternalFFR*>(parent)->evalG_bar();
    Vec3 u_bar = NLP->evalLocalPosition(frame->getZeta());
    Jacobian_trans.set(Index(0, 2), Index(3, 5), -A * tilde(u_bar) * G_bar);

    // elastic DOF
    Mat3xV modeShapeMatrix(qSize - 6, NONINIT);
    double positionU = frame->getEta();
    double positionV = frame->getXi();
    for (int k = 0; k < qSize - 6; k++) {
      Vec3 temp = surfaceModeShape.at(k).pointAt(positionU, positionV);
      modeShapeMatrix.set(k, temp);
    }

    Jacobian_trans.set(Index(0, 2), Index(6, qSize - 1), A * modeShapeMatrix);

    SqrMat3 wRA = (static_cast<FlexibleBodyLinearExternalFFR*>(parent))->getFrameOfReference()->evalOrientation();
    frame->setJacobianOfTranslation(wRA * Jacobian_trans,j);

    /******************************************************************  Jacobian of Rotation  *******************************************************************************/
    Mat3xV Jacobian_rot(qSize, INIT, 0.);
    Jacobian_rot.set(Index(0, 2), Index(3, 5), A * G_bar);
    frame->setJacobianOfRotation(wRA * Jacobian_rot,j);
  }

  int Contour2sNeutralLinearExternalFFR::getNumberOfTransNodesU() {
    return transNodes.rows(); //TODO: shouldn't it be the other way round?
  }

  int Contour2sNeutralLinearExternalFFR::getNumberOfTransNodesV() {
    return transNodes.cols(); //TODO: shouldn't it be the other way round?
  }

  void Contour2sNeutralLinearExternalFFR::setTransNodes(const MatVI & transNodes_) {
    transNodes.resize() = transNodes_;
  }


  MatVI Contour2sNeutralLinearExternalFFR::getTransNodes() {
    return transNodes;
  }

  void Contour2sNeutralLinearExternalFFR::setdegU(int deg) {
    degU = deg;
  }

  void Contour2sNeutralLinearExternalFFR::setdegV(int deg) {
    degV = deg;
  }

  void Contour2sNeutralLinearExternalFFR::setOpenStructure(bool openstructure_) {
    openStructure = openstructure_;
  }

  bool Contour2sNeutralLinearExternalFFR::getOpenStructure() {
    return openStructure;
  }

  void Contour2sNeutralLinearExternalFFR::readTransNodes(string file) {
    ifstream contourfile((file).c_str());
    if (!contourfile.is_open()) {
      THROW_MBSIMERROR("Can not open file " + file);
    }
    string s;
    getline(contourfile, s);

    setTransNodes(MatVI(s.c_str()));
  }

  NeutralNurbsVelocity2s* Contour2sNeutralLinearExternalFFR::createNeutralVelocity() {
    return new NeutralNurbsVelocity2s(parent, transNodes, nodeOffset, degU, degV, openStructure);
  }

  NeutralNurbsPosition2s* Contour2sNeutralLinearExternalFFR::createNeutralPosition() {
    return new NeutralNurbsPosition2s(parent, transNodes, nodeOffset, degU, degV, openStructure);
  }

  NeutralNurbsLocalPosition2s* Contour2sNeutralLinearExternalFFR::createNeutralLocalPosition() {
    return new NeutralNurbsLocalPosition2s(parent, transNodes, nodeOffset, degU, degV, openStructure);
  }

  void Contour2sNeutralLinearExternalFFR::resetUpToDate() {
    Contour2sNeutralFactory::resetUpToDate();
    NP->resetUpToDate();
    NLP->resetUpToDate();
    NV->resetUpToDate();
  }

} /* namespace MBSimFlexibleBody */
