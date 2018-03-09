/*
 * neutral_contour_1s_cosserat.cc
 *
 *  Created on: 25.10.2013
 *      Author: zwang
 */
#include <config.h>
#include "contour_1s_neutral_cosserat.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsimFlexibleBody/utils/cardan.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_33_cosserat.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_21_cosserat.h"
#include "mbsimFlexibleBody/utils/contact_utils.h"

using namespace MBSim;
using namespace fmatvec;

namespace MBSimFlexibleBody {

  Contour1sNeutralCosserat::Contour1sNeutralCosserat(const std::string &name_) :
      Contour1sNeutralFactory(name_), transNodes(0), rotNodes(0), nodeOffset(0), ANGLE(new Cardan()), NP(NULL), NV(NULL), NA(NULL), NDA(NULL) {

  }
  
  Contour1sNeutralCosserat::~Contour1sNeutralCosserat() {
    delete NP;
    delete NV;
    delete NA;
    delete NDA;
    NP = NULL;
    NV = NULL;
    NA = NULL;
    NDA = NULL;

  }

  NeutralNurbsPosition1s* Contour1sNeutralCosserat::createNeutralPosition() {
    return new NeutralNurbsPosition1s(parent, transNodes, nodeOffset, uMin, uMax, degU, openStructure);
  }

  NeutralNurbsVelocity1s* Contour1sNeutralCosserat::createNeutralVelocity() {
    return new NeutralNurbsVelocity1s(parent, transNodes, nodeOffset, uMin, uMax, degU, openStructure);
  }

  NeutralNurbsAngle1s* Contour1sNeutralCosserat::createNeutralAngle() {
    return new NeutralNurbsAngle1s(parent, rotNodes, nodeOffset, uMin, uMax, degU, openStructure);
  }

  NeutralNurbsDotangle1s* Contour1sNeutralCosserat::createNeutralDotangle() {
    return new NeutralNurbsDotangle1s(parent, rotNodes, nodeOffset, uMin, uMax, degU, openStructure);
  }

  void Contour1sNeutralCosserat::init(InitStage stage, const InitConfigSet &config) {

    if (stage == preInit) {
      NP = createNeutralPosition();
      NP->setBinormalDir(-R->evalOrientation().col(2));
      NV = createNeutralVelocity();
      NA = createNeutralAngle();
      NDA = createNeutralDotangle();

      Contour1sNeutralFactory::init(stage, config);
    }
    else if(stage == unknownStage) {
      NP->computeCurve(false); // the first time call the computeCurveVelocity, the flag should be false
      NV->computeCurve(false);
      NA->computeCurve(false);
      NDA->computeCurve(false);

      resetUpToDate();

      Vec u(NV->getuVec());
      for (int i = 0; i < u.size() - degU; i++)
        etaNodes.push_back(u(i));
      etaNodes.push_back(uMax);

      Contour1sNeutralFactory::init(stage, config);
    }

    Contour1sNeutralFactory::init(stage, config);
  }

  Vec3 Contour1sNeutralCosserat::evalPosition(const Vec2 &zeta) {
    return NP->evalPosition(zeta(0));
  }

  Vec3 Contour1sNeutralCosserat::evalWs(const Vec2 &zeta) {
    return NP->evalWs(zeta(0));
  }

  Vec3 Contour1sNeutralCosserat::evalWt(const Vec2 &zeta) {
    return NP->evalWt(zeta(0));
  }

  void Contour1sNeutralCosserat::updatePositions(ContourFrame *frame) {
    NP->update(frame);
    NP->updatePositionNormal(frame);
    NP->updatePositionFirstTangent(frame);
    NP->updatePositionSecondTangent(frame);
  }

  void Contour1sNeutralCosserat::updateVelocities(ContourFrame *frame) {
    NV->update(frame);
  }

  void Contour1sNeutralCosserat::updateJacobians(ContourFrame *frame, int j) {
    int qSize = (static_cast<FlexibleBody1sCosserat*>(parent))->getqSizeFull();

    /******************************************************************  Jacobian of Translation  *******************************************************************************/
    double sGlobal = frame->getEta(); // interpolation of Jacobian of Rotation starts from 0 --> \phi_{1/2} but Jacobian of Translation and contour starts from 0 --> r_0 therefore this difference of l0/2
    double sLocalTrans_bodySpace;
    int currentElementTrans;
    // transform form the contour space(uMin - uMax) to the FlexibleBody1sCosserat space (0 - L)
    double L = static_cast<FlexibleBody1sCosserat*>(parent)->getLength();
    double l0 = L / static_cast<FlexibleBody1sCosserat*>(parent)->getNumberElements();

    double sGlobal_bodySpace = (sGlobal - uMin) / (uMax - uMin) * L;
    static_cast<FlexibleBody1sCosserat*>(parent)->BuildElementTranslation(sGlobal_bodySpace, sLocalTrans_bodySpace, currentElementTrans); // Lagrange parameter and number of translational element
    double weightLeft = 1 - sLocalTrans_bodySpace / l0;
    double weightRight = 1 - weightLeft;

    Mat3xV Jacobian_trans(qSize, INIT, 0.);

    if (typeid(*parent) == typeid(FlexibleBody1s33Cosserat)) {
      // distribute the E to the nearby nodes
      SqrMat Jacobian_trans_total(3, EYE);
      // TODO: only for 1s closed structue, assuming numOfElements = numOfTransNodes
      if (currentElementTrans == transNodes.size() - 1) { // the last translation element
        Jacobian_trans.set(RangeV(0, 2), RangeV(6 * currentElementTrans, 6 * currentElementTrans + 2), weightLeft * Jacobian_trans_total);
        Jacobian_trans.set(RangeV(0, 2), RangeV(0, 2), weightRight * Jacobian_trans_total);
      }
      else {
        Jacobian_trans.set(RangeV(0, 2), RangeV(6 * currentElementTrans, 6 * currentElementTrans + 2), weightLeft * Jacobian_trans_total);
        Jacobian_trans.set(RangeV(0, 2), RangeV(6 * currentElementTrans + 6, 6 * currentElementTrans + 8), weightRight * Jacobian_trans_total);
      }
    }
    else if (typeid(*parent) == typeid(FlexibleBody1s21Cosserat)) {
      // distribute the E to the nearby nodes
      SqrMat Jacobian_trans_total(2, EYE);
      // TODO: only for 1s closed structue, assuming numOfElements = numOfTransNodes
      if (currentElementTrans == transNodes.size() - 1) { // the last translation element
        Jacobian_trans.set(RangeV(0, 1), RangeV(3 * currentElementTrans, 3 * currentElementTrans + 1), weightLeft * Jacobian_trans_total);
        Jacobian_trans.set(RangeV(0, 1), RangeV(0, 1), weightRight * Jacobian_trans_total);
      }
      else {
        Jacobian_trans.set(RangeV(0, 1), RangeV(3 * currentElementTrans, 3 * currentElementTrans + 1), weightLeft * Jacobian_trans_total);
        Jacobian_trans.set(RangeV(0, 1), RangeV(3 * currentElementTrans + 3, 3 * currentElementTrans + 4), weightRight * Jacobian_trans_total);
      }
    }

    frame->setJacobianOfTranslation(static_cast<FlexibleBody1sCosserat*>(parent)->transformJacobian(Jacobian_trans),j);

    /******************************************************************  Jacobian of Rotation  *******************************************************************************/
    // calculate the local position
    double sLocalRot_bodySpace;
    int currentElementRot;
    // linearly distribute the interpolated total Jacobian to the nearby nodes
    // here transformation from sGlobal to rotational element starting point is done by sGlobal+nodeOffset in oder to use the BuildElementTranslation()
    static_cast<FlexibleBody1s33Cosserat*>(parent)->BuildElementTranslation(sGlobal_bodySpace + l0 * 0.5, sLocalRot_bodySpace, currentElementRot);    // Lagrange parameter and number of translational element
    weightLeft = 1 - sLocalRot_bodySpace / l0;
    weightRight = 1 - weightLeft;

    // TODO: index are different for open structure
    Mat3xV Jacobian_rot(qSize, INIT, 0.);
    if (typeid(*parent) == typeid(FlexibleBody1s33Cosserat)) {
      Vec3 angles = NA->calculateStaggeredAngle(sGlobal);
      SqrMat3 Jacobian_rot_total = ANGLE->computeT(angles);

//      msg(Debug) << "angles: " << angles << endl;
//      msg(Debug) << "Jacobian_rot_total:  " << Jacobian_rot_total << endl << endl;
      if (currentElementRot == 0) { // the first rotational element
        Jacobian_rot.set(RangeV(0, 2), RangeV(qSize - 3, qSize - 1), weightLeft * Jacobian_rot_total); //TODO:  all of these are different for 1D and 3D case, maybe we need diffenent stucutre for 1D and 3D;
        Jacobian_rot.set(RangeV(0, 2), RangeV(6 * currentElementRot + 3, 6 * currentElementRot + 5), weightRight * Jacobian_rot_total);
      }
      else {
        Jacobian_rot.set(RangeV(0, 2), RangeV(6 * currentElementRot - 3, 6 * currentElementRot - 1), weightLeft * Jacobian_rot_total);  //TODO:  all of these are different for 1D and 3D case, maybe we need diffenent stucutre for 1D and 3D;
        Jacobian_rot.set(RangeV(0, 2), RangeV(6 * currentElementRot + 3, 6 * currentElementRot + 5), weightRight * Jacobian_rot_total);
      }
    }
    else if (typeid(*parent) == typeid(FlexibleBody1s21Cosserat)) {
      // TODO: only for 1s closed structue, assuming numOfElements = numOfTransNodes
      if (currentElementRot == 0) { // the first rotational element
        Jacobian_rot(2, qSize - 1) = weightLeft;
        Jacobian_rot(2, 3 * currentElementRot + 2) = weightRight;
      }
      else {
        Jacobian_rot(2, 3 * currentElementRot - 1) = weightLeft;
        Jacobian_rot(2, 3 * currentElementRot + 2) = weightRight;
      }
    }

    frame->setJacobianOfRotation(static_cast<FlexibleBody1sCosserat*>(parent)->transformJacobian(Jacobian_rot),j);
  }

  MBSim::ContactKinematics * Contour1sNeutralCosserat::findContactPairingWith(const std::type_info &type0, const std::type_info &type1) {
    return findContactPairingFlexible(type0, type1);
  }

  void Contour1sNeutralCosserat::setTransNodes(const fmatvec::VecInt & transNodes_) {
    transNodes.resize() = transNodes_;
  }

  void Contour1sNeutralCosserat::setRotNodes(const fmatvec::VecInt & rotNodes_) {
    rotNodes.resize() = rotNodes_;
  }

  void Contour1sNeutralCosserat::setNodeOffest(const double nodeOffset_) {
    nodeOffset = nodeOffset_;
  }

  void Contour1sNeutralCosserat::resetUpToDate() {
    Contour1sNeutralFactory::resetUpToDate();
    NP->resetUpToDate();
    NV->resetUpToDate();
    NA->resetUpToDate();
    NDA->resetUpToDate();
  }

}
/* namespace MBSimFlexibleBody */
