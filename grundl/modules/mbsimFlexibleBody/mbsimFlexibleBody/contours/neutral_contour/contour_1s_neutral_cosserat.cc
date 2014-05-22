/*
 * neutral_contour_1s_cosserat.cc
 *
 *  Created on: 25.10.2013
 *      Author: zwang
 */
#include <config.h>
#include "contour_1s_neutral_cosserat.h"
#include "mbsimFlexibleBody/utils/cardan.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_33_cosserat.h"
#include "mbsimFlexibleBody/utils/contact_utils.h"

using namespace MBSim;


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

  void Contour1sNeutralCosserat::init(MBSim::InitStage stage) {

    if (stage == preInit) {
      NP = createNeutralPosition();
      NV = createNeutralVelocity();
      NA = createNeutralAngle();
      NDA = createNeutralDotangle();

      NP->computeCurve(false); // the first time call the computeCurveVelocity, the flag should be false
      NV->computeCurve(false);
      NA->computeCurve(false);
      NDA->computeCurve(false);

      Vec u(NV->getuVec());
      for (int i = 0; i < u.size() - degU; i++)
        nodes.push_back(u(i));
      nodes.push_back(uMax);
    }
    else if (stage == resize) {
      // construct contourPoint for translation nodes
      nodes.reserve(transNodes.size() + 1);

//        nodeOffset = (static_cast<FlexibleBodyContinuum<double>*>(parent))->getNodeOffset();  // TODO change to be user set value

    }
    else if (stage == worldFrameContourLocation) {
      R->getOrientation() = (static_cast<FlexibleBody1sCosserat*>(parent))->getFrameOfReference()->getOrientation();
      R->getPosition() = (static_cast<FlexibleBody1sCosserat*>(parent))->getFrameOfReference()->getPosition();
    }

    Contour1sNeutralFactory::init(stage);
  }

  void Contour1sNeutralCosserat::updateKinematicsForFrame(MBSim::ContourPointData &cp, MBSim::FrameFeature ff) {
    if (ff == position || ff == position_cosy || ff == all)
      NP->update(cp);
    if (ff == velocity || ff == velocity_cosy || ff == velocities || ff == velocities_cosy || ff == all)
      NV->update(cp);
    if (ff == angle || ff == all)
      NA->update(cp); //TODO: use NP here also?
    if (ff == angularVelocity || ff == velocities || ff == velocities_cosy || ff == all) {
      // G(angle) * dotAngle
      //TODO: use NP here also??
      NA->update(cp); // staggered stuff is handled inside the updateAngle()
      NDA->update(cp);
      Vec3 angles = cp.getFrameOfReference().getAnglesOfOrientation();
      Vec3 dotAngles = cp.getFrameOfReference().getDotAnglesOfOrientation();
      cp.getFrameOfReference().setAngularVelocity(ANGLE->computeOmega(angles, dotAngles));
    }
    if (ff == normal || ff == cosy || ff == position_cosy || ff == velocity_cosy || ff == velocities_cosy || ff == all) {
      //TODO: using the position information of the angle curve seems not to work!
//      NA->updateAngleNormal(cp); // normal
      NP->updatePositionNormal(cp);
    }
    if (ff == firstTangent || ff == cosy || ff == position_cosy || ff == velocity_cosy || ff == velocities_cosy || ff == all) {
      //TODO: using the position information of the angle curve seems not to work!
//      NA->updateAngleFirstTangent(cp); // tangent
      NP->updatePositionFirstTangent(cp);
    }
    if (ff == secondTangent || ff == cosy || ff == position_cosy || ff == velocity_cosy || ff == velocities_cosy || ff == all) {
      //TODO: using the position information of the angle curve seems not to work!
//      NA->updateAngleSecondTangent(cp); // binormal (cartesian system)
      NP->updatePositionSecondTangent(cp);
    }
  }

  void Contour1sNeutralCosserat::updateJacobiansForFrame(MBSim::ContourPointData &cp, int j) {

    int qSize = (static_cast<FlexibleBody1sCosserat*>(parent))->getqSize();

    /******************************************************************  Jacobian of Translation  *******************************************************************************/
    double sGlobal = cp.getLagrangeParameterPosition()(0); // interpolation of Jacobian of Rotation starts from 0 --> \phi_{1/2} but Jacobian of Translation and contour starts from 0 --> r_0 therefore this difference of l0/2
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

    if (parent->getType() == "FlexibleBody1s33Cosserat") {
      // distribute the E to the nearby nodes
      SqrMat Jacobian_trans_total(3, EYE);
      // TODO: only for 1s closed structue, assuming numOfElements = numOfTransNodes
      if (currentElementTrans == transNodes.size() - 1) { // the last translation element
        Jacobian_trans.set(Index(0, 2), Index(6 * currentElementTrans, 6 * currentElementTrans + 2), weightLeft * Jacobian_trans_total);
        Jacobian_trans.set(Index(0, 2), Index(0, 2), weightRight * Jacobian_trans_total);
      }
      else {
        Jacobian_trans.set(Index(0, 2), Index(6 * currentElementTrans, 6 * currentElementTrans + 2), weightLeft * Jacobian_trans_total);
        Jacobian_trans.set(Index(0, 2), Index(6 * currentElementTrans + 6, 6 * currentElementTrans + 8), weightRight * Jacobian_trans_total);
      }
    }
    else if (parent->getType() == "FlexibleBody1s21Cosserat") {
      // distribute the E to the nearby nodes
      SqrMat Jacobian_trans_total(2, EYE);
      // TODO: only for 1s closed structue, assuming numOfElements = numOfTransNodes
      if (currentElementTrans == transNodes.size() - 1) { // the last translation element
        Jacobian_trans.set(Index(0, 1), Index(3 * currentElementTrans, 3 * currentElementTrans + 1), weightLeft * Jacobian_trans_total);
        Jacobian_trans.set(Index(0, 1), Index(0, 1), weightRight * Jacobian_trans_total);
      }
      else {
        Jacobian_trans.set(Index(0, 1), Index(3 * currentElementTrans, 3 * currentElementTrans + 1), weightLeft * Jacobian_trans_total);
        Jacobian_trans.set(Index(0, 1), Index(3 * currentElementTrans + 3, 3 * currentElementTrans + 4), weightRight * Jacobian_trans_total);
      }
    }

    cp.getFrameOfReference().setJacobianOfTranslation(Jacobian_trans);

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
    if (parent->getType() == "FlexibleBody1s33Cosserat") {
      NA->update(cp);  // transformation from sGlobal starting point to angle interplation stating point is done inside:  sGlobal-nodeOffset
      Vec3 angles = cp.getFrameOfReference().getAnglesOfOrientation();
      SqrMat3 Jacobian_rot_total = ANGLE->computeT(angles);

//      cout << "angles: " << angles << endl;
//      cout << "Jacobian_rot_total:  " << Jacobian_rot_total << endl << endl;
      if (currentElementRot == 0) { // the first rotational element
        Jacobian_rot.set(Index(0, 2), Index(qSize - 3, qSize - 1), weightLeft * Jacobian_rot_total); //TODO:  all of these are different for 1D and 3D case, maybe we need diffenent stucutre for 1D and 3D;
        Jacobian_rot.set(Index(0, 2), Index(6 * currentElementRot + 3, 6 * currentElementRot + 5), weightRight * Jacobian_rot_total);
      }
      else {
        Jacobian_rot.set(Index(0, 2), Index(6 * currentElementRot - 3, 6 * currentElementRot - 1), weightLeft * Jacobian_rot_total);  //TODO:  all of these are different for 1D and 3D case, maybe we need diffenent stucutre for 1D and 3D;
        Jacobian_rot.set(Index(0, 2), Index(6 * currentElementRot + 3, 6 * currentElementRot + 5), weightRight * Jacobian_rot_total);
      }
    }
    else if (parent->getType() == "FlexibleBody1s21Cosserat") {
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

    cp.getFrameOfReference().setJacobianOfRotation(Jacobian_rot);   // TODO: simplify computeT(), check which is to be used G() or A()*Gbar()

  }
  
  MBSim::ContactKinematics * Contour1sNeutralCosserat::findContactPairingWith(std::string type0, std::string type1) {
    return findContactPairingFlexible(type0.c_str(), type1.c_str());
  }

  void Contour1sNeutralCosserat::updateStateDependentVariables(double t) {
    // TODO: is this function called by the integrator at timestep???
    NP->computeCurve(true);   // the first time call the computeCurveVelocity, the flag should be false
    NV->computeCurve(true);
    NA->computeCurve(true);
    NDA->computeCurve(true);
    Contour1sNeutralFactory::updateStateDependentVariables(t);

    // debug

//    cout << "****************Velocity****************" << endl;
//    double num = 26;
//    Mat TestVelocity(num, 3, INIT);
//    Mat TestPosition(num, 3, INIT);
//    Mat TestAngle   (num, 3, INIT);
//    Mat TestDotAngle(num, 3, INIT);
//
//
//    for (int i = 0; i < num; i++ ) {
//      VecV alpha(3, INIT, 0);
//      alpha(0) = i * (uMax - uMin)/(num -1);
//      ContourPointData cpTest(alpha);
////      cout << "alpha: " << alpha << endl;
//      //cout << "position:" << cpTest.getLagrangeParameterPosition() << endl;
//      NP->update(cpTest);
//      NV->update(cpTest);
//      NA->update(cpTest);
//      NDA->update(cpTest);
//      Vec p = cpTest.getFrameOfReference().getPosition();
//      Vec v = cpTest.getFrameOfReference().getVelocity();
//      Vec a = cpTest.getFrameOfReference().getAnglesOfOrientation();
//      Vec da = cpTest.getFrameOfReference().getDotAnglesOfOrientation();
//      //cout << i << " : " << a << endl << endl;
//      TestPosition(Index(i,i), Index(0,2)) = p.T();
//      TestVelocity(Index(i,i), Index(0,2)) = v.T();
//      TestAngle   (Index(i,i), Index(0,2)) = a.T();
//      TestDotAngle(Index(i,i), Index(0,2)) = da.T();
//    }
//
//    cout << "positionCurve" << TestPosition << endl << endl;
//    cout << "velocityCurve" << TestVelocity << endl << endl;
//    cout << "angleCurve"    << TestAngle    << endl << endl;
//    cout << "dotAngleCurve" << TestDotAngle << endl << endl;

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

}
/* namespace MBSimFlexibleBody */
