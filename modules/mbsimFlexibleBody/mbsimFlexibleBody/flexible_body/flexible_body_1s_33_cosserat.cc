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
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_33_cosserat.h"
#include "mbsimFlexibleBody/frames/node_frame.h"
#include "mbsim/frames/fixed_contour_frame.h"
#include "mbsimFlexibleBody/utils/cardan.h"
#include "mbsim/dynamic_system_solver.h"
#include <mbsim/environment.h>
#include "mbsim/utils/eps.h"
#include "mbsim/utils/rotarymatrices.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  FlexibleBody1s33Cosserat::FlexibleBody1s33Cosserat(const string &name, bool openStructure_) :
      FlexibleBody1sCosserat(name, openStructure_), JTransInterp(false), I2(0.), I0(0.), R2(0.), cEps2D(0.) {
  }

  FlexibleBody1s33Cosserat::~FlexibleBody1s33Cosserat() {
    for (unsigned int i = 0; i < rotationDiscretization.size(); i++) {
      if (rotationDiscretization[i]) {
        delete rotationDiscretization[i];
        rotationDiscretization[i] = NULL;
      }
    }
  }

  void FlexibleBody1s33Cosserat::BuildElements(double t) {
    /* translational elements */
    for (int i = 0; i < Elements; i++) {
      int j = 6 * i; // start index in entire beam coordinates

      if (i < Elements - 1 || openStructure) {
        qElement[i] = q(j, j + 8);
        uElement[i] = u(j, j + 8);
      }
      else { // last FE-Beam for closed structure	
        qElement[i](0, 5) = q(j, j + 5);
        uElement[i](0, 5) = u(j, j + 5);
        qElement[i](6, 8) = q(0, 2);
        uElement[i](6, 8) = u(0, 2);
      }
    }

    /* rotational elements */
    if (openStructure)
      computeBoundaryCondition();

    for (int i = 0; i < rotationalElements; i++) {
      int j = 6 * i; // start index in entire beam coordinates

      if (i > 0 && i < rotationalElements - 1) { // no problem case
        qRotationElement[i] = q(j - 3, j + 5); // staggered grid -> rotation offset
        uRotationElement[i] = u(j - 3, j + 5);
      }
      else if (i == 0) { // first element
        if (openStructure) { // open structure
          qRotationElement[i](0, 2) = bound_ang_start;
          uRotationElement[i](0, 2) = bound_ang_vel_start;
          qRotationElement[i](3, 8) = q(j, j + 5);
          uRotationElement[i](3, 8) = u(j, j + 5);
        }
        else { // closed structure concerning gamma
          qRotationElement[i](0, 2) = q(q.size() - 3, q.size() - 1);
          uRotationElement[i](0, 2) = u(u.size() - 3, u.size() - 1);
          qRotationElement[i](3, 8) = q(j, j + 5);
          uRotationElement[i](3, 8) = u(j, j + 5);
          if (q(j + 5) < q(q.size() - 1))
            qRotationElement[i](2) -= 2. * M_PI;
          else
            qRotationElement[i](2) += 2. * M_PI;
        }
      }
      else if (i == rotationalElements - 1) { // last element
        if (openStructure) { // open structure
          qRotationElement[i](0, 5) = q(j - 3, j + 2);
          uRotationElement[i](0, 5) = u(j - 3, j + 2);
          qRotationElement[i](6, 8) = bound_ang_end;
          uRotationElement[i](6, 8) = bound_ang_vel_end;
        }
        else { // closed structure concerning gamma
          qRotationElement[i] = q(j - 3, j + 5);
          uRotationElement[i] = u(j - 3, j + 5);
        }
      }
    }
    updEle = false;
  }

  void FlexibleBody1s33Cosserat::GlobalVectorContribution(int n, const Vec& locVec, Vec& gloVec) {
    int j = 6 * n; // start index in entire beam coordinates

    if (n < Elements - 1 || openStructure) {
      gloVec(j, j + 8) += locVec;
    }
    else { // last FE for closed structure
      gloVec(j, j + 5) += locVec(0, 5);
      gloVec(0, 2) += locVec(6, 8);
    }
  }

  void FlexibleBody1s33Cosserat::GlobalMatrixContribution(int n, const Mat& locMat, Mat& gloMat) {
    int j = 6 * n; // start index in entire beam coordinates

    if (n < Elements - 1 || openStructure) {
      gloMat(Index(j, j + 8), Index(j, j + 8)) += locMat;
    }
    else { // last FE for closed structure
      gloMat(Index(j, j + 5), Index(j, j + 5)) += locMat(Index(0, 5), Index(0, 5));
      gloMat(Index(j, j + 5), Index(0, 2)) += locMat(Index(0, 5), Index(6, 8));
      gloMat(Index(0, 2), Index(j, j + 5)) += locMat(Index(6, 8), Index(0, 5));
      gloMat(Index(0, 2), Index(0, 2)) += locMat(Index(6, 8), Index(6, 8));
    }
  }

  void FlexibleBody1s33Cosserat::GlobalMatrixContribution(int n, const SymMat& locMat, SymMat& gloMat) {
    int j = 6 * n; // start index in entire beam coordinates

    if (n < Elements - 1 || openStructure) {
      gloMat(Index(j, j + 8)) += locMat;
    }
    else { // last FE for closed structure
      gloMat(Index(j, j + 5)) += locMat(Index(0, 5));
      gloMat(Index(j, j + 5), Index(0, 2)) += locMat(Index(0, 5), Index(6, 8));
      gloMat(Index(0, 2)) += locMat(Index(6, 8));
    }
  }

  void FlexibleBody1s33Cosserat::updatePositions(double t, Frame1s *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s33Cosserat::updatePositions): Not implemented.");
  }

  void FlexibleBody1s33Cosserat::updateVelocities(double t, Frame1s *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s33Cosserat::updateVelocities): Not implemented.");
  }

  void FlexibleBody1s33Cosserat::updateAccelerations(double t, Frame1s *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s33Cosserat::updateAccelerations): Not implemented.");
  }

  void FlexibleBody1s33Cosserat::updateJacobians(double t, Frame1s *frame, int j) {
    THROW_MBSIMERROR("(FlexibleBody1s33Cosserat::updateJacobians): Not implemented.");
  }

  void FlexibleBody1s33Cosserat::updateGyroscopicAccelerations(double t, Frame1s *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s33Cosserat::updateGyroscopicAccelerations): Not implemented.");
  }

  void FlexibleBody1s33Cosserat::updatePositions(double t, NodeFrame *frame) {
    Vec3 tmp(NONINIT);
    int node = frame->getNodeNumber();
    frame->setPosition(R->getPosition(t) + R->getOrientation(t) * q(6 * node + 0, 6 * node + 2));

    Vec3 angles = q(6 * node + 3, 6 * node + 5);


    frame->getOrientation(false).set(0, R->getOrientation() * angle->computet(angles));
    frame->getOrientation(false).set(1, R->getOrientation() * angle->computen(angles));
    frame->getOrientation(false).set(2, R->getOrientation() * angle->computeb(angles));
//    frame->setAngles(R->getOrientation(t) * angles);
  }

  void FlexibleBody1s33Cosserat::updateVelocities(double t, NodeFrame *frame) {
    Vec3 tmp(NONINIT);
    int node = frame->getNodeNumber();
    Vec3 angles = q(6 * node + 3, 6 * node + 5);
    Vec3 dotAngles = u(6 * node + 3, 6 * node + 5); //TODO
    frame->setVelocity(R->getOrientation(t) * u(6 * node + 0, 6 * node + 2));
    frame->setAngularVelocity(R->getOrientation() * angle->computeOmega(angles, dotAngles));
//    frame->setDerivativeOfAngles(R->getOrientation(t) * dotAngles);
 }

  void FlexibleBody1s33Cosserat::updateAccelerations(double t, NodeFrame *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s33Cosserat::updateAccelerations): Not implemented.");
  }

  void FlexibleBody1s33Cosserat::updateJacobians(double t, NodeFrame *frame, int j) {

    //Translational Node
    int node = frame->getNodeNumber();
    Mat3xV Jacobian_trans(qSize, INIT, 0.);

    Jacobian_trans.set(Index(0, 2), Index(6 * node, 6 * node + 2), SqrMat(3, EYE)); // translation

    frame->setJacobianOfTranslation(R->getOrientation(t) * Jacobian_trans, j);

    // Rotational Node
    // TODO: Is it necessary to separate in two functions?
    Mat3xV Jacobian_rot(qSize, INIT, 0.); // TODO open structure
    Vec p = q(6 * node + 3, 6 * node + 5);

    Jacobian_rot.set(Index(0, 2), Index(6 * node + 3, 6 * node + 5), angle->computeT(p)); // rotation

    frame->setJacobianOfRotation(R->getOrientation() * Jacobian_rot, j);
  }

  void FlexibleBody1s33Cosserat::updateGyroscopicAccelerations(double t, NodeFrame *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s33Cosserat::updateGyroscopicAccelerations): Not implemented.");
  }

  Vec3 FlexibleBody1s33Cosserat::getAngles(double t, int i) {
    return R->getOrientation(t) * q(6 * i + 3, 6 * i + 5);
  }

  Vec3 FlexibleBody1s33Cosserat::getDerivativeOfAngles(double t, int i) {
    return R->getOrientation(t) * u(6 * i + 3, 6 * i + 5);
  }

  void FlexibleBody1s33Cosserat::init(InitStage stage) {
    if (stage == preInit) {
      FlexibleBody1sCosserat::init(stage);
      l0 = L / Elements;
    }
    else if (stage == unknownStage) {
      FlexibleBody1sCosserat::init(stage);

      initialised = true;

      Vec g = R->getOrientation().T() * MBSimEnvironment::getInstance()->getAccelerationOfGravity();

      /* translational elements */
      for (int i = 0; i < Elements; i++) {
        discretization.push_back(new FiniteElement1s33CosseratTranslation(l0, rho, A, E, G, I1, I2, I0, g, angle));
        qElement.push_back(Vec(discretization[i]->getqSize(), INIT, 0.));
        uElement.push_back(Vec(discretization[i]->getuSize(), INIT, 0.));
        static_cast<FiniteElement1s33CosseratTranslation*>(discretization[i])->setMaterialDamping(Elements * cEps0D, cEps1D, cEps2D);
      }

      /* rotational elements */
      for (int i = 0; i < rotationalElements; i++) {
        rotationDiscretization.push_back(new FiniteElement1s33CosseratRotation(l0, E, G, I1, I2, I0, angle));
        qRotationElement.push_back(Vec(rotationDiscretization[i]->getqSize(), INIT, 0.));
        uRotationElement.push_back(Vec(rotationDiscretization[i]->getuSize(), INIT, 0.));
        if (fabs(R1) > epsroot() || fabs(R2) > epsroot())
          static_cast<FiniteElement1s33CosseratRotation*>(rotationDiscretization[i])->setCurlRadius(R1, R2);
      }

      initM();
    }
    else
      FlexibleBody1sCosserat::init(stage);

//curve->initContourFromBody(stage);
  }

  double FlexibleBody1s33Cosserat::computePotentialEnergy(double t) {
    /* translational elements */
    double V = FlexibleBody1sCosserat::computePotentialEnergy(t);

    /* rotational elements */
    for (unsigned int i = 0; i < rotationDiscretization.size(); i++) {
      V += rotationDiscretization[i]->computeElasticEnergy(getqRotationElement(t,i));
    }

    return V;
  }

  void FlexibleBody1s33Cosserat::updateLLM(double t, int k) {
//    FlexibleBody1sCosserat::updateLLM(t);
    getM(t,k); // be sure that M is update to date
    for (int i = 0; i < (int) discretization.size(); i++) {
      int j = 6 * i;
      LLM[k](Index(j + 3, j + 5)) = facLL(discretization[i]->getM()(Index(3, 5)));
    }
  }

  void FlexibleBody1s33Cosserat::setNumberElements(int n) {
    Elements = n;
    rotationalElements = n;
    if (openStructure) {
      qSize = 6 * n + 3;
      rotationalElements += 1;
    }
    else
      qSize = 6 * n;

    Vec q0Tmp;
    if (q0.size())
      q0Tmp = q0.copy();
    q0.resize(qSize, INIT, 0.);
    if (q0Tmp.size()) {
      if (q0Tmp.size() == q0.size())
        q0 = q0Tmp.copy();
      else
        THROW_MBSIMERROR("Dimension of q0 wrong!");
    }

    uSize[0] = qSize;
    uSize[1] = qSize; // TODO
    Vec u0Tmp;
    if (u0.size())
      u0Tmp = u0.copy();
    u0.resize(uSize[0], INIT, 0.);
    if (u0Tmp.size()) {
      if (u0Tmp.size() == u0.size())
        u0 = u0Tmp.copy();
      else
        THROW_MBSIMERROR("Dimension of u0 wrong !");
    }
  }

  fmatvec::Vector<Fixed<6>, double> FlexibleBody1s33Cosserat::getPositions(double sGlobal) {
//    double sLocal;
//    int currentElement;
//    BuildElement(sGlobal, sLocal, currentElement); // Lagrange parameter of affected FE
//    return static_cast<FiniteElement1s33RCM*>(discretization[currentElement])->getPositions(getqElement(currentElement), sLocal);

    THROW_MBSIMERROR("FlexibleBody1s33Cosserat::getPositions not implemented");
    fmatvec::Vector<Fixed<6>, double> temp(NONINIT);
//    ncc->updateKinematicsForFrame(cp, Frame::position);
//    ncc->updateKinematicsForFrame(cp, Frame::angle);
//    temp.set(Index(0, 2), cp.getFrameOfReference().getPosition());
//    temp.set(Index(3, 5), cp.getFrameOfReference().getAnglesOfOrientation());
    return temp;
  }

  fmatvec::Vector<Fixed<6>, double> FlexibleBody1s33Cosserat::getVelocities(double sGlobal) {
//    double sLocal;
//    int currentElement;
//    BuildElement(sGlobal, sLocal, currentElement); // Lagrange parameter of affected FE
//    return static_cast<FiniteElement1s33RCM*>(discretization[currentElement])->getVelocities(getqElement(currentElement), getuElement(currentElement), sLocal);

    THROW_MBSIMERROR("FlexibleBody1s33Cosserat::getVelocities not implemented");
    fmatvec::Vector<Fixed<6>, double> temp(NONINIT);
//    ncc->updateKinematicsForFrame(cp, Frame::position);
//    ncc->updateKinematicsForFrame(cp, Frame::angle);
//    ncc->updateKinematicsForFrame(cp, Frame::velocities);
//    temp.set(Index(6, 8), cp.getFrameOfReference().getVelocity());
//    temp.set(Index(9, 11), cp.getFrameOfReference().getAngularVelocity());
    return temp;
  }

  Vec3 FlexibleBody1s33Cosserat::computeAngles(double sGlobal, const Vec & vec) {
    Vec3 phiTmp, phi_L, phi_R;
    double sLocalRotation;
    int currentElementRotation; // TODO openstructure

    if (sGlobal < l0 / 2.) { // first rotation element (last half)
      sLocalRotation = sGlobal + l0 / 2.;
      phi_L = vec(vec.size() - 3, vec.size() - 1).copy();
      phi_R = vec(3, 5).copy();
      if (phi_L(2) < phi_R(2))
        phi_L(2) += 2. * M_PI;
      else
        phi_L(2) -= 2. * M_PI;
    }
    else if (sGlobal < L - l0 / 2.) {
      BuildElementTranslation(sGlobal + l0 / 2., sLocalRotation, currentElementRotation); // Lagrange parameter and number of rotational element (+l0/2)
      phi_L = vec(6 * currentElementRotation - 3, 6 * currentElementRotation - 1).copy();
      phi_R = vec(6 * currentElementRotation + 3, 6 * currentElementRotation + 5).copy();
    }
    else { // first rotation element (first half)
      sLocalRotation = sGlobal - (L - l0 / 2.);
      phi_L = vec(vec.size() - 3, vec.size() - 1).copy();
      phi_R = vec(3, 5).copy();
      if (phi_L(2) < phi_R(2))
        phi_L(2) += 2. * M_PI;
      else
        phi_L(2) -= 2. * M_PI;
    }
    phiTmp = phi_L + sLocalRotation / l0 * (phi_R - phi_L);

    return phiTmp;
  }

  void FlexibleBody1s33Cosserat::initInfo() {
    FlexibleBody1sCosserat::init(preInit);
    FlexibleBody1sCosserat::init(unknownStage);
    l0 = L / Elements;
    Vec g = Vec("[0.;0.;0.]");

    /* translational elements */
    for (int i = 0; i < Elements; i++) {
      discretization.push_back(new FiniteElement1s33CosseratTranslation(l0, rho, A, E, G, I1, I2, I0, g, angle));
      qElement.push_back(Vec(discretization[0]->getqSize(), INIT, 0.));
      uElement.push_back(Vec(discretization[0]->getuSize(), INIT, 0.));
    }

    /* rotational elements */
    for (int i = 0; i < rotationalElements; i++) {
      rotationDiscretization.push_back(new FiniteElement1s33CosseratRotation(l0, E, G, I1, I2, I0, angle));
      qRotationElement.push_back(Vec(rotationDiscretization[0]->getqSize(), INIT, 0.));
      uRotationElement.push_back(Vec(rotationDiscretization[0]->getuSize(), INIT, 0.));
    }

//curve->initContourFromBody(resize);
  }

  void FlexibleBody1s33Cosserat::BuildElementTranslation(const double& sGlobal, double& sLocal, int& currentElementTranslation) {
    double remainder = fmod(sGlobal, L);
    if (openStructure && sGlobal >= L)
      remainder += L; // remainder \in (-eps,L+eps)
    if (!openStructure && sGlobal < 0.)
      remainder += L; // remainder \in [0,L)

    currentElementTranslation = int(remainder / l0);
    sLocal = remainder - (currentElementTranslation) * l0; // Lagrange-Parameter of the affected FE with sLocal==0 and sGlobal==0 at the beginning of the beam

    assert(sLocal > -1e-8);
    assert(sLocal < l0 + 1e-8);

    if (currentElementTranslation >= Elements) { // contact solver computes to large sGlobal at the end of the entire beam (in open and closed structure)
      currentElementTranslation = Elements - 1;
      sLocal += l0;
    }
  }

  void FlexibleBody1s33Cosserat::initM() {
    for (int i = 0; i < (int) discretization.size(); i++)
      static_cast<FiniteElement1s33CosseratTranslation*>(discretization[i])->initM(); // compute attributes of finite element
    for (int i = 0; i < (int) discretization.size(); i++)
      GlobalMatrixContribution(i, discretization[i]->getM(), M[0]); // assemble
    for (int i = 0; i < (int) discretization.size(); i++) {
      int j = 6 * i;
      LLM[0](Index(j, j + 2)) = facLL(M[0](Index(j, j + 2)));
      if (openStructure && i == (int) discretization.size() - 1)
        LLM[0](Index(j + 6, j + 8)) = facLL(M[0](Index(j + 6, j + 8)));
    }
  }

  void FlexibleBody1s33Cosserat::computeBoundaryCondition() {
// TODO
  }

  void FlexibleBody1s33Cosserat::GlobalVectorContributionRotation(int n, const Vec& locVec, Vec& gloVec) {
    int j = 6 * n; // start index in entire beam coordinates
    if (n > 0 && n < rotationalElements - 1) { // no problem case
      gloVec(j - 3, j + 5) += locVec; // staggered grid -> rotation offset
    }
    else if (n == 0) { // first element
      if (openStructure) { // open structure
        gloVec(j, j + 5) += locVec(3, 8);
        gloVec(j + 3, j + 5) += locVec(0, 2); // TODO depends on computeBoundaryConditions()
      }
      else { // closed structure 
        gloVec(j, j + 5) += locVec(3, 8);
        gloVec(q.size() - 3, q.size() - 1) += locVec(0, 2);
      }
    }
    else if (n == rotationalElements - 1) { // last element
      if (openStructure) { // open structure
        gloVec(j - 3, j + 2) += locVec(0, 5);
        gloVec(j - 3, j - 1) += locVec(6, 8); // TODO depends on computeBoundaryConditions()
      }
      else { // closed structure
        gloVec(j - 3, j + 5) += locVec;
      }
    }
  }

  void FlexibleBody1s33Cosserat::exportPositionVelocity(const string & filenamePos, const string & filenameVel /*= string( )*/, const int & deg /* = 3*/, const bool &writePsFile /*= false*/) {
    THROW_MBSIMERROR("To be adapted to new internal nurbs ...");

//    PlNurbsCurved curvePos;
//    PlNurbsCurved curveVel;
//
//    if (!openStructure) {
//      PLib::Vector<PLib::HPoint3Dd> NodelistPos(Elements + deg);
//      PLib::Vector<PLib::HPoint3Dd> NodelistVel(Elements + deg);
//
//      for (int i = 0; i < Elements + deg; i++) {  // +deg-Elements are needed, as the curve is closed
//        ContourPointData cp(i);
//        if (i >= Elements)
//          cp.getNodeNumber() = i - Elements;
//
//        updateKinematicsForFrame(cp, position);
//        NodelistPos[i] = HPoint3Dd(cp.getFrameOfReference().getPosition()(0), cp.getFrameOfReference().getPosition()(1), cp.getFrameOfReference().getPosition()(2), 1);
//
//        if (not filenameVel.empty()) {
//          updateKinematicsForFrame(cp, velocity_cosy);
//
//          SqrMat3 TMPMat = cp.getFrameOfReference().getOrientation();
//          SqrMat3 AKI(INIT, 0.);
//          AKI.set(0, trans(TMPMat.col(1)));
//          AKI.set(1, trans(TMPMat.col(0)));
//          AKI.set(2, trans(TMPMat.col(2)));
//          Vec3 Vel(INIT, 0.);
//          Vel = AKI * cp.getFrameOfReference().getVelocity();
//
//          NodelistVel[i] = HPoint3Dd(Vel(0), Vel(1), Vel(2), 1);
//        }
//      }
//
//      /*create own uVec and uvec like in nurbsdisk_2s*/
//      PLib::Vector<double> uvec = PLib::Vector<double>(Elements + deg);
//      PLib::Vector<double> uVec = PLib::Vector<double>(Elements + deg + deg + 1);
//
//      const double stepU = L / Elements;
//
//      uvec[0] = 0;
//      for (int i = 1; i < uvec.size(); i++) {
//        uvec[i] = uvec[i - 1] + stepU;
//      }
//
//      uVec[0] = (-deg) * stepU;
//      for (int i = 1; i < uVec.size(); i++) {
//        uVec[i] = uVec[i - 1] + stepU;
//      }
//
//      curvePos.globalInterpClosedH(NodelistPos, uvec, uVec, deg);
//      curvePos.write(filenamePos.c_str());
//
//      if (writePsFile) {
//        string psfile = filenamePos + ".ps";
//
//        cout << curvePos.writePS(psfile.c_str(), 0, 2.0, 5, false) << endl;
//      }
//
//      if (not filenameVel.empty()) {
//        curveVel.globalInterpClosedH(NodelistVel, uvec, uVec, deg);
//        curveVel.write(filenameVel.c_str());
//      }
//    }
  }

  void FlexibleBody1s33Cosserat::importPositionVelocity(const string & filenamePos, const string & filenameVel /* = string( )*/) {
    THROW_MBSIMERROR("To be adapted to new internal nurbs ...");

//    int DEBUGLEVEL = 0;
//
//    PlNurbsCurved curvePos;
//    PlNurbsCurved curveVel;
//    curvePos.read(filenamePos.c_str());
//    if (not filenameVel.empty())
//      curveVel.read(filenameVel.c_str());
//
//    l0 = L / Elements;
//    Vec q0Dummy(q0.size(), INIT, 0.);
//    Vec u0Dummy(u0.size(), INIT, 0.);
//    Point3Dd refBinHalf;
//
//    for (int i = 0; i < Elements; i++) {
//      Point3Dd posStart, tangHalf, norHalf, binHalf;
//      posStart = curvePos.pointAt(i * l0);
//      tangHalf = curvePos.derive3D(i * l0 + l0 / 2., 1);
//      tangHalf /= norm(tangHalf);
//
//      if (i < 1) {
//        norHalf = curvePos.derive3D(i * l0 + l0 / 2., 2); // at START!!
//        norHalf /= norm(norHalf);
//        binHalf = crossProduct(norHalf, tangHalf);
//        norHalf = crossProduct(binHalf, tangHalf);
//        refBinHalf = binHalf; // set only in first element
//      }
//      else {
//        binHalf = refBinHalf;
//        norHalf = crossProduct(binHalf, tangHalf);
//        binHalf = crossProduct(tangHalf, norHalf);
//      }
//
//      q0Dummy(i * 6) = posStart.x(); // x
//      q0Dummy(i * 6 + 1) = posStart.y(); // y
//      q0Dummy(i * 6 + 2) = posStart.z(); // z
//
//      SqrMat AIK(3, INIT, 0.);
//      AIK(0, 0) = tangHalf.x();
//      AIK(1, 0) = tangHalf.y();
//      AIK(2, 0) = tangHalf.z();
//      AIK(0, 1) = norHalf.x();
//      AIK(1, 1) = norHalf.y();
//      AIK(2, 1) = norHalf.z();
//      AIK(0, 2) = binHalf.x();
//      AIK(1, 2) = binHalf.y();
//      AIK(2, 2) = binHalf.z();
//      Vec AlphaBetaGamma = AIK2Cardan(AIK);
//      q0Dummy(i * 6 + 3) = AlphaBetaGamma(0);
//      q0Dummy(i * 6 + 4) = AlphaBetaGamma(1);
//      q0Dummy(i * 6 + 5) = AlphaBetaGamma(2);
//
//      if (not filenameVel.empty()) {
//        Point3Dd velStart = curveVel.pointAt(i * l0);
//
//        Vec velK(3, INIT, 0.);
//        velK(0) = velStart.x();
//        velK(1) = velStart.y();
//        velK(2) = velStart.z();
//        Vec velI = trans(R->getOrientation()) * AIK * velK; // TODO AIK now from staggered nodes
//
//        u0Dummy(i * 6) = velI(0);
//        u0Dummy(i * 6 + 1) = velI(1);
//        u0Dummy(i * 6 + 2) = velI(2);
//      }
//
//      if (DEBUGLEVEL == 1) {
//        cout << "START(" << i + 1 << ",1:end) = [" << posStart << "];" << endl;
//        cout << "Tangent(" << i + 1 << ",1:end) = [" << tangHalf << "];" << endl;
//        cout << "Normal(" << i + 1 << ",1:end) = [" << norHalf << "];" << endl;
//        cout << "Binormal(" << i + 1 << ",1:end) = [" << binHalf << "];" << endl;
//        cout << "%----------------------------------" << endl;
//        cout << "alpha_New(" << i + 1 << ") = " << q0Dummy(i * 6 + 3) << ";" << endl;
//        cout << "beta_New(" << i + 1 << ") = " << q0Dummy(i * 6 + 4) << ";" << endl;
//        cout << "gamma_New(" << i + 1 << ") = " << q0Dummy(i * 6 + 5) << ";" << endl;
//        cout << "%----------------------------------" << endl;
//      }
//    }
//    setq0(q0Dummy);
//    if (not filenameVel.empty())
//      setu0(u0Dummy);

  }
}
