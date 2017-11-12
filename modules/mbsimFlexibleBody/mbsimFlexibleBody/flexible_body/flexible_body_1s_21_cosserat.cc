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
 * Contact: thomas.cebulla@mytum.de
 */

#include<config.h>
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_21_cosserat.h"
#include "mbsimFlexibleBody/frames/node_frame.h"
#include "mbsim/frames/fixed_contour_frame.h"
#include "mbsimFlexibleBody/contours/nurbs_curve_1s.h"
#include "mbsimFlexibleBody/utils/cardan.h"
#include <mbsim/environment.h>
#include "mbsim/utils/eps.h"
#include "mbsim/utils/rotarymatrices.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace H5;

namespace MBSimFlexibleBody {

  FlexibleBody1s21Cosserat::FlexibleBody1s21Cosserat(const string &name, bool openStructure_) :
      FlexibleBody1sCosserat(name, openStructure_), JInterp(false), PODreduced(false), U(), qFull(), uFull(), hFull() {
  }

  FlexibleBody1s21Cosserat::~FlexibleBody1s21Cosserat() {
    for (unsigned int i = 0; i < rotationDiscretization.size(); i++) {
      if (rotationDiscretization[i]) {
        delete rotationDiscretization[i];
        rotationDiscretization[i] = NULL;
      }
    }
  }

  void FlexibleBody1s21Cosserat::BuildElements() {
    if (PODreduced) {
      qFull << U * q;
      uFull << U * u;
    }
    else {
      qFull >> q;
      uFull >> u;
    }
    /* translational elements */
    for (int i = 0; i < Elements; i++) {
      int j = 3 * i; // start index in entire beam coordinates

      if (i < Elements - 1) {
        qElement[i] = qFull(j, j + 4);
        uElement[i] = uFull(j, j + 4);
      }
      else { // last FE-Beam for closed structure	
        qElement[i](0, 2) = qFull(j, j + 2);
        uElement[i](0, 2) = uFull(j, j + 2);
        qElement[i](3, 4) = qFull(0, 1);
        uElement[i](3, 4) = uFull(0, 1);
      }
    }

    /* rotational elements */
    for (int i = 0; i < rotationalElements; i++) {
      int j = 3 * i; // start index in entire beam coordinates

      if (i > 0 && i < rotationalElements - 1) { // no problem case
        qRotationElement[i] = qFull(j - 1, j + 2); // staggered grid -> rotation offset
        uRotationElement[i] = uFull(j - 1, j + 2);
      }
      else if (i == 0) { // first element
        qRotationElement[i](0) = qFull(qFull.size() - 1);
        uRotationElement[i](0) = uFull(uFull.size() - 1);
        qRotationElement[i](1, 3) = qFull(j, j + 2);
        uRotationElement[i](1, 3) = uFull(j, j + 2);
        if (qFull(j + 2) < qFull(qFull.size() - 1))
          qRotationElement[i](0) -= 2. * M_PI;
        else
          qRotationElement[i](0) += 2. * M_PI;
      }
      else if (i == rotationalElements - 1) { // last element
        qRotationElement[i] = qFull(j - 1, j + 2);
        uRotationElement[i] = uFull(j - 1, j + 2);
      }
    }
    updEle = false;
  }

  void FlexibleBody1s21Cosserat::GlobalVectorContribution(int n, const Vec& locVec, Vec& gloVec) {
    int j = 3 * n; // start index in entire beam coordinates

    if (n < Elements - 1) {
      gloVec(j, j + 4) += locVec;
    }
    else { // last FE for closed structure
      gloVec(j, j + 2) += locVec(0, 2);
      gloVec(0, 1) += locVec(3, 4);
    }
  }

  void FlexibleBody1s21Cosserat::GlobalVectorContributionRotation(int n, const Vec& locVec, Vec& gloVec) {
    int j = 3 * n;
    if (n == 0) {
      gloVec(0, 2) += locVec(1, 3);
      gloVec(qFull.size() - 1) += locVec(0);
    }
    else if (n > 0 && n < rotationalElements) {
      gloVec(j - 1, j + 2) += locVec;
    }
  }

  void FlexibleBody1s21Cosserat::GlobalMatrixContribution(int n, const Mat& locMat, Mat& gloMat) {
    int j = 3 * n; // start index in entire beam coordinates

    if (n < Elements - 1) {
      gloMat(RangeV(j, j + 4), RangeV(j, j + 4)) += locMat;
    }
    else { // last FE
      gloMat(RangeV(j, j + 2), RangeV(j, j + 2)) += locMat(RangeV(0, 2), RangeV(0, 2));
      gloMat(RangeV(0, 1), RangeV(0, 1)) += locMat(RangeV(3, 4), RangeV(3, 4));
    }
  }

  void FlexibleBody1s21Cosserat::GlobalMatrixContribution(int n, const SymMat& locMat, SymMat& gloMat) {
    int j = 3 * n; // start index in entire beam coordinates

    if (n < Elements - 1) {
      gloMat(RangeV(j, j + 4)) += locMat;
    }
    else { // last FE
      gloMat(RangeV(j, j + 2)) += locMat(RangeV(0, 2));
      gloMat(RangeV(0, 1)) += locMat(RangeV(3, 4));
    }
  }

  void FlexibleBody1s21Cosserat::updatePositions(Frame1s *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s21Cosserat::updatePositions): Not implemented.");
  }

  void FlexibleBody1s21Cosserat::updateVelocities(Frame1s *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s21Cosserat::updateVelocities): Not implemented.");
  }

  void FlexibleBody1s21Cosserat::updateAccelerations(Frame1s *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s21Cosserat::updateAccelerations): Not implemented.");
  }

  void FlexibleBody1s21Cosserat::updateJacobians(Frame1s *frame, int j) {
    THROW_MBSIMERROR("(FlexibleBody1s21Cosserat::updateJacobians): Not implemented.");
  }

  void FlexibleBody1s21Cosserat::updateGyroscopicAccelerations(Frame1s *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s21Cosserat::updateGyroscopicAccelerations): Not implemented.");
  }

  void FlexibleBody1s21Cosserat::updatePositions(NodeFrame *frame) {
    int node = frame->getNodeNumber();
    /* 2D -> 3D mapping */
    Vec qTmpNODE(3, INIT, 0.);
    qTmpNODE(0, 1) = evalqFull()(3 * node + 0, 3 * node + 1);
    Vec qTmpANGLE(3, INIT, 0.);
    qTmpANGLE(2) = qFull(3 * node + 2);

    frame->setPosition(R->evalPosition() + R->evalOrientation() * qTmpNODE);

    frame->getOrientation(false).set(0, R->getOrientation() * angle->computet(qTmpANGLE));
    frame->getOrientation(false).set(1, R->getOrientation() * angle->computen(qTmpANGLE));
    frame->getOrientation(false).set(2, R->getOrientation() * angle->computeb(qTmpANGLE));
  }

  void FlexibleBody1s21Cosserat::updateVelocities(NodeFrame *frame) {
    int node = frame->getNodeNumber();
    Vec qTmpANGLE(3, INIT, 0.);
    qTmpANGLE(2) = qFull(3 * node + 2);
    Vec uTmp(3, INIT, 0.);
    uTmp(0, 1) = uFull(3 * node + 0, 3 * node + 1);
    Vec uTmpANGLE(3, INIT, 0.);
    uTmpANGLE(2) = uFull(3 * node + 2);

    frame->setVelocity(R->evalOrientation() * uTmp);
    frame->setAngularVelocity(R->evalOrientation() * angle->computeOmega(qTmpANGLE, uTmpANGLE));
 }

  void FlexibleBody1s21Cosserat::updateAccelerations(NodeFrame *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s21Cosserat::updateAccelerations): Not implemented.");
  }

  void FlexibleBody1s21Cosserat::updateJacobians(NodeFrame *frame, int j) {
    int node = frame->getNodeNumber();
    /* Jacobian of translation element matrix [1,0,0;0,1,0], static */
    Mat3xV Jacobian_trans(evalqFull().size(), INIT, 0.);
    Jacobian_trans(0, 3 * node) = 1;
    Jacobian_trans(1, 3 * node + 1) = 1;

    frame->setJacobianOfTranslation(R->evalOrientation() * Jacobian_trans, j);

    /* Jacobian of rotation element matrix [1,0,0;0,1,0], static */
    Mat3xV Jacobian_rot(evalqFull().size(), INIT, 0.);
    Jacobian_rot(2, 3 * node + 2) = 1;

    frame->setJacobianOfRotation(R->getOrientation() * Jacobian_rot, j);
  }

  void FlexibleBody1s21Cosserat::updateGyroscopicAccelerations(NodeFrame *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s21Cosserat::updateGyroscopicAccelerations): Not implemented.");
  }

  void FlexibleBody1s21Cosserat::init(InitStage stage, const InitConfigSet &config) {
    if (stage == preInit) {
      l0 = L / Elements;

      if (PODreduced)
        qSize = U.cols();
      else
        qSize = 3 * Elements;

      uSize[0] = qSize;
      uSize[1] = qSize; // TODO

      if (PODreduced) {
        //TODO: move into readz0
        q0.resize() = U.T() * q0;
        u0.resize() = U.T() * u0;
        q << q0;
        u << u0;
      }

      FlexibleBody1sCosserat::init(stage, config);

      Vec g = R->getOrientation().T() * MBSimEnvironment::getInstance()->getAccelerationOfGravity();

      /* translational elements */
      for (int i = 0; i < Elements; i++) {
        discretization.push_back(new FiniteElement1s21CosseratTranslation(l0, rho, A, E, G, I1, g));
        qElement.push_back(Vec(discretization[i]->getqSize(), INIT, 0.));
        uElement.push_back(Vec(discretization[i]->getuSize(), INIT, 0.));
        static_cast<FiniteElement1s21CosseratTranslation*>(discretization[i])->setMaterialDamping(Elements * cEps0D, cEps1D);
      }

      /* rotational elements */
      for (int i = 0; i < rotationalElements; i++) {
        rotationDiscretization.push_back(new FiniteElement1s21CosseratRotation(l0, E, G, I1));
        qRotationElement.push_back(Vec(rotationDiscretization[i]->getqSize(), INIT, 0.));
        uRotationElement.push_back(Vec(rotationDiscretization[i]->getuSize(), INIT, 0.));
        if (fabs(R1) > epsroot)
          static_cast<FiniteElement1s21CosseratRotation*>(rotationDiscretization[i])->setCurlRadius(R1);
      }

      hFull.resize(3 * Elements);
    }

    else if (stage == unknownStage) {
      FlexibleBody1sCosserat::init(stage, config);

      initialised = true;

      initM();

      updEle = true;
    }

    else if (stage == plotting) {
//      ((OpenMBV::SpineExtrusion*) openMBVBody.get())->setInitialRotation(AIK2Cardan(R->getOrientation()));
      FlexibleBody1sCosserat::init(stage, config);
    }

    else
      FlexibleBody1sCosserat::init(stage, config);

    //curve->initContourFromBody(stage);
  }

  double FlexibleBody1s21Cosserat::computePotentialEnergy() {
    /* translational elements */
    double V = FlexibleBody1sCosserat::computePotentialEnergy();

    /* rotational elements */
    for (unsigned int i = 0; i < rotationDiscretization.size(); i++) {
      V += rotationDiscretization[i]->computeElasticEnergy(qRotationElement[i]);
    }

    return V;
  }

  void FlexibleBody1s21Cosserat::updateh(int k) {
    /* translational elements */
    hFull.init(0); //TODO: avoid this as values are overwritten in GlobalVectorContribution anyway?!
    for (int i = 0; i < (int) discretization.size(); i++)
      discretization[i]->computeh(getqElement(i), getuElement(i)); // compute attributes of finite element
    for (int i = 0; i < (int) discretization.size(); i++) {
      GlobalVectorContribution(i, discretization[i]->geth(), hFull); // assemble
    }

    /* rotational elements */
    for (int i = 0; i < (int) rotationDiscretization.size(); i++)
      rotationDiscretization[i]->computeh(evalqRotationElement(i), evaluRotationElement(i)); // compute attributes of finite element
    for (int i = 0; i < (int) rotationDiscretization.size(); i++)
      GlobalVectorContributionRotation(i, rotationDiscretization[i]->geth(), hFull); // assemble

    //reduce
    if (PODreduced) {
      h[k] << U.T() * hFull;
    }
    else
      h[k] << hFull;

    // mass proportional damping
    if (d_massproportional > 0) {
      h[k] -= d_massproportional * (M * u);
    }
  }

  void FlexibleBody1s21Cosserat::setNumberElements(int n) {
    Elements = n;
    rotationalElements = n;
  }

  void FlexibleBody1s21Cosserat::enablePOD(const string & h5Path, int reduceMode, int POMSize) {
    // Set POD-Reduction true

    Mat Snapshots;
    Snapshots.resize() << readPositionMatrix(h5Path, "q"); //TODO: which kind of "Job"...

    int fullDOFs = 3 * Elements;
    int nSnapshots = Snapshots.cols();

    Mat SVD(fullDOFs, nSnapshots, NONINIT);
    SqrMat POM(fullDOFs, NONINIT);
    SqrMat POV(nSnapshots, NONINIT);

    int info = svd(Snapshots, SVD, POM, POV, 1); //TODO: what is last parameter of svd for?

    if (info != 0) {
      THROW_MBSIMERROR("FlexibleBody1s21Cosserat::enablePOD(const string & h5Path, bool reduceEnergy): Single-Value-Decomposition was not succesfull");
    }

    if (reduceMode == 1) {
      if (POMSize <= 0)
        THROW_MBSIMERROR("FlexibleBody1s21Cosserat::enablePOD(): No valid POMSize chosen -> Has to be positive!");
    }
    else {
      // k: Reduce Total Energy
      POMSize = findPOMSize(POM, SVD);
    }

    U.resize() << POM(RangeV(0, fullDOFs - 1), RangeV(0, POMSize - 1));

    PODreduced = true;

  }

  fmatvec::Vector<Fixed<6>, double> FlexibleBody1s21Cosserat::getPositions(double sGlobal) {
    fmatvec::Vector<Fixed<6>, double> temp(NONINIT);
    Vec2 zeta;
    zeta(0) = sGlobal;
    FixedContourFrame P("P",zeta);

    temp.set(RangeV(0,2),P.evalPosition());
    temp.set(RangeV(3,5),computeAngles(sGlobal, evalqFull()));


    return temp;
  }

  fmatvec::Vector<Fixed<6>, double> FlexibleBody1s21Cosserat::getVelocities(double sGlobal) {
    fmatvec::Vector<Fixed<6>, double> temp(NONINIT);
    Vec2 zeta;
    zeta(0) = sGlobal;
    FixedContourFrame P("P",zeta);

    temp.set(RangeV(0,2),P.evalVelocity());
    temp.set(RangeV(3,5),P.getAngularVelocity());

    return temp;
  }

  Vec3 FlexibleBody1s21Cosserat::computeAngles(double sGlobal, const Vec & vec) {
    Vec3 left, right;
    double sLocalRotation;
    int currentElementRotation;

    if (sGlobal < l0 / 2.) { // first rotation element (last half)
      sLocalRotation = sGlobal + l0 / 2.;
      left(2) = vec(vec.size() - 1);
      right(2) = vec(2);

      if (left(2) < right(2))
        left(2) += 2. * M_PI;
      else
        left(2) -= 2. * M_PI;
    }
    else if (sGlobal < L - l0 / 2.) {
      BuildElementTranslation(sGlobal + l0 / 2., sLocalRotation, currentElementRotation); // Lagrange parameter and number of rotational element (+l0/2)
      left(2) = vec(3 * currentElementRotation - 1); //.copy();
      right(2) = vec(3 * currentElementRotation + 2); //.copy();
    }
    else { // first rotation element (first half)
      sLocalRotation = sGlobal - (L - l0 / 2.);
      left(2) = vec(vec.size() - 1);
      right(2) = vec(2);

      if (left(2) < right(2))
        left(2) += 2. * M_PI;
      else
        left(2) -= 2. * M_PI;
    }

    return left + sLocalRotation / l0 * (right - left);;
  }

  void FlexibleBody1s21Cosserat::initInfo() {
    init(preInit, InitConfigSet());
//    init(resize);
    l0 = L / Elements;
    Vec g = Vec("[0.;0.;0.]");

    /* translational elements */
    for (int i = 0; i < Elements; i++) {
      discretization.push_back(new FiniteElement1s21CosseratTranslation(l0, rho, A, E, G, I1, g));
      qElement.push_back(Vec(discretization[0]->getqSize(), INIT, 0.));
      uElement.push_back(Vec(discretization[0]->getuSize(), INIT, 0.));
    }

    /* rotational elements */
    for (int i = 0; i < rotationalElements; i++) {
      rotationDiscretization.push_back(new FiniteElement1s21CosseratRotation(l0, E, G, I1));
      qRotationElement.push_back(Vec(rotationDiscretization[0]->getqSize(), INIT, 0.));
      uRotationElement.push_back(Vec(rotationDiscretization[0]->getuSize(), INIT, 0.));
    }
    FlexibleBody1sCosserat::init(unknownStage, InitConfigSet());
//    BuildElements();

    //curve->initContourFromBody(resize);
  }

  void FlexibleBody1s21Cosserat::BuildElementTranslation(const double& sGlobal, double& sLocal, int& currentElementTranslation) {
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

  void FlexibleBody1s21Cosserat::initM() {
    MConst.resize(3 * Elements);
    LLMConst.resize(3 * Elements);
    for (int i = 0; i < (int) discretization.size(); i++)
      static_cast<FiniteElement1s21CosseratTranslation*>(discretization[i])->initM(); // compute attributes of finite element
    for (int i = 0; i < (int) discretization.size(); i++)
      GlobalMatrixContribution(i, discretization[i]->getM(), MConst); // assemble
    for (int i = 0; i < (int) discretization.size(); i++) {
      int j = 3 * i;
      LLMConst(RangeV(j, j + 2)) = facLL(MConst(RangeV(j, j + 2)));
      if (openStructure && i == (int) discretization.size() - 1)
        LLMConst(RangeV(j + 3, j + 4)) = facLL(MConst(RangeV(j + 3, j + 4)));
    }

    if (PODreduced) {
      //Mass matrix is reduced
      MConst.resize() << U.T() * MConst * U;
      LLMConst.resize() << facLL(MConst);
    }

//    updateM(0, 0);
//    facLLM(0);
  }

  void FlexibleBody1s21Cosserat::computeBoundaryCondition() {
    // TODO
  }

  void FlexibleBody1s21Cosserat::exportPositionVelocity(const string & filenamePos, const string & filenameVel /*= string( )*/, const int & deg /* = 3*/, const bool &writePsFile /*= false*/) {
    THROW_MBSIMERROR("To be adapted to new interface...");
//
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
//        cp.getNodeNumber() = i - Elements;
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

  void FlexibleBody1s21Cosserat::importPositionVelocity(const string & filenamePos, const string & filenameVel /* = string( )*/) {
    THROW_MBSIMERROR("To be adapted to new interface...");
//
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

  Mat FlexibleBody1s21Cosserat::readPositionMatrix(const string & h5File, const string & Job) {
    H5::File file(h5File, H5::File::read);
    H5::Group *group = file.openChildObject<H5::Group>(name);
    H5::VectorSerie<double> *data = group->openChildObject<H5::VectorSerie<double> >("data");

    int qsize = data->getColumns();
    int tsize = data->getColumn(0).size();

    int i;
    int start;
    int end;
    Mat X_(tsize, qsize, INIT, 0.);

    if (Job == "A") {
      start = 0;
      end = qsize;
      X_.resize(tsize, end - start, INIT, 0.);
    }
    else if (Job == "q") {
      start = 1;
      end = (qsize - 1) / 2 + 1;
      X_.resize(tsize, end - start, INIT, 0.);
    }
    else if (Job == "u") {
      start = (qsize - 1) / 2 + 1;
      end = qsize;
      X_.resize(tsize, end - start, INIT, 0.);
    }
    else if (Job == "z") {
      start = 1;
      end = qsize;
      X_.resize(tsize, end - start, INIT, 0.);
    }
    else {
      start = 0;
      end = qsize;
      cout << "---------------------------------------" << endl;
      cout << "no job has been selected: make job = A" << endl << endl;
    }

    for (i = start; i < end; i++) {
      Vec tmp(data->getColumn(i));
      X_(RangeV(0, tsize - 1), RangeV(i - start, i - start)) = tmp(RangeV(0, tsize - 1));
    }

    return X_.T();
  }

  int FlexibleBody1s21Cosserat::findPOMSize(const Mat & POM, const Mat & SVD, double precission) {
    double SVEnergy = 0;
    double TmpSVEnergy = 0;
    int m = POM.rows();
    int i = 0;
    for (i = 0; i < m - 1; i++) {
      SVEnergy += SVD(i, i);
    }

    int n = 0;
    while (TmpSVEnergy < precission * SVEnergy && n < m) {
      TmpSVEnergy += SVD(n, n);
      n++;
    }
    return n;

  }
}

