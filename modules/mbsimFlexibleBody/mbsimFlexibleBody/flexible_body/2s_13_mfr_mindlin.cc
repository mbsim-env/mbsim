/* Copyright (C) 2004-2015 MBSim Development Team
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

#include <config.h>
#include "mbsimFlexibleBody/flexible_body/2s_13_mfr_mindlin.h"
#include "mbsimFlexibleBody/frames/frame_2s.h"
#include "mbsimFlexibleBody/frames/node_frame.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/utils/eps.h"
#include <mbsim/utils/utils.h>
#include <mbsim/utils/rotarymatrices.h>

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  FlexibleBody2s13MFRMindlin::FlexibleBody2s13MFRMindlin(const string &name) :
      FlexibleBody2s13(name), N_compl(nullptr), R_compl(nullptr), R_ij(nullptr) {
    RefDofs = 6;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        N_ij[i][j] = nullptr;
        NR_ij[i][j] = nullptr;
      }
  }

  FlexibleBody2s13MFRMindlin::~FlexibleBody2s13MFRMindlin() {
    delete N_compl;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        delete N_ij[i][j];
        delete NR_ij[i][j];
      }
    delete R_compl;
    delete R_ij;
  }

  void FlexibleBody2s13MFRMindlin::updateM() {
    SymMat Mext = MConst; // copy constant mass matrix parts
    Vec qf = evalqExt()(RangeV(RefDofs, Dofs - 1));

    /* M_RR is constant */
    /* M_RTheta */
    Vec u_bar = (*R_compl) + (*N_compl) * qf;
    SqrMat3 M_RTheta = (-evalA()) * tilde(u_bar) * evalG();

    /* M_RF */
    Mat3xV M_RF = Mat3xV(Dofs - RefDofs, INIT, 0.); //= A*(*N_compl);

    /* M_ThetaTheta */
    SymMat3 I(INIT, 0.);

    I(0, 0) = (*R_ij)(1, 1) + (*R_ij)(2, 2) + 2. * ((*NR_ij[1][1]) + (*NR_ij[2][2])) * qf + qf.T() * ((*N_ij[1][1]) + (*N_ij[2][2])) * qf;
    I(0, 1) = -((*R_ij)(1, 0) + ((*NR_ij[1][0]) + (*NR_ij[0][1])) * qf + qf.T() * (*N_ij[1][0]) * qf);
    I(0, 2) = -((*R_ij)(2, 0) + ((*NR_ij[2][0]) + (*NR_ij[0][2])) * qf + qf.T() * (*N_ij[2][0]) * qf);
    I(1, 1) = (*R_ij)(2, 2) + (*R_ij)(0, 0) + 2. * ((*NR_ij[2][2]) + (*NR_ij[0][0])) * qf + qf.T() * ((*N_ij[2][2]) + (*N_ij[0][0])) * qf;
    I(1, 2) = -((*R_ij)(2, 1) + ((*NR_ij[2][1]) + (*NR_ij[1][2])) * qf + qf.T() * (*N_ij[2][1]) * qf);
    I(2, 2) = (*R_ij)(1, 1) + (*R_ij)(0, 0) + 2. * ((*NR_ij[1][1]) + (*NR_ij[0][0])) * qf + qf.T() * ((*N_ij[1][1]) + (*N_ij[0][0])) * qf;

    Mat3x3 M_ThetaTheta = G.T() * (I + J0) * G;

    /* M_ThetaF */
    Mat3xV qN(Dofs - RefDofs);

    qN.set(RangeV(0, 0), RangeV(0, Dofs - RefDofs - 1), qf.T() * ((*N_ij[1][2]) - (*N_ij[2][1]))); //+ (*NR_ij[1][2])-(*NR_ij[2][1]);
    qN.set(RangeV(1, 1), RangeV(0, Dofs - RefDofs - 1), qf.T() * ((*N_ij[2][0]) - (*N_ij[0][2]))); //+ (*NR_ij[2][0])-(*NR_ij[0][2]);
    qN.set(RangeV(2, 2), RangeV(0, Dofs - RefDofs - 1), qf.T() * ((*N_ij[0][1]) - (*N_ij[1][0]))); //+ (*NR_ij[0][1])-(*NR_ij[1][0]);

    Mat3xV M_ThetaF = G.T() * qN;

    /* M_FF is constant */

    /* sort into Mext */
    Mext.add(RangeV(0, 2), RangeV(3, 5), M_RTheta);
    Mext.add(RangeV(0, 2), RangeV(RefDofs, Dofs - 1), M_RF);

    for (int i = 3; i < RefDofs; i++)
      for (int j = i; j < RefDofs; j++)
        Mext(i, j) += M_ThetaTheta(i - 3, j - 3);

    Mext.add(RangeV(3, 5), RangeV(RefDofs, Dofs - 1), M_ThetaF);

    M = condenseMatrix(Mext, ILocked);

    /* Eigenvalues of M */
    if (msgAct(Debug)) {
      stringstream filenameM;
      filenameM << "M" << nr << "x" << nj << "t" << getTime() << ".txt";
      ofstream file_M(filenameM.str());
      file_M << M << endl;
      file_M << eigval(M) << endl;
      file_M.close();

      ofstream file_TS("LastTimeStep.txt");
      file_TS << getTime() << endl;
      file_TS.close();

      stringstream filenameMpart;
      filenameMpart << "Mpart" << nr << "x" << nj << ".txt";
      ofstream file_Mpart(filenameMpart.str());
      RangeV Ipart(3, 5);
      file_Mpart << "M_TT" << endl << M(Ipart) << endl;
      file_Mpart << eigval(M(Ipart)) << endl;
      file_Mpart.close();

      /* EIGENFREQUENCIES */
      if (eigval(M(Ipart))(0) < 0)
        throwError("TEST");

      //with MAPLE
      stringstream filename;
      filename << "Invertation" << nr << "x" << nj;
      MapleOutput(M, "M", filename.str());
      MapleOutput(K, "K", filename.str());

      //with MBSIM
      SqrMat H = static_cast<SqrMat>(slvLL(M, static_cast<Matrix<General, Ref, Ref, double>>(K)));
      Vector<Ref, std::complex<double>> EigVal = eigval(H);
      Vec NaturalHarmonics(EigVal.size(), INIT, 0.);
      for (int i = 0; i < EigVal.size() - 1; i++) {
        if (EigVal(i).imag() < 0 or EigVal(i).imag() > 0)
          throwError("updateM() - imaginary parts in EigVal");
        NaturalHarmonics(NaturalHarmonics.size() - 1 - i) = 1 / (M_PI * 2) * EigVal(i).real();
      }
      for (int i = 0; i < NaturalHarmonics.size(); i++)
        for (int j = 0; j < NaturalHarmonics.size() - i - 1; j++) {
          if (NaturalHarmonics(j) > NaturalHarmonics(j + 1)) {
            double tmp = NaturalHarmonics(j);
            NaturalHarmonics(j) = NaturalHarmonics(j + 1);
            NaturalHarmonics(j + 1) = tmp;
          }
        }
      stringstream filenameEigVal;
      filenameEigVal << "EigVal" << nr << "x" << nj << ".txt";
      ofstream file_eigval(filenameEigVal.str());
      file_eigval << NaturalHarmonics << endl;

      file_eigval << eigval(H) << endl;
      file_eigval.close();
    }

    //for testing
    //throwError("FlexibleBody2s13MFRMindlin::updateM -- Testing the Mass matrix");

    // LU-decomposition of M
    LLM = facLL(M);
  }

  void FlexibleBody2s13MFRMindlin::BuildElements() {
    for (int i = 0; i < Elements; i++) {
      //  ^ phi
      //  |
      //  |   4--------3
      //  |   |        |
      //  |   1--------2
      //  |
      //  | --------------> r
      // radial and azimuthal coordinates of the FE [ElementalNodes(r1,phi1,r2,phi2)]
      // r1 and phi1 are defined with node 1, r2 and phi2 with node 3
      ElementalNodes[i].set(RangeV(0, 1), NodeCoordinates.row(ElementNodeList(i, 0)).T()); // node 1
      ElementalNodes[i].set(RangeV(2, 3), NodeCoordinates.row(ElementNodeList(i, 2)).T()); // node 3

      if (ElementalNodes[i](3) <= ElementalNodes[i](1)) { // ring closure
        ElementalNodes[i](3) += 2 * M_PI;
      }
    }
    updEle = false;
  }

  void FlexibleBody2s13MFRMindlin::GlobalVectorContribution(int CurrentElement, const fmatvec::Vec& locVec, fmatvec::Vec& gloVec) {
    throwError("(FlexibleBody2s13MFRMindlin::GlobalVectorContribution): Not implemented!");
  }

  void FlexibleBody2s13MFRMindlin::GlobalMatrixContribution(int CurrentElement, const fmatvec::Mat& locMat, fmatvec::Mat& gloMat) {
    throwError("(FlexibleBody2s13MFRMindlin::GlobalMatrixContribution): Not implemented!");
  }

  void FlexibleBody2s13MFRMindlin::GlobalMatrixContribution(int CurrentElement, const fmatvec::SymMat& locMat, fmatvec::SymMat& gloMat) {
    throwError("(FlexibleBody2s13MFRMindlin::GlobalMatrixContribution): Not implemented!");
  }

  Vec3 FlexibleBody2s13MFRMindlin::evalPosition() {
    return R->evalPosition();
  }

  SqrMat3 FlexibleBody2s13MFRMindlin::evalOrientation() {
    return R->evalOrientation();
  }

  void FlexibleBody2s13MFRMindlin::updatePositions(Frame2s *frame) {
    if(nrm2(frame->getParameters()) < epsroot) { // center of gravity
      frame->setOrientation(R->evalOrientation()*evalA());
      switch(RefDofs) {
      case 2:
        frame->setPosition(R->evalPosition() + R->evalOrientation() * Vec3("[0;0;1]") * getq()(0));
        break;
      case 6:
        frame->setPosition(R->evalPosition() + R->evalOrientation() * getq()(RangeV(0,2)));
        break;
      default:
        throwError("(FlexibleBody2s13MFRMindlin::updateKinematicsForFrame): Unknown number of reference dofs!");
    }
    }
    else
      throwError("(FlexibleBody2s13MFRMindlin::updatePositions): Parameters must be zero!");
  }

  void FlexibleBody2s13MFRMindlin::updateVelocities(Frame2s *frame) {
   if(nrm2(frame->getParameters()) < epsroot) { // center of gravity
      switch(RefDofs) {
        case 2:
        frame->setVelocity(R->evalOrientation() * Vec3("[0;0;1]") * getu()(0));
        frame->setAngularVelocity(R->getOrientation() * Vec3("[0;0;1]") * getu()(1));
        break;
        case 6:
        frame->setPosition(R->evalOrientation() * getq()(RangeV(0,2)));
        frame->setVelocity(R->getOrientation() * getu()(RangeV(0,2)));
        frame->setAngularVelocity(R->getOrientation() * evalA() * evalG() * getu()(RangeV(3,5)));
        break;
        default:
        throwError("(FlexibleBody2s13MFRMindlin::updateVelocities): Unknown number of reference dofs!");
      }
    }
    else
      throwError("(FlexibleBody2s13MFRMindlin::updateVelocities): Parameters must be zero!");
  }

  void FlexibleBody2s13MFRMindlin::updateAccelerations(Frame2s *frame) {
    throwError("(FlexibleBody2s13MFRMindlin::updateAccelerations): Not implemented.");
  }

  void FlexibleBody2s13MFRMindlin::updateJacobians(Frame2s *frame, int j) {
    Vec2 alpha = frame->getParameters();

    if (nrm2(alpha) < epsroot) { // center of gravity
      Mat Jacext_trans(3, Dofs, INIT, 0.), Jacext_rot(3, Dofs, INIT, 0.);

      Jacext_trans.set(RangeV(0, 2), RangeV(0, 2), SqrMat(3, EYE));
      Jacext_rot.set(RangeV(0, 2), RangeV(3, 5), evalA() * evalG());

      // condensation
      Mat Jacobian_trans = condenseMatrixCols(Jacext_trans, ILocked);
      Mat Jacobian_rot = condenseMatrixCols(Jacext_rot, ILocked);

      // transformation
      frame->setJacobianOfTranslation(R->evalOrientation() * Jacobian_trans,j);
      frame->setJacobianOfRotation(R->getOrientation() * Jacobian_rot,j);
    }
    else { // on the disk
      throwError("(FlexibleBody2s13MFRMindlin::updateJacobians): Parameters must be zero!");
    }
  }

  void FlexibleBody2s13MFRMindlin::updateGyroscopicAccelerations(Frame2s *frame) {
    throwError("(FlexibleBody2s13MFRMindlin::updateGyroscopicAccelerations): Not implemented.");
  }

  void FlexibleBody2s13MFRMindlin::updatePositions(int node) {
    Vec3 r_ref(NONINIT);
    //first compute vector
    r_ref(0) = evalqExt()(RefDofs + node * NodeDofs + 1) * computeThickness(NodeCoordinates(node, 0)) / 2. + NodeCoordinates(node, 0); // radial component
    r_ref(1) = -qext(RefDofs + node * NodeDofs + 2) * computeThickness(NodeCoordinates(node, 0)) / 2.; // azimuthal component
    r_ref(2) = qext(RefDofs + node * NodeDofs) + computeThickness(NodeCoordinates(node, 0)) / 2.; //z-component

    r_ref = BasicRotAIKz(NodeCoordinates(node, 1)) * r_ref; //transformation into local frame  ---->?  transformation from intermediate frame(which are initially parallel to the local frame) to the FFR
    r_ref = evalA() * r_ref; //transformation from the moving frame of reference  ---->  ??? transformation from the moving frame of reference FFR to the Reference frame
    r_ref += qext(RangeV(0, 2)); //translation of moving frame of reference relative to frame of reference ---> add the translation displacement of the origin of FFR expressed in the Reference Frame
    // TODO:  is qext in Reference frame R or in the world frame ?
    WrOP[node] = R->evalPosition() + R->evalOrientation() * r_ref; //at last step: transformation into world frame
    updNodalPos[node] = false;
  }

  void FlexibleBody2s13MFRMindlin::updateVelocities(int node) {
    Vec3 u_ref_1(NONINIT);
    u_ref_1(0) = computeThickness(NodeCoordinates(node, 0)) / 2. * evaluExt()(RefDofs + node * NodeDofs + 1);
    u_ref_1(1) = -computeThickness(NodeCoordinates(node, 0)) / 2. * uext(RefDofs + node * NodeDofs + 2);
    u_ref_1(2) = uext(RefDofs + node * NodeDofs);

    Vec3 r_ref(NONINIT);
    r_ref(0) = evalqExt()(RefDofs + node * NodeDofs + 1) * computeThickness(NodeCoordinates(node, 0)) / 2. + NodeCoordinates(node, 0);
    r_ref(1) = -qext(RefDofs + node * NodeDofs + 2) * computeThickness(NodeCoordinates(node, 0)) / 2.;
    r_ref(2) = qext(RefDofs + node * NodeDofs) + computeThickness(NodeCoordinates(node, 0)) / 2.;

    r_ref = BasicRotAIKz(NodeCoordinates(node, 1)) * r_ref;

    Vec3 u_ref_2 = evalA() * (-tilde(r_ref) * evalG() * uext(RangeV(3, 5)) + BasicRotAIKz(NodeCoordinates(node, 1)) * u_ref_1);
    u_ref_2 += uext(RangeV(0, 2));

    WvP[node] = R->evalOrientation() * u_ref_2;

    Vec3 w_ref_1(INIT, 0.);
    w_ref_1(0) = -uext(RefDofs + node * NodeDofs + 2);
    w_ref_1(1) = uext(RefDofs + node * NodeDofs + 1);

    Vec w_ref_2 = A * (G * uext(RangeV(3, 5)) + BasicRotAIKz(NodeCoordinates(node, 1)) * w_ref_1);

    Wom[node] = R->getOrientation() * w_ref_2;

    updNodalVel[node] = false;
  }

  void FlexibleBody2s13MFRMindlin::updateAccelerations(int node) {
    throwError("(FlexibleBody2s13MFRMindlin::updateAccelerations): Not implemented.");
  }

  void FlexibleBody2s13MFRMindlin::updateJacobians(int node, int j) {

    // Jacobian of element
    Mat Jactmp_trans(3, RefDofs + NodeDofs, INIT, 0.), Jactmp_rot(3, RefDofs + NodeDofs, INIT, 0.); // initializing Ref + 1 Node

    // translational DOFs (d/dR)
    Jactmp_trans.set(RangeV(0, 2), RangeV(0, 2), SqrMat(3, EYE)); // ref

    // rotational DOFs (d/dTheta)
    SqrMat dAdalpha(3, NONINIT), dAdbeta(3, NONINIT), dAdgamma(3, NONINIT);

    double const &alpha = evalqExt()(3);
    double const &beta = qext(4);
    double const &gamma = qext(5);

    dAdalpha(0, 0) = 0.;
    dAdalpha(0, 1) = 0.;
    dAdalpha(0, 2) = 0.;
    dAdalpha(1, 0) = -sin(alpha) * sin(gamma) + cos(alpha) * sin(beta) * cos(gamma);
    dAdalpha(1, 1) = -sin(alpha) * cos(gamma) - cos(alpha) * sin(beta) * sin(gamma);
    dAdalpha(1, 2) = -cos(alpha) * cos(beta);
    dAdalpha(2, 0) = cos(alpha) * sin(gamma) + sin(alpha) * sin(beta) * cos(gamma);
    dAdalpha(2, 1) = cos(alpha) * cos(gamma) - sin(alpha) * sin(beta) * sin(gamma);
    dAdalpha(2, 2) = -sin(alpha) * cos(beta);

    dAdbeta(0, 0) = -sin(beta) * cos(gamma);
    dAdbeta(0, 1) = sin(beta) * sin(gamma);
    dAdbeta(0, 2) = cos(beta);
    dAdbeta(1, 0) = sin(alpha) * cos(beta) * cos(gamma);
    dAdbeta(1, 1) = -sin(alpha) * cos(beta) * sin(gamma);
    dAdbeta(1, 2) = sin(alpha) * sin(beta);
    dAdbeta(2, 0) = -cos(alpha) * cos(beta) * cos(gamma);
    dAdbeta(2, 1) = cos(alpha) * cos(beta) * sin(gamma);
    dAdbeta(2, 2) = -cos(alpha) * sin(beta);

    dAdgamma(0, 0) = -cos(beta) * sin(gamma);
    dAdgamma(0, 1) = -cos(beta) * cos(gamma);
    dAdgamma(0, 2) = 0.;
    dAdgamma(1, 0) = cos(alpha) * cos(gamma) - sin(alpha) * sin(beta) * sin(gamma);
    dAdgamma(1, 1) = -cos(alpha) * sin(gamma) - sin(alpha) * sin(beta) * cos(gamma);
    dAdgamma(1, 2) = 0.;
    dAdgamma(2, 0) = sin(alpha) * cos(gamma) + cos(alpha) * sin(beta) * sin(gamma);
    dAdgamma(2, 1) = -sin(alpha) * sin(gamma) + cos(alpha) * sin(beta) * cos(gamma);
    dAdgamma(2, 2) = 0.;

    Vec r_tmp(3, NONINIT);
    r_tmp(0) = NodeCoordinates(node, 0) + computeThickness(NodeCoordinates(node, 0)) / 2. * qext(RefDofs + node * NodeDofs + 1);
    r_tmp(1) = -computeThickness(NodeCoordinates(node, 0)) / 2. * qext(RefDofs + node * NodeDofs + 2);
    r_tmp(2) = qext(RefDofs + node * NodeDofs);

    r_tmp = BasicRotAIKz(NodeCoordinates(node, 1)) * r_tmp;

    Jactmp_trans.set(RangeV(0, 2), RangeV(3, 3), dAdalpha * r_tmp);
    Jactmp_trans.set(RangeV(0, 2), RangeV(4, 4), dAdbeta * r_tmp);
    Jactmp_trans.set(RangeV(0, 2), RangeV(5, 5), dAdgamma * r_tmp);

    Jactmp_rot.set(RangeV(0, 2), RangeV(3, 5), evalA() * evalG());

    // elastic DOFs
    // translation
    SqrMat u_tmp(3, INIT, 0.);
    u_tmp(0, 1) = computeThickness(NodeCoordinates(node, 0)) / 2.;
    u_tmp(1, 2) = -computeThickness(NodeCoordinates(node, 0)) / 2.;
    u_tmp(2, 0) = 1.;

    Jactmp_trans.set(RangeV(0, 2), RangeV(RefDofs, RefDofs + 2), A * BasicRotAIKz(NodeCoordinates(node, 1)) * u_tmp);

    // rotation
    SqrMat Z_tmp(3, INIT, 0.);
    Z_tmp(0, 2) = -1;
    Z_tmp(1, 1) = 1;
    Jactmp_rot.set(RangeV(0, 2), RangeV(RefDofs, RefDofs + 2), A * BasicRotAIKz(NodeCoordinates(node, 1)) * Z_tmp);

    // sort in the Jacobian of the disc disk
    // reference dofs
    Mat Jacext_trans(3, Dofs, INIT, 0.), Jacext_rot(3, Dofs, INIT, 0.);

    Jacext_trans.set(RangeV(0, 2), RangeV(0, RefDofs - 1), Jactmp_trans(RangeV(0, 2), RangeV(0, RefDofs - 1)));
    Jacext_rot.set(RangeV(0, 2), RangeV(0, RefDofs - 1), Jactmp_rot(RangeV(0, 2), RangeV(0, RefDofs - 1)));

    // elastic dofs
    Jacext_trans.set(RangeV(0, 2), RangeV(RefDofs + node * NodeDofs, RefDofs + node * NodeDofs + 2), Jactmp_trans(RangeV(0, 2), RangeV(RefDofs, RefDofs + 2)));
    Jacext_rot.set(RangeV(0, 2), RangeV(RefDofs + node * NodeDofs, RefDofs + node * NodeDofs + 2), Jactmp_rot(RangeV(0, 2), RangeV(RefDofs, RefDofs + 2)));

    // condensation
    Mat Jacobian_trans = condenseMatrixCols(Jacext_trans, ILocked);
    Mat Jacobian_rot = condenseMatrixCols(Jacext_rot, ILocked);

    // transformation
    WJP[j][node] = R->evalOrientation() * Jacobian_trans;
    WJR[j][node] = R->getOrientation() * Jacobian_rot;

    updNodalJac[j][node] = false;
  }

  void FlexibleBody2s13MFRMindlin::updateGyroscopicAccelerations(int node) {
    throwError("(FlexibleBody2s13MFRMindlin::updateGyroscopicAccelerations): Not implemented.");
  }

  void FlexibleBody2s13MFRMindlin::init(InitStage stage, const InitConfigSet &config) {
    if (stage == preInit) {
      assert(nr > 0); // at least on radial row
      assert(nj > 1); // at least two azimuthal elements

      nn = (nr+1)*nj;

      // condensation
      switch (LType) {
        case innerring: // 0: innerring
          ILocked = RangeV(RefDofs, RefDofs + NodeDofs * nj - 1);
          Jext.resize(Dofs, qSize, INIT, 0.);
          Jext.set(RangeV(0, RefDofs - 1), RangeV(0, RefDofs - 1), DiagMat(RefDofs, INIT, 1.));
          Jext.set(RangeV(RefDofs + NodeDofs * nj, Dofs - 1), RangeV(RefDofs, qSize - 1), DiagMat(qSize - RefDofs, INIT, 1.));
        break;

        case outerring: // 1: outerring
          ILocked = RangeV(qSize, Dofs - 1);
          Jext.resize(Dofs, qSize, INIT, 0.);
          Jext.set(RangeV(0, qSize - 1), RangeV(0, qSize - 1), DiagMat(qSize, INIT, 1.));
        break;
      }

      dr = (Ra - Ri) / nr;
      dj = 2 * M_PI / nj;

      NodeCoordinates.resize(Nodes, 2);
      ElementNodeList.resize(Elements, 4);

      // mapping nodes - node coordinates - elements
      for (int i = 0; i <= nr; i++) {
        for (int j = 0; j < nj; j++) {
          // node number increases azimuthally from the inner to the outer ring
          NodeCoordinates(j + i * nj, 0) = Ri + dr * i;
          NodeCoordinates(j + i * nj, 1) = 0. + dj * j;

          // element number increases azimuthally from the inner to the outer ring
          if (i < nr && j < nj - 1) {
            ElementNodeList(j + i * nj, 0) = j + i * nj; // elementnode 1
            ElementNodeList(j + i * nj, 1) = j + (i + 1) * nj; // elementnode 2
            ElementNodeList(j + i * nj, 2) = (j + 1) + (i + 1) * nj; // elementnode 3
            ElementNodeList(j + i * nj, 3) = (j + 1) + i * nj; // elementnode 4
          }
          else if (i < nr && j == nj - 1) { // ring closure
            ElementNodeList(j + i * nj, 0) = j + i * nj; // elementnode 1
            ElementNodeList(j + i * nj, 1) = j + (i + 1) * nj; // elementnode 2
            ElementNodeList(j + i * nj, 2) = 0 + (i + 1) * nj; // elementnode 3
            ElementNodeList(j + i * nj, 3) = 0 + i * nj; // elementnode 4
          }
        }
      }

      for (int i = 0; i < Elements; i++) {
        ElementalNodes.emplace_back(4, INIT, 0.);
      }

      BuildElements();

      for (int i = 0; i < Elements; i++) {
        discretization.push_back(new FiniteElement2s13MFRMindlin(E, nu, rho, d(0), d(1), d(2), ElementalNodes[i]));
      }

//      // borders of contour parametrisation
//      // beginning
//      Vec alphaS(2);
//      alphaS(0) = Ri; // radius
//      alphaS(1) = 0.; // angle
//
//      // end
//      Vec alphaE(2);
//      alphaE(0) = Ra; // radius
//      alphaE(1) = 2 * M_PI; // angle
//
//      contour->setAlphaStart(alphaS);
//      contour->setAlphaEnd(alphaE);


      FlexibleBody2s13::init(stage, config);
    }
    else if (stage == unknownStage) {
      initMatrices(); // calculate constant stiffness matrix and the constant parts of the mass-matrix

      FlexibleBody2s13::init(stage, config);
    }
    else
      FlexibleBody2s13::init(stage, config);
  }

  Vec FlexibleBody2s13MFRMindlin::transformCW(const Vec& WrPoint) {
    Vec CrPoint = WrPoint;

    CrPoint -= q(RangeV(0, 2));
    CrPoint = evalA().T() * CrPoint; // position in moving frame of reference

    const double xt = CrPoint(0);
    const double yt = CrPoint(1);

    CrPoint(0) = sqrt(xt * xt + yt * yt);
    CrPoint(1) = ArcTan(xt, yt);

    return CrPoint;
  }

  void FlexibleBody2s13MFRMindlin::initMatrices() {
    computeStiffnessMatrix();
    computeConstantMassMatrixParts();
    updateM();
  }

  void FlexibleBody2s13MFRMindlin::updateAG() {
    double sinalpha = sin(q(3));
    double cosalpha = cos(q(3));
    double sinbeta = sin(q(4));
    double cosbeta = cos(q(4));
    double singamma = sin(q(5));
    double cosgamma = cos(q(5));

    A(0, 0) = cosbeta * cosgamma;
    A(0, 1) = -cosbeta * singamma;
    A(0, 2) = sinbeta;

    A(1, 0) = cosalpha * singamma + sinalpha * sinbeta * cosgamma;
    A(1, 1) = cosalpha * cosgamma - sinalpha * sinbeta * singamma;
    A(1, 2) = -sinalpha * cosbeta;

    A(2, 0) = sinalpha * singamma - cosalpha * sinbeta * cosgamma;
    A(2, 1) = sinalpha * cosgamma + cosalpha * sinbeta * singamma;
    A(2, 2) = cosalpha * cosbeta;

    G(0, 0) = cosbeta * cosgamma;
    G(0, 1) = singamma;
    G(0, 2) = 0.;

    G(1, 0) = -cosbeta * singamma;
    G(1, 1) = cosgamma;
    G(1, 2) = 0.;

    G(2, 0) = sinbeta;
    G(2, 1) = 0.;
    G(2, 2) = 1.;

    updAG = false;
  }

  void FlexibleBody2s13MFRMindlin::computeStiffnessMatrix() {
    SymMat Kext(Dofs, INIT, 0.);
    int ElementNodes = 4;

    // element loop
    for (int element = 0; element < Elements; element++) {
      static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->computeStiffnessMatrix();
      SymMat ElK = static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->getK();

      for (int node = 0; node < ElementNodes; node++) {
        RangeV Ikges(RefDofs + ElementNodeList(element, node) * NodeDofs, RefDofs + (ElementNodeList(element, node) + 1) * NodeDofs - 1);
        RangeV Ikelement(node * NodeDofs, (node + 1) * NodeDofs - 1);

        Kext.add(Ikges, ElK(Ikelement)); // diagonal
        for (int n = node + 1; n < ElementNodes; n++) // secondary diagonals
          Kext.add(Ikges, RangeV(RefDofs + ElementNodeList(element, n) * NodeDofs, RefDofs + (ElementNodeList(element, n) + 1) * NodeDofs - 1), ElK(Ikelement, RangeV(n * NodeDofs, (n + 1) * NodeDofs - 1)));
      }
      static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->freeK();
    }

    // condensation
    K <<= condenseMatrix(Kext, ILocked);

    /* STATIC TEST */
    //RangeV Iall(RefDofs,K.size()-1);
    //// load
    //Vec F_test(K.size()-RefDofs,INIT,0.);
    //F_test((nr-1)*nj*3) = 1e10;
    //// displacements in MBSim
    //Vec q_test = slvLL(K(Iall),F_test);
    //Vec u_mbsim(12,NONINIT);
    //// first: positive x-axis
    //u_mbsim(0) = q_test(0);
    //u_mbsim(1) = q_test(nr/2*nj*3);
    //u_mbsim(2) = q_test((nr-1)*nj*3);
    //// second: positive y-axis
    //u_mbsim(3) = q_test(nj/4*3);
    //u_mbsim(4) = q_test(nr/2*nj*3+nj/4*3);
    //u_mbsim(5) = q_test((nr-1)*nj*3+nj/4*3);
    //// third: negative x-axis
    //u_mbsim(6) = q_test(nj/2*3);
    //u_mbsim(7) = q_test(nr/2*nj*3+nj/2*3);
    //u_mbsim(8) = q_test((nr-1)*nj*3+nj/2*3);
    //// fourth: negative y-axis
    //u_mbsim(9)  = q_test(3*nj/4*3);
    //u_mbsim(10) = q_test(nr/2*nj*3+3*nj/4*3);
    //u_mbsim(11) = q_test((nr-1)*nj*3+3*nj/4*3);
    //// displacements in ANSYS
    //Vec u_ansys("[0.10837E-15; 16.590; 50.111; -0.18542E-04; -0.85147; -2.4926; 0.0000; -0.17509; -0.31493; -0.18542E-04; -0.85147; -2.4926 ]");
    //// error
    //double maxerr = nrmInf(u_ansys-u_mbsim);
    //// output
    //ofstream file_static;
    //stringstream filename_static
    //filename_static <<  "Static" <<  nr << "x" << nj << ".txt";
    //file_static.open(filename_static.str());
    //file_static << "error=" << maxerr << endl;
    //file_static << "static_mbsim=matrix([" ;
    //for(int i=0;i<u_mbsim.size();i++)
    //  file_static << u_mbsim(i) << ",";
    //file_static << "]);" << endl;
    //file_static.close();
  }

  void FlexibleBody2s13MFRMindlin::computeConstantMassMatrixParts() {
    MConst.resize(Dofs, INIT, 0.);
    double ElementNodes = 4;

    /* M_RR */
    for (int i = 0; i < 3; i++) {
      MConst(i, i) += m0;
      for (int element = 0; element < Elements; element++) {
        static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->computeM_RR();
        MConst(i, i) += static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->getM_RR()(i, i);
        static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->freeM_RR();
      }
    }

    /* N_compl */
    delete N_compl;
    N_compl = new Mat(3, Dofs - RefDofs, INIT, 0.);
    for (int element = 0; element < Elements; element++) {
      static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->computeN_compl();
      Mat ElN_compl = static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->getN_compl();
      RangeV IRefTrans(0, 2);
      for (int node = 0; node < ElementNodes; node++) {
        RangeV Ikges(ElementNodeList(element, node) * NodeDofs, (ElementNodeList(element, node) + 1) * NodeDofs - 1);
        RangeV Ikelement(node * NodeDofs, (node + 1) * NodeDofs - 1);
        (*N_compl).add(IRefTrans, Ikges, ElN_compl(IRefTrans, Ikelement));
      }
      static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->freeN_compl();
    }

    /* N_ij */
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        delete N_ij[i][j];
        N_ij[i][j] = new SqrMat(Dofs - RefDofs, INIT, 0.);
        for (int element = 0; element < Elements; element++) {
          static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->computeN_ij(i, j);
          SqrMat ElN_ij = static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->getN_ij(i, j);

          for (int node = 0; node < ElementNodes; node++) {
            RangeV Ikges(ElementNodeList(element, node) * NodeDofs, (ElementNodeList(element, node) + 1) * NodeDofs - 1);
            RangeV Ikelement(node * NodeDofs, (node + 1) * NodeDofs - 1);

            (*(N_ij[i][j])).add(Ikges, Ikges, ElN_ij(Ikelement)); // diagonal
            for (int n = node + 1; n < ElementNodes; n++) // secondary diagonals
              (*(N_ij[i][j])).add(Ikges, RangeV(ElementNodeList(element, n) * NodeDofs, (ElementNodeList(element, n) + 1) * NodeDofs - 1), ElN_ij(Ikelement, RangeV(n * NodeDofs, (n + 1) * NodeDofs - 1)));
          }
          static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->freeN_ij(i, j);
        }
      }
    }

    /* M_FF */
    for (int i = 0; i < 3; i++)
      for (int k = RefDofs; k < Dofs; k++)
        for (int l = k; l < Dofs; l++)
          MConst(k, l) += (*(N_ij[i][i]))(k - RefDofs, l - RefDofs);

    /* NR_ij */
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        delete NR_ij[i][j];
        NR_ij[i][j] = new RowVec(Dofs - RefDofs, INIT, 0.);
        for (int element = 0; element < Elements; element++) {
          static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->computeNR_ij(i, j);
          RowVec ElNR_ij = static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->getNR_ij(i, j);
          for (int node = 0; node < ElementNodes; node++) {
            RangeV Ikges(ElementNodeList(element, node) * NodeDofs, (ElementNodeList(element, node) + 1) * NodeDofs - 1);
            RangeV Ikelement(node * NodeDofs, (node + 1) * NodeDofs - 1);
            (*(NR_ij[i][j])).add(Ikges, ElNR_ij(Ikelement));
          }
          static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->freeNR_ij(i, j);
        }
      }
    }

    /* R_compl */
    delete R_compl;
    R_compl = new Vec(3, INIT, 0.);
    for (int element = 0; element < Elements; element++) {
      static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->computeR_compl();
      *R_compl += static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->getR_compl();
      static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->freeR_compl();
    }

    /* R_ij */
    delete R_ij;
    R_ij = new SymMat(3, INIT, 0.);
    for (int element = 0; element < Elements; element++) {
      static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->computeR_ij();
      *R_ij += static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->getR_ij();
      static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->freeR_ij();
    }

  }
}

