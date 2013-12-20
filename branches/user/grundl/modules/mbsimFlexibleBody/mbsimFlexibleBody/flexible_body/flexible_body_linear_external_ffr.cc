/* Copyright (C) 20004-2011 MBSim Development Team
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
#include "mbsimFlexibleBody/flexible_body/flexible_body_linear_external_ffr.h"
#include "mbsimFlexibleBody/contours/nurbs_disk_2s.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/utils/eps.h"
#include <mbsim/utils/utils.h>
#include <mbsim/utils/rotarymatrices.h>
#include <mbsim/contours/sphere.h>

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/nurbsdisk.h"
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {
  
  FlexibleBodyLinearExternalFFR::FlexibleBodyLinearExternalFFR(const string &name, const bool &DEBUG_) :
      FlexibleBodyContinuum<Vec>(name), numOfElements(0), FFR(new Frame("FFR")), I_1(INIT, 0.0), I_kl(INIT, 0.0), S_bar(), I_kl_bar(), I_ThetaTheta(INIT, 0.0), I_ThetaF(), K(), G_bar(INIT, 0.0), G_bar_Dot(INIT, 0.0), A(INIT, 0), M_FF(), Qv(), phi(), nf(1), openStructure(true), DEBUG(DEBUG_) {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++)
        S_kl_bar[i][j] = SqrMat();
    }

    ContourPointData cp(0, FFRORIGIN);
    addFrame(FFR, cp);
    FFR->enableOpenMBV(1e-2);
  }
  
  FlexibleBodyLinearExternalFFR::~FlexibleBodyLinearExternalFFR() {
  }
  
  void FlexibleBodyLinearExternalFFR::init(InitStage stage) {
    if (stage == preInit) {
      // be careful the node index in abaqus starts from 1, but here starts 0. For example, if we want to add a frame to node 20 in abaqus, we should refer to node number 19 here.
      // TODO:  when wrinting a individual function for add nodes, we need do the transformation for user.
      vector<int> visNodelist;
//      for (int i = 0; i < 9; i++) {
//        visNodelist.push_back(227 + i);  // add frames to nodes ( N228 ~ N236) in abaqus.
//      }
      for (int i = 0; i < numOfElements; i++) {
        visNodelist.push_back(i);  // add frames to nodes ( N228 ~ N236) in abaqus.
      }

      for (size_t index = 0; index < visNodelist.size(); index++) {
        int nodeNumber = visNodelist[index];
        Frame * refFrame = new Frame("RefFrame" + numtostr(nodeNumber));
        addFrame(refFrame, nodeNumber);
        refFrame->enableOpenMBV(1e-3); //TODO: set this value
      }
    }
    else if (stage == resize) {
      // setNumberof qSize uSize[2]
      uSize[0] = qSize;
      uSize[1] = qSize; // TODO
    }
    else if (stage == MBSim::plot) {
      updatePlotFeatures();
    }
    else if (stage == unknownStage) {
      // read the input data file: mode shape vector, u0, mij, K.
//      readFEMData();  // move to public function, has to be executed before the init().
      
      assert(numOfElements > 1);
      // at least two azimuthal elements

      // create Elements and store them into discretization vector
      // the node index in abaqus strats from 1, but here starts from 0.
      for (int j = 0; j < numOfElements; j++) {
        discretization.push_back(new FiniteElementLinearExternalLumpedNode(mij[j], u0[j], phi(Index(3 * j, 3 * j + 2), Index(0, nf - 1))));
      }

      // compute the inertia shape integrals
      computeShapeIntegrals();

      updateAGbarGbardot();

      // set mode shape vector for each element.
//      BuildElements();  //
      
      initM(); // calculate constant stiffness matrix and the constant parts of the mass-matrix
      
      initQv();
    }

    FlexibleBodyContinuum<Vec>::init(stage);
  }
  
  void FlexibleBodyLinearExternalFFR::calcqSize() {
    qSize = 6 + nf;
  }

  void FlexibleBodyLinearExternalFFR::calcuSize(int j) {
    uSize[j] = 6 + nf;
  }

  void FlexibleBodyLinearExternalFFR::readFEMData(string inFilePath, const bool millimeterUnits) {
    // read mij
    double power = 1;
    if (millimeterUnits)
      power = 1000;

    ifstream mijInfile((inFilePath + "mij.dat").c_str());
    if (!mijInfile.is_open()) {
      cout << "Can not open file " << inFilePath << "mij.dat" << endl;
      throw 1;
    }
    for (double a; mijInfile >> a;)
      mij.push_back(a * power);
    
    // read u0
    ifstream u0Infile((inFilePath + "u0.dat").c_str());
    if (!u0Infile.is_open()) {
      cout << "Can not open file " << inFilePath << "u0.dat" << endl;
      throw 1;
    }
    for (string s; getline(u0Infile, s);) {
      fmatvec::Vec u0Line(3);
      istringstream sin(s);
      int i = 0;
      for (double temp; sin >> temp; i++) {
        u0Line(i) = temp;
      }
      u0.push_back(u0Line / power);
    }
    
    // get the number of lumped nodes(nj) = number of numOfElements in the model
    numOfElements = u0.size();
    
    // get the number of mode shapes(nf) used to describe the deformation
    ifstream phiInfile((inFilePath + "modeShapeMatrix.dat").c_str());
    if (!phiInfile.is_open()) {
      cout << "Can not open file " << inFilePath << "modeShapeMatrix.dat" << endl;
      throw 1;
    }
    string s1;
    std::vector<double> phiLine;
    getline(phiInfile, s1);
    istringstream sin1(s1);
    for (double temp; sin1 >> temp;)
      phiLine.push_back(temp);
    nf = phiLine.size();
    phiInfile.close();
    
    // read mode shape matrix
    phi.resize(3 * numOfElements, nf, INIT, 0.0);
    phiInfile.open((inFilePath + "modeShapeMatrix.dat").c_str());
    if (!phiInfile.is_open()) {
      cout << "Can not open file " << inFilePath << "modeShapeMatrix.dat" << endl;
      throw 1;
    }
    int row = 0;
    for (string s; getline(phiInfile, s); row++) {
      istringstream sin(s);  // TODO: use sin.clear() to avoid creating sin every time
      int i = 0;
      for (double temp; sin >> temp; i++)
        phi(row, i) = temp / power;
    }

    // read stiffness matrix
    SymMat KFull(3 * numOfElements, INIT, 0.0);
    ifstream KInfile((inFilePath + "stiffnessMatrix.dat").c_str());
    if (!KInfile.is_open()) {
      cout << "Can not open file " << inFilePath << "stiffnessMatrix.dat" << endl;
      throw 1;
    }
    for (string s; getline(KInfile, s);) {
      std::vector<double> KLine;
      istringstream sin(s);
      for (string tempxx; getline(sin, tempxx, ',');) {
        KLine.push_back(atof(tempxx.c_str()));  // atof()change char to double
      }
      KFull(3 * KLine[0] + KLine[1] - 4, 3 * KLine[2] + KLine[3] - 4) = KLine[4] * power;
    }
    
    K.resize(6 + nf, INIT, 0);

    SymMat Kff(phi.T() * KFull * phi);

    for (int i = 6; i < 6 + nf; i++)
      for (int j = i; j < 6 + nf; j++)
        K(i, j) = Kff(i - 6, j - 6);

    if (DEBUG) {

      cout.precision(6);
      cout << "numOfElements = " << numOfElements << endl;
      cout << "nf = " << nf << endl;
      
//      cout << "phi" << phi << endl;
      cout << "Kff" << Kff << endl;
      cout << "K" << K << endl;

      cout << "mij 0" << mij[0] << endl;
      cout << "mij 1" << mij[1] << endl;
      cout << "mij 2" << mij[2] << endl;
      cout << "mij last one" << mij[numOfElements - 1] << endl;

      cout << "u0[0] = " << u0.at(0) << endl;
      cout << "u0[1] = " << u0.at(1) << endl;
      cout << "u0[2] = " << u0.at(2) << endl;
      cout << "u0[" << numOfElements - 3 << "] = " << u0.at(numOfElements - 3) << endl;
      cout << "u0[" << numOfElements - 2 << "] = " << u0.at(numOfElements - 2) << endl;
      cout << "u0[" << numOfElements - 1 << "] = " << u0.back() << endl;
      cout << "u0.at(1)(2)" << u0.at(1)(2) << endl;

      cout << "phi node0,1 = " << phi(Index(0, 0), Index(0, nf - 1)) << endl;
      cout << "phi node0,2 = " << phi(Index(1, 1), Index(0, nf - 1)) << endl;
      cout << "phi node0,3 = " << phi(Index(2, 2), Index(0, nf - 1)) << endl;
      cout << "phi node" << numOfElements << ",1 = " << phi(Index(3 * numOfElements - 3, 3 * numOfElements - 3), Index(0, nf - 1)) << endl;
      cout << "phi node" << numOfElements << ",2 = " << phi(Index(3 * numOfElements - 2, 3 * numOfElements - 2), Index(0, nf - 1)) << endl;
      cout << "phi node" << numOfElements << ",3 = " << phi(Index(3 * numOfElements - 1, 3 * numOfElements - 1), Index(0, nf - 1)) << endl;
    }

  }
  
  void FlexibleBodyLinearExternalFFR::initM(int k) {
    // allocate memeory for M and Qv
    M[k].resize(6 + nf, INIT, 0.0);
    
    double M_RR = 0;  // constant during the simulation; for each DOF M_RR is same.  !!!!Every vaule has to be initialized before using it.
    for (int j = 0; j < numOfElements; j++) {
      M_RR += static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[j])->getMij();
    }

    // M_FF: nf*nf constant during the simulation;
    M_FF.resize(nf, INIT, 0.0);
    M_FF = S_kl_bar[0][0] + S_kl_bar[1][1] + S_kl_bar[2][2];

    // assemble M by the submatrix M_RR, M_RTheta, M_ThetaTheta, M_RF, M_ThetaF, M_FF
    updateM(0);

    M[k](0, 0) = M_RR;
    M[k](1, 1) = M_RR;
    M[k](2, 2) = M_RR;

    // copy M_FF to M in this way as the limitation of the submatrix opertor for symmetric matrix.
    for (int i = 6; i < nf + 6; i++)
      for (int j = i; j < nf + 6; j++)
        M[k](i, j) = M_FF(i - 6, j - 6);
    
    if (DEBUG) {
      cout << "M init " << M[k] << endl;
    }
  }
  
  void FlexibleBodyLinearExternalFFR::updateM(double t, int k) {
    // update M_ThetaR, M_ThetaTheta, M_FR, M_FTheta;  M_RR and M_FF is constant
    
    const fmatvec::Vec& qf = q(6, 5 + nf);
    
    // M_RTheta: 3*3
    fmatvec::Vec S_bar_t = I_1 + S_bar * qf;
    fmatvec::SqrMat M_RTheta((-A) * tilde(S_bar_t) * G_bar);
    
    // M_ThetaTheta: 3*3
    
    double u11 = I_kl(0, 0) + 2. * I_kl_bar[0][0] * qf + qf.T() * S_kl_bar[0][0] * qf;
    double u22 = I_kl(1, 1) + 2. * I_kl_bar[1][1] * qf + qf.T() * S_kl_bar[1][1] * qf;
    double u33 = I_kl(2, 2) + 2. * I_kl_bar[2][2] * qf + qf.T() * S_kl_bar[2][2] * qf;
    double u21 = I_kl(1, 0) + (I_kl_bar[1][0] + I_kl_bar[0][1]) * qf + qf.T() * S_kl_bar[1][0] * qf;
    double u31 = I_kl(2, 0) + (I_kl_bar[2][0] + I_kl_bar[0][2]) * qf + qf.T() * S_kl_bar[2][0] * qf;
    double u32 = I_kl(2, 1) + (I_kl_bar[2][1] + I_kl_bar[1][2]) * qf + qf.T() * S_kl_bar[2][1] * qf;
    
    I_ThetaTheta(0, 0) = u22 + u33;
    I_ThetaTheta(0, 1) = -u21;
    I_ThetaTheta(0, 2) = -u31;
    I_ThetaTheta(1, 1) = u11 + u33;
    I_ThetaTheta(1, 2) = -u32;
    I_ThetaTheta(2, 2) = u11 + u22;
    
    fmatvec::SymMat3 M_ThetaTheta(G_bar.T() * I_ThetaTheta * G_bar);
    
    // M_RF: 3*nf
    fmatvec::Mat M_RF = Mat(3, nf, INIT, 0.);
    M_RF = A * S_bar;
    
    // M_ThetaF: 3*nf
    I_ThetaF.resize(3, nf, INIT, 0.);
    
    I_ThetaF(Index(0, 0), Index(0, nf - 1)) = qf.T() * (S_kl_bar[1][2] - S_kl_bar[2][1]) + (I_kl_bar[1][2] - I_kl_bar[2][1]);
    I_ThetaF(Index(1, 1), Index(0, nf - 1)) = qf.T() * (S_kl_bar[2][0] - S_kl_bar[0][2]) + (I_kl_bar[2][0] - I_kl_bar[0][2]);
    I_ThetaF(Index(2, 2), Index(0, nf - 1)) = qf.T() * (S_kl_bar[0][1] - S_kl_bar[1][0]) + (I_kl_bar[0][1] - I_kl_bar[1][0]);
    
    fmatvec::Mat M_ThetaF(G_bar.T() * I_ThetaF);
    
    // update M by the submatrix M_RTheta, M_ThetaTheta, M_RF, M_ThetaF
    M[k](Index(0, 2), Index(3, 5)) = M_RTheta;
    M[k](Index(0, 2), Index(6, nf + 5)) = M_RF;

    // copy M_ThetaTheta to M in this way as the limitation of the submatrix opertor for symmetric matrix.
    for (int i = 3; i < 6; i++)
      for (int j = i; j < 6; j++)
        M[k](i, j) = M_ThetaTheta(i - 3, j - 3);

    M[k](Index(3, 5), Index(6, nf + 5)) = M_ThetaF;
    
    if (DEBUG) {

      cout << "qf " << qf << endl;
      cout << "I_kl(1,1) " << I_kl(0, 0) << endl;
      cout << "I_kl_bar(1,1) " << I_kl_bar[0][0] << endl;
      cout << "S_kl_bar(1,1) " << S_kl_bar[0][0] << endl;
      cout << "M_RTheta " << M_RTheta << endl;
      cout << "I_ThetaTheta " << I_ThetaTheta << endl;
      cout << "M_ThetaTheta " << M_ThetaTheta << endl;
      cout << "M_RF " << M_RF << endl;
      cout << "M_ThetaF " << M_ThetaF << endl;

    }

  }
  
  void FlexibleBodyLinearExternalFFR::initQv() {
    // initQv has to be called after initM
    Qv.resize(nf + 6, INIT, 0.0);
    updateQv();
    
    if (DEBUG) {
      cout << "Qv init" << Qv << endl;
    }
  }
  
  void FlexibleBodyLinearExternalFFR::updateQv() {
    
    // TODO: updateQv has to be called after updateM in each iteration
    fmatvec::Vec3 uTheta = u(3, 5);
    fmatvec::Vec uf = u(6, 5 + nf);
    fmatvec::Vec qf = q(6, 5 + nf);
    fmatvec::SqrMat omega_bar_skew(tilde(G_bar * uTheta));
    
    Qv(0, 2) = (-A) * (omega_bar_skew * omega_bar_skew * (I_1 + S_bar * qf) + 2. * omega_bar_skew * S_bar * uf);
    Qv(3, 5) = -2. * G_bar_Dot.T() * I_ThetaTheta * (G_bar * uTheta) - 2. * G_bar_Dot.T() * I_ThetaF * uf - G_bar.T() * I_ThetaTheta * (G_bar * uTheta);
    for (int j = 0; j < numOfElements; j++) {
      Qv(6, nf + 5) += -static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[j])->getMij() * (static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[j])->getModeShape().T() * (omega_bar_skew * omega_bar_skew * (static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[j])->getU0() + static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[j])->getModeShape() * qf) + 2. * omega_bar_skew * static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[j])->getModeShape() * uf));
    }

  }
  
  void FlexibleBodyLinearExternalFFR::updateh(double t, int k) {
    h[k] = Qv - K * q;
  }

  void FlexibleBodyLinearExternalFFR::computeShapeIntegrals() {
    // all shape integrals are constant, need to be calculated one time only.
//    if (DEBUG){
//      for (int j = 0; j < Elements; j++) {
//           cout << "phi entering computeShapeIntegrals" << static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[j])->getModeShape() << endl;
//      }
//    }
    //I_1  3*1,
    for (int j = 0; j < numOfElements; j++) {
      I_1 += static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[j])->getMij() * static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[j])->getU0();
    }
    
    // I_kl: each element is a scalar, and k,l =1,2,3  need define a matrix to store these nine scalar
    // as I_kl is symmetric, the second loop starts at l=k.
    for (int k = 0; k < 3; k++) {
      for (int l = k; l < 3; l++) {
        for (int j = 0; j < numOfElements; j++) {
          I_kl(k, l) += static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[j])->getMij() * static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[j])->getU0()(k) * static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[j])->getU0()(l);
        }
      }
    }

    // S_bar:  3*nf
    S_bar = Mat(3, nf, INIT, 0.);
    for (int j = 0; j < numOfElements; j++) {
      S_bar += static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[j])->getMij() * static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[j])->getModeShape();
    }
    
    //S_kl_bar each element is a nf*nf matrix and k,l = 1,2,3  need define a matrix to store these nine pointers of the nf*nf matrices
    for (int k = 0; k < 3; k++) {
      for (int l = 0; l < 3; l++) {
        S_kl_bar[k][l].resize(nf, INIT, 0.0);
        for (int j = 0; j < numOfElements; j++) {
          S_kl_bar[k][l] += static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[j])->getMij() * trans(static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[j])->getModeShape().row(k)) * static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[j])->getModeShape().row(l);
        }
      }
    }
    
    //I_kl_bar  each element is a 1*nf row vector, and k,l =1,2,3  need define a matrix to store these nine vector

    for (int k = 0; k < 3; k++) {
      for (int l = 0; l < 3; l++) {
        I_kl_bar[k][l].resize(nf, INIT, 0.0);
        for (int j = 0; j < numOfElements; j++) {
          I_kl_bar[k][l] += static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[j])->getMij() * static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[j])->getU0()(k) * static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[j])->getModeShape().row(l); // getModeShape() return matrix
        }
      }
    }
    
    if (DEBUG) {
      cout << "I_1" << I_1 << endl << endl;
      cout << "I_kl" << I_kl << endl << endl;
      cout << "S_bar" << S_bar << endl << endl;
      cout << "S_11_bar" << S_kl_bar[0][0] << endl << endl;
      cout << "S_22_bar" << S_kl_bar[1][1] << endl << endl;
      cout << "S_33_bar" << S_kl_bar[2][2] << endl << endl;
      cout << "S_23_bar" << S_kl_bar[1][2] << endl << endl;
      cout << "S_32_bar" << S_kl_bar[2][1] << endl << endl;
      cout << "I_13_bar" << I_kl_bar[0][2] << endl << endl;
      cout << "I_23_bar" << I_kl_bar[1][2] << endl << endl;
    }

  }
  
//  void FlexibleBodyLinearExternalFFR::BuildElements() {
//    // set mode shape vectors (3*nf) to each node.
//    for(int j=0;j<Elements;j++) {
//       // TODO: nodes which are in the boundary are missing in the output file of ABAQUS
//       static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[j])->setModeShape(phi( Index(3*j,3*j+2), Index(0,nf-1) ));
//    }
//  }
  
  void FlexibleBodyLinearExternalFFR::updateAGbarGbardot() {
    // use CardanPtr angle->computA() to compute A  and G, Gbar....

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
    
    G_bar(0, 0) = cosbeta * cosgamma;
    G_bar(0, 1) = singamma;
    G_bar(0, 2) = 0.;
    
    G_bar(1, 0) = -cosbeta * singamma;
    G_bar(1, 1) = cosgamma;
    G_bar(1, 2) = 0.;
    
    G_bar(2, 0) = sinbeta;
    G_bar(2, 1) = 0.;
    G_bar(2, 2) = 1.;
    
    // update G_bar_Dot
    //TODO:: check whether the formula for G_bar_Dot is correct?
    G_bar_Dot(0, 0) = -sinbeta * cosgamma - cosbeta * singamma;
    G_bar_Dot(0, 1) = cosgamma;
    
    G_bar_Dot(1, 0) = sinbeta * singamma - cosbeta * cosgamma;
    G_bar_Dot(1, 1) = -singamma;
    
    G_bar_Dot(2, 0) = cosbeta;
    
    if (DEBUG) {
      cout << "cosbeta" << cosbeta << endl << endl;
      cout << "cosgamma" << cosgamma << endl << endl;
      cout << "A" << A << endl << endl;
      cout << "G_bar" << G_bar << endl << endl;
      cout << "G_bar_Dot" << G_bar_Dot << endl << endl;
    }

  }

  //TODO: should we add updateAGbarGbardot() before updateKinematics and updateJacobian.
  void FlexibleBodyLinearExternalFFR::updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff, Frame *frame) {
    if (cp.getContourParameterType() == NODE) { // frame on the lumped node
      const int node = cp.getNodeNumber();

      if (ff == position || ff == position_cosy || ff == all) {

        Vec3 u_bar = static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[node])->getU0() + static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[node])->getModeShape() * q(6, 5 + nf);
//        u_bar = A * u_bar; // A*u_bar: transform local position vector expressed in FFR into Reference Frame R
//        u_bar += q(0, 2); // r_p = R + A*U_bar: add the translational displacement of FFR (based on the reference frame R) to the local position vector expressed in Reference Frame R

        cp.getFrameOfReference().setPosition(R->getPosition() + R->getOrientation() * (q(0, 2) + A * u_bar)); // transformation from Reference Frame R into world frame
        // TODO:  why in cosserat is there not  plus R->getPosition() ?
      }

      if (ff == localPosition || ff == all) {

        Vec3 u_bar = static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[node])->getU0() + static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[node])->getModeShape() * q(6, 5 + nf);

        cp.getFrameOfReference().setLocalPosition(u_bar);  // don't need to transform to the systerm coordinates,  TODO: need to be done in neutral contour when calcualting the position
      }

      // TODO: interpolate the position of lumped node to get a smooth surface, and then get A from that curve.
      SqrMat3 wA(R->getOrientation() * A);
      if (ff == normal || ff == cosy || ff == position_cosy || ff == velocity_cosy || ff == velocities_cosy || ff == all)
        cp.getFrameOfReference().getOrientation().set(0, wA.col(0));
      if (ff == firstTangent || ff == cosy || ff == position_cosy || ff == velocity_cosy || ff == velocities_cosy || ff == all)
        cp.getFrameOfReference().getOrientation().set(1, wA.col(1));
      if (ff == secondTangent || ff == cosy || ff == position_cosy || ff == velocity_cosy || ff == velocities_cosy || ff == all)
        cp.getFrameOfReference().getOrientation().set(2, wA.col(2));

      if (ff == velocity || ff == velocities || ff == velocity_cosy || ff == velocities_cosy || ff == all) {
        Vec3 A_S_qfDot = A * static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[node])->getModeShape() * u(6, 5 + nf);

        Vec3 u_bar = static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[node])->getU0() + static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[node])->getModeShape() * q(6, 5 + nf);

        Vec3 u_ref_1 = -A * tilde(u_bar) * G_bar * u(3, 5);

        cp.getFrameOfReference().setVelocity(R->getOrientation() * (u(0, 2) + u_ref_1 + A_S_qfDot));  // Schabana 5.15
      }

      if (ff == angularVelocity || ff == velocities || ff == velocities_cosy || ff == all) {
        // In reality, each point on the flexible body should has a different angular rotation velocity omega as the effect of deformation. But as we assume the deformation is small and thus linearizable, in calculating
        // deformation u_f, we neglect the effect of the change of orientation, simply set the deformation to be equal a weighted summation of different modes vectors.
        // For kinematics, it means that we assume that every point in the flexible body have the same angular velocity omega of the FFR, neglecting the effect of deformation on angular velocity.

        Vec omega_ref = A * G_bar * u(3, 5);

        cp.getFrameOfReference().setAngularVelocity(R->getOrientation() * omega_ref);
      }
    }
    else if (cp.getContourParameterType() == FFRORIGIN) { // origin of Floating frame
      if (ff == position || ff == position_cosy || ff == all) {
        cp.getFrameOfReference().setPosition(R->getPosition() + R->getOrientation() * q(0, 2)); // transformation from Reference Frame R into world frame
      }

      SqrMat3 wA(R->getOrientation() * A);
      if (ff == normal || ff == cosy || ff == position_cosy || ff == velocity_cosy || ff == velocities_cosy || ff == all)
        cp.getFrameOfReference().getOrientation().set(0, wA.col(0));
      if (ff == firstTangent || ff == cosy || ff == position_cosy || ff == velocity_cosy || ff == velocities_cosy || ff == all)
        cp.getFrameOfReference().getOrientation().set(1, wA.col(1));
      if (ff == secondTangent || ff == cosy || ff == position_cosy || ff == velocity_cosy || ff == velocities_cosy || ff == all)
        cp.getFrameOfReference().getOrientation().set(2, wA.col(2));

      if (ff == velocity || ff == velocities || ff == velocity_cosy || ff == velocities_cosy || ff == all) {
        cp.getFrameOfReference().setVelocity(R->getOrientation() * u(0, 2));
      }

      if (ff == angularVelocity || ff == velocities || ff == velocities_cosy || ff == all) {
        Vec3 omega_ref = A * G_bar * u(3, 5);

        cp.getFrameOfReference().setAngularVelocity(R->getOrientation() * omega_ref);
      }

    }
    else if (cp.getContourParameterType() == FIXEDRELATIVEFRAME) { // origin of Floating frame
      FixedRelativeFrame * frf = static_cast<FixedRelativeFrame*>(frame);
      if (ff == position || ff == position_cosy || ff == all) {
        cp.getFrameOfReference().setPosition(R->getPosition() +  R->getOrientation() * (q(0,2) +  A * frf->getRelativePosition())); // frf->getRelativePosition() is expressed in FFR frame, and FFR position is expressed in the inertial frame.
      }

      SqrMat3 wA(R->getOrientation() * A * frf->getRelativeOrientation());
      if (ff == normal || ff == cosy || ff == position_cosy || ff == velocity_cosy || ff == velocities_cosy || ff == all)
        cp.getFrameOfReference().getOrientation().set(0, wA.col(0));
      if (ff == firstTangent || ff == cosy || ff == position_cosy || ff == velocity_cosy || ff == velocities_cosy || ff == all)
        cp.getFrameOfReference().getOrientation().set(1, wA.col(1));
      if (ff == secondTangent || ff == cosy || ff == position_cosy || ff == velocity_cosy || ff == velocities_cosy || ff == all)
        cp.getFrameOfReference().getOrientation().set(2, wA.col(2));

      if (ff == velocity || ff == velocities || ff == velocity_cosy || ff == velocities_cosy || ff == all) {
        Vec3 u_bar = frf->getRelativePosition();
        Vec3 u_ref_1 = -A * tilde(u_bar) * G_bar * u(3, 5);
        cp.getFrameOfReference().setVelocity(R->getOrientation() * (u(0, 2) + u_ref_1));  // Schabana 5.15
      }

      if (ff == angularVelocity || ff == velocities || ff == velocities_cosy || ff == all) {
        Vec3 omega_ref = A * G_bar * u(3, 5);
        cp.getFrameOfReference().setAngularVelocity(R->getOrientation() * omega_ref);
      }

    }
    else
      throw MBSimError("ERROR(FlexibleBodyLinearExternalFFR::updateKinematicsForFrame): ContourPointDataType should be 'NODE' or 'CONTINUUM'");

    if (frame != 0) { // frame should be linked to contour point data     // TODO:  is ff is not full feature, how to update these four values?
      frame->setPosition(cp.getFrameOfReference().getPosition());
      frame->setOrientation(cp.getFrameOfReference().getOrientation());
      frame->setVelocity(cp.getFrameOfReference().getVelocity());
      frame->setAngularVelocity(cp.getFrameOfReference().getAngularVelocity());
    }
  }

  void FlexibleBodyLinearExternalFFR::updateJacobiansForFrame(ContourPointData &cp, Frame *frame) {
    if (cp.getContourParameterType() == NODE) { // force on node
      int node = cp.getNodeNumber();

      // Jacobian of element
      Mat Jactmp_trans(3, 6 + nf, INIT, 0.), Jactmp_rot(3, 6 + nf, INIT, 0.); // initializing Ref + 1 Node

      // translational DOFs (d/dR)
      Jactmp_trans(Index(0, 2), Index(0, 2)) = SqrMat(3, EYE); // ref
      Vec3 u_bar = static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[node])->getU0() + static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[node])->getModeShape() * q(6, 5 + nf);
      Jactmp_trans(Index(0, 2), Index(3, 5)) = -A * tilde(u_bar) * G_bar;
      Jactmp_rot(Index(0, 2), Index(3, 5)) = A * G_bar;

      // elastic DOFs
      // translation (A*phi)
      Jactmp_trans(Index(0, 2), Index(6, 5 + nf)) = A * static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[node])->getModeShape();

      // rotation part for elastic DOFs is zero.

      // transformation
      cp.getFrameOfReference().setJacobianOfTranslation(R->getOrientation() * Jactmp_trans);
      cp.getFrameOfReference().setJacobianOfRotation(R->getOrientation() * Jactmp_rot);

    }
    else if (cp.getContourParameterType() == FFRORIGIN) {
      Mat3xV Jactmp_trans(6 + nf, INIT, 0.);
      Mat3xV Jactmp_rot(6 + nf, INIT, 0.);

      Jactmp_trans.set(Index(0, 2), Index(0, 2), Mat3x3(EYE));

      Jactmp_rot.set(Index(0, 2), Index(3, 5), A * G_bar);

      cp.getFrameOfReference().setJacobianOfTranslation(R->getOrientation() * Jactmp_trans);
      cp.getFrameOfReference().setJacobianOfRotation(R->getOrientation() * Jactmp_rot);
    }
    else if (cp.getContourParameterType() == FIXEDRELATIVEFRAME) {
      FixedRelativeFrame * frf = static_cast<FixedRelativeFrame*>(frame);

      Mat3xV Jactmp_trans(6 + nf, INIT, 0.);
      Mat3xV Jactmp_rot(6 + nf, INIT, 0.);

      Jactmp_trans.set(Index(0, 2), Index(0, 2), Mat3x3(EYE));
      Jactmp_trans.set(Index(0, 2), Index(3, 5), -A * tilde(frf->getRelativePosition()) * G_bar);
      Jactmp_rot.set(Index(0, 2), Index(3, 5), A * G_bar);

      cp.getFrameOfReference().setJacobianOfTranslation(R->getOrientation() * Jactmp_trans);
      cp.getFrameOfReference().setJacobianOfRotation(R->getOrientation() * Jactmp_rot);
    }
    else
      throw MBSimError("ERROR(FlexibleBodyLinearExternalFFR::updateJacobiansForFrame): ContourPointDataType should be 'TRANSNODE', 'FFRORIGIN' or 'FIXEDRELATIVEFRAME'");

    // cp.getFrameOfReference().setGyroscopicAccelerationOfTranslation(TODO)
    // cp.getFrameOfReference().setGyroscopicAccelerationOfRotation(TODO)

    if (frame != 0) { // frame should be linked to contour point data
      frame->setJacobianOfTranslation(cp.getFrameOfReference().getJacobianOfTranslation());
      frame->setJacobianOfRotation(cp.getFrameOfReference().getJacobianOfRotation());
      frame->setGyroscopicAccelerationOfTranslation(cp.getFrameOfReference().getGyroscopicAccelerationOfTranslation());
      frame->setGyroscopicAccelerationOfRotation(cp.getFrameOfReference().getGyroscopicAccelerationOfRotation());
    }
  }

  void FlexibleBodyLinearExternalFFR::updateStateDependentVariables(double t) {
    updateAGbarGbardot();

    FlexibleBodyContinuum<Vec>::updateStateDependentVariables(t);
  }

  const Vec3 FlexibleBodyLinearExternalFFR::getModeShapeVector(int node, int column) const {
    return static_cast<FiniteElementLinearExternalLumpedNode*>(discretization[node])->getModeShape().col(column);
  }

  void FlexibleBodyLinearExternalFFR::addFrame(FixedRelativeFrame * frame) {
    ContourPointData cp(0, FIXEDRELATIVEFRAME);
    addFrame(frame, cp);
  }
}

