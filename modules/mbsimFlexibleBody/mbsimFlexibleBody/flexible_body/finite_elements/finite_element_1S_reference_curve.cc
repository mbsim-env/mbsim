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
 * Created on: Jul 30, 2013
 * Contact: kilian.grundl@gmail.com
 */

#include "config.h"

#include "finite_element_1S_reference_curve.h"

#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1S_reference_curve_functions.h"

#include <mbsim/numerics/gaussian_quadratur.h>
#include <mbsim/utils/eps.h>

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimFlexibleBody;

namespace MBSimFlexibleBody {

  FlexibleBody1SReferenceCurveFE::FlexibleBody1SReferenceCurveFE(FlexibleBody1SReferenceCurve * parent, int eleNo, Vec2 alpha, int order, int nodeDofs) :
      DiscretizationInterface(), parent(parent), element(eleNo), alpha(alpha), order(order), nodeDirs(nodeDofs), coeffs(0) {

    if (0) {
      // initialize Files for debug-output
      ofstream myfile;
      myfile.open("hEle.txt", ios::trunc);
      myfile.close();

      myfile.open("MEle.txt", ios::trunc);
      myfile.close();
    }

  }

  FlexibleBody1SReferenceCurveFE::~FlexibleBody1SReferenceCurveFE() {

  }

  void FlexibleBody1SReferenceCurveFE::init(Element::InitStage stage, const InitConfigSet &config) {

    // initialize the coefficients
    if (stage == Element::resize) {
      coeffs.resize(order + 1);
      for (int i = 0; i < order + 1; i++) {
        coeffs[i].resize(order + 1);
        coeffs[i][0].resize(order + 1);
//        coeffs[i][0] = getLagrangeCoeffs(i);
        coeffs[i][0] = getHermiteCoeffs(i);
        // build derivatives
        for (int der = 1; der <= order; der++) {
          coeffs[i][der].resize(order + 1 - der);
          for (int j = 0; j < order + 1 - der; j++) {
            coeffs[i][der][j] = coeffs[i][der - 1][j + 1] * (j + 1);
          }
        }
      }

      freeDoFs.clear();
      for (int i = 0; i < dofDirs.size(); i++) {
        if (dofDirs(i) >= 0) {
          freeDoFs.push_back(i);
        }
      }

      if (0) {
        cout << "Free Local DoFs of Element #" << element << "  are: \t";
        for (size_t i = 0; i < freeDoFs.size(); i++) {
          cout << setfill(' ') << setw(3) << freeDoFs[i] << ", ";
        }
        cout << endl;
        cout << "Corresponding Global DoFs are \t\t";
        for (size_t i = 0; i < freeDoFs.size(); i++) {
          cout << setfill(' ') << setw(3) << dofDirs(freeDoFs[i]) << ", ";
        }
        cout << endl;
      }

      h.resize(freeDoFs.size(), INIT, 0.);
      // resize the single parts of the h vector
      hT1.resize(freeDoFs.size(), INIT, 0.);
      hT2.resize(freeDoFs.size(), INIT, 0.);
      hT3.resize(freeDoFs.size(), INIT, 0.);
      hV1.resize(freeDoFs.size(), INIT, 0.);
      hV2.resize(freeDoFs.size(), INIT, 0.);
      hV3.resize(freeDoFs.size(), INIT, 0.);
      hV4.resize(freeDoFs.size(), INIT, 0.);
    }

    else if (stage == Element::unknownStage) {
      if (0) {
        //testing the Legendre polynoms
        int pnts = 30;
        Vec out(pnts, NONINIT);
        for (int j = 0; j < 6; j++) {
          for (int i = 0; i < pnts; i++) {
            double xi = -1 + 2. * i / (pnts - 1.);
            out(i) = computeLegendre(j, xi, 0);
          }
          cout << out << endl;
        }
      }

      if (0) {
        //testing the ansatz functions
        int pnts = 100;
        Vec y(pnts, NONINIT);
        Vec x(pnts, NONINIT);

        //Values
        for (int der = 0; der <= order; der++) {
          for (int j = 0; j < getNodesSize(); j++) {
            for (int i = 0; i < pnts; i++) {
              double xi = -1 + 2. * i / (pnts - 1.);
              x(i) = xi;
              y(i) = computeN(j, xi, der);
            }
            if (der == 0 and j == 0) {
              //plot x only once
              cout << "x" << " = [ ";
              for (int k = 0; k < y.size(); k++) {
                if (k > 0)
                  cout << ", ";
                cout << x(k);
              }
              cout << "];" << endl;
            }
            // plot y
            cout << "y" << der << "(" << j + 1 << ",:) = [ ";
            for (int k = 0; k < y.size(); k++) {
              if (k > 0)
                cout << ", ";
              cout << y(k);
            }
            cout << "];" << endl;
          }
        }
        throwError("init failed");
      }
    }
  }

  void FlexibleBody1SReferenceCurveFE::computeh(const Vec & qEle, const Vec & uEle) {
    // The h-vector of this body is the sum of the following (h = h1+h2+h3...):
    // PARTS that come from the kinetic energy
    //  hT1 = - 2 * rho*A * sdot * qdot^T * intPTdPdxi
    //  hT2 = - 2 *  rho * A * dotq^T intPTdPdt
    //  hT3_k = dT/dqk = rho * A * dotq^T intPTdPdqk dotq
    // PARTS that come from the potential energy
    //  hV1_k = - dWgammadq = -EA intForWgamma
    //  hV2_k = - dWndqk = - EIn intForWn_k
    //  hV3_k = - dWbdqk = - EIb intForWb_k
    //  hV4_k = - dWtdqk = - GIt  intForWtau_k

    if (1) {
      // hT1 shhould be zero for this closed curve
//      hT1 = -2 * parent->getrho() * parent->getA() * uEle(0) * trans(uEle.T() * integratePTdPdxi());

      hT2 = -2 * parent->getrho() * parent->getA() * trans(uEle.T() * integratePTdPdt());

      // For qInd == 1 it is the full matrix, otherwise its just the first two entries
      hT3(1) = parent->getrho() * parent->getA() * uEle.T() * integratePTdPdqk(1) * uEle;
      for (size_t qIndLoc = 2; qIndLoc < freeDoFs.size(); qIndLoc++) {
        hT3(qIndLoc) = 0.;
        SqrMatV tmp(integratePTdPdqk(freeDoFs[qIndLoc]));
        for (int i = 0; i < 2; i++) {
          hT3(qIndLoc) += (uEle.T() * tmp.col(i)) * uEle(i);
        }
        hT3(qIndLoc) *= parent->getrho() * parent->getA();
      }

      // parts of the potential energy
      for (size_t qIndLoc = 1; qIndLoc < freeDoFs.size(); qIndLoc++) {

        // Elongation (Wgamma)
        if (parent->elongationActive)
          hV1(qIndLoc) = -parent->getE() * parent->getA() * integrateForWgamma(freeDoFs[qIndLoc]);

        // Bending in main-plane (Wn)
        if (parent->normalBendingActive)
          hV2(qIndLoc) = -parent->getE() * parent->getIn() * integrateForWn(freeDoFs[qIndLoc]);

        if (nodeDirs == 3) {
          // Bending outside main-plane (Wb)
          hV3(qIndLoc) = -parent->getE() * parent->getIb() * integrateForWb(freeDoFs[qIndLoc]);

          // Torsion (Wtau)
          //        hV123(qInd) += - computeG() * It * forhV123(2);
        }
      }

      h = hT1 + hT2 + hT3 + hV1 + hV2 + hV3;

      if (0) {
        ofstream myfile;
        myfile.open("hEle.txt", ios::app);
        myfile << "Element No " << element << endl;
        myfile.precision(2);
        myfile << "h  =" << h.T();
        myfile << "hT1=" << hT1.T();
        myfile << "hT2=" << hT2.T();
        myfile << "hT3=" << hT3.T();
        myfile << "hV1=" << hV1.T();
        myfile << "hV2=" << hV2.T();
        myfile << "hV3=" << hV3.T() << endl;
        myfile.close();
      }
    }
  }

  void FlexibleBody1SReferenceCurveFE::computeM(const Vec & qEle) {
    if (1) {
      M = parent->getrho() * parent->getA() * integratePTP();

      if (0) {
        ofstream myfile;
        myfile.open("MEle.txt", ios::app);
        myfile << "Element No " << element << endl;
        myfile.precision(2);
        myfile << M << endl;
        myfile.close();
      }
    }
    else {
      M = SymMatV(freeDoFs.size(), EYE);
    }
  }

  double FlexibleBody1SReferenceCurveFE::computeXiLoc(double xiGlob) {
    if ((xiGlob >= alpha(0)) and (xiGlob <= alpha(1))) {
      return (2 * xiGlob - alpha(1) - alpha(0)) / (alpha(1) - alpha(0));
    }
    throw MBSimError("Invalid global coordinate xiGlob = " + toString(xiGlob) + " given.");
    return -2.;
  }

  double FlexibleBody1SReferenceCurveFE::computeXiGlob(double xiLoc) {
    if (xiLoc >= -1 and xiLoc <= 1) {
      return ((alpha(1) - alpha(0)) * xiLoc + (alpha(1) + alpha(0))) / 2;
    }
    throw MBSimError("Invalid local coordinate xiloc = " + toString(xiLoc) + " given.");
    return -2.;
  }

  fmatvec::Vec3 FlexibleBody1SReferenceCurveFE::computer(double xi, int derXi, int derTheta) {
    Vec3 rRef = parent->computerRef(xi, derXi, derTheta);
    Vec3 rf = computeBqf(xi, derXi, derTheta);

    return rRef + rf;
  }

  fmatvec::Vec3 FlexibleBody1SReferenceCurveFE::computev(double xi, int derXi) {
    Vec3 vsdot = computer(xi, derXi + 1, 0) * parent->uElementAll[element](0);
    Vec3 vthetadot = computer(xi, derXi, 1) * parent->uElementAll[element](1);
    Vec3 vf = computeBuf(xi, derXi, 0);
    return vsdot + vthetadot + vf;
  }

  fmatvec::Vec3 FlexibleBody1SReferenceCurveFE::computedrdqk(double xi, int derXi, int dofDirLoc) {
    Vec3 drdqk;
    if (dofDirLoc == 0) {
      // stays zero
    }
    else if (dofDirLoc == 1) {
      drdqk = computer(xi, derXi, 1);
    }
    else {
      drdqk = computeB(dofDirLoc, xi, derXi, 0);
    }

    return drdqk;
  }

  double FlexibleBody1SReferenceCurveFE::computeN(int nodeNo, double xiLoc, int derXi) {
    /**!
     * The following code uses the horner rule to evaluate the ansatz function that is a polynom with given coefficients!
     */
    if (derXi > order)
      return 0.;

    int i = order - derXi; // = coeffs[nodeNo].size()-1;

    double res = coeffs[nodeNo][derXi][i];
    for (i--; i >= 0; i--) {
      res = res * xiLoc + coeffs[nodeNo][derXi][i];
    }

    // consider the after derivative
    double b = 2. / (alpha(1) - alpha(0));

    double dxidx = 1.;
    for (int i = 0; i < derXi; i++) //avoiding the pow-function
      dxidx *= b;

    return res * dxidx;

  }

  double FlexibleBody1SReferenceCurveFE::computeLegendre(int j, double xiLoc, int derXi) {
    if (j == 0) {
      if (derXi == 0)
        return 1.;
      else
        return 0;
    }
    else if (j == 1) {
      if (derXi == 0)
        return xiLoc;
      else if (derXi == 1) {
        return 1.;
      }
      else
        return 0;
    }
    else {
      if (derXi == 0) {
        return (2. * j - 1.) / j * xiLoc * computeLegendre(j - 1, xiLoc, derXi) - (j - 1.) / j * computeLegendre(j - 2, xiLoc, derXi);
      }
      else {
        double sum = 0;
        for (int i = j - 1; i >= 0; i -= 2)
          sum += (2 * i + 1) * computeLegendre(i, xiLoc, derXi - 1);
        return sum;
      }
    }
  }

  fmatvec::Vec3 FlexibleBody1SReferenceCurveFE::computeB(int dofDirLocal, double xiGlob, int derXi, int derTheta) {
    if (AVers == 0) {
      if (derXi == 0) {
        return parent->computeARef(xiGlob, 0, derTheta) * computeS(dofDirLocal - 2, xiGlob, 0);
      }
      else if (derXi == 1) {
        Vec3 ret;
        ret = parent->computeARef(xiGlob, 1, derTheta) * computeS(dofDirLocal - 2, xiGlob, 0);
        ret += parent->computeARef(xiGlob, 0, derTheta) * computeS(dofDirLocal - 2, xiGlob, 1);
        return ret;
      }
      else if (derXi == 2) {
        Vec3 ret;
        ret = parent->computeARef(xiGlob, 2, derTheta) * computeS(dofDirLocal - 2, xiGlob, 0);
        ret += 2. * parent->computeARef(xiGlob, 1, derTheta) * computeS(dofDirLocal - 2, xiGlob, 1);
        ret += parent->computeARef(xiGlob, 0, derTheta) * computeS(dofDirLocal - 2, xiGlob, 2);
        return ret;
      }
      else if (derXi == 3) {
        Vec3 ret;
        ret = parent->computeARef(xiGlob, 3, derTheta) * computeS(dofDirLocal - 2, xiGlob, 0);
        ret += 3. * parent->computeARef(xiGlob, 2, derTheta) * computeS(dofDirLocal - 2, xiGlob, 1);
        ret += 3. * parent->computeARef(xiGlob, 1, derTheta) * computeS(dofDirLocal - 2, xiGlob, 2);
        ret += parent->computeARef(xiGlob, 0, derTheta) * computeS(dofDirLocal - 2, xiGlob, 3);
        return ret;
      }
      else {
        throw MBSimError("Not implemented yet " + string(__func__));
      }
    }

    if (AVers == 1) {
      double xiOfDOF = computeXiOfDOF(dofDirLocal);
      return parent->computeARef(xiOfDOF, 0, derTheta) * computeS(dofDirLocal - 2, xiGlob, derXi);
    }

    if (AVers == 2) {
      if (derTheta > 0) {
        return Vec3();
      }

      return computeS(dofDirLocal - 2, xiGlob, derXi);
    }

    return 0.;
  }

  fmatvec::Vec3 FlexibleBody1SReferenceCurveFE::computeBqf(double xiGlob, int derXi, int derTheta) {
    Vec3 rf;
    if (AVers == 0) {
      if (derXi == 0) {
        Vec3 rfLoc = computeSTimeslocVec(xiGlob, 0, parent->qElementAll[element]);
        SqrMat3 ARef = parent->computeARef(xiGlob, 0, derTheta);
        rf = ARef * rfLoc;
      }
      else if (derXi == 1) {
        SqrMat3 ARdxi = parent->computeARef(xiGlob, 1, derTheta);
        Vec3 Sqf = computeSTimeslocVec(xiGlob, 0, parent->qElementAll[element]);
        SqrMat3 AR = parent->computeARef(xiGlob, 0, derTheta);
        Vec3 Sqdfdxi = computeSTimeslocVec(xiGlob, 1, parent->qElementAll[element]);
        rf = ARdxi * Sqf + AR * Sqdfdxi;
      }
      else if (derXi == 2) {
        Vec3 part1 = parent->computeARef(xiGlob, 2, derTheta) * computeSTimeslocVec(xiGlob, 0, parent->qElementAll[element]);
        Vec3 part2 = 2. * parent->computeARef(xiGlob, 1, derTheta) * computeSTimeslocVec(xiGlob, 1, parent->qElementAll[element]);
        Vec3 part3 = parent->computeARef(xiGlob, 0, derTheta) * computeSTimeslocVec(xiGlob, 2, parent->qElementAll[element]);
        rf = part1 + part2 + part3;
      }
      else if (derXi == 3) {
        Vec3 part1 = parent->computeARef(xiGlob, 3, derTheta) * computeSTimeslocVec(xiGlob, 0, parent->qElementAll[element]);
        Vec3 part2 = 3. * parent->computeARef(xiGlob, 2, derTheta) * computeSTimeslocVec(xiGlob, 1, parent->qElementAll[element]);
        Vec3 part3 = 3. * parent->computeARef(xiGlob, 1, derTheta) * computeSTimeslocVec(xiGlob, 2, parent->qElementAll[element]);
        Vec3 part4 = parent->computeARef(xiGlob, 0, derTheta) * computeSTimeslocVec(xiGlob, 3, parent->qElementAll[element]);
        rf = part1 + part2 + part3 + part4;
      }
      else
        throw MBSimError("Not Implemented for higher derivatives!");

    }

    if (AVers == 1) {
      rf = computeAnodeTimesSTimeslocVec(xiGlob, derXi, derTheta, parent->qElementAll[element]);
    }

    if (AVers == 2) {
      if (derTheta == 0) {
        rf = computeSTimeslocVec(xiGlob, derXi, parent->qElementAll[element]);
      }
    }
    return rf;
  }

  fmatvec::Vec3 FlexibleBody1SReferenceCurveFE::computeBuf(double xiGlob, int derXi, int derTheta) {
    Vec3 vf;
    if (AVers == 0) {
      if (derXi == 0) {
        Vec3 vfLoc = computeSTimeslocVec(xiGlob, 0, parent->uElementAll[element]);
        SqrMat3 ARef = parent->computeARef(xiGlob, 0, derTheta);
        vf = ARef * vfLoc;
      }
      else if (derXi == 1) {
        SqrMat3 ARdxi = parent->computeARef(xiGlob, 1, derTheta);
        Vec3 Suf = computeSTimeslocVec(xiGlob, 0, parent->uElementAll[element]);
        SqrMat3 AR = parent->computeARef(xiGlob, 0, derTheta);
        Vec3 Sudfdxi = computeSTimeslocVec(xiGlob, 1, parent->uElementAll[element]);
        vf = ARdxi * Suf + AR * Sudfdxi;
      }
      else if (derXi == 2) {
        Vec3 part1 = parent->computeARef(xiGlob, 2, derTheta) * computeSTimeslocVec(xiGlob, 0, parent->uElementAll[element]);
        Vec3 part2 = 2. * parent->computeARef(xiGlob, 1, derTheta) * computeSTimeslocVec(xiGlob, 1, parent->uElementAll[element]);
        Vec3 part3 = parent->computeARef(xiGlob, 0, derTheta) * computeSTimeslocVec(xiGlob, 2, parent->uElementAll[element]);
        vf = part1 + part2 + part3;
      }
      else if (derXi == 3) {
        Vec3 part1 = parent->computeARef(xiGlob, 3, derTheta) * computeSTimeslocVec(xiGlob, 0, parent->uElementAll[element]);
        Vec3 part2 = 3. * parent->computeARef(xiGlob, 2, derTheta) * computeSTimeslocVec(xiGlob, 1, parent->uElementAll[element]);
        Vec3 part3 = 3. * parent->computeARef(xiGlob, 1, derTheta) * computeSTimeslocVec(xiGlob, 2, parent->uElementAll[element]);
        Vec3 part4 = parent->computeARef(xiGlob, 0, derTheta) * computeSTimeslocVec(xiGlob, 3, parent->uElementAll[element]);
        vf = part1 + part2 + part3 + part4;
      }
      else
        throw MBSimError("Not Implemented for higher derivatives!");

    }

    if (AVers == 1) {
      vf = computeAnodeTimesSTimeslocVec(xiGlob, derXi, derTheta, parent->uElementAll[element]);
    }

    if (AVers == 2) {
      if (derTheta == 0) {
        vf = computeSTimeslocVec(xiGlob, derXi, parent->uElementAll[element]);
      }
    }
    return vf;
  }

  Vec3 FlexibleBody1SReferenceCurveFE::computeS(int columnEle, double xiGlob, int derXi) {
    Vec3 vec;

    // find local position
    double xiLoc = computeXiLoc(xiGlob);

    // The index of the ansatz function
    int vecInd = columnEle % nodeDirs;

    // Index of the affected ansatz function
    int j = columnEle / nodeDirs;

    vec(vecInd) = computeN(j, xiLoc, derXi);

    return vec;
  }

  Vec3 FlexibleBody1SReferenceCurveFE::computeSTimeslocVec(double xiGlob, int derXi, const Vec & locVec) {
    Vec3 vec;

    // find local position
    double xiLoc = computeXiLoc(xiGlob);

    // compute the vector
    for (int j = 0; j < getNodesSize(); j++) {
      // Run over all Dofs
      const double N = computeN(j, xiLoc, derXi);
      for (int i = 0; i < nodeDirs; i++) {
        //Do the same for each direction
        int ind = j * nodeDirs + i; // find the index of the DoF
        const double val = locVec(ind + 2); // due to global dofs add 2
        vec(i) += N * val;
      }
    }

    return vec;
  }

  Vec3 FlexibleBody1SReferenceCurveFE::computeAnodeTimesSTimeslocVec(double xiGlob, int derXi, int derTheta, const Vec & locVec) {
    Vec3 vec;
    SqrMat3 A;
    //TODO: only implemented for Hermite splines
    A = parent->computeARef(alpha(0), 0, derTheta);

    // find local position
    double xiLoc = computeXiLoc(xiGlob);

    // compute the vector
    for (int j = 0; j < getNodesSize(); j++) {
      // update the A matrix if necessary
      if (j == getNodesSize() / 2)
        A = parent->computeARef(alpha(1), 0, derTheta);

      // Run over all Dofs
      const double N = computeN(j, xiLoc, derXi);
      for (int i = 0; i < nodeDirs; i++) {
        //Do the same for each direction
        int ind = j * nodeDirs + i; // find the index of the DoF
        const double val = locVec(ind + 2); // due to global dofs add 2
        vec += A.col(i) * N * val; // compared to the other approaches the A-Matrix has to be directly evaluated as it can not be taken out of the sum: "\sum A_i N_i q_i" as basically the A now depends not on xi but in i...
      }
    }

    return vec;
  }

  vector<double> FlexibleBody1SReferenceCurveFE::getLagrangeCoeffs(int nodeNo) {
    vector<double> ret;
    if (order == 1) {
      if (nodeNo == 0) {
        ret.push_back(1. / 2.);
        ret.push_back(-1. / 2.);
      }
      else {
        ret.push_back(1. / 2.);
        ret.push_back(1. / 2.);
      }
    }
    if (order == 2) {
      if (nodeNo == 0) {
        ret.push_back(0.);
        ret.push_back(-1. / 2.);
        ret.push_back(1. / 2.);
      }
      else if (nodeNo == 1) {
        ret.push_back(1.);
        ret.push_back(0.);
        ret.push_back(-1.);
      }
      else {
        ret.push_back(0.);
        ret.push_back(1. / 2.);
        ret.push_back(1. / 2.);
      }
    }
    if (order == 3) {
      if (nodeNo == 0) {
        ret.push_back(-1. / 16.);
        ret.push_back(1. / 16.);
        ret.push_back(9. / 16.);
        ret.push_back(-9. / 16.);
      }
      else if (nodeNo == 1) {
        ret.push_back(9. / 16.);
        ret.push_back(-27. / 16.);
        ret.push_back(-9. / 16.);
        ret.push_back(27. / 16.);
      }
      else if (nodeNo == 2) {
        ret.push_back(9. / 16.);
        ret.push_back(27. / 16.);
        ret.push_back(-9. / 16.);
        ret.push_back(-27. / 16.);
      }
      else {
        ret.push_back(-1. / 16.);
        ret.push_back(-1. / 16.);
        ret.push_back(9. / 16.);
        ret.push_back(9. / 16.);
      }
    }
    if (order == 4) {
      if (nodeNo == 0) {
        ret.push_back(0.);
        ret.push_back(1. / 6.);
        ret.push_back(-1. / 6.);
        ret.push_back(-2. / 3.);
        ret.push_back(2. / 3.);
      }
      else if (nodeNo == 1) {
        ret.push_back(0.);
        ret.push_back(-4. / 3.);
        ret.push_back(8. / 3.);
        ret.push_back(4. / 3.);
        ret.push_back(-8. / 3.);
      }
      else if (nodeNo == 2) {
        ret.push_back(1.);
        ret.push_back(0.);
        ret.push_back(-5.);
        ret.push_back(0.);
        ret.push_back(4.);
      }
      else if (nodeNo == 3) {
        ret.push_back(0.);
        ret.push_back(4. / 3.);
        ret.push_back(8. / 3.);
        ret.push_back(-4. / 3.);
        ret.push_back(-8. / 3.);
      }
      else {
        ret.push_back(0.);
        ret.push_back(-1. / 6.);
        ret.push_back(-1. / 6.);
        ret.push_back(2. / 3.);
        ret.push_back(2. / 3.);
      }
    }
    return ret;
  }

  vector<double> FlexibleBody1SReferenceCurveFE::getHermiteCoeffs(int ansatzFunctionNo) {
    vector<double> ret;
    if (order == 3) {
      double le = alpha(1) - alpha(0);
      if (ansatzFunctionNo == 0) {
        // position level left node
        ret.push_back(1. / 2.);
        ret.push_back(-3. / 4.);
        ret.push_back(0);
        ret.push_back(1. / 4.);
      }
      else if (ansatzFunctionNo == 1) {
        //first derivative level left node
        ret.push_back(le / 8.);
        ret.push_back(-le / 8.);
        ret.push_back(-le / 8.);
        ret.push_back(le / 8.);
      }
      else if (ansatzFunctionNo == 2) {
        // position level right node
        ret.push_back(1. / 2.);
        ret.push_back(3. / 4.);
        ret.push_back(0.);
        ret.push_back(-1. / 4.);
      }
      else {
        // first derivative level right node
        ret.push_back(-le / 8.);
        ret.push_back(-le / 8.);
        ret.push_back(le / 8.);
        ret.push_back(le / 8.);
      }
    }
    if (order == 5) {
      double le = alpha(1) - alpha(0);
      if (ansatzFunctionNo == 0) {
        // position level left node
        ret.push_back(1. / 2.);
        ret.push_back(-15. / 16.);
        ret.push_back(0.);
        ret.push_back(5. / 8.);
        ret.push_back(0.);
        ret.push_back(-3. / 16.);
      }
      else if (ansatzFunctionNo == 1) {
        //first derivative level left node
        ret.push_back(le * 5. / 32.);
        ret.push_back(-le * 7. / 32.);
        ret.push_back(-le * 3. / 16.);
        ret.push_back(le * 5. / 16.);
        ret.push_back(le * 1. / 32.);
        ret.push_back(-le * 3. / 32.);
      }
      else if (ansatzFunctionNo == 2) {
        //second derivative level left node
        ret.push_back(le * le / 64.);
        ret.push_back(-le * le / 64.);
        ret.push_back(-le * le / 32.);
        ret.push_back(le * le / 32.);
        ret.push_back(le * le / 64.);
        ret.push_back(-le * le / 64.);
      }
      else if (ansatzFunctionNo == 3) {
        // position level right node
        ret.push_back(1. / 2.);
        ret.push_back(15. / 16.);
        ret.push_back(0.);
        ret.push_back(-5. / 8.);
        ret.push_back(0.);
        ret.push_back(3. / 16.);
      }
      else if (ansatzFunctionNo == 4) {
        //first derivative level right node
        ret.push_back(-le * 5. / 32.);
        ret.push_back(-le * 7. / 32.);
        ret.push_back(le * 3. / 16.);
        ret.push_back(le * 5. / 16.);
        ret.push_back(-le * 1. / 32.);
        ret.push_back(-le * 3. / 32.);
      }
      else {
        //second derivative level right node
        ret.push_back(le * le / 64.);
        ret.push_back(le * le / 64.);
        ret.push_back(-le * le / 32.);
        ret.push_back(-le * le / 32.);
        ret.push_back(le * le / 64.);
        ret.push_back(le * le / 64.);
      }
    }

    if (ret.size() == 0)
      throw MBSimError("Hermite Coefficients only implemented for order 3 and order 5");

    return ret;
  }

  fmatvec::Mat3xV FlexibleBody1SReferenceCurveFE::computeP(double xiGlob, int derXi) {
    Mat3xV P(freeDoFs.size(), INIT, 0.);

    // Reference dofs
    P.set(0, computer(xiGlob, 1 + derXi, 0));
    P.set(1, computer(xiGlob, derXi, 1));

    // local dofs
    for (size_t qIndLoc = 2; qIndLoc < freeDoFs.size(); qIndLoc++) {
      P.set(qIndLoc, computeB(freeDoFs[qIndLoc], xiGlob, derXi, 0));
    }

    return P;
  }

  fmatvec::Mat3xV FlexibleBody1SReferenceCurveFE::computedPdqk(double xiGlob, int dofDirLocal) {
    Mat3xV dPdqk(freeDoFs.size(), INIT, 0.);

    //s
    if (dofDirLocal == 0) {
      //derivative is zero here
    }
    //Theta
    else if (dofDirLocal == 1) {
      //reference DOFs
      dPdqk.set(0, computer(xiGlob, 1, 1));
      dPdqk.set(1, computer(xiGlob, 0, 2));

      // local dofs
      for (size_t qIndLoc = 2; qIndLoc < freeDoFs.size(); qIndLoc++) {
        dPdqk.set(qIndLoc, computeB(freeDoFs[qIndLoc], xiGlob, 0, 1));
      }
    }
    else {
      dPdqk.set(0, computeB(dofDirLocal, xiGlob, 1, 0));
      dPdqk.set(1, computeB(dofDirLocal, xiGlob, 0, 1));

      //Rest is zero --> TODO: use this in multiplication!!!
    }

    return dPdqk;
  }

  fmatvec::SymMatV FlexibleBody1SReferenceCurveFE::integratePTP() {
    funcPTP func(this);
    GaussLegendreQuadrature<SymMatV> quad(&func, freeDoFs.size(), 1, 5);
    quad.setSubIntegrals(2);

    SymMatV ret(quad.integrate(borders[0], borders[1]));
    for (size_t i = 2; i < borders.size() - 1; i += 2) {
      ret += quad.integrate(borders[i], borders[i + 1]);
    }
    return ret;
  }

  fmatvec::SqrMatV FlexibleBody1SReferenceCurveFE::integratePTdPdxi() {
    funcPTdPdxi func(this);
    GaussLegendreQuadrature<SqrMatV> quad(&func, freeDoFs.size(), 1, 5);
    quad.setSubIntegrals(2);

    SqrMatV ret(quad.integrate(borders[0], borders[1]));
    for (size_t i = 2; i < borders.size() - 1; i += 2) {
      ret += quad.integrate(borders[i], borders[i + 1]);
    }
    return ret;
  }

  fmatvec::SqrMatV FlexibleBody1SReferenceCurveFE::integratePTdPdt() {
    funcPTdPdt func(this);
    GaussLegendreQuadrature<SqrMatV> quad(&func, freeDoFs.size(), 1, 5);
    quad.setSubIntegrals(2);

    SqrMatV ret(quad.integrate(borders[0], borders[1]));
    for (size_t i = 2; i < borders.size() - 1; i += 2) {
      ret += quad.integrate(borders[i], borders[i + 1]);
    }
    return ret;
  }

  fmatvec::SqrMatV FlexibleBody1SReferenceCurveFE::integratePTdPdqk(int qInd) {
    funcPTdPdqk func(this);
    func.setqInd(qInd);

    GaussLegendreQuadrature<SqrMatV> quad(&func, freeDoFs.size(), 1, 5);
    quad.setSubIntegrals(2);

    SqrMatV ret(quad.integrate(borders[0], borders[1]));
    for (size_t i = 2; i < borders.size() - 1; i += 2) {
      ret += quad.integrate(borders[i], borders[i + 1]);
    }
    return ret;
  }

  double FlexibleBody1SReferenceCurveFE::integrateForWgamma(int dofDirOfElement) {
    funcForWgamma func(this);
    func.setDofDirOfElement(dofDirOfElement);

    GaussLegendreQuadrature<Vec1> quad(&func, 1, 1, 5);
    quad.setSubIntegrals(2);

    Vec1 ret(quad.integrate(borders[0], borders[1]));
    for (size_t i = 2; i < borders.size() - 1; i += 2) {
      ret += quad.integrate(borders[i], borders[i + 1]);
    }
    return ret(0);
  }

  double FlexibleBody1SReferenceCurveFE::integrateForWn(int qIndLoc) {
    funcForWn func(this);
    func.setqIndLoc(qIndLoc);

    GaussLegendreQuadrature<Vec1> quad(&func, 1, 1, 5);
    quad.setSubIntegrals(2);

    Vec1 ret(quad.integrate(borders[0], borders[1]));
    for (size_t i = 2; i < borders.size() - 1; i += 2) {
      ret += quad.integrate(borders[i], borders[i + 1]);
    }
    return ret(0);
  }

  double FlexibleBody1SReferenceCurveFE::integrateForWb(int qIndLoc) {
    funcForWb func(this);
    func.setqIndLoc(qIndLoc);

    GaussLegendreQuadrature<Vec1> quad(&func, 1, 1, 5);
    quad.setSubIntegrals(2);

    Vec1 ret(quad.integrate(borders[0], borders[1]));
    for (size_t i = 2; i < borders.size() - 1; i += 2) {
      ret += quad.integrate(borders[i], borders[i + 1]);
    }
    return ret(0);
  }

  double FlexibleBody1SReferenceCurveFE::computeeps(double xiGlob) {

    double eps = 0.;

    throw MBSimError("Not implemented!");
    return eps;
  }

  double FlexibleBody1SReferenceCurveFE::computedepsdqk(int localDofDir, double xiGlob) {

    double depsdqk = 0.;

    throw MBSimError("Not implemented!");
    return depsdqk;

    //TODO: The following is wrong!!!

//    if (localDofDir < 2) {
//      return 0.;
//    }
//    else if ((localDofDir - 2) % nodeDirs == 0) {
//      // tangential direction
//      return computeS(localDofDir - 2, xiGlob, 1)(0);
//    }
//    else if ((localDofDir - 2) % nodeDirs == 1) {
//      // normal direction
//      return computeS(localDofDir - 2, xiGlob, 0)(1);
//    }
//    else if (localDofDir > 1 and (localDofDir - 2) % nodeDirs == 2) {
//      // binormal direction
//      return 0.;
//    }
//    else {
//      // kappa of overlaid curve --> general formula (should not be used/ called) just stated for sake of completeness
//      throw MBSimError("This is a case that should never be reached. Please ask your system administrator for help.");
//      //TODO: maybe exclude the option here, that it depends on the binormal deflection? --> Could speed up the process
//      Vec3 nR = parent->computenRef(xiGlob, 0, 0);
//      return nR.T() * computeB(localDofDir, xiGlob, 2, 0);
//    }
  }

  double FlexibleBody1SReferenceCurveFE::computeKappan(double xiGlob) {

    Vec3 nR = parent->computenRef(xiGlob, 0, 0);
    double kappanRef = nR.T() * computer(xiGlob, 2, 0);

    return kappanRef;
  }

  double FlexibleBody1SReferenceCurveFE::computeKappab(double xiGlob) {
    double kappabRef = parent->b.T() * computer(xiGlob, 2, 0);
    return kappabRef;
  }

  double FlexibleBody1SReferenceCurveFE::computedKappandqk(double xiGlob, int qIndLocal) {
    if (AVers == 0) {
      if (qIndLocal == 1) {
        // only reference curve depends on theta
        Vec3 dtdTheta = parent->computerRef(xiGlob, 1, 1);
        double part1 = crossProduct(parent->b, dtdTheta).T() * computer(xiGlob, 2, 0);

        Vec3 nR = parent->computenRef(xiGlob, 0, 0);
        double part2 = nR.T() * computer(xiGlob, 2, 1);

        return part1 + part2;
      }
      else if (qIndLocal > 1 and (qIndLocal - 2) % nodeDirs == 0) {
        // tangential direction
        return -2. * computeS(qIndLocal - 2, xiGlob, 1)(0);
      }
      else if (qIndLocal > 1 and (qIndLocal - 2) % nodeDirs == 1) {
        // normal direction
        double part1 = computeS(qIndLocal - 2, xiGlob, 2)(1);
        double part2 = computeS(qIndLocal - 2, xiGlob, 0)(1);
        return part1 - part2;
      }
      else if (qIndLocal > 1 and (qIndLocal - 2) % nodeDirs == 2) {
        // binormal direction
        return 0.;
      }
      else {

        // kappa of overlaid curve --> general formula (should not be used/ called) just stated for sake of completeness
        throw MBSimError("This is a case that should never be reached. Please ask your system administrator for help.");
        //TODO: maybe exclude the option here, that it depends on the binormal deflection? --> Could speed up the process
        Vec3 nR = parent->computenRef(xiGlob, 0, 0);
        return nR.T() * computeB(qIndLocal, xiGlob, 2, 0);
      }

      return 0.;
    }

    if (AVers == 1) {
      if (qIndLocal == 1) {
        // only reference curve depends on theta
        Vec3 dtdTheta = parent->computerRef(xiGlob, 1, 1);
        double part1 = crossProduct(parent->b, dtdTheta).T() * computer(xiGlob, 2, 0);

        Vec3 nR = parent->computenRef(xiGlob, 0, 0);
        double part2 = nR.T() * computer(xiGlob, 2, 1);

        return part1 + part2;
      }
      else if (qIndLocal > 1 and (qIndLocal - 2) % nodeDirs == 1) {
        double xiNode = computeXiOfDOF(qIndLocal);
        Vec3 nR = parent->computenRef(xiGlob, 0, 0);
        return nR.T() * parent->computeARef(xiNode, 0, 0) * computeS(qIndLocal - 2, xiGlob, 2);
      }

      return 0.;
    }

    if (AVers == 2) {
      if (qIndLocal == 1) {
        // only reference curve depends on theta
        Vec3 dtdTheta = parent->computerRef(xiGlob, 1, 1);
        double part1 = crossProduct(parent->b, dtdTheta).T() * computer(xiGlob, 2, 0);

        Vec3 nR = parent->computenRef(xiGlob, 0, 0);
        double part2 = nR.T() * computer(xiGlob, 2, 1);

        return part1 + part2;
      }
      else if (qIndLocal > 1 and (qIndLocal - 2) % nodeDirs <= 1) {
        // tangential direction
        Vec3 nR = parent->computenRef(xiGlob, 0, 0);
        return scalarProduct(nR, computeS(qIndLocal - 2, xiGlob, 2));
      }

      return 0.;
    }

    return 0.;

  }

  double FlexibleBody1SReferenceCurveFE::computedKappabdqk(double xiGlob, int qIndLocal) {
    // REMARK: it is the same for all AVers!
    if (qIndLocal == 1) {
      return parent->b.T() * computer(xiGlob, 2, 1);
    }
    else if (qIndLocal > 1 and (qIndLocal - 2) % nodeDirs == 0) {
      // tangential direction
      return 0.;
    }
    else if (qIndLocal > 1 and (qIndLocal - 2) % nodeDirs == 1) {
      // normal direction
      return 0.;
    }
    else if (qIndLocal > 1 and (qIndLocal - 2) % nodeDirs == 2) {
      // binormal direction
      return computeS(qIndLocal - 2, xiGlob, 2)(2);
    }
    else {

      // kappa of overlaid curve --> general formula (should not be used/ called) just stated for sake of completeness
      throw MBSimError("This is a case that should never be reached. Please ask your system administrator for help.");
      //TODO: maybe exclude the option here, that it depends on the binormal deflection? --> Could speed up the process
      Vec3 nR = parent->computenRef(xiGlob, 0, 0);
      return nR.T() * computeB(qIndLocal, xiGlob, 2, 0);
    }

    return 0.;
  }

  double FlexibleBody1SReferenceCurveFE::dWndqk(double xiGlob, int qInd) {
    funcForWn func(this);
    func.setqIndLoc(qInd);

    return func(xiGlob)(0);
  }

  void FlexibleBody1SReferenceCurveFE::updateIntegrationBorders(double t) {
    borders.clear();

//    double pulleyDistance = parent->dPulley;
//    double theta = parent->getq()(1);
    fmatvec::VecV C1Pos(parent->refCurve->computeC1Positions(parent->getq()));
//    double rS, phi, endSecondary, endPushPart, endPrimary, endLoosePart;
//    computeBeltNumbers(rP, theta, pulleyDistance, rS, phi, endSecondary, endPushPart, endPrimary, endLoosePart);

    borders.push_back(alpha(0));

    for (int i = 0; i < C1Pos.size(); i++) {
      if (alpha(0) < C1Pos(i) and alpha(1) > C1Pos(i)) {
        borders.push_back(C1Pos(i));
        borders.push_back(C1Pos(i) + macheps);
      }
    }

    borders.push_back(alpha(1));
  }

  double FlexibleBody1SReferenceCurveFE::computeXiOfDOF(int dofDirLocal) {
    if (dofDirLocal < 2) {
      throw MBSimError("This is not a valid local dof direction. It must be bigger than 1!");
    }
    //TODO: only implemented for Hermite splines
    if (dofDirLocal < dofDirs.size() / 2)
      return alpha(0);

    return alpha(1);
  }
}

