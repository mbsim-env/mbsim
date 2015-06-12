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
 * Created on: Sep 06, 2013
 * Contact: kilian.grundl@gmail.com
 */

#ifndef FINITE_ELEMENT_1S_REFERENCE_CURVE_FUNCTIONS_H_
#define FINITE_ELEMENT_1S_REFERENCE_CURVE_FUNCTIONS_H_

#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1S_reference_curve.h"

namespace MBSimFlexibleBody {

  class funcPTP : public MBSim::Function<fmatvec::SymMatV(double)> {
    public:
      funcPTP(FlexibleBody1SReferenceCurveFE * refBody) :
          refBody(refBody) {
      }

      ~funcPTP() {
      }

      fmatvec::SymMatV operator()(const double & xi) {
        fmatvec::Mat3xV P = refBody->computeP(xi, 0);
        fmatvec::SymMatV PTP(P.cols(), INIT, 0.);
        for (int i = 0; i < PTP.size(); i++) {
          Vec3 coli = P.col(i);
          for (int j = i; j < PTP.size(); j++) {
            PTP(i, j) = scalarProduct(coli, P.col(j));
          }
        }
        return PTP;
      }
    private:
      FlexibleBody1SReferenceCurveFE * refBody;
  };

  class funcPTdPdxi : public MBSim::Function<fmatvec::SqrMatV(double)> {
    public:
      funcPTdPdxi(FlexibleBody1SReferenceCurveFE * refBody) :
          refBody(refBody) {
      }

      ~funcPTdPdxi() {
      }

      fmatvec::SqrMatV operator()(const double & xi) {
        fmatvec::Mat3xV P = refBody->computeP(xi, 0);
        fmatvec::Mat3xV dPdxi = refBody->computeP(xi, 1);

        SqrMatV ret(P.cols(), INIT, 0.);
        for (int i = 0; i < P.cols(); i++) {
          Vec3 colP = P.col(i);
          for (int j = 0; j < dPdxi.cols(); j++)
            ret(i, j) = scalarProduct(colP, dPdxi.col(j));
        }

        return ret;
      }
    private:
      FlexibleBody1SReferenceCurveFE * refBody;
  };

  class funcPTdPdt : public MBSim::Function<fmatvec::SqrMatV(double)> {
    public:
      funcPTdPdt(FlexibleBody1SReferenceCurveFE * refBody) :
          refBody(refBody) {
      }

      ~funcPTdPdt() {
      }

      fmatvec::SqrMatV operator()(const double & xi) {
        fmatvec::Mat3xV P = refBody->computeP(xi, 0);
        fmatvec::Mat3xV dPdt(P.cols(), INIT, 0.);

        SqrMatV ret(P.cols(), INIT, 0.);

        /* Compute dPdt*/
        // Theta
        Vec q = refBody->getq();
        dPdt = refBody->computedPdqk(xi, 1) * q(1);

        //flexible DoFs
        for (int qInd = 2; qInd < P.cols(); qInd++) {
          dPdt.add(0, q(qInd) * refBody->computeB(qInd, xi, 1, 0));
          dPdt.add(1, q(qInd) * refBody->computeB(qInd, xi, 0, 1));

        }

        /* Multiplication with P^T*/
        for (int i = 0; i < P.cols(); i++) {
          Vec3 colP = P.col(i);
          for (int j = 0; j < dPdt.cols(); j++)
            ret(i, j) += scalarProduct(colP, dPdt.col(j));
        }

        return ret;
      }
    private:
      FlexibleBody1SReferenceCurveFE * refBody;
  };

  class funcPTdPdqk : public MBSim::Function<fmatvec::SqrMatV(double)> {
    public:
      funcPTdPdqk(FlexibleBody1SReferenceCurveFE * refBody) :
          refBody(refBody), qInd(-1) {
      }

      void setqInd(int qInd_) {
        qInd = qInd_;
      }

      ~funcPTdPdqk() {
      }

      fmatvec::SqrMatV operator()(const double & xi) {
        fmatvec::Mat3xV P = refBody->computeP(xi, 0);
        fmatvec::Mat3xV dPdqk = refBody->computedPdqk(xi, qInd);
        fmatvec::SqrMatV ret(P.cols(), INIT, 0.);

        // for the flexible DoFs dPdqk is zero...
        int endCol = dPdqk.cols();
        if (qInd > 1)
          endCol = 2;

        for (int i = 0; i < P.cols(); i++) {
          Vec3 col = P.col(i);
          for (int j = 0; j < endCol; j++)
            ret(i, j) = scalarProduct(col, dPdqk.col(j));
        }

        return ret;
      }
    private:
      FlexibleBody1SReferenceCurveFE * refBody;
      int qInd;
  };

  /*!
   * \brief this function returns basically three values. As these share some values it makes sense to not compute them all the time again...
   */
  class funcForWgamma : public MBSim::Function<fmatvec::Vec1(double)> {
    public:
      funcForWgamma(FlexibleBody1SReferenceCurveFE * refBody) :
          refBody(refBody), dofDirOfElement(-1) {
      }

      void setDofDirOfElement(int qInd_) {
        dofDirOfElement = qInd_;
      }

      ~funcForWgamma() {
      }

      fmatvec::Vec1 operator()(const double & xi) {
        fmatvec::Vec1 retVal;

        Vec3 drdxi = refBody->computer(xi, 1, 0);
        double drdxipw2 = scalarProduct(drdxi, drdxi);
//        Vec3 drdxidxi = refBody->computer(xi, 2, 0);
//        double drdxidxipw2 = scalarProduct(drdxidxi, drdxidxi);
        Vec3 drdxidqk = refBody->computedrdqk(xi, 1, dofDirOfElement);
//        Vec3 drdxidxidqk = refBody->computedrdqk(xi, 2, qInd);

// Part for W_gamma (elongation deformation)
// Full influence
        double sqrdrdxipw2 = sqrt(drdxipw2);
        double ForW_gamma = scalarProduct(drdxi, drdxidqk);
        retVal(0) = (sqrdrdxipw2 - 1.) * ForW_gamma / sqrdrdxipw2;

        return retVal;
      }
    private:
      FlexibleBody1SReferenceCurveFE * refBody;
      int dofDirOfElement;
  };

  class funcForWn : public MBSim::Function<fmatvec::Vec1(double)> {
    public:
      funcForWn(FlexibleBody1SReferenceCurveFE * refBody) :
          refBody(refBody), qIndLoc(-1) {
      }

      void setqIndLoc(int qIndLoc_) {
        qIndLoc = qIndLoc_;
      }

      ~funcForWn() {
      }

      fmatvec::Vec1 operator()(const double & xi) {
        double kappan = refBody->computeKappan(xi);
        double dkappandqk = refBody->computedKappandqk(xi, qIndLoc);
        return Vec1(INIT, kappan * dkappandqk);
      }
    private:
      FlexibleBody1SReferenceCurveFE * refBody;
      int qIndLoc;
  };

  class funcForWb : public MBSim::Function<fmatvec::Vec1(double)> {
    public:
      funcForWb(FlexibleBody1SReferenceCurveFE * refBody) :
          refBody(refBody), qIndLoc(-1) {
      }

      void setqIndLoc(int qIndLoc_) {
        qIndLoc = qIndLoc_;
      }

      ~funcForWb() {
      }

      fmatvec::Vec1 operator()(const double & xi) {
        double kappab = refBody->computeKappab(xi);
        double dkappabdqk = refBody->computedKappabdqk(xi, qIndLoc);
        return Vec1(INIT, kappab * dkappabdqk);
      }
    private:
      FlexibleBody1SReferenceCurveFE * refBody;
      int qIndLoc;
  };
}

#endif /* FINITE_ELEMENT_1S_REFERENCE_CURVE_FUNCTIONS_H_ */
