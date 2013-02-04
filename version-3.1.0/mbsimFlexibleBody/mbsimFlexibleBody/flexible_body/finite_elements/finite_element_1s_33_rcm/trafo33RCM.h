/* Copyright (C) 2004-2011 MBSim Development Team
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

#ifndef TRAFO33RCM_H_
#define TRAFO33RCM_H_

#include "fmatvec.h"
#include <mbsim/numerics/nonlinear_algebra/multi_dimensional_newton_method.h>
#include "mbsim/utils/function.h"
#include "mbsimFlexibleBody/pointer.h"
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_33_rcm/trafo33RCM.h"
#include "mbsimFlexibleBody/utils/revcardan.h"
#include "mbsim/utils/nonlinear_algebra.h"
#include <mbsim/numerics/nonlinear_algebra/multi_dimensional_newton_method.h>
#include "mbsim/utils/eps.h"
#include "mbsim/mbsim_event.h"

namespace MBSimFlexibleBody {
  /***
   * \brief computation of FiniteElement1s33RCM position coordinates (cp-angles and bending)
   * \author Thorsten Schindler
   * \date 2009-04-24 initial commit (Thorsten Schindler)
   *
   * \todo: use FmatVec2.0!!!
   */
  class PositionFunction : public MBSim::Function1<fmatvec::Vec11, fmatvec::Vec11> {
    public:
      /**
       * \brief constructor
       * \param reversed Cardan-Object
       * \param length of FiniteElement1s33RCM
       * \param angles of COG of rigid left-right connection
       * \param global bending parameters
       * \param transpose of difference of right and left beam position
       */
      PositionFunction(RevCardanPtr angle_, double l0_, const fmatvec::Vec3& pL_, const fmatvec::Vec3& pR_, double cL1_, double cR1_, double cL2_, double cR2_, const fmatvec::RowVec3& rRrLmH_);

      /**
       * \brief destructor 
       */
      virtual ~PositionFunction();

      /* INHERITED INTERFACE */
      fmatvec::Vec11 operator()(const fmatvec::Vec11& pos, const void * = NULL);
      /***************************************************/

    private:
      /** 
       * \brief reversed Cardan-Object
       */
      RevCardanPtr angle;

      /** 
       * \brief length of FiniteElement1s33RCM 
       */
      double l0;

      /** 
       * \brief angles of COG of rigid left connection
       */
      fmatvec::Vec3 pL;

      /**
       * \brief angles of COG of rigid right connection
       */
      fmatvec::Vec3 pR;

      /** 
       * \brief global bending parameters
       */
      double cL1, cR1, cL2, cR2;

      /** 
       * \brief transpose of difference of right and left beam position
       */
      fmatvec::RowVec3 rRrLmH;
  };

  /**
   * \brief computation of FiniteElement1s33RCM position JACOBIAN (cp-angles and bending)
   * \author Thorsten Schindler
   * \date 2009-04-24 initial commit (Thorsten Schindler)
   */
  class PositionJacobian : public MBSim::NJacobianFunction<11> {
    public:
      /**
       * \brief constructor
       * \param reversed Cardan-Object
       * \param length of FiniteElement1s33RCM
       * \param transpose of difference of right and left beam position
       * \param differentiated centre angles with respect to bending coordinates
       */
      PositionJacobian(RevCardanPtr angle_, double l0_, const fmatvec::RowVec3 &rRrLmH_, const fmatvec::Mat3x11 &pSbE_);

      /**
       * \brief destructor
       */
      virtual ~PositionJacobian();

      /* INHERITED INTERFACE */
      fmatvec::SqrMat11 operator()(const fmatvec::Vec11 &pos, const void * = NULL);
      /***************************************************/

    private:
      /** 
       * \brief reversed Cardan-Object
       */
      RevCardanPtr angle;

      /** 
       * \brief length of FiniteElement1s33RCM 
       */
      double l0;

      /** 
       * \brief transpose of difference of right and left beam position
       */
      fmatvec::RowVec3 rRrLmH;

      /** 
       * \brief differentiated centre angles with respect to bending coordinates
       *
       * It is a constant 3x11 Matrix with [1 0 0 0 ...
       *                                    0 1 0 0 ...
       *                                    0 0 1 0 ...]
       */
      fmatvec::Mat3x11 pSbE;
  };
  /*******************************************************************/

  /*! 
   * \brief transformation of coordinates for FiniteElement1s33RCM
   * \author Thorsten Schindler
   * \date 2009-04-24 initial commit (Thorsten Schindler)
   */
  class Trafo33RCM {
    public:
      /**
       * \brief constructor
       */
      Trafo33RCM(RevCardanPtr angle_, double l0_);

      /**
       * \brief destructor
       */
      virtual ~Trafo33RCM();

      /* GETTER / SETTER */
      const fmatvec::SqrMat16& getJIG() const;
      const fmatvec::SqrMat16& getJIGt() const;
      const fmatvec::Vec3& getpS() const;
      const fmatvec::Vec3& getpSt() const;
      const fmatvec::Vec3& gettS() const;
      const fmatvec::Vec3& getnS() const;
      const fmatvec::Vec3& getbS() const;
      const fmatvec::RowVec3& getnSH() const;
      const fmatvec::RowVec3& getbSH() const;
      const fmatvec::Vec3& getntilS() const;
      const fmatvec::Vec3& getbtilS() const;
      const fmatvec::RowVec3& getntilSH() const;
      const fmatvec::RowVec3& getbtilSH() const;
      const fmatvec::Vec3& gettSt() const;
      const fmatvec::Vec3& getnSt() const;
      const fmatvec::Vec3& getbSt() const;
      const fmatvec::RowVec3& gettStH() const;
      const fmatvec::RowVec3& getnStH() const;
      const fmatvec::RowVec3& getbStH() const;
      const fmatvec::RowVec3& getntilStH() const;
      const fmatvec::RowVec3& getbtilStH() const;
      const fmatvec::SqrMat3& gettSpS() const;
      const fmatvec::SqrMat3& getnSpS() const;
      const fmatvec::SqrMat3& getbSpS() const;
      const fmatvec::SqrMat3& getntilSpS() const;
      const fmatvec::SqrMat3& getbtilSpS() const;
      const fmatvec::SqrMat3& gettSpSt() const;
      const fmatvec::SqrMat3& getnSpSt() const;
      const fmatvec::SqrMat3& getbSpSt() const;
      const fmatvec::SqrMat3& getntilSpSt() const;
      const fmatvec::SqrMat3& getbtilSpSt() const;

      const fmatvec::Vec16& getqIt() const;
      const double& getepstil() const;
      const double& getepstilt() const;
      const fmatvec::Vec3& getrS() const;
      const fmatvec::Vec3& getrSt() const;
      const double& getk0() const;
      const double& getk0t() const;
      const fmatvec::Vec11& getbe() const;
      const fmatvec::Vec11& getbet() const;

      const double& getxintil() const;
      const double& getxibtil() const;
      const double& getetantil() const;
      const double& getetabtil() const;
      const double& getxintilt() const;
      const double& getxibtilt() const;
      const double& getetantilt() const;
      const double& getetabtilt() const;
      const fmatvec::SqrMat4& getV() const;
      /***************************************************/

      /**
       * \brief compute the internal coordinates
       * \param global coordinates
       */
      void computeqI(const fmatvec::Vec16 & qG);

      /**
       * \brief compute the Jacobian of the trafo 
       * \param global coordinates
       */
      void computeJIG(const fmatvec::Vec16& qG);

      /**
       * \brief compute the internal state
       * \param global coordinates
       * \param global velocities
       */
      void computezI(const fmatvec::Vec16& qG, const fmatvec::Vec16& qGt);

      /**
       * \brief compute the internal time differentiated COSY 
       * \param global coordinates
       * \param global velocities
       */
      void computeCOSYt(const fmatvec::Vec16& qG, const fmatvec::Vec16& qGt);

      /**
       * \brief compute the internal coordinates and velocities as well as JIGt 
       * \param global coordinates
       * \param global velocities
       */
      void computeTrafo(const fmatvec::Vec16& qG, const fmatvec::Vec16& qGt);

    private:
      /** 
       * \brief reversed Cardan-Object
       */
      RevCardanPtr angle;

      /** 
       * \brief length of FiniteElement1s33RCM and powers
       */
      double l0, l0h2, l0h3, l0h4, l0h5;
      double xstar, xstarh2, xstarh3;

      /** 
       * \brief internal coordinates of the FiniteElement1s33RCM
       */
      double epstil, k0;
      fmatvec::Vec3 rS, pS;

      /** 
       * \brief sum of right and left translational vector as well as transpose of the difference
       */
      fmatvec::Vec3 rRrLp;
      fmatvec::RowVec3 rRrLmH;

      /** 
       * \brief bending coordinates
       *
       * be = (pS0, pS1, pS2, dL1, dR1, betaL1, betaR1, dL2, dR2, betaL2, betaR2)
       */
      fmatvec::Vec11 be;

      /** 
       * \brief COSY definitions 
       */
      fmatvec::Vec3 tS, nS, bS, ntilS, btilS;
      fmatvec::RowVec3 nSH, bSH, ntilSH, btilSH;
      fmatvec::SqrMat3 tSpS, nSpS, bSpS, ntilSpS, btilSpS;
      double xibtil, xintil, etabtil, etantil;

      /** 
       * \brief system matrix and right hand side for computebe() 
       */
      fmatvec::Mat11x27 SMRHS_Jac;

      /** 
       * \brief derivative of coefficients of bending polynomials w with respect to bending coordinates
       */
      fmatvec::SqrMat4 V;

      /** 
       * \brief delta matrices for computebe()
       */
      fmatvec::Mat3x16 drRdrLp, drRdrLm;

      fmatvec::Mat3x11 pSbE;

      /**
       * \brief COSY definitions for computebe() 
       */
      fmatvec::Mat8x9 SMRHS;
      fmatvec::Mat3x11 nSbE, bSbE, ntilSbE, btilSbE;
      fmatvec::RowVec11 xibtilbE, xintilbE, etabtilbE, etantilbE;

      /** 
       * \brief derivative of be and COSY with respect to qG
       */
      fmatvec::Mat11x16 beqG;
      fmatvec::Mat3x16 tSqG, nSqG, bSqG, ntilSqG, btilSqG;
      fmatvec::RowVec16 xintilqG, xibtilqG, etantilqG, etabtilqG;

      /** 
       * \brief Jacobian and differentiated Jacobian
       */
      fmatvec::SqrMat16 JIG, JIGt;

      /** 
       * \brief derivative of k0, epstilt, qI, rS, be and COSY with respect to time
       */
      double k0t, epstilt;
      fmatvec::Vec16 qIt;
      fmatvec::Vec11 bet;
      fmatvec::Vec3 rSt, pSt, tSt, nSt, bSt;
      fmatvec::RowVec3 tStH, nStH, bStH, ntilStH, btilStH;
      fmatvec::SqrMat3 tSpSt, nSpSt, bSpSt, ntilSpSt, btilSpSt;
      double xibtilt, xintilt, etabtilt, etantilt;

      /**
       * \brief computes preliminaries 
       * \param global coordinates
       */
      void computeprelim(const fmatvec::Vec16& qG);

      /**
       * \brief compute the initial value for computebe() 
       * \param global coordinates
       */
      fmatvec::Vec computes0(const fmatvec::Vec16& qG);

      /**
       * \brief compute the angle and bending positions
       * \param global coordinates
       */
      void computebe(const fmatvec::Vec16& qG);

      /**
       * \brief compute the COSY
       */
      void computeCOSY();

      /**
       * \brief compute the CP, prolongation and torsion
       * \param global coordinates
       */
      void computerSepstk0(const fmatvec::Vec16& qG);

      /**
       * \brief compute delta matrix for right and left beam end 
       */
      void computedrRdrL();

      /**
       * \brief compute the derivative of coefficients of bending polynomials w with respect to bending coordinates 
       */
      void computeV();

      /**
       * \brief compute the derivative of bE with respect to qG
       */
      void computebeqG();

      /**
       * \brief compute the derivative of COSY with respect to qG
       */
      void computeCOSYqG();

      /**
       * \brief compute the derivative of COSY with respect to time
       */
      void computeCOSYt();

      /**
       * \brief compute the derivative of JIG with respect to time
       * \param global velocities
       */
      void computeJIGt(const fmatvec::Vec16& qGt);
  };

  inline const fmatvec::SqrMat16& Trafo33RCM::getJIG() const {
    return JIG;
  }
  inline const fmatvec::SqrMat16& Trafo33RCM::getJIGt() const {
    return JIGt;
  }
  inline const fmatvec::Vec3& Trafo33RCM::getpS() const {
    return pS;
  }
  inline const fmatvec::Vec3& Trafo33RCM::getpSt() const {
    return pSt;
  }
  inline const fmatvec::Vec3& Trafo33RCM::gettS() const {
    return tS;
  }
  inline const fmatvec::Vec3& Trafo33RCM::getnS() const {
    return nS;
  }
  inline const fmatvec::Vec3& Trafo33RCM::getbS() const {
    return bS;
  }
  inline const fmatvec::RowVec3& Trafo33RCM::getnSH() const {
    return nSH;
  }
  inline const fmatvec::RowVec3& Trafo33RCM::getbSH() const {
    return bSH;
  }
  inline const fmatvec::Vec3& Trafo33RCM::getntilS() const {
    return ntilS;
  }
  inline const fmatvec::Vec3& Trafo33RCM::getbtilS() const {
    return btilS;
  }
  inline const fmatvec::RowVec3& Trafo33RCM::getntilSH() const {
    return ntilSH;
  }
  inline const fmatvec::RowVec3& Trafo33RCM::getbtilSH() const {
    return btilSH;
  }
  inline const fmatvec::Vec3& Trafo33RCM::gettSt() const {
    return tSt;
  }
  inline const fmatvec::Vec3& Trafo33RCM::getnSt() const {
    return nSt;
  }
  inline const fmatvec::Vec3& Trafo33RCM::getbSt() const {
    return bSt;
  }
  inline const fmatvec::RowVec3& Trafo33RCM::gettStH() const {
    return tStH;
  }
  inline const fmatvec::RowVec3& Trafo33RCM::getnStH() const {
    return nStH;
  }
  inline const fmatvec::RowVec3& Trafo33RCM::getbStH() const {
    return bStH;
  }
  inline const fmatvec::RowVec3& Trafo33RCM::getntilStH() const {
    return ntilStH;
  }
  inline const fmatvec::RowVec3& Trafo33RCM::getbtilStH() const {
    return btilStH;
  }
  inline const fmatvec::SqrMat3& Trafo33RCM::gettSpS() const {
    return tSpS;
  }
  inline const fmatvec::SqrMat3& Trafo33RCM::getnSpS() const {
    return nSpS;
  }
  inline const fmatvec::SqrMat3& Trafo33RCM::getbSpS() const {
    return bSpS;
  }
  inline const fmatvec::SqrMat3& Trafo33RCM::getntilSpS() const {
    return ntilSpS;
  }
  inline const fmatvec::SqrMat3& Trafo33RCM::getbtilSpS() const {
    return btilSpS;
  }
  inline const fmatvec::SqrMat3& Trafo33RCM::gettSpSt() const {
    return tSpSt;
  }
  inline const fmatvec::SqrMat3& Trafo33RCM::getnSpSt() const {
    return nSpSt;
  }
  inline const fmatvec::SqrMat3& Trafo33RCM::getbSpSt() const {
    return bSpSt;
  }
  inline const fmatvec::SqrMat3& Trafo33RCM::getntilSpSt() const {
    return ntilSpSt;
  }
  inline const fmatvec::SqrMat3& Trafo33RCM::getbtilSpSt() const {
    return btilSpSt;
  }
  inline const fmatvec::Vec16& Trafo33RCM::getqIt() const {
    return qIt;
  }
  inline const double& Trafo33RCM::getepstil() const {
    return epstil;
  }
  inline const double& Trafo33RCM::getepstilt() const {
    return epstilt;
  }
  inline const fmatvec::Vec3& Trafo33RCM::getrS() const {
    return rS;
  }
  inline const fmatvec::Vec3& Trafo33RCM::getrSt() const {
    return rSt;
  }
  inline const double& Trafo33RCM::getk0() const {
    return k0;
  }
  inline const double& Trafo33RCM::getk0t() const {
    return k0t;
  }
  inline const fmatvec::Vec11& Trafo33RCM::getbe() const {
    return be;
  }
  inline const fmatvec::Vec11& Trafo33RCM::getbet() const {
    return bet;
  }
  inline const double& Trafo33RCM::getxintil() const {
    return xintil;
  }
  inline const double& Trafo33RCM::getxibtil() const {
    return xibtil;
  }
  inline const double& Trafo33RCM::getetantil() const {
    return etantil;
  }
  inline const double& Trafo33RCM::getetabtil() const {
    return etabtil;
  }
  inline const double& Trafo33RCM::getxintilt() const {
    return xintilt;
  }
  inline const double& Trafo33RCM::getxibtilt() const {
    return xibtilt;
  }
  inline const double& Trafo33RCM::getetantilt() const {
    return etantilt;
  }
  inline const double& Trafo33RCM::getetabtilt() const {
    return etabtilt;
  }
  inline const fmatvec::SqrMat4& Trafo33RCM::getV() const {
    return V;
  }
/*******************************************************************/


  /* Class PositionFunction */
  inline PositionFunction::PositionFunction(RevCardanPtr angle_, double l0_, const fmatvec::Vec3& pL_, const fmatvec::Vec3& pR_, double cL1_, double cR1_, double cL2_, double cR2_, const fmatvec::RowVec3& rRrLmH_) :
      angle(angle_), l0(l0_), pL(pL_), pR(pR_), cL1(cL1_), cR1(cR1_), cL2(cL2_), cR2(cR2_), rRrLmH(rRrLmH_) {
  }

  inline PositionFunction::~PositionFunction() {
  }

  inline fmatvec::Vec11 PositionFunction::operator()(const fmatvec::Vec11& pos, const void *) {
    fmatvec::Vec3 pS = pos(fmatvec::Range0x2());

    fmatvec::Vec3 nS = angle->computen(pS);
    fmatvec::Vec3 bS = angle->computeb(pS);
    fmatvec::RowVec3 ntilSH = angle->computentil(pS).T();
    fmatvec::RowVec3 btilSH = angle->computebtil(pS).T();
    double xibtil = btilSH * nS;
    double xintil = ntilSH * nS;
    double etabtil = btilSH * bS;
    double etantil = ntilSH * bS;

    fmatvec::Vec11 F;
    F(0) = pos(0) - 0.5 * (pR(0) + pL(0) - sin(pos(1)) * (pos(9) + pos(10)));
    F(1) = pos(1) - 0.5 * (pR(1) + pL(1) - pos(5) - pos(6));
    F(2) = pos(2) - 0.5 * (pR(2) + pL(2) - pos(9) - pos(10));
    F(3) = pos(6) - pos(5) - pR(1) + pL(1);
    F(4) = pos(10) - pos(9) - pR(2) + pL(2);
    F(5) = xintil * (pos(4) - pos(3)) + xibtil * (pos(8) - pos(7)) - rRrLmH * nS;
    F(6) = etantil * (pos(4) - pos(3)) + etabtil * (pos(8) - pos(7)) - rRrLmH * bS;
    F(7) = 3. * l0 * (pos(5) - pos(6)) / 64. + 7. * (pos(3) + pos(4)) / 16. - cL1 - cR1;
    F(8) = -3. * l0 * (pos(5) + pos(6)) / 128. + 17. * (pos(4) - pos(3)) / 64. + cL1 - cR1;
    F(9) = 3. * l0 * (pos(9) - pos(10)) / 64. + 7. * (pos(7) + pos(8)) / 16. - cL2 - cR2;
    F(10) = -3. * l0 * (pos(9) + pos(10)) / 128. + 17. * (pos(8) - pos(7)) / 64. + cL2 - cR2;
    return F;
  }
  /*******************************************************************/

  /* Class PositionJacobian */
  inline PositionJacobian::PositionJacobian(RevCardanPtr angle_, double l0_, const fmatvec::RowVec3 &rRrLmH_, const fmatvec::Mat3x11 &pSbE_) :
      angle(angle_), l0(l0_), rRrLmH(rRrLmH_), pSbE(pSbE_) {
  }

  inline PositionJacobian::~PositionJacobian() {
  }

  inline fmatvec::SqrMat11 PositionJacobian::operator()(const fmatvec::Vec11& pos, const void *) {
    fmatvec::Vec3 pS = pos(fmatvec::Range0x2());

    fmatvec::Vec3 nS = angle->computen(pS);
    fmatvec::Vec3 bS = angle->computeb(pS);
    fmatvec::RowVec3 ntilSH = angle->computentil(pS).T();
    fmatvec::RowVec3 btilSH = angle->computebtil(pS).T();
    double xibtil = btilSH * nS;
    double xintil = ntilSH * nS;
    double etabtil = btilSH * bS;
    double etantil = ntilSH * bS;

    fmatvec::Mat3x11 nSbE = angle->computenq(pS) * pSbE;
    fmatvec::Mat3x11 bSbE = angle->computebq(pS) * pSbE;
    fmatvec::Mat11V ntilSbEH = (angle->computentilq(pS) * pSbE).T(); //TODO:_size okay??
    fmatvec::Mat11V btilSbEH = (angle->computebtilq(pS) * pSbE).T(); //TODO:_size okay??

    fmatvec::RowVec11 xibtilbE = (btilSbEH * nS).T() + btilSH * nSbE;
    fmatvec::RowVec11 xintilbE = (ntilSbEH * nS).T() + ntilSH * nSbE;
    fmatvec::RowVec11 etabtilbE = (btilSbEH * bS).T() + btilSH * bSbE;
    fmatvec::RowVec11 etantilbE = (ntilSbEH * bS).T() + ntilSH * bSbE;

    fmatvec::SqrMat11 SMRHS_Jac;
    SMRHS_Jac(0, 0) = 1.;
    SMRHS_Jac(0, 1) = 0.5 * cos(pS(1)) * (pos(9) + pos(10));
    SMRHS_Jac(0, 9) = 0.5 * sin(pS(1));
    SMRHS_Jac(0, 10) = 0.5 * sin(pS(1));

    SMRHS_Jac(1, 1) = 1.;
    SMRHS_Jac(1, 5) = 0.5;
    SMRHS_Jac(1, 6) = 0.5;

    SMRHS_Jac(2, 2) = 1.;
    SMRHS_Jac(2, 9) = 0.5;
    SMRHS_Jac(2, 10) = 0.5;

    SMRHS_Jac(3, 5) = -1.;
    SMRHS_Jac(3, 6) = 1.;

    SMRHS_Jac(4, 9) = -1.;
    SMRHS_Jac(4, 10) = 1.;

    SMRHS_Jac.set(fmatvec::Index(5, 5), fmatvec::Index(0, 10), -rRrLmH * nSbE);
    SMRHS_Jac.add(fmatvec::Index(5, 5), fmatvec::Index(0, 10), xintilbE * (pos(4) - pos(3)) + xibtilbE * (pos(8) - pos(7)));
    SMRHS_Jac(5, 3) -= xintil;
    SMRHS_Jac(5, 4) += xintil;
    SMRHS_Jac(5, 7) -= xibtil;
    SMRHS_Jac(5, 8) += xibtil;

    SMRHS_Jac.set(fmatvec::Index(6, 6), fmatvec::Index(0, 10), -rRrLmH * bSbE);
    SMRHS_Jac.add(fmatvec::Index(6, 6), fmatvec::Index(0, 10), etantilbE * (pos(4) - pos(3)) + etabtilbE * (pos(8) - pos(7)));
    SMRHS_Jac(6, 3) -= etantil;
    SMRHS_Jac(6, 4) += etantil;
    SMRHS_Jac(6, 7) -= etabtil;
    SMRHS_Jac(6, 8) += etabtil;

    SMRHS_Jac(7, 3) = 7. / 16.;
    SMRHS_Jac(7, 4) = 7. / 16.;
    SMRHS_Jac(7, 5) = 3. * l0 / 64.;
    SMRHS_Jac(7, 6) = -3. * l0 / 64.;

    SMRHS_Jac(8, 3) = -17. / 64.;
    SMRHS_Jac(8, 4) = 17. / 64.;
    SMRHS_Jac(8, 5) = -3. * l0 / 128.;
    SMRHS_Jac(8, 6) = -3. * l0 / 128.;

    SMRHS_Jac(9, 7) = 7. / 16.;
    SMRHS_Jac(9, 8) = 7. / 16.;
    SMRHS_Jac(9, 9) = 3. * l0 / 64.;
    SMRHS_Jac(9, 10) = -3. * l0 / 64.;

    SMRHS_Jac(10, 7) = -17. / 64.;
    SMRHS_Jac(10, 8) = 17. / 64.;
    SMRHS_Jac(10, 9) = -3. * l0 / 128.;
    SMRHS_Jac(10, 10) = -3. * l0 / 128.;

    return SMRHS_Jac;
  }
  /*******************************************************************/

  /* CLASS TRAFO33RCM */
  inline Trafo33RCM::Trafo33RCM(RevCardanPtr angle_, double l0_) :
      angle(angle_), l0(l0_), l0h2(l0 * l0), l0h3(l0h2 * l0), l0h4(l0h3 * l0), l0h5(l0h4 * l0), xstar(0.25 * l0), xstarh2(xstar * xstar), xstarh3(xstarh2 * xstar), epstil(0.), k0(0.), rS(), pS(), rRrLp(), rRrLmH(), be(), tS(), nS(), bS(), ntilS(), btilS(), nSH(), bSH(), ntilSH(), btilSH(), tSpS(), nSpS(), bSpS(), ntilSpS(), btilSpS(), xibtil(0.), xintil(0.), etabtil(0.), etantil(0.), SMRHS_Jac(), V(), drRdrLp(), drRdrLm(), pSbE(), SMRHS(), nSbE(), bSbE(), ntilSbE(), btilSbE(), xibtilbE(), xintilbE(), etabtilbE(), etantilbE(), beqG(), tSqG(), nSqG(), bSqG(), ntilSqG(), btilSqG(), xintilqG(), xibtilqG(), etantilqG(), etabtilqG(), JIG(), JIGt(), k0t(0.), qIt(), bet(), rSt(), pSt(), tSt(), nSt(), bSt(), tStH(), nStH(), bStH(), ntilStH(), btilStH(), tSpSt(), nSpSt(), bSpSt(), ntilSpSt(), btilSpSt(), xibtilt(0.), xintilt(0.), etabtilt(0.), etantilt(0.) {
    /* trafo initial value */
    SMRHS(0, 2) = -1.;
    SMRHS(0, 3) = 1.;
    SMRHS(1, 6) = -1.;
    SMRHS(1, 7) = 1.;

    SMRHS(4, 0) = 7. / 16.;
    SMRHS(4, 1) = 7. / 16.;
    SMRHS(4, 2) = 3. * l0 / 64.;
    SMRHS(4, 3) = -3. * l0 / 64.;
    SMRHS(5, 0) = -17. / 64.;
    SMRHS(5, 1) = 17. / 64.;
    SMRHS(5, 2) = -3. * l0 / 128.;
    SMRHS(5, 3) = -3. * l0 / 128.;
    SMRHS(6, 4) = 7. / 16.;
    SMRHS(6, 5) = 7. / 16.;
    SMRHS(6, 6) = 3. * l0 / 64.;
    SMRHS(6, 7) = -3. * l0 / 64.;
    SMRHS(7, 4) = -17. / 64.;
    SMRHS(7, 5) = 17. / 64.;
    SMRHS(7, 6) = -3. * l0 / 128.;
    SMRHS(7, 7) = -3. * l0 / 128.;

    /* trafo Jacobian */
    computeV();

    SMRHS_Jac(0, 0) = 1.;
    SMRHS_Jac(1, 1) = 1.;
    SMRHS_Jac(1, 5) = 0.5;
    SMRHS_Jac(1, 6) = 0.5;
    SMRHS_Jac(2, 2) = 1.;
    SMRHS_Jac(2, 9) = 0.5;
    SMRHS_Jac(2, 10) = 0.5;
    SMRHS_Jac(3, 6) = 1.;
    SMRHS_Jac(3, 5) = -1.;
    SMRHS_Jac(4, 10) = 1.;
    SMRHS_Jac(4, 9) = -1.;
    SMRHS_Jac.set(fmatvec::Index(7, 7), fmatvec::Index(3, 6), 2. * xstarh2 * (V(fmatvec::Index(1, 1), fmatvec::Index(0, 3)) * xstarh2 + V(fmatvec::Index(3, 3), fmatvec::Index(0, 3))));
    SMRHS_Jac.set(fmatvec::Index(8, 8), fmatvec::Index(3, 6), 2. * xstarh3 * (V(fmatvec::Index(0, 0), fmatvec::Index(0, 3)) * xstarh2 + V(fmatvec::Index(2, 2), fmatvec::Index(0, 3))));
    SMRHS_Jac.set(fmatvec::Index(9, 9), fmatvec::Index(7, 10), 2. * xstarh2 * (V(fmatvec::Index(1, 1), fmatvec::Index(0, 3)) * xstarh2 + V(fmatvec::Index(3, 3), fmatvec::Index(0, 3))));
    SMRHS_Jac.set(fmatvec::Index(10, 10), fmatvec::Index(7, 10), 2. * xstarh3 * (V(fmatvec::Index(0, 0), fmatvec::Index(0, 3)) * xstarh2 + V(fmatvec::Index(2, 2), fmatvec::Index(0, 3))));

    SMRHS_Jac(0, 14) = 0.5;
    SMRHS_Jac(0, 24) = 0.5;
    SMRHS_Jac(1, 15) = 0.5;
    SMRHS_Jac(1, 25) = 0.5;
    SMRHS_Jac(2, 16) = 0.5;
    SMRHS_Jac(2, 26) = 0.5;
    SMRHS_Jac(3, 15) = -1.;
    SMRHS_Jac(3, 25) = 1.;
    SMRHS_Jac(4, 16) = -1.;
    SMRHS_Jac(4, 26) = 1.;
    SMRHS_Jac(7, 17) = 1.;
    SMRHS_Jac(7, 18) = 1.;
    SMRHS_Jac(8, 17) = -1.;
    SMRHS_Jac(8, 18) = 1.;
    SMRHS_Jac(9, 19) = 1.;
    SMRHS_Jac(9, 20) = 1.;
    SMRHS_Jac(10, 19) = -1.;
    SMRHS_Jac(10, 20) = 1.;

    computedrRdrL();

    pSbE(0, 0) = 1.;
    pSbE(1, 1) = 1.;
    pSbE(2, 2) = 1.;
  }

  inline Trafo33RCM::~Trafo33RCM() {
  }

  inline void Trafo33RCM::computeprelim(const fmatvec::Vec16& qG) {
    rRrLmH = (qG(fmatvec::Range10x12()) - qG(fmatvec::Range0x2())).T();
  }

  inline fmatvec::Vec Trafo33RCM::computes0(const fmatvec::Vec16& qG) {
    fmatvec::Vec pM = 0.5 * (qG(fmatvec::Range3x5()) + qG(fmatvec::Range13x15()));
    fmatvec::Vec3 tM = angle->computet(pM);
    fmatvec::Vec3 nM = angle->computen(pM);
    fmatvec::Vec3 bM = angle->computeb(pM);
    fmatvec::Vec nMtil = angle->computentil(pM);
    fmatvec::Vec bMtil = angle->computebtil(pM);
    fmatvec::SqrMat3 nMpM = angle->computenq(pM);
    fmatvec::SqrMat3 bMpM = angle->computebq(pM);

    double xinMtil = nM.T() * nMtil;
    double xibMtil = nM.T() * bMtil;
    double etanMtil = bM.T() * nMtil;
    double etabMtil = bM.T() * bMtil;

    fmatvec::RowVec3 rRrLmnMpM = 0.5 * rRrLmH * nMpM;
    SMRHS(2, 0) = -xinMtil;
    SMRHS(2, 1) = xinMtil;
    SMRHS(2, 4) = -xibMtil;
    SMRHS(2, 5) = xibMtil;
    SMRHS(2, 2) = rRrLmnMpM(1);
    SMRHS(2, 3) = rRrLmnMpM(1);
    SMRHS(2, 6) = rRrLmnMpM(0) * sin(pM(1)) + rRrLmnMpM(2);
    SMRHS(2, 7) = rRrLmnMpM(0) * sin(pM(1)) + rRrLmnMpM(2);

    fmatvec::RowVec3 rRrLmbMpM = 0.5 * rRrLmH * bMpM;
    SMRHS(3, 0) = -etanMtil;
    SMRHS(3, 1) = etanMtil;
    SMRHS(3, 4) = -etabMtil;
    SMRHS(3, 5) = etabMtil;
    SMRHS(3, 2) = rRrLmbMpM(1);
    SMRHS(3, 3) = rRrLmbMpM(1);
    SMRHS(3, 6) = rRrLmbMpM(0) * sin(pM(1)) + rRrLmbMpM(2);
    SMRHS(3, 7) = rRrLmbMpM(0) * sin(pM(1)) + rRrLmbMpM(2);

    SMRHS(0, 8) = qG(14) - qG(4);
    SMRHS(1, 8) = qG(15) - qG(5);
    SMRHS(2, 8) = rRrLmH * nM;
    SMRHS(3, 8) = rRrLmH * bM;
    SMRHS(4, 8) = qG(6) + qG(7);
    SMRHS(5, 8) = qG(7) - qG(6);
    SMRHS(6, 8) = qG(8) + qG(9);
    SMRHS(7, 8) = qG(9) - qG(8);

    fmatvec::Vec sol = slvLU(static_cast<fmatvec::SqrMat>(SMRHS(fmatvec::Index(0, 7), fmatvec::Index(0, 7))), static_cast<fmatvec::Vec>(SMRHS(fmatvec::Index(0, 7), fmatvec::Index(8, 8))));
    fmatvec::Vec s0(11);
    s0(3,10) = sol;

    s0(1) = 0.5 * ((qG(4) + qG(14)) - (sol(2) + sol(3)));
    s0(2) = 0.5 * ((qG(5) + qG(15)) - (sol(6) + sol(7)));
    s0(0) = 0.5 * (qG(3) + qG(13) - sin(s0(1)) * (sol(6) + sol(7)));

    return s0.copy();
  }

  inline void Trafo33RCM::computebe(const fmatvec::Vec16& qG) {
    // REQUIRED computeprelim()

    fmatvec::Vec3 pL = qG(fmatvec::Range3x5());
    fmatvec::Vec3 pR = qG(fmatvec::Range13x15());

    PositionFunction fun(angle, l0, pL, pR, qG(6), qG(7), qG(8), qG(9), rRrLmH); // (object itself no reference)
    PositionJacobian jac(angle, l0, rRrLmH, pSbE);
    MBSim::GlobalResidualCFunction<11> crit;
    MBSim::MDNM<11> rf;
    rf.setFunction(&fun);
    rf.setJacobianFunction(&jac);
    rf.setCriteriaFunction(&crit);
    rf.setMaximumNumberOfIterations(10);

    if (nrm2(be) < MBSim::epsroot()) {
      fmatvec::Vec11 s0 = computes0(qG); // initial value
      be = rf.solve(s0);
    }
    else
      be = rf.solve(be);

    if (rf.getInfo() != 0) {
      throw MBSim::MBSimError("ERROR (TRAFO33RCM:computebe): No convergence of Newton method during bending correction.");
    }
  }

  inline void Trafo33RCM::computeCOSY() {
    // REQUIRED computebe()

    pS = be(fmatvec::Range0x2());
    tS = angle->computet(pS);
    nS = angle->computen(pS);
    bS = angle->computeb(pS);
    ntilS = angle->computentil(pS);
    btilS = angle->computebtil(pS);
    nSH = nS.T();
    bSH = bS.T();
    ntilSH = ntilS.T();
    btilSH = btilS.T();
    tSpS = angle->computetq(pS);
    nSpS = angle->computenq(pS);
    bSpS = angle->computebq(pS);
    ntilSpS = angle->computentilq(pS);
    btilSpS = angle->computebtilq(pS);

    xibtil = nSH * btilS;
    xintil = nSH * ntilS;
    etabtil = bSH * btilS;
    etantil = bSH * ntilS;
  }

  inline void Trafo33RCM::computerSepstk0(const fmatvec::Vec16& qG) {
    // REQUIRED computeCOSY()

    fmatvec::Vec3 rRrLp = qG(fmatvec::Range10x12()) + qG(fmatvec::Range0x2());

    rS = 0.5 * (rRrLp - (xintil * (be(3) + be(4)) + xibtil * (be(7) + be(8))) * nS - (etantil * (be(3) + be(4)) + etabtil * (be(7) + be(8))) * bS);
    epstil = rRrLmH * tS / l0 - 1.;
    k0 = (qG(13) - qG(3) - sin(pS(1)) * (be(10) - be(9))) / l0;
  }

  inline void Trafo33RCM::computeqI(const fmatvec::Vec16& qG) {
    computeprelim(qG);
    computebe(qG);
    computeCOSY();
    computerSepstk0(qG);
  }

  inline void Trafo33RCM::computedrRdrL() {
    drRdrLp(0, 0) = 1.;
    drRdrLp(1, 1) = 1.;
    drRdrLp(2, 2) = 1.;
    drRdrLp(0, 10) = 1.;
    drRdrLp(1, 11) = 1.;
    drRdrLp(2, 12) = 1.;

    drRdrLm(0, 0) = -1.;
    drRdrLm(1, 1) = -1.;
    drRdrLm(2, 2) = -1.;
    drRdrLm(0, 10) = 1.;
    drRdrLm(1, 11) = 1.;
    drRdrLm(2, 12) = 1.;
  }

  inline void Trafo33RCM::computeV() {
    V(0, 0) = 24. / l0h5;
    V(1, 0) = -8. / l0h4;
    V(2, 0) = -10. / l0h3;
    V(3, 0) = 4. / l0h2;

    V(0, 1) = -24. / l0h5;
    V(1, 1) = -8. / l0h4;
    V(2, 1) = 10. / l0h3;
    V(3, 1) = 4. / l0h2;

    V(0, 2) = 4. / l0h4;
    V(1, 2) = -2. / l0h3;
    V(2, 2) = -1. / l0h2;
    V(3, 2) = 1. / (2. * l0);

    V(0, 3) = 4. / l0h4;
    V(1, 3) = 2. / l0h3;
    V(2, 3) = -1. / l0h2;
    V(3, 3) = -1. / (2. * l0);
  }

  inline void Trafo33RCM::computebeqG() {
    // REQUIRED computeqI()

    nSbE = nSpS * pSbE;
    bSbE = bSpS * pSbE;
    ntilSbE = ntilSpS * pSbE;
    btilSbE = btilSpS * pSbE;

    xibtilbE = nSH * btilSbE + btilSH * nSbE;
    xintilbE = nSH * ntilSbE + ntilSH * nSbE;
    etabtilbE = bSH * btilSbE + btilSH * bSbE;
    etantilbE = bSH * ntilSbE + ntilSH * bSbE;

    SMRHS_Jac(0, 1) = 0.5 * cos(pS(1)) * (be(9) + be(10));
    SMRHS_Jac(0, 9) = 0.5 * sin(pS(1));
    SMRHS_Jac(0, 10) = 0.5 * sin(pS(1));
    SMRHS_Jac.set(fmatvec::Index(5, 5), fmatvec::Index(0, 10), -rRrLmH * nSbE);
    SMRHS_Jac.add(fmatvec::Index(5, 5), fmatvec::Index(0, 10), xintilbE * (be(4) - be(3)) + xibtilbE * (be(8) - be(7)));
    SMRHS_Jac(5, 4) += xintil;
    SMRHS_Jac(5, 3) -= xintil;
    SMRHS_Jac(5, 8) += xibtil;
    SMRHS_Jac(5, 7) -= xibtil;

    SMRHS_Jac.set(fmatvec::Index(6, 6), fmatvec::Index(0, 10), -rRrLmH * bSbE);
    SMRHS_Jac.add(fmatvec::Index(6, 6), fmatvec::Index(0, 10), etantilbE * (be(4) - be(3)) + etabtilbE * (be(8) - be(7)));
    SMRHS_Jac(6, 4) += etantil;
    SMRHS_Jac(6, 3) -= etantil;
    SMRHS_Jac(6, 8) += etabtil;
    SMRHS_Jac(6, 7) -= etabtil;

    SMRHS_Jac.set(fmatvec::Index(5, 5), fmatvec::Index(11, 26), nSH * drRdrLm);
    SMRHS_Jac.set(fmatvec::Index(6, 6), fmatvec::Index(11, 26), bSH * drRdrLm);

    //TODO: Is this done every timestep --> use different solver
    beqG = slvLU(static_cast<fmatvec::SqrMat>(SMRHS_Jac(fmatvec::Index(0, 10), fmatvec::Index(0, 10))), static_cast<fmatvec::Mat>(SMRHS_Jac(fmatvec::Index(0, 10), fmatvec::Index(11, 26)))); //TODO??
  }

  inline void Trafo33RCM::computeCOSYqG() {
    // REQUIRED computebeqG()

    tSqG = tSpS * beqG(fmatvec::Index(0, 2), fmatvec::Index(0, 15));
    nSqG = nSpS * beqG(fmatvec::Index(0, 2), fmatvec::Index(0, 15));
    bSqG = bSpS * beqG(fmatvec::Index(0, 2), fmatvec::Index(0, 15));
    ntilSqG = ntilSpS * beqG(fmatvec::Index(0, 2), fmatvec::Index(0, 15));
    btilSqG = btilSpS * beqG(fmatvec::Index(0, 2), fmatvec::Index(0, 15));

    xintilqG = nSH * ntilSqG + ntilSH * nSqG;
    xibtilqG = nSH * btilSqG + btilSH * nSqG;
    etantilqG = bSH * ntilSqG + ntilSH * bSqG;
    etabtilqG = bSH * btilSqG + btilSH * bSqG;
  }

  inline void Trafo33RCM::computeJIG(const fmatvec::Vec16& qG) {
    computeqI(qG);
    computebeqG();
    computeCOSYqG();

    fmatvec::RowVec3 tSH = tS.T();

    JIG.set(fmatvec::Index(0, 2), fmatvec::Index(0, 15), drRdrLp);
    JIG.add(fmatvec::Index(0, 2), fmatvec::Index(0, 15), -(nS * (xintilqG * (be(3) + be(4)) + xibtilqG * (be(7) + be(8)) + xintil * (beqG(fmatvec::Index(3, 3), fmatvec::Index(0, 15)) + beqG(fmatvec::Index(4, 4), fmatvec::Index(0, 15)) + xibtil * (beqG(fmatvec::Index(7, 7), fmatvec::Index(0, 15)) + beqG(fmatvec::Index(8, 8), fmatvec::Index(0, 15)))))));
    JIG.add(fmatvec::Index(0, 2), fmatvec::Index(0, 15), -((xintil * (be(3) + be(4)) + xibtil * (be(7) + be(8))) * nSqG));
    JIG.add(fmatvec::Index(0, 2), fmatvec::Index(0, 15), -(bS * (etantilqG * (be(3) + be(4)) + etabtilqG * (be(7) + be(8)) + etantil * (beqG(fmatvec::Index(3, 3), fmatvec::Index(0, 15)) + beqG(fmatvec::Index(4, 4), fmatvec::Index(0, 15))) + etabtil * (beqG(fmatvec::Index(7, 7), fmatvec::Index(0, 15)) + beqG(fmatvec::Index(8, 8), fmatvec::Index(0, 15))))));
    JIG.add(fmatvec::Index(0, 2), fmatvec::Index(0, 15), -((etantil * (be(3) + be(4)) + etabtil * (be(7) + be(8))) * bSqG));
    JIG(fmatvec::Index(0, 2), fmatvec::Index(0, 15)) *= 0.5;
    JIG.set(fmatvec::Index(3, 5), fmatvec::Index(0, 15), beqG(fmatvec::Index(0, 2), fmatvec::Index(0, 15)));
    JIG.set(fmatvec::Index(6, 6), fmatvec::Index(0, 15), (tSH * drRdrLm + rRrLmH * tSqG) / l0);
    JIG.set(fmatvec::Index(7, 14), fmatvec::Index(0, 15), beqG(fmatvec::Index(3, 10), fmatvec::Index(0, 15)));
    JIG.set(fmatvec::Index(15, 15), fmatvec::Index(0, 15), -sin(pS(1)) * (beqG(fmatvec::Index(10, 10), fmatvec::Index(0, 15)) - beqG(fmatvec::Index(9, 9), fmatvec::Index(0, 15))) - (be(10) - be(9)) * cos(pS(1)) * beqG(fmatvec::Index(1, 1), fmatvec::Index(0, 15)));
    JIG(15, 3) -= 1.;
    JIG(15, 13) += 1.;
    JIG(fmatvec::Index(15, 15), fmatvec::Index(0, 15)) /= l0;
  }

  inline void Trafo33RCM::computezI(const fmatvec::Vec16& qG, const fmatvec::Vec16& qGt) {
    computeJIG(qG);

    qIt = JIG * qGt;
    rSt = qIt(fmatvec::Range0x2());
    pSt = qIt(fmatvec::Range3x5());
    epstilt = qIt(6);
    bet.set(fmatvec::Range0x2(), pSt(fmatvec::Range0x2()));
    bet.set(fmatvec::Range3x10(), qIt(fmatvec::Range7x14()));
    k0t = qIt(15);
  }

  inline void Trafo33RCM::computeCOSYt(const fmatvec::Vec16& qG, const fmatvec::Vec16& qGt) {
    computezI(qG, qGt);

    tSt = tSpS * pSt;
    nSt = nSpS * pSt;
    bSt = bSpS * pSt;
    fmatvec::Vec3 ntilSt = ntilSpS * pSt;
    fmatvec::Vec3 btilSt = btilSpS * pSt;

    nStH = nSt.T();
    bStH = bSt.T();
    ntilStH = ntilSt.T();
    btilStH = btilSt.T();

    tSpSt = angle->computetqt(pS, pSt);
    nSpSt = angle->computenqt(pS, pSt);
    bSpSt = angle->computebqt(pS, pSt);
    ntilSpSt = angle->computentilqt(pS, pSt);
    btilSpSt = angle->computebtilqt(pS, pSt);

    xibtilt = nStH * btilS + btilStH * nS;
    xintilt = nStH * ntilS + ntilStH * nS;
    etabtilt = bStH * btilS + btilStH * bS;
    etantilt = bStH * ntilS + ntilStH * bS;
  }

  inline void Trafo33RCM::computeJIGt(const fmatvec::Vec16& qGt) {
    // REQUIRED computeCOSYt()

    fmatvec::RowVec3 rRrLtmH = (qGt(fmatvec::Range10x12()) - qGt(fmatvec::Range0x2())).T();
    fmatvec::RowVec3 tStH = tSt.T();

    fmatvec::Mat3x11 nSbEt = nSpSt * pSbE;
    fmatvec::Mat3x11 bSbEt = bSpSt * pSbE;
    fmatvec::Mat3x11 ntilSbEt = ntilSpSt * pSbE;
    fmatvec::Mat3x11 btilSbEt = btilSpSt * pSbE;

    fmatvec::RowVec11 xibtilbEt = nStH * btilSbE + btilStH * nSbE + nSH * btilSbEt + btilSH * nSbEt;
    fmatvec::RowVec11 xintilbEt = nStH * ntilSbE + ntilStH * nSbE + nSH * ntilSbEt + ntilSH * nSbEt;
    fmatvec::RowVec11 etabtilbEt = bStH * btilSbE + btilStH * bSbE + bSH * btilSbEt + btilSH * bSbEt;
    fmatvec::RowVec11 etantilbEt = bStH * ntilSbE + ntilStH * bSbE + bSH * ntilSbEt + ntilSH * bSbEt;

    fmatvec::Mat3V tSqGt = tSpSt * beqG(fmatvec::Index(0, 2), fmatvec::Index(0, 15)) + tSpS * JIGt(fmatvec::Index(3, 5), fmatvec::Index(0, 15));
    fmatvec::Mat3V nSqGt = nSpSt * beqG(fmatvec::Index(0, 2), fmatvec::Index(0, 15)) + nSpS * JIGt(fmatvec::Index(3, 5), fmatvec::Index(0, 15));
    fmatvec::Mat3V bSqGt = bSpSt * beqG(fmatvec::Index(0, 2), fmatvec::Index(0, 15)) + bSpS * JIGt(fmatvec::Index(3, 5), fmatvec::Index(0, 15));
    fmatvec::Mat3V ntilSqGt = ntilSpSt * beqG(fmatvec::Index(0, 2), fmatvec::Index(0, 15)) + ntilSpS * JIGt(fmatvec::Index(3, 5), fmatvec::Index(0, 15));
    fmatvec::Mat3V btilSqGt = btilSpSt * beqG(fmatvec::Index(0, 2), fmatvec::Index(0, 15)) + btilSpS * JIGt(fmatvec::Index(3, 5), fmatvec::Index(0, 15));

    fmatvec::RowVec16 xibtilqGt = nStH * btilSqG + btilStH * nSqG + nSH * btilSqGt + btilSH * nSqGt;
    fmatvec::RowVec16 xintilqGt = nStH * ntilSqG + ntilStH * nSqG + nSH * ntilSqGt + ntilSH * nSqGt;
    fmatvec::RowVec16 etabtilqGt = bStH * btilSqG + btilStH * bSqG + bSH * btilSqGt + btilSH * bSqGt;
    fmatvec::RowVec16 etantilqGt = bStH * ntilSqG + ntilStH * bSqG + bSH * ntilSqGt + ntilSH * bSqGt;

    fmatvec::SqrMat11 SMt;
    SMt(0, 1) = 0.5 * (-sin(pS(1)) * pSt(1) * (be(9) + be(10)) + cos(pS(1)) * (bet(9) + bet(10)));
    SMt(0, 9) = 0.5 * cos(pS(1)) * pSt(1);
    SMt(0, 10) = 0.5 * cos(pS(1)) * pSt(1);

    SMt.set(fmatvec::Index(5, 5), fmatvec::Index(0, 10), -rRrLtmH * nSbE - rRrLmH * nSbEt);
    SMt.add(fmatvec::Index(5, 5), fmatvec::Index(0, 10),  xintilbEt * (be(4) - be(3)) + xibtilbEt * (be(8) - be(7)));
    SMt.add(fmatvec::Index(5, 5), fmatvec::Index(0, 10),  xintilbE * (bet(4) - bet(3)) + xibtilbE * (bet(8) - bet(7)));
    SMt(5, 4) += xintilt;
    SMt(5, 3) -= xintilt;
    SMt(5, 8) += xibtilt;
    SMt(5, 7) -= xibtilt;

    SMt.set(fmatvec::Index(6, 6), fmatvec::Index(0, 10), -rRrLtmH * bSbE - rRrLmH * bSbEt);
    SMt.add(fmatvec::Index(6, 6), fmatvec::Index(0, 10), etantilbEt * (be(4) - be(3)) + etabtilbEt * (be(8) - be(7)));
    SMt.add(fmatvec::Index(6, 6), fmatvec::Index(0, 10), etantilbE * (bet(4) - bet(3)) + etabtilbE * (bet(8) - bet(7)));
    SMt(6, 4) += etantilt;
    SMt(6, 3) -= etantilt;
    SMt(6, 8) += etabtilt;
    SMt(6, 7) -= etabtilt;

    fmatvec::MatV16 RHS_JIGt = -SMt * beqG;
    RHS_JIGt.add(fmatvec::Index(5, 5), fmatvec::Index(0, 15), nStH * drRdrLm);
    RHS_JIGt.add(fmatvec::Index(6, 6), fmatvec::Index(0, 15), bStH * drRdrLm);

    fmatvec::Mat beqGt = slvLU(static_cast<fmatvec::SqrMat>(SMRHS_Jac(fmatvec::Index(0, 10), fmatvec::Index(0, 10))), static_cast<fmatvec::Mat>(RHS_JIGt)); //TODO??

    JIGt.set(fmatvec::Index(3, 5), fmatvec::Index(0, 15), beqGt(fmatvec::Index(0, 2), fmatvec::Index(0, 15)));
    JIGt.set(fmatvec::Index(7, 14), fmatvec::Index(0, 15), beqGt(fmatvec::Index(3, 10), fmatvec::Index(0, 15)));

    JIGt.set(fmatvec::Index(6, 6), fmatvec::Index(0, 15),(tStH * drRdrLm + rRrLtmH * tSqG + rRrLmH * tSqGt) / l0);

    JIGt.set(fmatvec::Index(0, 2), fmatvec::Index(0, 15), -nSt * (xintilqG * (be(3) + be(4)) + xibtilqG * (be(7) + be(8)) + xintil * (beqG(fmatvec::Index(3, 3), fmatvec::Index(0, 15)) + beqG(fmatvec::Index(4, 4), fmatvec::Index(0, 15))) + xibtil * (beqG(fmatvec::Index(7, 7), fmatvec::Index(0, 15)) + beqG(fmatvec::Index(8, 8), fmatvec::Index(0, 15)))));
    JIGt.add(fmatvec::Index(0, 2), fmatvec::Index(0, 15), - (nS * (xintilqGt * (be(3) + be(4)) + xibtilqGt * (be(7) + be(8)) + xintilt * (beqG(fmatvec::Index(3, 3), fmatvec::Index(0, 15)) + beqG(fmatvec::Index(4, 4), fmatvec::Index(0, 15))) + xibtilt * (beqG(fmatvec::Index(7, 7), fmatvec::Index(0, 15)) + beqG(fmatvec::Index(8, 8), fmatvec::Index(0, 15))))));
    JIGt.add(fmatvec::Index(0, 2), fmatvec::Index(0, 15), - (nS * (xintilqG * (bet(3) + bet(4)) + xibtilqG * (bet(7) + bet(8)) + xintil * (beqGt(fmatvec::Index(3, 3), fmatvec::Index(0, 15)) + beqGt(fmatvec::Index(4, 4), fmatvec::Index(0, 15))) + xibtil * (beqGt(fmatvec::Index(7, 7), fmatvec::Index(0, 15)) + beqGt(fmatvec::Index(8, 8), fmatvec::Index(0, 15))))));

    JIGt.add(fmatvec::Index(0, 2), fmatvec::Index(0, 15), -( (xintil * (be(3) + be(4)) + xibtil * (be(7) + be(8))) * nSqGt));
    JIGt.add(fmatvec::Index(0, 2), fmatvec::Index(0, 15), -( (xintilt * (be(3) + be(4)) + xibtilt * (be(7) + be(8))) * nSqG));
    JIGt.add(fmatvec::Index(0, 2), fmatvec::Index(0, 15), -( (xintil * (bet(3) + bet(4)) + xibtil * (bet(7) + bet(8))) * nSqG));

    JIGt.add(fmatvec::Index(0, 2), fmatvec::Index(0, 15), -( bSt * (etantilqG * (be(3) + be(4)) + etabtilqG * (be(7) + be(8)) + etantil * (beqG(fmatvec::Index(3, 3), fmatvec::Index(0, 15)) + beqG(fmatvec::Index(4, 4), fmatvec::Index(0, 15))) + etabtil * (beqG(fmatvec::Index(7, 7), fmatvec::Index(0, 15)) + beqG(fmatvec::Index(8, 8), fmatvec::Index(0, 15))))));
    JIGt.add(fmatvec::Index(0, 2), fmatvec::Index(0, 15), -( bS * (etantilqGt * (be(3) + be(4)) + etabtilqGt * (be(7) + be(8)) + etantilt * (beqG(fmatvec::Index(3, 3), fmatvec::Index(0, 15)) + beqG(fmatvec::Index(4, 4), fmatvec::Index(0, 15))) + etabtilt * (beqG(fmatvec::Index(7, 7), fmatvec::Index(0, 15)) + beqG(fmatvec::Index(8, 8), fmatvec::Index(0, 15))))));
    JIGt.add(fmatvec::Index(0, 2), fmatvec::Index(0, 15), -( bS * (etantilqG * (bet(3) + bet(4)) + etabtilqG * (bet(7) + bet(8)) + etantil * (beqGt(fmatvec::Index(3, 3), fmatvec::Index(0, 15)) + beqGt(fmatvec::Index(4, 4), fmatvec::Index(0, 15))) + etabtil * (beqGt(fmatvec::Index(7, 7), fmatvec::Index(0, 15)) + beqGt(fmatvec::Index(8, 8), fmatvec::Index(0, 15))))));

    JIGt.add(fmatvec::Index(0, 2), fmatvec::Index(0, 15), -( (etantil * (be(3) + be(4)) + etabtil * (be(7) + be(8))) * bSqGt));
    JIGt.add(fmatvec::Index(0, 2), fmatvec::Index(0, 15), -( (etantilt * (be(3) + be(4)) + etabtilt * (be(7) + be(8))) * bSqG));
    JIGt.add(fmatvec::Index(0, 2), fmatvec::Index(0, 15), -( (etantil * (bet(3) + bet(4)) + etabtil * (bet(7) + bet(8))) * bSqG));

    JIGt(fmatvec::Index(0, 2), fmatvec::Index(0, 15)) *= 0.5;

    JIGt.set(fmatvec::Index(15, 15), fmatvec::Index(0, 15), -cos(pS(1)) * pSt(1) * (beqG(fmatvec::Index(10, 10), fmatvec::Index(0, 15)) - beqG(fmatvec::Index(9, 9), fmatvec::Index(0, 15))) - ((bet(10) - bet(9)) * cos(pS(1)) - (be(10) - be(9)) * sin(pS(1)) * pSt(1)) * beqG(fmatvec::Index(1, 1), fmatvec::Index(0, 15)));
    JIGt.add(fmatvec::Index(15, 15), fmatvec::Index(0, 15), -sin(pS(1)) * (beqGt(fmatvec::Index(10, 10), fmatvec::Index(0, 15)) - beqGt(fmatvec::Index(9, 9), fmatvec::Index(0, 15))) - (be(10) - be(9)) * cos(pS(1)) * beqGt(fmatvec::Index(1, 1), fmatvec::Index(0, 15)));
    JIGt(fmatvec::Index(15, 15), fmatvec::Index(0, 15)) /= l0;
  }

  inline void Trafo33RCM::computeTrafo(const fmatvec::Vec16& qG, const fmatvec::Vec16& qGt) {
    computeCOSYt(qG, qGt);
    computeJIGt(qGt);
  }
/*******************************************************************/



}

#endif /*TRAFO33RCM_H_*/

