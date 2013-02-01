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
      void computeprelim(const fmatvec::Vec& qG);

      /**
       * \brief compute the initial value for computebe() 
       * \param global coordinates
       */
      fmatvec::Vec computes0(const fmatvec::Vec& qG);

      /**
       * \brief compute the angle and bending positions
       * \param global coordinates
       */
      void computebe(const fmatvec::Vec& qG);

      /**
       * \brief compute the COSY
       */
      void computeCOSY();

      /**
       * \brief compute the CP, prolongation and torsion
       * \param global coordinates
       */
      void computerSepstk0(const fmatvec::Vec& qG);

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

}

#endif /*TRAFO33RCM_H_*/

