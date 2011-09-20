/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: thschindler@users.berlios.de
 */

#ifndef TRAFO33RCM_H_
#define TRAFO33RCM_H_

#include "fmatvec.h"
#include "mbsim/utils/function.h"
#include "mbsimFlexibleBody/pointer.h"

namespace MBSimFlexibleBody {

  /***
   * \brief computation of FiniteElement1s33RCM position coordinates (cp-angles and bending)
   * \author Thorsten Schindler
   * \date 2009-04-24 initial commit (Thorsten Schindler)
   */
  class PositionFunction : public MBSim::Function1<fmatvec::Vec,fmatvec::Vec> {    
    public:
      /**
       * \brief constructor
       * \param reversed Cardan-Object
       * \param length of FiniteElement1s33RCM
       * \param angles of COG of rigid left-right connection
       * \param global bending parameters
       * \param transpose of difference of right and left beam position
       */
      PositionFunction(RevCardanPtr angle_,double l0_,const fmatvec::Vec& pL_,const fmatvec::Vec& pR_,
          double cL1_,double cR1_,double cL2_,double cR2_,const fmatvec::RowVec& rRrLmH_);
      
      /**
       * \brief destructor 
       */
      virtual ~PositionFunction();
      
      /* INHERITED INTERFACE */
      fmatvec::Vec operator()(const fmatvec::Vec& pos, const void * =NULL);
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
       * \brief angles of COG of rigid left-right connection
       */
      fmatvec::Vec pL, pR;

      /** 
       * \brief global bending parameters
       */
      double cL1, cR1, cL2, cR2;

      /** 
       * \brief transpose of difference of right and left beam position
       */
      fmatvec::RowVec rRrLmH;  
  };

  /**
   * \brief computation of FiniteElement1s33RCM position JACOBIAN (cp-angles and bending)
   * \author Thorsten Schindler
   * \date 2009-04-24 initial commit (Thorsten Schindler)
   */
  class PositionJacobian : public MBSim::Function1<fmatvec::SqrMat,fmatvec::Vec> {
    public:
      /**
       * \brief constructor
       * \param reversed Cardan-Object
       * \param length of FiniteElement1s33RCM
       * \param transpose of difference of right and left beam position
       * \param differentiated centre angles with respect to bending coordinates
       */
      PositionJacobian(RevCardanPtr angle_,double l0_,const fmatvec::RowVec &rRrLmH_,const fmatvec::Mat &pSbE_);
      
      /**
       * \brief destructor
       */
      virtual ~PositionJacobian();
      
      /* INHERITED INTERFACE */
      fmatvec::SqrMat operator()(const fmatvec::Vec &pos, const void * =NULL);
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
      fmatvec::RowVec rRrLmH;

      /** 
       * \brief differentiated centre angles with respect to bending coordinates
       */
      fmatvec::Mat pSbE;
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
      Trafo33RCM(RevCardanPtr angle_,double l0_);
      
      /**
       * \brief destructor
       */
      virtual ~Trafo33RCM();

      /* GETTER / SETTER */
      const fmatvec::SqrMat& getJIG() const;		
      const fmatvec::SqrMat& getJIGt() const;
      const fmatvec::Vec& getpS() const;
      const fmatvec::Vec& getpSt() const;
      const fmatvec::Vec& gettS() const;
      const fmatvec::Vec& getnS() const;
      const fmatvec::Vec& getbS() const;
      const fmatvec::RowVec& getnSH() const;
      const fmatvec::RowVec& getbSH() const;
      const fmatvec::Vec& getntilS() const;
      const fmatvec::Vec& getbtilS() const;
      const fmatvec::RowVec& getntilSH() const;
      const fmatvec::RowVec& getbtilSH() const;
      const fmatvec::Vec& gettSt() const;
      const fmatvec::Vec& getnSt() const;
      const fmatvec::Vec& getbSt() const;
      const fmatvec::RowVec& gettStH() const;
      const fmatvec::RowVec& getnStH() const;
      const fmatvec::RowVec& getbStH() const;
      const fmatvec::RowVec& getntilStH() const;
      const fmatvec::RowVec& getbtilStH() const;
      const fmatvec::SqrMat& gettSpS() const;
      const fmatvec::SqrMat& getnSpS() const;
      const fmatvec::SqrMat& getbSpS() const;
      const fmatvec::SqrMat& getntilSpS() const;
      const fmatvec::SqrMat& getbtilSpS() const;
      const fmatvec::SqrMat& gettSpSt() const;
      const fmatvec::SqrMat& getnSpSt() const;
      const fmatvec::SqrMat& getbSpSt() const;
      const fmatvec::SqrMat& getntilSpSt() const;
      const fmatvec::SqrMat& getbtilSpSt() const;

      const fmatvec::Vec& getqIt() const;
      const double& getepstil() const;
      const double& getepstilt() const;
      const fmatvec::Vec& getrS() const;
      const fmatvec::Vec& getrSt() const;
      const double& getk0() const;
      const double& getk0t() const;
      const fmatvec::Vec& getbe() const;
      const fmatvec::Vec& getbet() const;

      const double& getxintil() const;
      const double& getxibtil() const;
      const double& getetantil() const;
      const double& getetabtil() const;
      const double& getxintilt() const;
      const double& getxibtilt() const;
      const double& getetantilt() const;
      const double& getetabtilt() const;
      const fmatvec::SqrMat& getV() const;
      /***************************************************/

      /**
       * \brief compute the internal coordinates
       * \param global coordinates
       */
      void computeqI(const fmatvec::Vec& qG);
      
      /**
       * \brief compute the Jacobian of the trafo 
       * \param global coordinates
       */
      void computeJIG(const fmatvec::Vec& qG);
      
      /**
       * \brief compute the internal state
       * \param global coordinates
       * \param global velocities
       */
      void computezI(const fmatvec::Vec& qG,const fmatvec::Vec& qGt);
      
      /**
       * \brief compute the internal time differentiated COSY 
       * \param global coordinates
       * \param global velocities
       */
      void computeCOSYt(const fmatvec::Vec& qG,const fmatvec::Vec& qGt);
      
      /**
       * \brief compute the internal coordinates and velocities as well as JIGt 
       * \param global coordinates
       * \param global velocities
       */
      void computeTrafo(const fmatvec::Vec& qG,const fmatvec::Vec& qGt);

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
      fmatvec::Vec rS, pS;

      /** 
       * \brief sum of right and left translational vector as well as transpose of the difference
       */ 
      fmatvec::Vec rRrLp;
      fmatvec::RowVec rRrLmH;

      /** 
       * \brief bending coordinates
       */
      fmatvec::Vec be;

      /** 
       * \brief COSY definitions 
       */
      fmatvec::Vec tS, nS, bS, ntilS, btilS;
      fmatvec::RowVec nSH, bSH, ntilSH, btilSH;
      fmatvec::SqrMat tSpS, nSpS, bSpS, ntilSpS, btilSpS;
      double xibtil, xintil, etabtil, etantil;

      /** 
       * \brief system matrix and right hand side for computebe() 
       */
      fmatvec::Mat SMRHS_Jac;

      /** 
       * \brief derivative of coefficients of bending polynomials w with respect to bending coordinates
       */
      fmatvec::SqrMat V;

      /** 
       * \brief delta matrices for computebe()
       */
      fmatvec::Mat drRdrLp, drRdrLm, pSbE; 		

      /**
       * \brief COSY definitions for computebe() 
       */
      fmatvec::Mat SMRHS, nSbE, bSbE, ntilSbE, btilSbE;
      fmatvec::RowVec xibtilbE, xintilbE, etabtilbE, etantilbE;

      /** 
       * \brief derivative of be and COSY with respect to qG
       */
      fmatvec::Mat beqG, tSqG, nSqG, bSqG, ntilSqG, btilSqG;
      fmatvec::RowVec xintilqG, xibtilqG, etantilqG, etabtilqG;

      /** 
       * \brief Jacobian and differentiated Jacobian
       */
      fmatvec::SqrMat JIG, JIGt;

      /** 
       * \brief derivative of k0, epstilt, qI, rS, be and COSY with respect to time
       */
      double k0t, epstilt;
      fmatvec::Vec qIt, rSt, bet, pSt, tSt, nSt, bSt;
      fmatvec::RowVec tStH, nStH, bStH, ntilStH, btilStH;
      fmatvec::SqrMat tSpSt, nSpSt, bSpSt, ntilSpSt, btilSpSt;
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
      void computeJIGt(const fmatvec::Vec& qGt);	
  };

  inline const fmatvec::SqrMat& Trafo33RCM::getJIG() const { return JIG; }
  inline const fmatvec::SqrMat& Trafo33RCM::getJIGt() const { return JIGt; }
  inline const fmatvec::Vec& Trafo33RCM::getpS() const { return pS; }
  inline const fmatvec::Vec& Trafo33RCM::getpSt() const { return pSt; }
  inline const fmatvec::Vec& Trafo33RCM::gettS() const { return tS; }
  inline const fmatvec::Vec& Trafo33RCM::getnS() const { return nS; }
  inline const fmatvec::Vec& Trafo33RCM::getbS() const { return bS; }
  inline const fmatvec::RowVec& Trafo33RCM::getnSH() const { return nSH; }
  inline const fmatvec::RowVec& Trafo33RCM::getbSH() const { return bSH; }
  inline const fmatvec::Vec& Trafo33RCM::getntilS() const { return ntilS; }
  inline const fmatvec::Vec& Trafo33RCM::getbtilS() const { return btilS; }
  inline const fmatvec::RowVec& Trafo33RCM::getntilSH() const { return ntilSH; }
  inline const fmatvec::RowVec& Trafo33RCM::getbtilSH() const { return btilSH; }
  inline const fmatvec::Vec& Trafo33RCM::gettSt() const { return tSt; }
  inline const fmatvec::Vec& Trafo33RCM::getnSt() const { return nSt; }
  inline const fmatvec::Vec& Trafo33RCM::getbSt() const { return bSt; }
  inline const fmatvec::RowVec& Trafo33RCM::gettStH() const { return tStH; }
  inline const fmatvec::RowVec& Trafo33RCM::getnStH() const { return nStH; }
  inline const fmatvec::RowVec& Trafo33RCM::getbStH() const { return bStH; }
  inline const fmatvec::RowVec& Trafo33RCM::getntilStH() const { return ntilStH; }
  inline const fmatvec::RowVec& Trafo33RCM::getbtilStH() const { return btilStH; }
  inline const fmatvec::SqrMat& Trafo33RCM::gettSpS() const { return tSpS; }
  inline const fmatvec::SqrMat& Trafo33RCM::getnSpS() const { return nSpS; }
  inline const fmatvec::SqrMat& Trafo33RCM::getbSpS() const { return bSpS; }
  inline const fmatvec::SqrMat& Trafo33RCM::getntilSpS() const { return ntilSpS; }
  inline const fmatvec::SqrMat& Trafo33RCM::getbtilSpS() const { return btilSpS; }
  inline const fmatvec::SqrMat& Trafo33RCM::gettSpSt() const { return tSpSt; }
  inline const fmatvec::SqrMat& Trafo33RCM::getnSpSt() const { return nSpSt; }
  inline const fmatvec::SqrMat& Trafo33RCM::getbSpSt() const { return bSpSt; }
  inline const fmatvec::SqrMat& Trafo33RCM::getntilSpSt() const { return ntilSpSt; }
  inline const fmatvec::SqrMat& Trafo33RCM::getbtilSpSt() const { return btilSpSt; }
  inline const fmatvec::Vec& Trafo33RCM::getqIt() const { return qIt; }
  inline const double& Trafo33RCM::getepstil() const { return epstil; }
  inline const double& Trafo33RCM::getepstilt() const { return epstilt; }
  inline const fmatvec::Vec& Trafo33RCM::getrS() const { return rS; }
  inline const fmatvec::Vec& Trafo33RCM::getrSt() const { return rSt; }
  inline const double& Trafo33RCM::getk0() const { return k0; }
  inline const double& Trafo33RCM::getk0t() const { return k0t; }
  inline const fmatvec::Vec& Trafo33RCM::getbe() const { return be; }
  inline const fmatvec::Vec& Trafo33RCM::getbet() const { return bet; }
  inline const double& Trafo33RCM::getxintil() const { return xintil; }
  inline const double& Trafo33RCM::getxibtil() const { return xibtil; }
  inline const double& Trafo33RCM::getetantil() const { return etantil; }
  inline const double& Trafo33RCM::getetabtil() const { return etabtil; }
  inline const double& Trafo33RCM::getxintilt() const { return xintilt; }
  inline const double& Trafo33RCM::getxibtilt() const { return xibtilt; }
  inline const double& Trafo33RCM::getetantilt() const { return etantilt; }
  inline const double& Trafo33RCM::getetabtilt() const { return etabtilt; }
  inline const fmatvec::SqrMat& Trafo33RCM::getV() const { return V; } 
  /*******************************************************************/

}

#endif /*TRAFO33RCM_H_*/

