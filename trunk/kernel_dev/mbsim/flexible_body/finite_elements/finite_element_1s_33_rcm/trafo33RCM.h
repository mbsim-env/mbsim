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

namespace MBSim {

  class RevCardan;

  /*! 
   * \brief computation of FiniteElement1s33RCM position coordinates (cp-angles and bending)
   * \author Thorsten Schindler
   * \date 2009-04-24 initial commit (Thorsten Schindler)
   */
  class PositionFunction : public MBSim::Function<fmatvec::Vec,fmatvec::Vec> {    
    public:
      /*! Constructor */
      PositionFunction(RevCardan *angle_,double l0_,const fmatvec::Vec& pL_,const fmatvec::Vec& pR_,
          double cL1_,double cR1_,double cL2_,double cR2_,const fmatvec::RowVec& rRrLmH_);
      /*! Destructor */
      ~PositionFunction();
      /*! Rootfunction for computation of position coordinates (CP-angles and bending) */
      fmatvec::Vec operator()(const fmatvec::Vec& pos);

    private:
      /** reversed Cardan-Object */
      RevCardan* angle;

      /** length of FEM1s33RCM */
      double l0;

      /** angles of COG of rigid left-right connection */
      fmatvec::Vec pL, pR;

      /** global bending parameters */
      double cL1, cR1, cL2, cR2;

      /** trans(rR-rL) */
      fmatvec::RowVec rRrLmH;  
  };

  /*! 
   * \brief computation of FiniteElement1s33RCM position JACOBIAN (cp-angles and bending)
   * \author Thorsten Schindler
   * \date 2009-04-24 initial commit (Thorsten Schindler)
   */
  class PositionJacobian : public MBSim::Function<fmatvec::SqrMat,fmatvec::Vec> {
    public:
      /*! Constructor */
      PositionJacobian(RevCardan *angle_,double l0_,const fmatvec::RowVec &rRrLmH_,const fmatvec::Mat &pSbE_);
      /*! Destructor */
      ~PositionJacobian();
      /*! Jacobian for computation of position coordinates (CP-angles and bending) */
      fmatvec::SqrMat operator()(const fmatvec::Vec &pos);

    private:
      /** reversed Cardan-Object */
      RevCardan* angle;

      /** length of FEM1s33RCM */
      double l0;

      /** trans(rR-rL) */
      fmatvec::RowVec rRrLmH;

      /** pS differentiated with respect to bending coordinates */
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
      /*! Constructor */
      Trafo33RCM(RevCardan *angle_,double l0_);
      /*! Destructor */
      virtual ~Trafo33RCM();

      /*! Compute the internal coordinates */
      void computeqI(const fmatvec::Vec& qG);
      /*! Compute the Jacobian of the trafo */
      void computeJIG(const fmatvec::Vec& qG);
      /*! Compute the internal state */
      void computezI(const fmatvec::Vec& qG,const fmatvec::Vec& qGt);
      /*! Compute the internal time differentiated COSY */
      void computeCOSYt(const fmatvec::Vec& qG,const fmatvec::Vec& qGt);
      /*! Compute the internal coordinates and velocities as well as JIGt */
      void computeTrafo(const fmatvec::Vec& qG,const fmatvec::Vec& qGt);

      /*! Return JIG */
      fmatvec::SqrMat getJIG() const;		
      /*! Return JIGt */
      fmatvec::SqrMat getJIGt() const;

      /*! Return pS */
      fmatvec::Vec getpS() const;
      /*! Return pSt */
      fmatvec::Vec getpSt() const;
      /*! Return tS */
      fmatvec::Vec gettS() const;
      /*! Return nS */
      fmatvec::Vec getnS() const;
      /*! Return bS */
      fmatvec::Vec getbS() const;
      /*! Return nSH */
      fmatvec::RowVec getnSH() const;
      /*! Return bSH */
      fmatvec::RowVec getbSH() const;
      /*! Return ntilS */
      fmatvec::Vec getntilS() const;
      /*! Return btilS */
      fmatvec::Vec getbtilS() const;
      /*! Return ntilSH */
      fmatvec::RowVec getntilSH() const;
      /*! Return btilSH */
      fmatvec::RowVec getbtilSH() const;
      /*! Return tSt */
      fmatvec::Vec gettSt() const;
      /*! Return nSt */
      fmatvec::Vec getnSt() const;
      /*! Return bSt */
      fmatvec::Vec getbSt() const;
      /*! Return tStH */
      fmatvec::RowVec gettStH() const;
      /*! Return nStH */
      fmatvec::RowVec getnStH() const;
      /*! Return bStH */
      fmatvec::RowVec getbStH() const;
      /*! Return ntilStH */
      fmatvec::RowVec getntilStH() const;
      /*! Return btilStH */
      fmatvec::RowVec getbtilStH() const;
      /*! Return tSpS */
      fmatvec::SqrMat gettSpS() const;
      /*! Return nSpS */
      fmatvec::SqrMat getnSpS() const;
      /*! Return bSpS */
      fmatvec::SqrMat getbSpS() const;
      /*! Return ntilSpS */
      fmatvec::SqrMat getntilSpS() const;
      /*! Return btilSpS */
      fmatvec::SqrMat getbtilSpS() const;
      /*! Return tSpSt */
      fmatvec::SqrMat gettSpSt() const;
      /*! Return nSpSt */
      fmatvec::SqrMat getnSpSt() const;
      /*! Return bSpSt */
      fmatvec::SqrMat getbSpSt() const;
      /*! Return ntilSpSt */
      fmatvec::SqrMat getntilSpSt() const;
      /*! Return btilSpSt */
      fmatvec::SqrMat getbtilSpSt() const;

      /*! Return qIt */
      fmatvec::Vec getqIt() const;
      /*! Return epstil */
      double getepstil() const;
      /*! Return epstilt */
      double getepstilt() const;
      /*! Return rS */
      fmatvec::Vec getrS() const;
      /*! Return rSt */
      fmatvec::Vec getrSt() const;
      /*! Return k0 */
      double getk0() const;
      /*! Return k0t */
      double getk0t() const;
      /*! Return be */
      fmatvec::Vec getbe() const;
      /*! Return bet */
      fmatvec::Vec getbet() const;

      /*! Return xintil */
      double getxintil() const;
      /*! Return xibtil */
      double getxibtil() const;
      /*! Return etantil */
      double getetantil() const;
      /*! Return etabtil */
      double getetabtil() const;
      /*! Return xintilt */
      double getxintilt() const;
      /*! Return xibtilt */
      double getxibtilt() const;
      /*! Return etantilt */
      double getetantilt() const;
      /*! Return etabtilt */
      double getetabtilt() const;

      /*! Return V */
      fmatvec::SqrMat getV() const;

    private:
      /** reversed Cardan-Object */
      RevCardan* angle;

      /** length of FEM1s33RCM and powers */
      double l0, l0h2, l0h3, l0h4, l0h5;
      double xstar, xstarh2, xstarh3;

      /** internal coordinates of the FEM1s33RCM */
      double epstil, k0;
      fmatvec::Vec rS, pS;

      /** sum of right and left translational vector as well as transpose of the difference */ 
      fmatvec::Vec rRrLp;
      fmatvec::RowVec rRrLmH;

      /** bending coordinates */
      fmatvec::Vec be;

      /** COSY definitions */
      fmatvec::Vec tS, nS, bS, ntilS, btilS;
      fmatvec::RowVec nSH, bSH, ntilSH, btilSH;
      fmatvec::SqrMat tSpS, nSpS, bSpS, ntilSpS, btilSpS;
      double xibtil, xintil, etabtil, etantil;

      /** system matrix and right hand side for computebe() */
      fmatvec::Mat SMRHS_Jac;

      /** derivative of coefficients of bending polynomials w with respect to bending coordinates */
      fmatvec::SqrMat V;

      /** delta matrices for computebe() */
      fmatvec::Mat drRdrLp, drRdrLm, pSbE; 		

      /** COSY definitions for computebe() */
      fmatvec::Mat SMRHS, nSbE, bSbE, ntilSbE, btilSbE;
      fmatvec::RowVec xibtilbE, xintilbE, etabtilbE, etantilbE;

      /** derivative of be and COSY with respect to qG */
      fmatvec::Mat beqG, tSqG, nSqG, bSqG, ntilSqG, btilSqG;
      fmatvec::RowVec xintilqG, xibtilqG, etantilqG, etabtilqG;

      /** Jacobian and differentiated Jacobian */
      fmatvec::SqrMat JIG, JIGt;

      /** derivative of k0, epstilt, qI, rS, be and COSY with respect to time */
      double k0t, epstilt;
      fmatvec::Vec qIt, rSt, bet, pSt, tSt, nSt, bSt;
      fmatvec::RowVec tStH, nStH, bStH, ntilStH, btilStH;
      fmatvec::SqrMat tSpSt, nSpSt, bSpSt, ntilSpSt, btilSpSt;
      double xibtilt, xintilt, etabtilt, etantilt;

      /*! Computes preliminaries */
      void computeprelim(const fmatvec::Vec& qG);
      /*! Compute the initial value for computebe() */
      fmatvec::Vec computes0(const fmatvec::Vec& qG);
      /*! Compute the angle and bending positions */
      void computebe(const fmatvec::Vec& qG);
      /*! Compute the COSY */
      void computeCOSY();
      /*! Compute the CP, prolongation and torsion */
      void computerSepstk0(const fmatvec::Vec& qG);
      /*! Compute delta matrix for right and left beam end */
      void computedrRdrL();
      /*! Compute the derivative of coefficients of bending polynomials w with respect to bending coordinates */
      void computeV();
      /*! Compute the derivative of bE with respect to qG */
      void computebeqG();
      /*! Compute the derivative of COSY with respect to qG */
      void computeCOSYqG();
      /*! Compute the derivative of COSY with respect to time */
      void computeCOSYt();
      /*! Compute the derivative of JIG with respect to time */
      void computeJIGt(const fmatvec::Vec& qGt);	
  };

  inline fmatvec::SqrMat Trafo33RCM::getJIG() const {return JIG;}
  inline fmatvec::SqrMat Trafo33RCM::getJIGt() const {return JIGt;}
  inline fmatvec::Vec Trafo33RCM::getpS() const {return pS;}
  inline fmatvec::Vec Trafo33RCM::getpSt() const {return pSt;}
  inline fmatvec::Vec Trafo33RCM::gettS() const {return tS;}
  inline fmatvec::Vec Trafo33RCM::getnS() const {return nS;}
  inline fmatvec::Vec Trafo33RCM::getbS() const {return bS;}
  inline fmatvec::RowVec Trafo33RCM::getnSH() const {return nSH;}
  inline fmatvec::RowVec Trafo33RCM::getbSH() const {return bSH;}
  inline fmatvec::Vec Trafo33RCM::getntilS() const {return ntilS;}
  inline fmatvec::Vec Trafo33RCM::getbtilS() const {return btilS;}
  inline fmatvec::RowVec Trafo33RCM::getntilSH() const {return ntilSH;}
  inline fmatvec::RowVec Trafo33RCM::getbtilSH() const {return btilSH;}
  inline fmatvec::Vec Trafo33RCM::gettSt() const {return tSt;}
  inline fmatvec::Vec Trafo33RCM::getnSt() const {return nSt;}
  inline fmatvec::Vec Trafo33RCM::getbSt() const {return bSt;}
  inline fmatvec::RowVec Trafo33RCM::gettStH() const {return tStH;}
  inline fmatvec::RowVec Trafo33RCM::getnStH() const {return nStH;}
  inline fmatvec::RowVec Trafo33RCM::getbStH() const {return bStH;}
  inline fmatvec::RowVec Trafo33RCM::getntilStH() const {return ntilStH;}
  inline fmatvec::RowVec Trafo33RCM::getbtilStH() const {return btilStH;}
  inline fmatvec::SqrMat Trafo33RCM::gettSpS() const {return tSpS;}
  inline fmatvec::SqrMat Trafo33RCM::getnSpS() const {return nSpS;}
  inline fmatvec::SqrMat Trafo33RCM::getbSpS() const {return bSpS;}
  inline fmatvec::SqrMat Trafo33RCM::getntilSpS() const {return ntilSpS;}
  inline fmatvec::SqrMat Trafo33RCM::getbtilSpS() const {return btilSpS;}
  inline fmatvec::SqrMat Trafo33RCM::gettSpSt() const {return tSpSt;}
  inline fmatvec::SqrMat Trafo33RCM::getnSpSt() const {return nSpSt;}
  inline fmatvec::SqrMat Trafo33RCM::getbSpSt() const {return bSpSt;}
  inline fmatvec::SqrMat Trafo33RCM::getntilSpSt() const {return ntilSpSt;}
  inline fmatvec::SqrMat Trafo33RCM::getbtilSpSt() const {return btilSpSt;}
  inline fmatvec::Vec Trafo33RCM::getqIt() const {return qIt;}
  inline double Trafo33RCM::getepstil() const {return epstil;}
  inline double Trafo33RCM::getepstilt() const {return epstilt;}
  inline fmatvec::Vec Trafo33RCM::getrS() const {return rS;}
  inline fmatvec::Vec Trafo33RCM::getrSt() const {return rSt;}
  inline double Trafo33RCM::getk0() const {return k0;}
  inline double Trafo33RCM::getk0t() const {return k0t;}
  inline fmatvec::Vec Trafo33RCM::getbe() const {return be;}
  inline fmatvec::Vec Trafo33RCM::getbet() const {return bet;}
  inline double Trafo33RCM::getxintil() const {return xintil;}
  inline double Trafo33RCM::getxibtil() const {return xibtil;}
  inline double Trafo33RCM::getetantil() const {return etantil;}
  inline double Trafo33RCM::getetabtil() const {return etabtil;}
  inline double Trafo33RCM::getxintilt() const {return xintilt;}
  inline double Trafo33RCM::getxibtilt() const {return xibtilt;}
  inline double Trafo33RCM::getetantilt() const {return etantilt;}
  inline double Trafo33RCM::getetabtilt() const {return etabtilt;}
  inline fmatvec::SqrMat Trafo33RCM::getV() const {return V;}
  /*******************************************************************/

}

#endif /*TRAFO33RCM_H_*/

