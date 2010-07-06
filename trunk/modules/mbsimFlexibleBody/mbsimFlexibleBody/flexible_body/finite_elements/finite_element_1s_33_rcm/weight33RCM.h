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

#ifndef WEIGHT33RCM_H_
#define WEIGHT33RCM_H_

#include "fmatvec.h"

namespace MBSimFlexibleBody {

  class Trafo33RCM;

  /**
   * \brief integrals of bending parametrisation for FiniteElement1s33RCM
   * \author Thorsten Schindler
   * \date 2009-04-24 initial commit (Thorsten Schindler) 
   */
  class Weight33RCM {
    public:
      /**
       * \brief constructor
       */
      Weight33RCM(double l0_,double l0h2_,double l0h3_,Trafo33RCM* tf_);
      /**
       * \brief Destructor */
      virtual ~Weight33RCM();

      /* GETTER / SETTER */
      /**
       * \param first curvature
       * \param second curvature
       */
      void setCurvature(double k10_,double k20_);
      
      /**
       * \param number of Gauss points
       */
      void setGauss(int nGauss);

      const fmatvec::RowVec& getvxvt() const;
      const fmatvec::Vec& getvxvtH() const;
      const fmatvec::RowVec& getxvxvt() const;
      const fmatvec::Vec& getxvxvtH() const;
      const fmatvec::SymMat& getvxvtwxwt() const;
      const fmatvec::RowVec& getvvt() const;
      const fmatvec::Vec& getvvtH() const;
      const fmatvec::RowVec& getxvvt() const;
      const fmatvec::Vec& getxvvtH() const;
      const fmatvec::SymMat& getvvtwwt() const;
      
      const double& getIwh1() const;
      const double& getIwh2() const;
      const double& getIwh1t() const;	
      const double& getIwh2t() const;
      const double& getIxwh1() const;
      const double& getIxwh2() const;
      const double& getIxwh1t() const;
      const double& getIxwh2t() const;		
      const double& getIwh1twh1() const;
      const double& getIwh1twh2() const;
      const double& getIwh1twh1t() const;
      const double& getIwh1wh1() const;
      const double& getIwh1wh2t() const;		
      const double& getIwh1wh2() const;
      const double& getIwh2twh2t() const;
      const double& getIwh2twh2() const;
      const double& getIwh2wh2() const;
      const double& getIwh1twh2t() const;		
      const double& getIwh1xwh1x() const;
      const double& getIwh2xwh2x() const;
      const double& getIwh1xxwh1xx() const;
      const double& getIwh2xxwh2xx() const;
      const fmatvec::RowVec& getIwh1xxwxxwt() const;
      const fmatvec::RowVec& getIwh2xxwxxwt() const;
      const fmatvec::RowVec& getIwh1wwt() const;
      const fmatvec::RowVec& getIwh2wwt() const;
      const fmatvec::Vec& getIwh1wwtH() const;
      const fmatvec::Vec& getIwh2wwtH() const;
      const fmatvec::RowVec& getIwh1twwt() const;			
      const fmatvec::RowVec& getIwh2twwt() const;
      const fmatvec::Vec& getIwh1twwtH() const;			
      const fmatvec::Vec& getIwh2twwtH() const;
      const fmatvec::RowVec& getIwh1xwxwt() const;
      const fmatvec::RowVec& getIwh2xwxwt() const;

      const fmatvec::Vec& getw1coef() const;
      const fmatvec::Vec& getw2coef() const;
      const fmatvec::Vec& getw1tcoef() const;
      const fmatvec::Vec& getw2tcoef() const;
      const fmatvec::Mat& getw1coefqI() const;
      const fmatvec::Mat& getw2coefqI() const;
      const fmatvec::Vec& getwh1coef() const;
      const fmatvec::Vec& getwh2coef() const;
      const fmatvec::Vec& getwh1tcoef() const;
      const fmatvec::Vec& getwh2tcoef() const;		
      const fmatvec::Mat& getwh1coefqI() const;
      const fmatvec::Mat& getwh2coefqI() const;
      fmatvec::Mat getwh1coefqIH() const;
      fmatvec::Mat getwh2coefqIH() const;
      const fmatvec::Mat& getwh1tcoefqI() const;
      const fmatvec::Mat& getwh2tcoefqI() const;
      const fmatvec::Mat& getwh1coefqInunutH() const;
      const fmatvec::Mat& getwh2coefqInunutH() const;

      const fmatvec::Mat& gettSqI() const;
      const fmatvec::Mat& getnSqI() const;
      const fmatvec::Mat& getbSqI() const;
      const fmatvec::Mat& getnSqIH() const;
      const fmatvec::Mat& getbSqIH() const;
      const fmatvec::Mat& gettStqI() const;
      const fmatvec::Mat& getnStqI() const;
      const fmatvec::Mat& getbStqI() const;

      const double& getTtil() const;
      const fmatvec::RowVec& getTtilqI() const;
      const fmatvec::SymMat& getTtilqItqIt() const;
      const fmatvec::Vec& getTtilqItqIqIt() const;

      const fmatvec::Mat& getdpS() const;		
      const fmatvec::Mat& getdpSH() const;
      /***************************************************/

      /* BASIC INTEGRALS */
      double intv(const fmatvec::Vec& vt) const;
      double intvx(const fmatvec::Vec& vt) const;
      double intxv(const fmatvec::Vec& vt) const;
      double intxvx(const fmatvec::Vec& vt) const;
      double intvw(const fmatvec::Vec& vt,const fmatvec::Vec& wt) const;
      double intvxwx(const fmatvec::Vec& vt,const fmatvec::Vec& wt) const;		
      double intvxxvxx(const fmatvec::Vec& vt,double C) const;
      void intvvt();		
      void intvvtH();
      void intvxvt();		
      void intvxvtH();		
      void intxvvt();		
      void intxvvtH();
      void intxvxvt();		
      void intxvxvtH();		
      fmatvec::RowVec intvwwt(const fmatvec::Vec& vt) const;
      fmatvec::RowVec intvxwxwt(const fmatvec::Vec& vt) const;
      fmatvec::RowVec intvxxwxxwt(const fmatvec::Vec& vt,double C) const;
      void intvvtwwt();
      void intvxvtwxwt();
      /***************************************************/

      /**
       * \brief computes the integrals of bending polynomials 
       * \param global coordinates
       * \param global velocities
       */
      void computeint(const fmatvec::Vec& qG,const fmatvec::Vec& qGt);
      
      /**
       * \brief computes the vector integrals of bending polynomials 
       * \param global coordinates
       * \param global velocities
       */
      void computeintD(const fmatvec::Vec& qG,const fmatvec::Vec& qGt);
      
      /**
       * \brief computes the coefficients of bending polynomials w 
       * \param global coordinates
       */
      void computewcoefPos(const fmatvec::Vec& qG);
      
      /**
       * \brief computes the time differentiated coefficients of bending polynomials w 
       * \param global coordinates
       * \param global velocities
       */
      void computewcoefVel(const fmatvec::Vec& qG,const fmatvec::Vec& qGt);
      
      /**
       * \brief computes the coefficients of bending polynomials w and wh
       * \param global coordinates
       */
      void computewhcoefPos(const fmatvec::Vec& qG);
      
      /**
       * \brief computes the coefficients of bending polynomials w and wh and their time derivatives 
       * \param global coordinates
       * \param global velocities
       */
      void computewhcoefVel(const fmatvec::Vec& qG,const fmatvec::Vec& qGt);
      
      /**
       * \brief computes the derivative of w-coefficients with respect to bending coordinates
       */
      void computewcoefPosD();
      
      /**
       * \brief computes bending polynomial values on position level
       * \param global coordinates
       */
      void computewhcoefPosD(const fmatvec::Vec& qG);
      
      /**
       * \brief computes the coefficients of w and wt
       * \param left translational deflection
       * \param right translational deflection
       * \param left rotational deflection
       * \param right rotational deflection
       */
      fmatvec::Vec computewcoef(double dL,double dR,double bL,double bR) const;	
      
      /**
       * \brief evaluates the bending polynomial and its x-derivative
       * \param point of evaluation 
       */
      fmatvec::Vec computew(const fmatvec::Vec& wt,double x) const;

    private:
      /** 
       * \brief Trafo-Object
       */
      Trafo33RCM* tf;

      /** 
       * \brief length of FEM1s33RCM
       */
      double l0, l0h2, l0h3, l0h4, l0h5, l0h7, l0h9, l0h11;

      /** 
       * \brief predefined bendings
       */
      double k10, k20;

      /** 
       * \brief general integrals
       */
      fmatvec::RowVec Ivvt, Ivxvt, Ixvvt, Ixvxvt;
      fmatvec::SymMat Ivvtwwt, Ivxvtwxwt;
      fmatvec::Vec IvvtH, IvxvtH, IxvvtH, IxvxvtH;

      /**
       * \brief special integrals
       */
      double Iwh1, Iwh2, Iwh1t, Iwh2t, Ixwh1, Ixwh2, Ixwh1t, Ixwh2t;
      double Iwh1twh1, Iwh1twh2, Iwh1twh1t, Iwh1wh1, Iwh1wh2t, Iwh1wh2, Iwh2twh2t;	
      double Iwh2twh2, Iwh2wh2, Iwh1twh2t;
      fmatvec::RowVec Iwh1wwt, Iwh1twwt, Iwh2wwt, Iwh2twwt;
      fmatvec::Vec Iwh1wwtH, Iwh1twwtH, Iwh2wwtH, Iwh2twwtH;

      double Iwh1xwh1x, Iwh2xwh2x, Iwh1xxwh1xx, Iwh2xxwh2xx;
      fmatvec::RowVec Iwh1xwxwt, Iwh2xwxwt;
      fmatvec::RowVec Iwh1xxwxxwt, Iwh2xxwxxwt;

      /** 
       * \brief bending coefficients
       */
      fmatvec::Vec w1coef, w2coef, w1tcoef, w2tcoef;
      fmatvec::Vec wh1coef, wh2coef, wh1tcoef, wh2tcoef;
      fmatvec::Mat w1coefqI, w2coefqI;
      fmatvec::Mat wh1coefqI, wh2coefqI, wh1tcoefqI, wh2tcoefqI;
      fmatvec::Mat wh1coefqInunutH, wh2coefqInunutH;

      /**
       * \brief COSY 
       */
      fmatvec::Mat tSqI, nSqI, bSqI;
      fmatvec::Mat tStqI, nStqI, bStqI;
      fmatvec::Mat nSqIH, bSqIH;
      fmatvec::Mat ntilSqI, btilSqI;
      fmatvec::RowVec xintilqI, xibtilqI, etantilqI, etabtilqI;

      /** 
       * \brief omgtS
       */
      double omgt;
      fmatvec::RowVec omgtqI, omgtqIt;
      fmatvec::Vec omgtqItqIqIt;

      /**
       * \brief rotational kinetic energy 
       */
      double Ttil;
      fmatvec::RowVec TtilqI;
      fmatvec::SymMat TtilqItqIt;
      fmatvec::Vec TtilqItqIqIt;

      /** 
       * \brief Gauss integration
       */
      fmatvec::Vec gp, xip; // cannot be initialised in constructor
      double bam;

      /** 
       * \brief delta matrix for pS
       */
      fmatvec::Mat dpS, dpSH;

      /**
       * \brief computes the integrals of bending polynomials 
       */
      void computeint();
      
      /**
       * \brief computes the coefficients of bending polynomials w 
       */
      void computewcoefPos();
      
      /**
       * \brief computes the time differentiated coefficients of bending polynomials w 
       */
      void computewcoefVel();
      
      /**
       * \brief computes the coefficients of bending polynomials w and wh
       */
      void computewhcoefPos();
      
      /**
       * \brief computes the coefficients of bending polynomials w and wh and their time derivatives
       */
      void computewhcoefVel();
      
      /**
       * \brief computes bending polynomial values on position level
       */
      void computewhcoefPosD();
      
      /**
       * \brief computes bending polynomial values on velocity level
       */
      void computewhcoefVelD();

      /**
       * \brief compute angular velocity around tangent */
      void computeomgt(double x);
      
      /**
       * \brief compute rotational kinetic energy
       */
      void computeT();
      
      /**
       * \brief compute delta matrix for CP with respect to rotation 
       */
      void computedpS();			
  };

  inline void Weight33RCM::setCurvature(double k10_,double k20_) { k10 = k10_; k20 = k20_; }
  inline void Weight33RCM::intvvtH() { IvvtH = Ivvt.T(); }
  inline void Weight33RCM::intvxvtH() { IvxvtH = Ivxvt.T(); }	
  inline void Weight33RCM::intxvvtH() { IxvvtH = Ixvvt.T(); }
  inline void Weight33RCM::intxvxvtH() { IxvxvtH = Ixvxvt.T(); }

  inline const fmatvec::RowVec& Weight33RCM::getvxvt() const { return Ivxvt; }
  inline const fmatvec::Vec& Weight33RCM::getvxvtH() const { return IvxvtH; }
  inline const fmatvec::RowVec& Weight33RCM::getxvxvt() const { return Ixvxvt; }
  inline const fmatvec::Vec& Weight33RCM::getxvxvtH() const { return IxvxvtH; }
  inline const fmatvec::SymMat& Weight33RCM::getvxvtwxwt() const { return Ivxvtwxwt; }
  inline const fmatvec::RowVec& Weight33RCM::getvvt() const { return Ivvt; }
  inline const fmatvec::Vec& Weight33RCM::getvvtH() const { return IvvtH; }
  inline const fmatvec::RowVec& Weight33RCM::getxvvt() const { return Ixvvt; }
  inline const fmatvec::Vec& Weight33RCM::getxvvtH() const { return IxvvtH; }
  inline const fmatvec::SymMat& Weight33RCM::getvvtwwt() const { return Ivvtwwt; }

  inline const double& Weight33RCM::getIwh1() const { return Iwh1; }
  inline const double& Weight33RCM::getIwh2() const { return Iwh2; }
  inline const double& Weight33RCM::getIwh1t() const { return Iwh1t; }	
  inline const double& Weight33RCM::getIwh2t() const { return Iwh2t; }
  inline const double& Weight33RCM::getIxwh1() const { return Ixwh1; }
  inline const double& Weight33RCM::getIxwh2() const { return Ixwh2; }
  inline const double& Weight33RCM::getIxwh1t() const { return Ixwh1t; }
  inline const double& Weight33RCM::getIxwh2t() const { return Ixwh2t; }		
  inline const double& Weight33RCM::getIwh1twh1() const { return Iwh1twh1; }
  inline const double& Weight33RCM::getIwh1twh2() const { return Iwh1twh2; }
  inline const double& Weight33RCM::getIwh1twh1t() const { return Iwh1twh1t; }
  inline const double& Weight33RCM::getIwh1wh1() const { return Iwh1wh1; }
  inline const double& Weight33RCM::getIwh1wh2t() const { return Iwh1wh2t; }		
  inline const double& Weight33RCM::getIwh1wh2() const { return Iwh1wh2; }
  inline const double& Weight33RCM::getIwh2twh2t() const { return Iwh2twh2t; }
  inline const double& Weight33RCM::getIwh2twh2() const { return Iwh2twh2; }
  inline const double& Weight33RCM::getIwh2wh2() const { return Iwh2wh2; }
  inline const double& Weight33RCM::getIwh1twh2t() const { return Iwh1twh2t; }		
  inline const double& Weight33RCM::getIwh1xwh1x() const { return Iwh1xwh1x; }
  inline const double& Weight33RCM::getIwh2xwh2x() const { return Iwh2xwh2x; }
  inline const double& Weight33RCM::getIwh1xxwh1xx() const { return Iwh1xxwh1xx; }
  inline const double& Weight33RCM::getIwh2xxwh2xx() const { return Iwh2xxwh2xx; }	
  inline const fmatvec::RowVec& Weight33RCM::getIwh1xxwxxwt() const { return Iwh1xxwxxwt; }
  inline const fmatvec::RowVec& Weight33RCM::getIwh2xxwxxwt() const { return Iwh2xxwxxwt; }
  inline const fmatvec::RowVec& Weight33RCM::getIwh1wwt() const { return Iwh1wwt; }
  inline const fmatvec::RowVec& Weight33RCM::getIwh2wwt() const { return Iwh2wwt; }
  inline const fmatvec::Vec& Weight33RCM::getIwh1wwtH() const { return Iwh1wwtH; }
  inline const fmatvec::Vec& Weight33RCM::getIwh2wwtH() const { return Iwh2wwtH; }
  inline const fmatvec::RowVec& Weight33RCM::getIwh1twwt() const { return Iwh1twwt; }			
  inline const fmatvec::RowVec& Weight33RCM::getIwh2twwt() const { return Iwh2twwt; }
  inline const fmatvec::Vec& Weight33RCM::getIwh1twwtH() const { return Iwh1twwtH; }			
  inline const fmatvec::Vec& Weight33RCM::getIwh2twwtH() const { return Iwh2twwtH; }
  inline const fmatvec::RowVec& Weight33RCM::getIwh1xwxwt() const { return Iwh1xwxwt; }
  inline const fmatvec::RowVec& Weight33RCM::getIwh2xwxwt() const { return Iwh2xwxwt; }

  inline const fmatvec::Vec& Weight33RCM::getw1coef() const { return w1coef; }
  inline const fmatvec::Vec& Weight33RCM::getw2coef() const { return w2coef; }
  inline const fmatvec::Vec& Weight33RCM::getw1tcoef() const { return w1tcoef; }
  inline const fmatvec::Vec& Weight33RCM::getw2tcoef() const { return w2tcoef; }
  inline const fmatvec::Mat& Weight33RCM::getw1coefqI() const { return w1coefqI; }
  inline const fmatvec::Mat& Weight33RCM::getw2coefqI() const { return w2coefqI; }
  inline const fmatvec::Vec& Weight33RCM::getwh1coef() const { return wh1coef; }
  inline const fmatvec::Vec& Weight33RCM::getwh2coef() const { return wh2coef; }
  inline const fmatvec::Vec& Weight33RCM::getwh1tcoef() const { return wh1tcoef; }
  inline const fmatvec::Vec& Weight33RCM::getwh2tcoef() const { return wh2tcoef; }
  inline const fmatvec::Mat& Weight33RCM::getwh1coefqI() const { return wh1coefqI; }
  inline const fmatvec::Mat& Weight33RCM::getwh2coefqI() const { return wh2coefqI; }
  inline fmatvec::Mat Weight33RCM::getwh1coefqIH() const { return wh1coefqI.T(); }
  inline fmatvec::Mat Weight33RCM::getwh2coefqIH() const { return wh2coefqI.T(); }
  inline const fmatvec::Mat& Weight33RCM::getwh1tcoefqI() const { return wh1tcoefqI; }
  inline const fmatvec::Mat& Weight33RCM::getwh2tcoefqI() const { return wh2tcoefqI; }
  inline const fmatvec::Mat& Weight33RCM::getwh1coefqInunutH() const { return wh1coefqInunutH; }
  inline const fmatvec::Mat& Weight33RCM::getwh2coefqInunutH() const { return wh2coefqInunutH; }

  inline const fmatvec::Mat& Weight33RCM::gettSqI() const { return tSqI; }
  inline const fmatvec::Mat& Weight33RCM::getnSqI() const { return nSqI; }
  inline const fmatvec::Mat& Weight33RCM::getbSqI() const { return bSqI; }
  inline const fmatvec::Mat& Weight33RCM::getnSqIH() const { return nSqIH; }
  inline const fmatvec::Mat& Weight33RCM::getbSqIH() const { return bSqIH; }
  inline const fmatvec::Mat& Weight33RCM::gettStqI() const { return tStqI; }
  inline const fmatvec::Mat& Weight33RCM::getnStqI() const { return nStqI; }
  inline const fmatvec::Mat& Weight33RCM::getbStqI() const { return bStqI; }

  inline const double& Weight33RCM::getTtil() const { return Ttil; }
  inline const fmatvec::RowVec& Weight33RCM::getTtilqI() const { return TtilqI; }
  inline const fmatvec::SymMat& Weight33RCM::getTtilqItqIt() const { return TtilqItqIt; }
  inline const fmatvec::Vec& Weight33RCM::getTtilqItqIqIt() const { return TtilqItqIqIt; }

  inline const fmatvec::Mat& Weight33RCM::getdpS() const { return dpS; }
  inline const fmatvec::Mat& Weight33RCM::getdpSH() const { return dpSH; }
  /*******************************************************************/

}

#endif /*WEIGHT33RCM_H_*/

