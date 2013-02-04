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

#ifndef WEIGHT33RCM_H_
#define WEIGHT33RCM_H_

#include "fmatvec.h"
#include "mbsimFlexibleBody/pointer.h"
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_33_rcm/weight33RCM.h"
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_33_rcm/trafo33RCM.h"
#include "mbsim/mbsim_event.h"

namespace MBSimFlexibleBody {

  /**
   * \brief integrals of bending parametrisation for FiniteElement1s33RCM
   * \author Thorsten Schindler
   * \date 2009-04-24 initial commit (Thorsten Schindler) 
   *
   * \todo: use fmatvec2.0
   */
  class Weight33RCM {
    public:
      /**
       * \brief constructor
       */
      Weight33RCM(double l0_,double l0h2_,double l0h3_,Trafo33RCMPtr tf_);
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

      const fmatvec::RowVec4& getvxvt() const;
      const fmatvec::Vec4& getvxvtH() const;
      const fmatvec::RowVec4& getxvxvt() const;
      const fmatvec::Vec4& getxvxvtH() const;
      const fmatvec::SymMat4& getvxvtwxwt() const;
      const fmatvec::RowVec4& getvvt() const;
      const fmatvec::Vec4& getvvtH() const;
      const fmatvec::RowVec4& getxvvt() const;
      const fmatvec::Vec4& getxvvtH() const;
      const fmatvec::SymMat4& getvvtwwt() const;
      
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
      const fmatvec::RowVec4& getIwh1xxwxxwt() const;
      const fmatvec::RowVec4& getIwh2xxwxxwt() const;
      const fmatvec::RowVec4& getIwh1wwt() const;
      const fmatvec::RowVec4& getIwh2wwt() const;
      const fmatvec::Vec4& getIwh1wwtH() const;
      const fmatvec::Vec4& getIwh2wwtH() const;
      const fmatvec::RowVec4& getIwh1twwt() const;
      const fmatvec::RowVec4& getIwh2twwt() const;
      const fmatvec::Vec4& getIwh1twwtH() const;
      const fmatvec::Vec4& getIwh2twwtH() const;
      const fmatvec::RowVec4& getIwh1xwxwt() const;
      const fmatvec::RowVec4& getIwh2xwxwt() const;

      const fmatvec::Vec4& getw1coef() const;
      const fmatvec::Vec4& getw2coef() const;
      const fmatvec::Vec4& getw1tcoef() const;
      const fmatvec::Vec4& getw2tcoef() const;
      const fmatvec::Mat4x16& getw1coefqI() const;
      const fmatvec::Mat4x16& getw2coefqI() const;
      const fmatvec::Vec4& getwh1coef() const;
      const fmatvec::Vec4& getwh2coef() const;
      const fmatvec::Vec4& getwh1tcoef() const;
      const fmatvec::Vec4& getwh2tcoef() const;
      const fmatvec::Mat4x16& getwh1coefqI() const;
      const fmatvec::Mat4x16& getwh2coefqI() const;
      fmatvec::Mat16x4 getwh1coefqIH() const;
      fmatvec::Mat16x4 getwh2coefqIH() const;
      const fmatvec::Mat4x16& getwh1tcoefqI() const;
      const fmatvec::Mat4x16& getwh2tcoefqI() const;
      const fmatvec::Mat16x4& getwh1coefqInunutH() const;
      const fmatvec::Mat16x4& getwh2coefqInunutH() const;

      const fmatvec::Mat3x16& gettSqI() const;
      const fmatvec::Mat3x16& getnSqI() const;
      const fmatvec::Mat3x16& getbSqI() const;
      const fmatvec::Mat16x3& getnSqIH() const;
      const fmatvec::Mat16x3& getbSqIH() const;
      const fmatvec::Mat3x16& gettStqI() const;
      const fmatvec::Mat3x16& getnStqI() const;
      const fmatvec::Mat3x16& getbStqI() const;

      const double& getTtil() const;
      const fmatvec::RowVec16& getTtilqI() const;
      const fmatvec::SymMat16& getTtilqItqIt() const;
      const fmatvec::Vec16& getTtilqItqIqIt() const;

      const fmatvec::Mat3x16& getdpS() const;
      const fmatvec::Mat16x3& getdpSH() const;
      /***************************************************/

      /* BASIC INTEGRALS */
      double intv(const fmatvec::Vec4& vt) const;
      double intvx(const fmatvec::Vec4& vt) const;
      double intxv(const fmatvec::Vec4& vt) const;
      double intxvx(const fmatvec::Vec4& vt) const;
      double intvw(const fmatvec::Vec4& vt,const fmatvec::Vec4& wt) const;
      double intvxwx(const fmatvec::Vec4& vt,const fmatvec::Vec4& wt) const;
      double intvxxvxx(const fmatvec::Vec4& vt,double C) const;
      void intvvt();		
      void intvvtH();
      void intvxvt();		
      void intvxvtH();		
      void intxvvt();		
      void intxvvtH();
      void intxvxvt();		
      void intxvxvtH();		
      fmatvec::RowVec4 intvwwt(const fmatvec::Vec4& vt) const;
      fmatvec::RowVec4 intvxwxwt(const fmatvec::Vec4& vt) const;
      fmatvec::RowVec4 intvxxwxxwt(const fmatvec::Vec4& vt,double C) const;
      void intvvtwwt();
      void intvxvtwxwt();
      /***************************************************/

      /**
       * \brief computes the integrals of bending polynomials 
       * \param global coordinates
       * \param global velocities
       */
      void computeint(const fmatvec::Vec16& qG,const fmatvec::Vec16& qGt);
      
      /**
       * \brief computes the vector integrals of bending polynomials 
       * \param global coordinates
       * \param global velocities
       */
      void computeintD(const fmatvec::Vec16& qG,const fmatvec::Vec16& qGt);
      
      /**
       * \brief computes the coefficients of bending polynomials w 
       * \param global coordinates
       */
      void computewcoefPos(const fmatvec::Vec16& qG);
      
      /**
       * \brief computes the time differentiated coefficients of bending polynomials w 
       * \param global coordinates
       * \param global velocities
       */
      void computewcoefVel(const fmatvec::Vec16& qG,const fmatvec::Vec16& qGt);
      
      /**
       * \brief computes the coefficients of bending polynomials w and wh
       * \param global coordinates
       */
      void computewhcoefPos(const fmatvec::Vec16& qG);
      
      /**
       * \brief computes the coefficients of bending polynomials w and wh and their time derivatives 
       * \param global coordinates
       * \param global velocities
       */
      void computewhcoefVel(const fmatvec::Vec16& qG,const fmatvec::Vec16& qGt);
      
      /**
       * \brief computes the derivative of w-coefficients with respect to bending coordinates
       */
      void computewcoefPosD();
      
      /**
       * \brief computes bending polynomial values on position level
       * \param global coordinates
       */
      void computewhcoefPosD(const fmatvec::Vec16& qG);
      
      /**
       * \brief computes the coefficients of w and wt
       * \param left translational deflection
       * \param right translational deflection
       * \param left rotational deflection
       * \param right rotational deflection
       */
      fmatvec::Vec4 computewcoef(double dL,double dR,double bL,double bR) const;
      
      /**
       * \brief evaluates the bending polynomial and its x-derivative
       * \param point of evaluation 
       */
      fmatvec::Vec2 computew(const fmatvec::Vec4& wt,double x) const;

    private:
      /** 
       * \brief Trafo-Object
       */
      Trafo33RCMPtr tf;

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
      fmatvec::RowVec4 Ivvt, Ivxvt, Ixvvt, Ixvxvt;
      fmatvec::SymMat4 Ivvtwwt, Ivxvtwxwt;
      fmatvec::Vec4 IvvtH, IvxvtH, IxvvtH, IxvxvtH;

      /**
       * \brief special integrals
       */
      double Iwh1, Iwh2, Iwh1t, Iwh2t, Ixwh1, Ixwh2, Ixwh1t, Ixwh2t;
      double Iwh1twh1, Iwh1twh2, Iwh1twh1t, Iwh1wh1, Iwh1wh2t, Iwh1wh2, Iwh2twh2t;	
      double Iwh2twh2, Iwh2wh2, Iwh1twh2t;
      fmatvec::RowVec4 Iwh1wwt, Iwh1twwt, Iwh2wwt, Iwh2twwt;
      fmatvec::Vec4 Iwh1wwtH, Iwh1twwtH, Iwh2wwtH, Iwh2twwtH;

      double Iwh1xwh1x, Iwh2xwh2x, Iwh1xxwh1xx, Iwh2xxwh2xx;
      fmatvec::RowVec4 Iwh1xwxwt, Iwh2xwxwt;
      fmatvec::RowVec4 Iwh1xxwxxwt, Iwh2xxwxxwt;

      /** 
       * \brief bending coefficients
       */
      fmatvec::Vec4 w1coef, w2coef, w1tcoef, w2tcoef;
      fmatvec::Vec4 wh1coef, wh2coef, wh1tcoef, wh2tcoef;
      fmatvec::Mat4x16 w1coefqI, w2coefqI;
      fmatvec::Mat4x16 wh1coefqI, wh2coefqI, wh1tcoefqI, wh2tcoefqI;
      fmatvec::Mat16x4 wh1coefqInunutH, wh2coefqInunutH;

      /**
       * \brief COSY 
       */
      fmatvec::Mat3x16 tSqI, nSqI, bSqI;
      fmatvec::Mat3x16 tStqI, nStqI, bStqI;
      fmatvec::Mat16x3 nSqIH, bSqIH;
      fmatvec::Mat3x16 ntilSqI, btilSqI;
      fmatvec::RowVec16 xintilqI, xibtilqI, etantilqI, etabtilqI;

      /** 
       * \brief omgtS
       */
      double omgt;
      fmatvec::RowVec16 omgtqI, omgtqIt;
      fmatvec::Vec16 omgtqItqIqIt;

      /**
       * \brief rotational kinetic energy 
       */
      double Ttil;
      fmatvec::RowVec16 TtilqI;
      fmatvec::SymMat16 TtilqItqIt;
      fmatvec::Vec16 TtilqItqIqIt;

      /** 
       * \brief Gauss integration
       */
      fmatvec::Vec gp, xip; // cannot be initialised in constructor
      double bam;

      /** 
       * \brief delta matrix for pS
       */
      fmatvec::Mat3x16 dpS;
      fmatvec::Mat16x3 dpSH;

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

  inline const fmatvec::RowVec4& Weight33RCM::getvxvt() const { return Ivxvt; }
  inline const fmatvec::Vec4& Weight33RCM::getvxvtH() const { return IvxvtH; }
  inline const fmatvec::RowVec4& Weight33RCM::getxvxvt() const { return Ixvxvt; }
  inline const fmatvec::Vec4& Weight33RCM::getxvxvtH() const { return IxvxvtH; }
  inline const fmatvec::SymMat4& Weight33RCM::getvxvtwxwt() const { return Ivxvtwxwt; }
  inline const fmatvec::RowVec4& Weight33RCM::getvvt() const { return Ivvt; }
  inline const fmatvec::Vec4& Weight33RCM::getvvtH() const { return IvvtH; }
  inline const fmatvec::RowVec4& Weight33RCM::getxvvt() const { return Ixvvt; }
  inline const fmatvec::Vec4& Weight33RCM::getxvvtH() const { return IxvvtH; }
  inline const fmatvec::SymMat4& Weight33RCM::getvvtwwt() const { return Ivvtwwt; }

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
  inline const fmatvec::RowVec4& Weight33RCM::getIwh1xxwxxwt() const { return Iwh1xxwxxwt; }
  inline const fmatvec::RowVec4& Weight33RCM::getIwh2xxwxxwt() const { return Iwh2xxwxxwt; }
  inline const fmatvec::RowVec4& Weight33RCM::getIwh1wwt() const { return Iwh1wwt; }
  inline const fmatvec::RowVec4& Weight33RCM::getIwh2wwt() const { return Iwh2wwt; }
  inline const fmatvec::Vec4& Weight33RCM::getIwh1wwtH() const { return Iwh1wwtH; }
  inline const fmatvec::Vec4& Weight33RCM::getIwh2wwtH() const { return Iwh2wwtH; }
  inline const fmatvec::RowVec4& Weight33RCM::getIwh1twwt() const { return Iwh1twwt; }
  inline const fmatvec::RowVec4& Weight33RCM::getIwh2twwt() const { return Iwh2twwt; }
  inline const fmatvec::Vec4& Weight33RCM::getIwh1twwtH() const { return Iwh1twwtH; }
  inline const fmatvec::Vec4& Weight33RCM::getIwh2twwtH() const { return Iwh2twwtH; }
  inline const fmatvec::RowVec4& Weight33RCM::getIwh1xwxwt() const { return Iwh1xwxwt; }
  inline const fmatvec::RowVec4& Weight33RCM::getIwh2xwxwt() const { return Iwh2xwxwt; }

  inline const fmatvec::Vec4& Weight33RCM::getw1coef() const { return w1coef; }
  inline const fmatvec::Vec4& Weight33RCM::getw2coef() const { return w2coef; }
  inline const fmatvec::Vec4& Weight33RCM::getw1tcoef() const { return w1tcoef; }
  inline const fmatvec::Vec4& Weight33RCM::getw2tcoef() const { return w2tcoef; }
  inline const fmatvec::Mat4x16& Weight33RCM::getw1coefqI() const { return w1coefqI; }
  inline const fmatvec::Mat4x16& Weight33RCM::getw2coefqI() const { return w2coefqI; }
  inline const fmatvec::Vec4& Weight33RCM::getwh1coef() const { return wh1coef; }
  inline const fmatvec::Vec4& Weight33RCM::getwh2coef() const { return wh2coef; }
  inline const fmatvec::Vec4& Weight33RCM::getwh1tcoef() const { return wh1tcoef; }
  inline const fmatvec::Vec4& Weight33RCM::getwh2tcoef() const { return wh2tcoef; }
  inline const fmatvec::Mat4x16& Weight33RCM::getwh1coefqI() const { return wh1coefqI; }
  inline const fmatvec::Mat4x16& Weight33RCM::getwh2coefqI() const { return wh2coefqI; }
  inline fmatvec::Mat16x4 Weight33RCM::getwh1coefqIH() const { return wh1coefqI.T(); }
  inline fmatvec::Mat16x4 Weight33RCM::getwh2coefqIH() const { return wh2coefqI.T(); }
  inline const fmatvec::Mat4x16& Weight33RCM::getwh1tcoefqI() const { return wh1tcoefqI; }
  inline const fmatvec::Mat4x16& Weight33RCM::getwh2tcoefqI() const { return wh2tcoefqI; }
  inline const fmatvec::Mat16x4& Weight33RCM::getwh1coefqInunutH() const { return wh1coefqInunutH; }
  inline const fmatvec::Mat16x4& Weight33RCM::getwh2coefqInunutH() const { return wh2coefqInunutH; }

  inline const fmatvec::Mat3x16& Weight33RCM::gettSqI() const { return tSqI; }
  inline const fmatvec::Mat3x16& Weight33RCM::getnSqI() const { return nSqI; }
  inline const fmatvec::Mat3x16& Weight33RCM::getbSqI() const { return bSqI; }
  inline const fmatvec::Mat16x3& Weight33RCM::getnSqIH() const { return nSqIH; }
  inline const fmatvec::Mat16x3& Weight33RCM::getbSqIH() const { return bSqIH; }
  inline const fmatvec::Mat3x16& Weight33RCM::gettStqI() const { return tStqI; }
  inline const fmatvec::Mat3x16& Weight33RCM::getnStqI() const { return nStqI; }
  inline const fmatvec::Mat3x16& Weight33RCM::getbStqI() const { return bStqI; }

  inline const double& Weight33RCM::getTtil() const { return Ttil; }
  inline const fmatvec::RowVec16& Weight33RCM::getTtilqI() const { return TtilqI; }
  inline const fmatvec::SymMat16& Weight33RCM::getTtilqItqIt() const { return TtilqItqIt; }
  inline const fmatvec::Vec16& Weight33RCM::getTtilqItqIqIt() const { return TtilqItqIqIt; }

  inline const fmatvec::Mat3x16& Weight33RCM::getdpS() const { return dpS; }
  inline const fmatvec::Mat16x3& Weight33RCM::getdpSH() const { return dpSH; }
  /*******************************************************************/

  inline Weight33RCM::Weight33RCM(double l0_, double l0h2_, double l0h3_, Trafo33RCMPtr tf_) :
      tf(tf_), l0(l0_), l0h2(l0h2_), l0h3(l0h3_), l0h4(l0h3 * l0), l0h5(l0h3 * l0h2), l0h7(l0h5 * l0h2), l0h9(l0h7 * l0h2), l0h11(l0h9 * l0h2), k10(0.), k20(0.), Ivvt(), Ivxvt(), Ixvvt(), Ixvxvt(), Ivvtwwt(), Ivxvtwxwt(), Iwh1(0.), Iwh2(0.), Iwh1t(0.), Iwh2t(0.), Ixwh1(0.), Ixwh2(0.), Ixwh1t(0.), Ixwh2t(0.), Iwh1twh1(0.), Iwh1twh2(0.), Iwh1twh1t(0.), Iwh1wh1(0.), Iwh1wh2t(0.), Iwh1wh2(0.), Iwh2twh2t(0.), Iwh2twh2(0.), Iwh2wh2(0.), Iwh1twh2t(0.), Iwh1wwt(), Iwh1twwt(), Iwh2wwt(), Iwh2twwt(), Iwh1wwtH(), Iwh1twwtH(), Iwh2wwtH(), Iwh2twwtH(), Iwh1xwh1x(0.), Iwh2xwh2x(0.), Iwh1xxwh1xx(0.), Iwh2xxwh2xx(0.), Iwh1xwxwt(), Iwh2xwxwt(), Iwh1xxwxxwt(), Iwh2xxwxxwt(), w1coef(), w2coef(), w1tcoef(), w2tcoef(), wh1coef(), wh2coef(), wh1tcoef(), wh2tcoef(), w1coefqI(), w2coefqI(), wh1coefqI(), wh2coefqI(), wh1tcoefqI(), wh2tcoefqI(), wh1coefqInunutH(), wh2coefqInunutH(), tSqI(), nSqI(), bSqI(), tStqI(), nStqI(), bStqI(), nSqIH(), bSqIH(), ntilSqI(), btilSqI(), xintilqI(), xibtilqI(), etantilqI(), etabtilqI(), omgt(0.), omgtqI(), omgtqIt(), omgtqItqIqIt(), Ttil(0.), TtilqI(), TtilqItqIt(), TtilqItqIqIt(), bam(0.5 * l0), dpS() {
    /* general integrals */
    intvvt();
    intvvtH();
    intvxvt();
    intvxvtH();
    intxvvt();
    intxvvtH();
    intxvxvt();
    intxvxvtH();
    intvvtwwt();
    intvxvtwxwt();

    /* constant coefficient terms*/
    computewcoefPosD();

    /* delta-matrix */
    computedpS();
  }

  inline Weight33RCM::~Weight33RCM() {
  }

  inline void Weight33RCM::setGauss(int nGauss) {
    gp = fmatvec::Vec(nGauss); // Gauss weights
    xip = fmatvec::Vec(nGauss); // Gauss points

    switch (nGauss) {
      case 1:
        xip(0) = 0.;

        gp(0) = 2.;
      break;
      case 2:
        xip(0) = -sqrt(1. / 3.);
        xip(1) = sqrt(1. / 3.);

        gp(0) = 1.;
        gp(1) = 1.;
      break;
      case 3:
        xip(0) = -sqrt(3. / 5.);
        xip(1) = 0.;
        xip(2) = sqrt(3. / 5.);

        gp(0) = 5. / 9.;
        gp(1) = 8. / 9.;
        gp(2) = 5. / 9.;
      break;
      case 4:
        xip(0) = -sqrt((15. + 2. * sqrt(30.)) / 35.);
        xip(1) = -sqrt((15. - 2. * sqrt(30.)) / 35.);
        xip(2) = sqrt((15. - 2. * sqrt(30.)) / 35.);
        xip(3) = sqrt((15. + 2. * sqrt(30.)) / 35.);

        gp(0) = (18. - sqrt(30.)) / 36.;
        gp(1) = (18. + sqrt(30.)) / 36.;
        gp(2) = (18. + sqrt(30.)) / 36.;
        gp(3) = (18. - sqrt(30.)) / 36.;
      break;
      case 5:
        xip(0) = -sqrt((35. + 2. * sqrt(70.)) / 63.);
        xip(1) = -sqrt((35. - 2. * sqrt(70.)) / 63.);
        xip(2) = 0.;
        xip(3) = sqrt((35. - 2. * sqrt(70.)) / 63.);
        xip(4) = sqrt((35. + 2. * sqrt(70.)) / 63.);

        gp(0) = (322. - 13. * sqrt(70.)) / 900.;
        gp(1) = (322. + 13. * sqrt(70.)) / 900.;
        gp(2) = 128. / 225.;
        gp(3) = (322. + 13. * sqrt(70.)) / 900.;
        gp(4) = (322. - 13. * sqrt(70.)) / 900.;
      break;
      default:
        throw MBSim::MBSimError("ERROR (Weight33RCM::setGauss): Maximum of 5 Gauss points supported");
    }
  }

  inline void Weight33RCM::computeint(const fmatvec::Vec16& qG, const fmatvec::Vec16& qGt) {
    computewhcoefVel(qG, qGt);
    computeint();
  }

  inline void Weight33RCM::computeint() {
    // REQUIRED computewhcoefVel();
    computeT();

    Iwh1 = intv(wh1coef);
    Iwh2 = intv(wh2coef);
    Ixwh1 = intxv(wh1coef);
    Ixwh2 = intxv(wh2coef);
    Iwh1wh1 = intvw(wh1coef, wh1coef);
    Iwh1wh2 = intvw(wh1coef, wh2coef);
    Iwh2wh2 = intvw(wh2coef, wh2coef);
    Iwh1xwh1x = intvxwx(wh1coef, wh1coef);
    Iwh2xwh2x = intvxwx(wh2coef, wh2coef);
    Iwh1xxwh1xx = intvxxvxx(wh1coef, k10);
    Iwh2xxwh2xx = intvxxvxx(wh2coef, k20);

    Iwh1t = intv(wh1tcoef);
    Iwh2t = intv(wh2tcoef);
    Ixwh1t = intxv(wh1tcoef);
    Ixwh2t = intxv(wh2tcoef);
    Iwh1twh1 = intvw(wh1tcoef, wh1coef);
    Iwh1twh2 = intvw(wh1tcoef, wh2coef);
    Iwh1twh1t = intvw(wh1tcoef, wh1tcoef);
    Iwh1wh2t = intvw(wh1coef, wh2tcoef);
    Iwh2twh2t = intvw(wh2tcoef, wh2tcoef);
    Iwh2twh2 = intvw(wh2tcoef, wh2coef);
    Iwh1twh2t = intvw(wh1tcoef, wh2tcoef);
  }

  inline void Weight33RCM::computeintD(const fmatvec::Vec16& qG, const fmatvec::Vec16& qGt) {
    tf->computeTrafo(qG, qGt);
    computewhcoefVelD();
    computeint();

    Iwh1wwt = intvwwt(wh1coef);
    Iwh1wwtH = Iwh1wwt.T();
    Iwh1twwt = intvwwt(wh1tcoef);
    Iwh1twwtH = Iwh1twwt.T();
    Iwh2wwt = intvwwt(wh2coef);
    Iwh2wwtH = Iwh2wwt.T();
    Iwh2twwt = intvwwt(wh2tcoef);
    Iwh2twwtH = Iwh2twwt.T();

    Iwh1xxwxxwt = intvxxwxxwt(wh1coef, k10);
    Iwh2xxwxxwt = intvxxwxxwt(wh2coef, k20);
    Iwh1xwxwt = intvxwxwt(wh1coef);
    Iwh2xwxwt = intvxwxwt(wh2coef);
  }

  inline fmatvec::Vec2 Weight33RCM::computew(const fmatvec::Vec4& wt, double x) const {
    fmatvec::Vec2 W;
    W(0) = (((wt(0) * x + wt(1)) * x + wt(2)) * x + wt(3)) * pow(x, 2);
    W(1) = (((5 * wt(0) * x + 4 * wt(1)) * x + 3 * wt(2)) * x + 2 * wt(3)) * x;

    return W;
  }

  inline void Weight33RCM::computewcoefPos(const fmatvec::Vec16& qG) {
    tf->computeqI(qG);
    computewcoefPos();
  }

  inline void Weight33RCM::computewcoefPos() {
    // REQUIRED trafo->computeqI()
    fmatvec::Vec11 be = tf->getbe();

    w1coef = computewcoef(be(3), be(4), be(5), be(6));
    w2coef = computewcoef(be(7), be(8), be(9), be(10));
  }

  inline void Weight33RCM::computewcoefVel(const fmatvec::Vec16& qG, const fmatvec::Vec16& qGt) {
    tf->computezI(qG, qGt);
    computewcoefVel();
  }

  inline void Weight33RCM::computewcoefVel() {
    // REQUIRED trafo->computezI()
    fmatvec::Vec11 bet = tf->getbet();

    w1tcoef = computewcoef(bet(3), bet(4), bet(5), bet(6));
    w2tcoef = computewcoef(bet(7), bet(8), bet(9), bet(10));
  }

  inline void Weight33RCM::computewhcoefPos(const fmatvec::Vec16& qG) {
    tf->computeqI(qG);
    computewhcoefPos();
  }

  inline void Weight33RCM::computewhcoefPos() {
    // REQUIRED trafo->computeqI()
    computewcoefPos();

    wh1coef = tf->getxintil() * w1coef + tf->getxibtil() * w2coef;
    wh2coef = tf->getetantil() * w1coef + tf->getetabtil() * w2coef;
  }

  inline void Weight33RCM::computewhcoefVel(const fmatvec::Vec16& qG, const fmatvec::Vec16& qGt) {
    tf->computeCOSYt(qG, qGt);
    computewhcoefPos();
    computewcoefVel();
    computewhcoefVel();
  }

  inline void Weight33RCM::computewhcoefVel() {
    // REQUIRED trafo->computeCOSYt(qG,qGt);
    //      computewhcoefPos();
    //      computewcoefVel();

    wh1tcoef = tf->getxintilt() * w1coef + tf->getxibtilt() * w2coef + tf->getxintil() * w1tcoef + tf->getxibtil() * w2tcoef;
    wh2tcoef = tf->getetantilt() * w1coef + tf->getetabtilt() * w2coef + tf->getetantil() * w1tcoef + tf->getetabtil() * w2tcoef;
  }

  inline void Weight33RCM::computewcoefPosD() {
    w1coefqI.set(fmatvec::Index(0, 3), fmatvec::Index(7, 10), tf->getV());
    w2coefqI.set(fmatvec::Index(0, 3), fmatvec::Index(11, 14), tf->getV());
  }

  inline void Weight33RCM::computewhcoefPosD(const fmatvec::Vec16& qG) {
    tf->computeJIG(qG);
    computewhcoefPosD();
  }

  inline void Weight33RCM::computewhcoefPosD() {
    // REQUIRED trafo->computeqI
    computewhcoefPos();

    tSqI = tf->gettSpS() * dpS;
    nSqI = tf->getnSpS() * dpS;
    bSqI = tf->getbSpS() * dpS;

    ntilSqI = tf->getntilSpS() * dpS;
    btilSqI = tf->getbtilSpS() * dpS;

    xintilqI = tf->getnSH() * ntilSqI + tf->getntilSH() * nSqI;
    xibtilqI = tf->getnSH() * btilSqI + tf->getbtilSH() * nSqI;
    etantilqI = tf->getbSH() * ntilSqI + tf->getntilSH() * bSqI;
    etabtilqI = tf->getbSH() * btilSqI + tf->getbtilSH() * bSqI;

    wh1coefqI = w1coef * xintilqI + w2coef * xibtilqI + tf->getxintil() * w1coefqI + tf->getxibtil() * w2coefqI;
    wh2coefqI = w1coef * etantilqI + w2coef * etabtilqI + tf->getetantil() * w1coefqI + tf->getetabtil() * w2coefqI;
  }

  inline void Weight33RCM::computewhcoefVelD() {
    computewhcoefPosD();
    computewcoefVel();
    computewhcoefVel();

    tStqI.set(fmatvec::Index(0, 2), fmatvec::Index(3, 5), tf->gettSpSt());
    nStqI.set(fmatvec::Index(0, 2), fmatvec::Index(3, 5), tf->getnSpSt());
    bStqI.set(fmatvec::Index(0, 2), fmatvec::Index(3, 5), tf->getbSpSt());

    fmatvec::Mat3x16 ntilStqI;
    ntilStqI.set(fmatvec::Index(0, 2), fmatvec::Index(3, 5), tf->getntilSpSt());

    fmatvec::Mat3x16 btilStqI;
    btilStqI.set(fmatvec::Index(0, 2), fmatvec::Index(3, 5), tf->getbtilSpSt());

    fmatvec::RowVec16 xintiltqI = tf->getntilSH() * nStqI + tf->getnStH() * ntilSqI + tf->getntilStH() * nSqI + tf->getnSH() * ntilStqI;
    fmatvec::RowVec16 xibtiltqI = tf->getbtilSH() * nStqI + tf->getnStH() * btilSqI + tf->getbtilStH() * nSqI + tf->getnSH() * btilStqI;
    fmatvec::RowVec16 etantiltqI = tf->getntilSH() * bStqI + tf->getbStH() * ntilSqI + tf->getntilStH() * bSqI + tf->getbSH() * ntilStqI;
    fmatvec::RowVec16 etabtiltqI = tf->getbtilSH() * bStqI + tf->getbStH() * btilSqI + tf->getbtilStH() * bSqI + tf->getbSH() * btilStqI;

    wh1tcoefqI = w1coef * xintiltqI + w2coef * xibtiltqI + tf->getxintilt() * w1coefqI + tf->getxibtilt() * w2coefqI + w1tcoef * xintilqI + w2tcoef * xibtilqI;
    wh2tcoefqI = w1coef * etantiltqI + w2coef * etabtiltqI + tf->getetantilt() * w1coefqI + tf->getetabtilt() * w2coefqI + w1tcoef * etantilqI + w2tcoef * etabtilqI;

    nSqIH = nSqI.T();
    bSqIH = bSqI.T();

    fmatvec::Mat16x3 ntilSqIH = ntilSqI.T();
    fmatvec::Mat16x3 btilSqIH = btilSqI.T();

    fmatvec::RowVec16 xintilqInunut = (nSqIH * ntilSqI * tf->getqIt() + dpSH * tf->getnSpSt().T() * tf->getntilS() + ntilSqIH * nSqI * tf->getqIt() + dpSH * tf->getntilSpSt().T() * tf->getnS()).T();
    fmatvec::RowVec16 xibtilqInunut = (nSqIH * btilSqI * tf->getqIt() + dpSH * tf->getnSpSt().T() * tf->getbtilS() + btilSqIH * nSqI * tf->getqIt() + dpSH * tf->getbtilSpSt().T() * tf->getnS()).T();
    fmatvec::RowVec16 etantilqInunut = (bSqIH * ntilSqI * tf->getqIt() + dpSH * tf->getbSpSt().T() * tf->getntilS() + ntilSqIH * bSqI * tf->getqIt() + dpSH * tf->getntilSpSt().T() * tf->getbS()).T();
    fmatvec::RowVec16 etabtilqInunut = (bSqIH * btilSqI * tf->getqIt() + dpSH * tf->getbSpSt().T() * tf->getbtilS() + btilSqIH * bSqI * tf->getqIt() + dpSH * tf->getbtilSpSt().T() * tf->getbS()).T();

    wh1coefqInunutH = (w1coefqI * tf->getqIt() * xintilqI + w2coefqI * tf->getqIt() * xibtilqI + xintilqI * tf->getqIt() * w1coefqI + xibtilqI * tf->getqIt() * w2coefqI + w1coef * xintilqInunut + w2coef * xibtilqInunut).T();
    wh2coefqInunutH = (w1coefqI * tf->getqIt() * etantilqI + w2coefqI * tf->getqIt() * etabtilqI + etantilqI * tf->getqIt() * w1coefqI + etabtilqI * tf->getqIt() * w2coefqI + w1coef * etantilqInunut + w2coef * etabtilqInunut).T();
  }

  inline fmatvec::Vec4 Weight33RCM::computewcoef(double dL, double dR, double bL, double bR) const {
    double dLdRp = dL + dR;
    double dLdRm = dL - dR;
    double bLbRp = bL + bR;
    double bLbRm = bL - bR;

    fmatvec::Vec4 wcoef(fmatvec::NONINIT);
    wcoef(0) = (4. * l0 * bLbRp + 24. * dLdRm) / l0h5;
    wcoef(1) = (-2. * l0 * bLbRm - 8. * dLdRp) / l0h4;
    wcoef(2) = (-l0 * bLbRp - 10. * dLdRm) / l0h3;
    wcoef(3) = (l0 * bLbRm + 8. * dLdRp) / (2. * l0h2);

    return wcoef;
  }

  inline void Weight33RCM::computeomgt(double x) {
    // REQUIRED tf->computezI()
    // REQUIRED computewcoef_Pos()
    // REQUIRED computewcoef_Vel()

    fmatvec::Vec3 pS = tf->getpS();
    fmatvec::Vec3 pSt = tf->getpSt();

    double spS1 = sin(pS(1));
    double cpS1 = cos(pS(1));

    fmatvec::Vec2 w1 = computew(w1coef, x);
    fmatvec::Vec2 w2 = computew(w2coef, x);
    fmatvec::Vec2 w2t = computew(w2tcoef, x);

    fmatvec::RowVec4 dwxdwt;
    dwxdwt(3) = 2 * x;
    dwxdwt(2) = x * x * 3.;
    dwxdwt(1) = x * x * x * 4.;
    dwxdwt(0) = x * x * x * x * 5.;

    fmatvec::RowVec16 w1xqI = dwxdwt * w1coefqI;
    fmatvec::RowVec16 w2xqI = dwxdwt * w2coefqI;

    omgt = pSt(0) + tf->getk0t() * x + cpS1 * pSt(1) * w2(1) + spS1 * w2t(1) - (spS1 + cpS1 * w1(1)) * (pSt(2) + w2t(1));

    omgtqI = w1xqI;
    omgtqI(4) += 1.;
    omgtqI *= -(pSt(2) + w2t(1)) * (cpS1 - spS1 * w1(1));
    omgtqI += cpS1 * pSt(1) * w2xqI;
    omgtqI(4) += cpS1 * w2t(1) - spS1 * pSt(1) * w2(1);

    omgtqIt = w2xqI;
    omgtqIt(5) += 1.;
    omgtqIt *= -(spS1 + cpS1 * w1(1));
    omgtqIt(3) += 1;
    omgtqIt += spS1 * w2xqI;
    omgtqIt(15) += x;
    omgtqIt(4) += cpS1 * w2(1);

    omgtqItqIqIt = w2xqI.T();
    omgtqItqIqIt(5) += 1.;
    omgtqItqIqIt *= -((cpS1 - spS1 * w1(1)) * (pSt(1) + w1xqI * tf->getqIt()));
    omgtqItqIqIt += w2xqI.T() * cpS1 * pSt(1);
    omgtqItqIqIt(4) += -spS1 * w2(1) * pSt(1) + cpS1 * w2xqI * tf->getqIt();
  }

  inline void Weight33RCM::computeT() {
    // REQUIRED tf->computezI()
    // REQUIRED computewcoef_Pos()
    // REQUIRED computewcoef_Vel()

    Ttil = 0.;
    TtilqI = fmatvec::RowVec16();
    TtilqItqIt = fmatvec::SymMat16();
    TtilqItqIqIt = fmatvec::Vec16();

    for (int i = 0; i < gp.size(); i++) {
      computeomgt(bam * xip(i));
      Ttil += bam * gp(i) * omgt * omgt;
      TtilqI += bam * gp(i) * omgt * omgtqI;
      TtilqItqIt += static_cast<fmatvec::SymMat16>(bam * gp(i) * omgtqIt.T() * omgtqIt);
      TtilqItqIqIt += bam * gp(i) * (omgtqIt.T() * omgtqI * tf->getqIt() + omgt * omgtqItqIqIt);
    }
  }

  inline void Weight33RCM::computedpS() {
    dpS(0, 3) = 1.;
    dpS(1, 4) = 1.;
    dpS(2, 5) = 1.;
    dpSH = dpS.T();
  }

  inline double Weight33RCM::intv(const fmatvec::Vec4& vt) const {
    const double& bv = vt(1);
    const double& dv = vt(3);

    return bv / 80. * l0h5 + dv / 12. * l0h3;
  }

  inline double Weight33RCM::intvx(const fmatvec::Vec4& vt) const {
    const double& av = vt(0);
    const double& cv = vt(2);

    return av * l0h5 / 16. + cv * l0h3 / 4.;
  }

  inline double Weight33RCM::intxv(const fmatvec::Vec4& vt) const {
    const double& av = vt(0);
    const double& cv = vt(2);

    return av / 448. * l0h7 + cv / 80. * l0h5;
  }

  inline double Weight33RCM::intxvx(const fmatvec::Vec4& vt) const {
    const double& bv = vt(1);
    const double& dv = vt(3);

    return bv * l0h5 / 20. + dv * l0h3 / 6.;
  }

  inline double Weight33RCM::intvw(const fmatvec::Vec4& vt, const fmatvec::Vec4& wt) const {
    const double& av = vt(0);
    const double& bv = vt(1);
    const double& cv = vt(2);
    const double& dv = vt(3);
    const double& aw = wt(0);
    const double& bw = wt(1);
    const double& cw = wt(2);
    const double& dw = wt(3);

    return av * aw / 11264. * l0h11 + (av * cw + aw * cv + bv * bw) / 2304. * l0h9 + (bv * dw + bw * dv + cv * cw) / 448. * l0h7 + dv * dw / 80. * l0h5;
  }

  inline double Weight33RCM::intvxwx(const fmatvec::Vec4& vt, const fmatvec::Vec4& wt) const {
    const double& av = vt(0);
    const double& bv = vt(1);
    const double& cv = vt(2);
    const double& dv = vt(3);
    const double& aw = wt(0);
    const double& bw = wt(1);
    const double& cw = wt(2);
    const double& dw = wt(3);

    return 25. * av * aw / 2304. * l0h9 + (15. * cv * aw / 448. + bv * bw / 28. + 15. * av * cw / 448.) * l0h7 + (dv * bw / 10. + 9. * cv * cw / 80. + bv * dw / 10.) * l0h5 + dv * dw / 3. * l0h3;
  }

  inline double Weight33RCM::intvxxvxx(const fmatvec::Vec4& vt, double C) const {
    const double& av = vt(0);
    const double& bv = vt(1);
    const double& cv = vt(2);
    const double& dv = vt(3);

    return 25. * pow(av, 2) / 28. * l0h7 + (3. * av * cv + 9. * pow(bv, 2) / 5.) * l0h5 + (4. * bv * dv + 3. * pow(cv, 2) - 2. * bv * C) * l0h3 + (pow(C, 2) - 4. * C * dv + 4. * pow(dv, 2)) * l0;
  }

  inline void Weight33RCM::intvvt() {
    Ivvt(1) = l0h5 / 80.;
    Ivvt(3) = l0h3 / 12.;
  }

  inline void Weight33RCM::intvxvt() {
    Ivxvt(0) = l0h5 / 16.;
    Ivxvt(2) = l0h3 / 4.;
  }

  inline void Weight33RCM::intxvvt() {
    Ixvvt(0) = l0h7 / 448.;
    Ixvvt(2) = l0h5 / 80.;
  }

  inline void Weight33RCM::intxvxvt() {
    Ixvxvt(1) = l0h5 / 20.;
    Ixvxvt(3) = l0h3 / 6.;
  }

  inline fmatvec::RowVec4 Weight33RCM::intvwwt(const fmatvec::Vec4& vt) const {
    const double& av = vt(0);
    const double& bv = vt(1);
    const double& cv = vt(2);
    const double& dv = vt(3);

    fmatvec::RowVec4 Ivwwt(fmatvec::NONINIT);
    Ivwwt(0) = av / 11264. * l0h11 + cv / 2304. * l0h9;
    Ivwwt(1) = bv / 2304. * l0h9 + dv / 448. * l0h7;
    Ivwwt(2) = av / 2304. * l0h9 + cv / 448. * l0h7;
    Ivwwt(3) = bv / 448. * l0h7 + dv / 80. * l0h5;
    return Ivwwt;
  }

  inline fmatvec::RowVec4 Weight33RCM::intvxwxwt(const fmatvec::Vec4& vt) const {
    const double& av = vt(0);
    const double& bv = vt(1);
    const double& cv = vt(2);
    const double& dv = vt(3);

    fmatvec::RowVec4 Ivxwxwt(fmatvec::NONINIT);
    Ivxwxwt(0) = 25. * av / 2304. * l0h9 + 15. * cv / 448. * l0h7;
    Ivxwxwt(1) = bv / 28. * l0h7 + dv / 10. * l0h5;
    Ivxwxwt(2) = 15. * av / 448. * l0h7 + 9. * cv / 80. * l0h5;
    Ivxwxwt(3) = bv / 10. * l0h5 + dv / 3. * l0h3;
    return Ivxwxwt;
  }

  inline fmatvec::RowVec4 Weight33RCM::intvxxwxxwt(const fmatvec::Vec4& vt, double C) const {
    const double& av = vt(0);
    const double& bv = vt(1);
    const double& cv = vt(2);
    const double& dv = vt(3);

    fmatvec::RowVec4 Ivxxwxxwt(fmatvec::NONINIT);
    Ivxxwxxwt(0) = 25. * av / 28. * l0h7 + 3. * cv / 2. * l0h5;
    Ivxxwxxwt(1) = 9. * bv / 5. * l0h5 + (2. * dv - C) * l0h3;
    Ivxxwxxwt(2) = 3. * av / 2. * l0h5 + 3. * cv * l0h3;
    Ivxxwxxwt(3) = 2. * bv * l0h3 + (4. * dv - 2. * C) * l0;
    return Ivxxwxxwt;
  }

  inline void Weight33RCM::intvvtwwt() {
    Ivvtwwt(0, 0) = l0h11 / 11264.;
    Ivvtwwt(0, 2) = l0h9 / 2304.;
    Ivvtwwt(1, 1) = Ivvtwwt(0, 2);
    Ivvtwwt(1, 3) = l0h7 / 448.;
    Ivvtwwt(2, 2) = Ivvtwwt(1, 3);
    Ivvtwwt(3, 3) = l0h5 / 80.;
  }

  inline void Weight33RCM::intvxvtwxwt() {
    Ivxvtwxwt(0, 0) = 25. * l0h9 / 2304.;
    Ivxvtwxwt(0, 2) = 15. * l0h7 / 448.;
    Ivxvtwxwt(1, 1) = l0h7 / 28.;
    Ivxvtwxwt(1, 3) = l0h5 / 10.;
    Ivxvtwxwt(2, 2) = 9. * l0h5 / 80.;
    Ivxvtwxwt(3, 3) = l0h3 / 3.;
  }

}

#endif /*WEIGHT33RCM_H_*/

