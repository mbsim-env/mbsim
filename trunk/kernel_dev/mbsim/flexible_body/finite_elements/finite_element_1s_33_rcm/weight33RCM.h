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

namespace MBSim {

  class Trafo33RCM;

  /*! 
   * \brief integrals of bending parametrisation for FiniteElement1s33RCM
   * \author Thorsten Schindler
   * \date 2009-04-24 initial commit (Thorsten Schindler) 
   */
  class Weight33RCM {
    public:
      /*! Constructor */
      Weight33RCM(double l0_,double l0h2_,double l0h3_,Trafo33RCM* tf_);
      /*! Destructor */
      virtual ~Weight33RCM();

      /*! Set curvature */
      void setCurvature(double k10_,double k20_);
      /*! Set number of Gauss points */
      void setGauss(int nGauss);

      /*! Computes Iv */
      double intv(const fmatvec::Vec& vt) const;
      /*! Computes Ivx */
      double intvx(const fmatvec::Vec& vt) const;
      /*! Computes Ixv */
      double intxv(const fmatvec::Vec& vt) const;
      /*! Computes Ixvx */
      double intxvx(const fmatvec::Vec& vt) const;
      /*! Computes Ivw */
      double intvw(const fmatvec::Vec& vt,const fmatvec::Vec& wt) const;
      /*! Computes Ivxwx */
      double intvxwx(const fmatvec::Vec& vt,const fmatvec::Vec& wt) const;		
      /*! Computes I(vxx-C)^2 */
      double intvxxvxx(const fmatvec::Vec& vt,double C) const;
      /*! Computes Ivvt */
      void intvvt();		
      /*! Computes IvvtH */
      void intvvtH();
      /*! Computes Ivxvt */
      void intvxvt();		
      /*! Computes IvxvtH */
      void intvxvtH();		
      /*! Computes Ixvvt */
      void intxvvt();		
      /*! Computes IxvvtH */
      void intxvvtH();
      /*! Computes Ixvxvt */
      void intxvxvt();		
      /*! Computes IxvxvtH */
      void intxvxvtH();		
      /*! Computes Ivwwt */
      fmatvec::RowVec intvwwt(const fmatvec::Vec& vt) const;
      /*! Computes Ivxwxwt */
      fmatvec::RowVec intvxwxwt(const fmatvec::Vec& vt) const;
      /*! Computes I(vxx-C)wxxwt */
      fmatvec::RowVec intvxxwxxwt(const fmatvec::Vec& vt,double C) const;
      /*! Computes Ivvtwwt */
      void intvvtwwt();
      /*! Computes Ivxvtwxwt */
      void intvxvtwxwt();

      /*! Computes the integrals of bending polynomials */
      void computeint(const fmatvec::Vec& qG,const fmatvec::Vec& qGt);
      /*! Computes the vector integrals of bending polynomials */
      void computeintD(const fmatvec::Vec& qG,const fmatvec::Vec& qGt);
      /*! Computes the coefficients of bending polynomials w */
      void computewcoefPos(const fmatvec::Vec& qG);
      /*! Computes the time differentiated coefficients of bending polynomials w */
      void computewcoefVel(const fmatvec::Vec& qG,const fmatvec::Vec& qGt);
      /*! Computes the coefficients of bending polynomials w and wh */
      void computewhcoefPos(const fmatvec::Vec& qG);
      /*! Computes the coefficients of bending polynomials w and wh and their time derivatives */
      void computewhcoefVel(const fmatvec::Vec& qG,const fmatvec::Vec& qGt);
      /*! Computes the derivative of w-coefficients with respect to bending coordinates */
      void computewcoefPosD();
      /*! Computes bending polynomial values on position level */
      void computewhcoefPosD(const fmatvec::Vec& qG);
      /*! Computes the coefficients of w and wt */
      fmatvec::Vec computewcoef(double dL,double dR,double bL,double bR) const;	
      /*! Evaluates the bending polynomial and its x-derivative at \param x */
      fmatvec::Vec computew(const fmatvec::Vec& wt,double x) const;

      /*! Returns Ivxvt */
      fmatvec::RowVec getvxvt() const;
      /*! Returns IvxvtH */
      fmatvec::Vec getvxvtH() const;
      /*! Returns Ixvxvt */
      fmatvec::RowVec getxvxvt() const;
      /*! Returns IxvxvtH */
      fmatvec::Vec getxvxvtH() const;
      /*! Returns Ivxvtwxwt */
      fmatvec::SymMat getvxvtwxwt() const;
      /*! Returns Ivvt */
      fmatvec::RowVec getvvt() const;
      /*! Returns IvvtH */
      fmatvec::Vec getvvtH() const;
      /*! Returns Ixvvt */
      fmatvec::RowVec getxvvt() const;
      /*! Returns IxvvtH */
      fmatvec::Vec getxvvtH() const;
      /*! Returns Ivvtwwt */
      fmatvec::SymMat getvvtwwt() const;

      /*! Return Iwh1 */
      double getIwh1() const;
      /*! Return Iwh2 */
      double getIwh2() const;
      /*! Return  Iwh1t */
      double getIwh1t() const;	
      /*! Return Iwh2t */
      double getIwh2t() const;
      /*! Return Ixwh1 */
      double getIxwh1() const;
      /*! Return Ixwh2 */
      double getIxwh2() const;
      /*! Return Ixwh1t */
      double getIxwh1t() const;
      /*! Return Ixwh2t */
      double getIxwh2t() const;		
      /*! Return Iwh1twh1 */
      double getIwh1twh1() const;
      /*! Return Iwh1twh2 */
      double getIwh1twh2() const;
      /*! Return Iwh1twh1t */
      double getIwh1twh1t() const;
      /*! Return Iwh1wh1 */
      double getIwh1wh1() const;
      /*! Return Iwh1wh2t */
      double getIwh1wh2t() const;		
      /*! Return Iwh1wh2 */
      double getIwh1wh2() const;
      /*! Return Iwh2twh2t */
      double getIwh2twh2t() const;
      /*! Return Iwh2twh2 */
      double getIwh2twh2() const;
      /*! Return Iwh2wh2 */
      double getIwh2wh2() const;
      /*! Return Iwh1twh2t */
      double getIwh1twh2t() const;		
      /*! Return Iwh1xwh1x */
      double getIwh1xwh1x() const;
      /*! Return Iwh2xwh2x */
      double getIwh2xwh2x() const;
      /*! Return Iwh1xxwh1xx */
      double getIwh1xxwh1xx() const;
      /*! Return Iwh2xxwh2xx */
      double getIwh2xxwh2xx() const;
      /*! Return Iwh1xxwxxwt */
      fmatvec::RowVec getIwh1xxwxxwt() const;
      /*! Return Iwh2xxwxxwt */
      fmatvec::RowVec getIwh2xxwxxwt() const;
      /*! Return Iwh1wwt */
      fmatvec::RowVec getIwh1wwt() const;
      /*! Return Iwh2wwt */
      fmatvec::RowVec getIwh2wwt() const;
      /*! Return Iwh1wwtH */
      fmatvec::Vec getIwh1wwtH() const;
      /*! Return Iwh2wwtH */
      fmatvec::Vec getIwh2wwtH() const;
      /*! Return Iwh1twwt */
      fmatvec::RowVec getIwh1twwt() const;			
      /*! Return Iwh2twwt */
      fmatvec::RowVec getIwh2twwt() const;
      /*! Return Iwh1twwtH */
      fmatvec::Vec getIwh1twwtH() const;			
      /*! Return Iwh2twwtH */
      fmatvec::Vec getIwh2twwtH() const;
      /*! Return Iwh1xwxwt */
      fmatvec::RowVec getIwh1xwxwt() const;
      /*! Return Iwh2xwxwt */
      fmatvec::RowVec getIwh2xwxwt() const;

      /*! Return w1coef */
      fmatvec::Vec getw1coef() const;
      /*! Return w2coef */
      fmatvec::Vec getw2coef() const;
      /*! Return w1tcoef */
      fmatvec::Vec getw1tcoef() const;
      /*! Return w2tcoef */
      fmatvec::Vec getw2tcoef() const;
      /*! Return w1coefqI */
      fmatvec::Mat getw1coefqI() const;
      /*! Return w2coefqI */
      fmatvec::Mat getw2coefqI() const;
      /*! Return wh1coef */
      fmatvec::Vec getwh1coef() const;
      /*! Return wh2coef */
      fmatvec::Vec getwh2coef() const;
      /*! Return wh1tcoef */
      fmatvec::Vec getwh1tcoef() const;
      /*! Return wh2tcoef */
      fmatvec::Vec getwh2tcoef() const;		
      /*! Return wh1coefqI */
      fmatvec::Mat getwh1coefqI() const;
      /*! Return wh2coefqI */
      fmatvec::Mat getwh2coefqI() const;
      /*! Return wh1coefqIH */
      fmatvec::Mat getwh1coefqIH() const;
      /*! Return wh2coefqIH */
      fmatvec::Mat getwh2coefqIH() const;
      /*! Return wh1tcoefqI */
      fmatvec::Mat getwh1tcoefqI() const;
      /*! Return wh2tcoefqI */
      fmatvec::Mat getwh2tcoefqI() const;
      /*! Return wh1coefqInunutH */
      fmatvec::Mat getwh1coefqInunutH() const;
      /*! Return wh2coefqInunutH */
      fmatvec::Mat getwh2coefqInunutH() const;

      /*! Return tSqI */
      fmatvec::Mat gettSqI() const;
      /*! Return nSqI */
      fmatvec::Mat getnSqI() const;
      /*! Return bSqI */
      fmatvec::Mat getbSqI() const;
      /*! Return nSqIH */
      fmatvec::Mat getnSqIH() const;
      /*! Return bSqIH */
      fmatvec::Mat getbSqIH() const;
      /*! Return tStqI */
      fmatvec::Mat gettStqI() const;
      /*! Return nStqI */
      fmatvec::Mat getnStqI() const;
      /*! Return bStqI */
      fmatvec::Mat getbStqI() const;

      /*! Return omgtS */
      double getTtil() const;
      /*! Return omgtSqI */
      fmatvec::RowVec getTtilqI() const;
      /*! Return omgtSqIt */
      fmatvec::SymMat getTtilqItqIt() const;
      /*! Return omgtSqItqIqIt */
      fmatvec::Vec getTtilqItqIqIt() const;

      /*! Return dpS */
      fmatvec::Mat getdpS() const;		
      /*! Return dpSH */
      fmatvec::Mat getdpSH() const;

    private:
      /** Trafo-Object */
      Trafo33RCM* tf;

      /** length of FEM1s33RCM */
      double l0, l0h2, l0h3, l0h4, l0h5, l0h7, l0h9, l0h11;

      /** predefined bendings */
      double k10, k20;

      /** general integrals */
      fmatvec::RowVec Ivvt, Ivxvt, Ixvvt, Ixvxvt;
      fmatvec::SymMat Ivvtwwt, Ivxvtwxwt;
      fmatvec::Vec IvvtH, IvxvtH, IxvvtH, IxvxvtH;

      /** special integrals */
      double Iwh1, Iwh2, Iwh1t, Iwh2t, Ixwh1, Ixwh2, Ixwh1t, Ixwh2t;
      double Iwh1twh1, Iwh1twh2, Iwh1twh1t, Iwh1wh1, Iwh1wh2t, Iwh1wh2, Iwh2twh2t;	
      double Iwh2twh2, Iwh2wh2, Iwh1twh2t;
      fmatvec::RowVec Iwh1wwt, Iwh1twwt, Iwh2wwt, Iwh2twwt;
      fmatvec::Vec Iwh1wwtH, Iwh1twwtH, Iwh2wwtH, Iwh2twwtH;

      double Iwh1xwh1x, Iwh2xwh2x, Iwh1xxwh1xx, Iwh2xxwh2xx;
      fmatvec::RowVec Iwh1xwxwt, Iwh2xwxwt;
      fmatvec::RowVec Iwh1xxwxxwt, Iwh2xxwxxwt;

      /** bending coefficients */
      fmatvec::Vec w1coef, w2coef, w1tcoef, w2tcoef;
      fmatvec::Vec wh1coef, wh2coef, wh1tcoef, wh2tcoef;
      fmatvec::Mat w1coefqI, w2coefqI;
      fmatvec::Mat wh1coefqI, wh2coefqI, wh1tcoefqI, wh2tcoefqI;
      fmatvec::Mat wh1coefqInunutH, wh2coefqInunutH;

      /** COSY */
      fmatvec::Mat tSqI, nSqI, bSqI;
      fmatvec::Mat tStqI, nStqI, bStqI;
      fmatvec::Mat nSqIH, bSqIH;
      fmatvec::Mat ntilSqI, btilSqI;
      fmatvec::RowVec xintilqI, xibtilqI, etantilqI, etabtilqI;

      /** omgtS */
      double omgt;
      fmatvec::RowVec omgtqI, omgtqIt;
      fmatvec::Vec omgtqItqIqIt;

      /** rotational kinetic energy */
      double Ttil;
      fmatvec::RowVec TtilqI;
      fmatvec::SymMat TtilqItqIt;
      fmatvec::Vec TtilqItqIqIt;

      /** Gauss integration */
      fmatvec::Vec gp, xip; // cannot be initialised in constructor
      double bam;

      /** delta matrix for pS */
      fmatvec::Mat dpS, dpSH;

      /*! Computes the integrals of bending polynomials */
      void computeint();
      /*! Computes the coefficients of bending polynomials w */
      void computewcoefPos();
      /*! Computes the time differentiated coefficients of bending polynomials w */
      void computewcoefVel();
      /*! Computes the coefficients of bending polynomials w and wh */
      void computewhcoefPos();
      /*! Computes the coefficients of bending polynomials w and wh and their time derivatives */
      void computewhcoefVel();
      /*! Computes bending polynomial values on position level */
      void computewhcoefPosD();
      /*! Computes bending polynomial values on velocity level */
      void computewhcoefVelD();

      /*! Compute angular velocity around tangent */
      void computeomgt(double x);
      /*! Compute rotational kinetic energy */
      void computeT();
      /*! Compute delta matrix for CP with respect to rotation */
      void computedpS();			
  };

  inline void Weight33RCM::setCurvature(double k10_,double k20_) {k10 = k10_; k20 = k20_;}
  inline void Weight33RCM::intvvtH() {IvvtH = trans(Ivvt);}
  inline void Weight33RCM::intvxvtH() {IvxvtH = trans(Ivxvt);}	
  inline void Weight33RCM::intxvvtH() {IxvvtH = trans(Ixvvt);}
  inline void Weight33RCM::intxvxvtH() {IxvxvtH = trans(Ixvxvt);}

  inline fmatvec::RowVec Weight33RCM::getvxvt() const {return Ivxvt;}
  inline fmatvec::Vec Weight33RCM::getvxvtH() const {return IvxvtH;}
  inline fmatvec::RowVec Weight33RCM::getxvxvt() const {return Ixvxvt;}
  inline fmatvec::Vec Weight33RCM::getxvxvtH() const {return IxvxvtH;}
  inline fmatvec::SymMat Weight33RCM::getvxvtwxwt() const {return Ivxvtwxwt;}
  inline fmatvec::RowVec Weight33RCM::getvvt() const {return Ivvt;}
  inline fmatvec::Vec Weight33RCM::getvvtH() const {return IvvtH;}
  inline fmatvec::RowVec Weight33RCM::getxvvt() const {return Ixvvt;}
  inline fmatvec::Vec Weight33RCM::getxvvtH() const {return IxvvtH;}
  inline fmatvec::SymMat Weight33RCM::getvvtwwt() const {return Ivvtwwt;}

  inline double Weight33RCM::getIwh1() const {return Iwh1;}
  inline double Weight33RCM::getIwh2() const {return Iwh2;}
  inline double Weight33RCM::getIwh1t() const {return Iwh1t;}	
  inline double Weight33RCM::getIwh2t() const {return Iwh2t;}
  inline double Weight33RCM::getIxwh1() const {return Ixwh1;}
  inline double Weight33RCM::getIxwh2() const {return Ixwh2;}
  inline double Weight33RCM::getIxwh1t() const {return Ixwh1t;}
  inline double Weight33RCM::getIxwh2t() const {return Ixwh2t;}		
  inline double Weight33RCM::getIwh1twh1() const {return Iwh1twh1;}
  inline double Weight33RCM::getIwh1twh2() const {return Iwh1twh2;}
  inline double Weight33RCM::getIwh1twh1t() const {return Iwh1twh1t;}
  inline double Weight33RCM::getIwh1wh1() const {return Iwh1wh1;}
  inline double Weight33RCM::getIwh1wh2t() const {return Iwh1wh2t;}		
  inline double Weight33RCM::getIwh1wh2() const {return Iwh1wh2;}
  inline double Weight33RCM::getIwh2twh2t() const {return Iwh2twh2t;}
  inline double Weight33RCM::getIwh2twh2() const {return Iwh2twh2;}
  inline double Weight33RCM::getIwh2wh2() const {return Iwh2wh2;}
  inline double Weight33RCM::getIwh1twh2t() const {return Iwh1twh2t;}		
  inline double Weight33RCM::getIwh1xwh1x() const {return Iwh1xwh1x;}
  inline double Weight33RCM::getIwh2xwh2x() const {return Iwh2xwh2x;}
  inline double Weight33RCM::getIwh1xxwh1xx() const {return Iwh1xxwh1xx;}
  inline double Weight33RCM::getIwh2xxwh2xx() const {return Iwh2xxwh2xx;}	
  inline fmatvec::RowVec Weight33RCM::getIwh1xxwxxwt() const {return Iwh1xxwxxwt;}
  inline fmatvec::RowVec Weight33RCM::getIwh2xxwxxwt() const {return Iwh2xxwxxwt;}
  inline fmatvec::RowVec Weight33RCM::getIwh1wwt() const {return Iwh1wwt;}
  inline fmatvec::RowVec Weight33RCM::getIwh2wwt() const {return Iwh2wwt;}
  inline fmatvec::Vec Weight33RCM::getIwh1wwtH() const {return Iwh1wwtH;}
  inline fmatvec::Vec Weight33RCM::getIwh2wwtH() const {return Iwh2wwtH;}
  inline fmatvec::RowVec Weight33RCM::getIwh1twwt() const {return Iwh1twwt;}			
  inline fmatvec::RowVec Weight33RCM::getIwh2twwt() const {return Iwh2twwt;}
  inline fmatvec::Vec Weight33RCM::getIwh1twwtH() const {return Iwh1twwtH;}			
  inline fmatvec::Vec Weight33RCM::getIwh2twwtH() const {return Iwh2twwtH;}
  inline fmatvec::RowVec Weight33RCM::getIwh1xwxwt() const {return Iwh1xwxwt;}
  inline fmatvec::RowVec Weight33RCM::getIwh2xwxwt() const {return Iwh2xwxwt;}

  inline fmatvec::Vec Weight33RCM::getw1coef() const {return w1coef;}
  inline fmatvec::Vec Weight33RCM::getw2coef() const {return w2coef;}
  inline fmatvec::Vec Weight33RCM::getw1tcoef() const {return w1tcoef;}
  inline fmatvec::Vec Weight33RCM::getw2tcoef() const {return w2tcoef;}
  inline fmatvec::Mat Weight33RCM::getw1coefqI() const {return w1coefqI;}
  inline fmatvec::Mat Weight33RCM::getw2coefqI() const {return w2coefqI;}
  inline fmatvec::Vec Weight33RCM::getwh1coef() const {return wh1coef;}
  inline fmatvec::Vec Weight33RCM::getwh2coef() const {return wh2coef;}
  inline fmatvec::Vec Weight33RCM::getwh1tcoef() const {return wh1tcoef;}
  inline fmatvec::Vec Weight33RCM::getwh2tcoef() const {return wh2tcoef;}
  inline fmatvec::Mat Weight33RCM::getwh1coefqI() const {return wh1coefqI;}
  inline fmatvec::Mat Weight33RCM::getwh2coefqI() const {return wh2coefqI;}
  inline fmatvec::Mat Weight33RCM::getwh1coefqIH() const {return trans(wh1coefqI);}
  inline fmatvec::Mat Weight33RCM::getwh2coefqIH() const {return trans(wh2coefqI);}
  inline fmatvec::Mat Weight33RCM::getwh1tcoefqI() const {return wh1tcoefqI;}
  inline fmatvec::Mat Weight33RCM::getwh2tcoefqI() const {return wh2tcoefqI;}
  inline fmatvec::Mat Weight33RCM::getwh1coefqInunutH() const {return wh1coefqInunutH;}
  inline fmatvec::Mat Weight33RCM::getwh2coefqInunutH() const {return wh2coefqInunutH;}

  inline fmatvec::Mat Weight33RCM::gettSqI() const {return tSqI;}
  inline fmatvec::Mat Weight33RCM::getnSqI() const {return nSqI;}
  inline fmatvec::Mat Weight33RCM::getbSqI() const {return bSqI;}
  inline fmatvec::Mat Weight33RCM::getnSqIH() const {return nSqIH;}
  inline fmatvec::Mat Weight33RCM::getbSqIH() const {return bSqIH;}
  inline fmatvec::Mat Weight33RCM::gettStqI() const {return tStqI;}
  inline fmatvec::Mat Weight33RCM::getnStqI() const {return nStqI;}
  inline fmatvec::Mat Weight33RCM::getbStqI() const {return bStqI;}

  inline double Weight33RCM::getTtil() const {return Ttil;}
  inline fmatvec::RowVec Weight33RCM::getTtilqI() const {return TtilqI;}
  inline fmatvec::SymMat Weight33RCM::getTtilqItqIt() const {return TtilqItqIt;}
  inline fmatvec::Vec Weight33RCM::getTtilqItqIqIt() const {return TtilqItqIqIt;}

  inline fmatvec::Mat Weight33RCM::getdpS() const {return dpS;}
  inline fmatvec::Mat Weight33RCM::getdpSH() const {return dpSH;}
  /*******************************************************************/

}

#endif /*WEIGHT33RCM_H_*/

