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

#include<config.h>
#define FMATVEC_NO_INITIALIZATION
#define FMATVEC_NO_BOUNDS_CHECK

#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_33_rcm/weight33RCM.h"
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_33_rcm/trafo33RCM.h"
#include "mbsim/mbsim_event.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  Weight33RCM::Weight33RCM(double l0_,double l0h2_,double l0h3_,Trafo33RCMPtr tf_) : 
    tf(tf_),l0(l0_),l0h2(l0h2_),l0h3(l0h3_),l0h4(l0h3*l0),l0h5(l0h3*l0h2),l0h7(l0h5*l0h2),l0h9(l0h7*l0h2),l0h11(l0h9*l0h2),
    k10(0.),k20(0.),Ivvt(4,INIT,0.),Ivxvt(4,INIT,0.),Ixvvt(4,INIT,0.),Ixvxvt(4,INIT,0.),Ivvtwwt(4,INIT,0.),Ivxvtwxwt(4,INIT,0.),
    Iwh1(0.),Iwh2(0.),Iwh1t(0.),Iwh2t(0.),Ixwh1(0.),Ixwh2(0.),Ixwh1t(0.),Ixwh2t(0.),
    Iwh1twh1(0.),Iwh1twh2(0.),Iwh1twh1t(0.),Iwh1wh1(0.),Iwh1wh2t(0.),Iwh1wh2(0.),Iwh2twh2t(0.),	
    Iwh2twh2(0.),Iwh2wh2(0.),Iwh1twh2t(0.),Iwh1wwt(4,INIT,0.),Iwh1twwt(4,INIT,0.),Iwh2wwt(4,INIT,0.),Iwh2twwt(4,INIT,0.),
    Iwh1wwtH(4,INIT,0.),Iwh1twwtH(4,INIT,0.),Iwh2wwtH(4,INIT,0.),Iwh2twwtH(4,INIT,0.),
    Iwh1xwh1x(0.),Iwh2xwh2x(0.),Iwh1xxwh1xx(0.),Iwh2xxwh2xx(0.),
    Iwh1xwxwt(4,INIT,0.),Iwh2xwxwt(4,INIT,0.),Iwh1xxwxxwt(4,INIT,0.),Iwh2xxwxxwt(4,INIT,0.),
    w1coef(4,INIT,0.),w2coef(4,INIT,0.),w1tcoef(4,INIT,0.),w2tcoef(4,INIT,0.),
    wh1coef(4,INIT,0.),wh2coef(4,INIT,0.),wh1tcoef(4,INIT,0.),wh2tcoef(4,INIT,0.),
    w1coefqI(4,16,INIT,0.),w2coefqI(4,16,INIT,0.),
    wh1coefqI(4,16,INIT,0.),wh2coefqI(4,16,INIT,0.),wh1tcoefqI(4,16,INIT,0.),wh2tcoefqI(4,16,INIT,0.),
    wh1coefqInunutH(16,4,INIT,0.),wh2coefqInunutH(16,4,INIT,0.),
    tSqI(3,16,INIT,0.),nSqI(3,16,INIT,0.),bSqI(3,16,INIT,0.),
    tStqI(3,16,INIT,0.),nStqI(3,16,INIT,0.),bStqI(3,16,INIT,0.),
    nSqIH(16,3,INIT,0.),bSqIH(16,3,INIT,0.),ntilSqI(3,16,INIT,0.),btilSqI(3,16,INIT,0.),
    xintilqI(16,INIT,0.),xibtilqI(16,INIT,0.),etantilqI(16,INIT,0.),etabtilqI(16,INIT,0.),
    omgt(0.),omgtqI(16,INIT,0.),omgtqIt(16,INIT,0.),omgtqItqIqIt(16,INIT,0.),
    Ttil(0.),TtilqI(16,INIT,0.),TtilqItqIt(16,INIT,0.),TtilqItqIqIt(16,INIT,0.),
    bam(0.5*l0),dpS(3,16,INIT,0.)
  {	
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

  Weight33RCM::~Weight33RCM() {}

  void Weight33RCM::setGauss(int nGauss) {
    gp = Vec(nGauss); // Gauss weights
    xip = Vec(nGauss); // Gauss points

    switch(nGauss) {
      case 1:
        xip(0) = 0.;

        gp(0) = 2.;
        break;
      case 2:
        xip(0) = -sqrt(1./3.);
        xip(1) = sqrt(1./3.);

        gp(0) = 1.;
        gp(1) = 1.;
        break;
      case 3:
        xip(0) = -sqrt(3./5.);
        xip(1) = 0.;
        xip(2) = sqrt(3./5.);

        gp(0) = 5./9.;
        gp(1) = 8./9.;
        gp(2) = 5./9.;
        break;
      case 4:
        xip(0) = -sqrt((15.+2.*sqrt(30.))/35.);
        xip(1) = -sqrt((15.-2.*sqrt(30.))/35.);
        xip(2) = sqrt((15.-2.*sqrt(30.))/35.);
        xip(3) = sqrt((15.+2.*sqrt(30.))/35.);

        gp(0) = (18.-sqrt(30.))/36.;
        gp(1) = (18.+sqrt(30.))/36.;
        gp(2) = (18.+sqrt(30.))/36.;
        gp(3) = (18.-sqrt(30.))/36.;
        break;
      case 5:
        xip(0) = -sqrt((35.+2.*sqrt(70.))/63.);
        xip(1) = -sqrt((35.-2.*sqrt(70.))/63.);
        xip(2) = 0.;
        xip(3) = sqrt((35.-2.*sqrt(70.))/63.);
        xip(4) = sqrt((35.+2.*sqrt(70.))/63.);

        gp(0) = (322.-13.*sqrt(70.))/900.;
        gp(1) = (322.+13.*sqrt(70.))/900.;
        gp(2) = 128./225.;
        gp(3) = (322.+13.*sqrt(70.))/900.;
        gp(4) = (322.-13.*sqrt(70.))/900.;
        break;
      default:
        throw MBSimError("ERROR (Weight33RCM::setGauss): Maximum of 5 Gauss points supported");
    }
  }

  void Weight33RCM::computeint(const Vec& qG,const Vec& qGt) {
    computewhcoefVel(qG,qGt);
    computeint();
  }

  void Weight33RCM::computeint() {
    // REQUIRED computewhcoefVel();
    computeT();

    Iwh1 = intv(wh1coef);
    Iwh2 = intv(wh2coef); 
    Ixwh1 = intxv(wh1coef);
    Ixwh2 = intxv(wh2coef);
    Iwh1wh1 = intvw(wh1coef,wh1coef);
    Iwh1wh2 = intvw(wh1coef,wh2coef);
    Iwh2wh2 = intvw(wh2coef,wh2coef);
    Iwh1xwh1x = intvxwx(wh1coef,wh1coef);
    Iwh2xwh2x = intvxwx(wh2coef,wh2coef);
    Iwh1xxwh1xx = intvxxvxx(wh1coef,k10);
    Iwh2xxwh2xx = intvxxvxx(wh2coef,k20);

    Iwh1t = intv(wh1tcoef); 
    Iwh2t = intv(wh2tcoef);	
    Ixwh1t = intxv(wh1tcoef); 
    Ixwh2t = intxv(wh2tcoef);
    Iwh1twh1 = intvw(wh1tcoef,wh1coef);
    Iwh1twh2 = intvw(wh1tcoef,wh2coef);
    Iwh1twh1t = intvw(wh1tcoef,wh1tcoef);	
    Iwh1wh2t = intvw(wh1coef,wh2tcoef);	
    Iwh2twh2t = intvw(wh2tcoef,wh2tcoef);
    Iwh2twh2 = intvw(wh2tcoef,wh2coef); 	
    Iwh1twh2t = intvw(wh1tcoef,wh2tcoef);
  }

  void Weight33RCM::computeintD(const Vec& qG,const Vec& qGt) {	
    tf->computeTrafo(qG,qGt);
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

    Iwh1xxwxxwt = intvxxwxxwt(wh1coef,k10); 
    Iwh2xxwxxwt  = intvxxwxxwt(wh2coef,k20);
    Iwh1xwxwt = intvxwxwt(wh1coef);
    Iwh2xwxwt = intvxwxwt(wh2coef);
  }

  Vec Weight33RCM::computew(const Vec& wt,double x) const {	
    Vec W(2);	
    W(0) = (((wt(0)*x+wt(1))*x+wt(2))*x+wt(3))*pow(x,2);
    W(1) = (((5*wt(0)*x+4*wt(1))*x+3*wt(2))*x+2*wt(3))*x;

    return W.copy();
  }

  void Weight33RCM::computewcoefPos(const Vec& qG) {
    tf->computeqI(qG);
    computewcoefPos();
  }

  void Weight33RCM::computewcoefPos() {
    // REQUIRED trafo->computeqI()
    Vec be = tf->getbe();

    w1coef = computewcoef(be(3),be(4),be(5),be(6));
    w2coef = computewcoef(be(7),be(8),be(9),be(10));
  }

  void Weight33RCM::computewcoefVel(const Vec& qG,const Vec& qGt) {
    tf->computezI(qG,qGt);
    computewcoefVel();
  }

  void Weight33RCM::computewcoefVel() {
    // REQUIRED trafo->computezI()
    Vec bet = tf->getbet();

    w1tcoef = computewcoef(bet(3),bet(4),bet(5),bet(6));
    w2tcoef = computewcoef(bet(7),bet(8),bet(9),bet(10));
  }

  void Weight33RCM::computewhcoefPos(const Vec& qG) {	
    tf->computeqI(qG);
    computewhcoefPos();		
  }

  void Weight33RCM::computewhcoefPos() {	
    // REQUIRED trafo->computeqI()
    computewcoefPos();

    wh1coef = tf->getxintil()*w1coef+tf->getxibtil()*w2coef;
    wh2coef = tf->getetantil()*w1coef+tf->getetabtil()*w2coef;		
  }

  void Weight33RCM::computewhcoefVel(const Vec& qG,const Vec& qGt) {
    tf->computeCOSYt(qG,qGt);
    computewhcoefPos();
    computewcoefVel();	
    computewhcoefVel();
  }

  void Weight33RCM::computewhcoefVel() {
    // REQUIRED trafo->computeCOSYt(qG,qGt);
    //			computewhcoefPos();
    //			computewcoefVel();

    wh1tcoef = tf->getxintilt()*w1coef+tf->getxibtilt()*w2coef + tf->getxintil()*w1tcoef+tf->getxibtil()*w2tcoef;
    wh2tcoef = tf->getetantilt()*w1coef+tf->getetabtilt()*w2coef + tf->getetantil()*w1tcoef+tf->getetabtil()*w2tcoef;
  }

  void Weight33RCM::computewcoefPosD() {
    w1coefqI(0,7,3,10) = tf->getV();
    w2coefqI(0,11,3,14) = tf->getV();
  }

  void Weight33RCM::computewhcoefPosD(const Vec& qG) {	 
    tf->computeJIG(qG);
    computewhcoefPosD();
  }

  void Weight33RCM::computewhcoefPosD() {	 
    // REQUIRED trafo->computeqI
    computewhcoefPos();

    tSqI = tf->gettSpS()*dpS;
    nSqI = tf->getnSpS()*dpS;
    bSqI = tf->getbSpS()*dpS;

    ntilSqI = tf->getntilSpS()*dpS;
    btilSqI = tf->getbtilSpS()*dpS;

    xintilqI = tf->getnSH()*ntilSqI+tf->getntilSH()*nSqI; 
    xibtilqI = tf->getnSH()*btilSqI+tf->getbtilSH()*nSqI;
    etantilqI = tf->getbSH()*ntilSqI+tf->getntilSH()*bSqI;
    etabtilqI = tf->getbSH()*btilSqI+tf->getbtilSH()*bSqI;

    wh1coefqI = w1coef*xintilqI+w2coef*xibtilqI + tf->getxintil()*w1coefqI+tf->getxibtil()*w2coefqI;	
    wh2coefqI = w1coef*etantilqI+w2coef*etabtilqI + tf->getetantil()*w1coefqI+tf->getetabtil()*w2coefqI;	
  }

  void Weight33RCM::computewhcoefVelD() {	
    computewhcoefPosD();
    computewcoefVel();
    computewhcoefVel();

    tStqI(0,3,2,5) = tf->gettSpSt();
    nStqI(0,3,2,5) = tf->getnSpSt();
    bStqI(0,3,2,5) = tf->getbSpSt();

    Mat ntilStqI(3,16,INIT,0.);
    ntilStqI(0,3,2,5) = tf->getntilSpSt();

    Mat btilStqI(3,16,INIT,0.);
    btilStqI(0,3,2,5) = tf->getbtilSpSt();

    RowVec xintiltqI = tf->getntilSH()*nStqI+tf->getnStH()*ntilSqI+tf->getntilStH()*nSqI+tf->getnSH()*ntilStqI;
    RowVec xibtiltqI = tf->getbtilSH()*nStqI+tf->getnStH()*btilSqI+tf->getbtilStH()*nSqI+tf->getnSH()*btilStqI;
    RowVec etantiltqI = tf->getntilSH()*bStqI+tf->getbStH()*ntilSqI+tf->getntilStH()*bSqI+tf->getbSH()*ntilStqI;
    RowVec etabtiltqI = tf->getbtilSH()*bStqI+tf->getbStH()*btilSqI+tf->getbtilStH()*bSqI+tf->getbSH()*btilStqI;

    wh1tcoefqI = w1coef*xintiltqI+w2coef*xibtiltqI
      + tf->getxintilt()*w1coefqI+tf->getxibtilt()*w2coefqI
      + w1tcoef*xintilqI+w2tcoef*xibtilqI;
    wh2tcoefqI = w1coef*etantiltqI+w2coef*etabtiltqI
      + tf->getetantilt()*w1coefqI+tf->getetabtilt()*w2coefqI
      + w1tcoef*etantilqI+w2tcoef*etabtilqI;

    nSqIH = nSqI.T();
    bSqIH = bSqI.T();

    Mat ntilSqIH = ntilSqI.T();
    Mat btilSqIH = btilSqI.T();

    RowVec xintilqInunut = (nSqIH*ntilSqI*tf->getqIt()+dpSH*tf->getnSpSt().T()*tf->getntilS()+ntilSqIH*nSqI*tf->getqIt()+dpSH*tf->getntilSpSt().T()*tf->getnS()).T();
    RowVec xibtilqInunut = (nSqIH*btilSqI*tf->getqIt()+dpSH*tf->getnSpSt().T()*tf->getbtilS()+btilSqIH*nSqI*tf->getqIt()+dpSH*tf->getbtilSpSt().T()*tf->getnS()).T();
    RowVec etantilqInunut = (bSqIH*ntilSqI*tf->getqIt()+dpSH*tf->getbSpSt().T()*tf->getntilS()+ntilSqIH*bSqI*tf->getqIt()+dpSH*tf->getntilSpSt().T()*tf->getbS()).T();
    RowVec etabtilqInunut = (bSqIH*btilSqI*tf->getqIt()+dpSH*tf->getbSpSt().T()*tf->getbtilS()+btilSqIH*bSqI*tf->getqIt()+dpSH*tf->getbtilSpSt().T()*tf->getbS()).T();

    wh1coefqInunutH = (w1coefqI*tf->getqIt()*xintilqI+w2coefqI*tf->getqIt()*xibtilqI+xintilqI*tf->getqIt()*w1coefqI+xibtilqI*tf->getqIt()*w2coefqI
        +w1coef*xintilqInunut+w2coef*xibtilqInunut).T();
    wh2coefqInunutH = (w1coefqI*tf->getqIt()*etantilqI+w2coefqI*tf->getqIt()*etabtilqI+etantilqI*tf->getqIt()*w1coefqI+etabtilqI*tf->getqIt()*w2coefqI
        +w1coef*etantilqInunut+w2coef*etabtilqInunut).T();
  }

  Vec Weight33RCM::computewcoef(double dL,double dR,double bL,double bR) const {
    double dLdRp = dL+dR;
    double dLdRm = dL-dR;
    double bLbRp = bL+bR;
    double bLbRm = bL-bR;

    Vec wcoef(4); 	
    wcoef(0) = (4.*l0*bLbRp+24.*dLdRm)/l0h5; 			
    wcoef(1) = (-2.*l0*bLbRm-8.*dLdRp)/l0h4;
    wcoef(2) = (-l0*bLbRp-10.*dLdRm)/l0h3;  			
    wcoef(3) = (l0*bLbRm+8.*dLdRp)/(2.*l0h2);

    return wcoef.copy();
  }

  void Weight33RCM::computeomgt(double x) {
    // REQUIRED tf->computezI()
    // REQUIRED computewcoef_Pos()
    // REQUIRED computewcoef_Vel()

    Vec pS = tf->getpS();
    Vec pSt = tf->getpSt();

    double spS1 = sin(pS(1));
    double cpS1 = cos(pS(1));

    Vec w1 = computew(w1coef,x);
    Vec w2 = computew(w2coef,x);
    Vec w2t = computew(w2tcoef,x);

    RowVec dwxdwt(4);
    dwxdwt(3) = 2*x;
    dwxdwt(2) = x*x*3.;
    dwxdwt(1) = x*x*x*4.;
    dwxdwt(0) = x*x*x*x*5.;

    RowVec w1xqI = dwxdwt*w1coefqI;
    RowVec w2xqI = dwxdwt*w2coefqI;

    omgt = pSt(0)+tf->getk0t()*x+cpS1*pSt(1)*w2(1)+spS1*w2t(1)-(spS1+cpS1*w1(1))*(pSt(2)+w2t(1));

    omgtqI = w1xqI;
    omgtqI(4) += 1.;
    omgtqI *= -(pSt(2)+w2t(1))*(cpS1-spS1*w1(1));
    omgtqI += cpS1*pSt(1)*w2xqI;
    omgtqI(4) += cpS1*w2t(1)-spS1*pSt(1)*w2(1);

    omgtqIt = w2xqI;
    omgtqIt(5) += 1.;
    omgtqIt *= -(spS1+cpS1*w1(1));
    omgtqIt(3) += 1;
    omgtqIt += spS1*w2xqI;
    omgtqIt(15) += x;
    omgtqIt(4) += cpS1*w2(1);

    omgtqItqIqIt = w2xqI.T();
    omgtqItqIqIt(5) += 1.;	
    omgtqItqIqIt *= -((cpS1-spS1*w1(1))*(pSt(1)+w1xqI*tf->getqIt()));
    omgtqItqIqIt += w2xqI.T()*cpS1*pSt(1);
    omgtqItqIqIt(4) += -spS1*w2(1)*pSt(1)+cpS1*w2xqI*tf->getqIt();
  }

  void Weight33RCM::computeT() {
    // REQUIRED tf->computezI()
    // REQUIRED computewcoef_Pos()
    // REQUIRED computewcoef_Vel()

    Ttil = 0.;
    TtilqI = RowVec(16,INIT,0.);
    TtilqItqIt = SymMat(16,INIT,0.);
    TtilqItqIqIt = Vec(16,INIT,0.);

    for(int i=0; i<gp.size();i++) {
      computeomgt(bam*xip(i));
      Ttil += bam*gp(i)*omgt*omgt;
      TtilqI += bam*gp(i)*omgt*omgtqI;
      TtilqItqIt += static_cast<SymMat>(bam*gp(i)*omgtqIt.T()*omgtqIt);
      TtilqItqIqIt += bam*gp(i)*(omgtqIt.T()*omgtqI*tf->getqIt()+omgt*omgtqItqIqIt);
    } 
  }

  void Weight33RCM::computedpS() {
    dpS(0,3) = 1.;
    dpS(1,4) = 1.;
    dpS(2,5) = 1.;
    dpSH = dpS.T();
  }

  double Weight33RCM::intv(const Vec& vt) const {
    const double& bv = vt(1); 
    const double& dv = vt(3);	

    return bv/80.*l0h5+dv/12.*l0h3;
  }

  double Weight33RCM::intvx(const Vec& vt) const {
    const double& av = vt(0); 
    const double& cv = vt(2); 

    return av*l0h5/16.+cv*l0h3/4.;
  }

  double Weight33RCM::intxv(const Vec& vt) const {
    const double& av = vt(0); 
    const double& cv = vt(2);	

    return av/448.*l0h7+cv/80.*l0h5;
  }

  double Weight33RCM::intxvx(const Vec& vt) const {
    const double& bv = vt(1); 
    const double& dv = vt(3);	

    return bv*l0h5/20.+dv*l0h3/6.;
  }

  double Weight33RCM::intvw(const Vec& vt,const Vec& wt) const {
    const double& av = vt(0); 
    const double& bv = vt(1); 
    const double& cv = vt(2);
    const double& dv = vt(3);
    const double& aw = wt(0); 
    const double& bw = wt(1); 
    const double& cw = wt(2); 
    const double& dw = wt(3); 

    return av*aw/11264.*l0h11+(av*cw+aw*cv+bv*bw)/2304.*l0h9+(bv*dw+bw*dv+cv*cw)/448.*l0h7+dv*dw/80.*l0h5;
  }

  double Weight33RCM::intvxwx(const Vec& vt,const Vec& wt) const {
    const double& av = vt(0); 
    const double& bv = vt(1); 
    const double& cv = vt(2); 
    const double& dv = vt(3);	
    const double& aw = wt(0); 
    const double& bw = wt(1); 
    const double& cw = wt(2); 
    const double& dw = wt(3);

    return 25.*av*aw/2304.*l0h9+(15.*cv*aw/448.+bv*bw/28.+15.*av*cw/448.)*l0h7
      +(dv*bw/10.+9.*cv*cw/80.+bv*dw/10.)*l0h5+dv*dw/3.*l0h3;
  }

  double Weight33RCM::intvxxvxx(const Vec& vt,double C) const {
    const double& av = vt(0); 
    const double& bv = vt(1); 
    const double& cv = vt(2); 
    const double& dv = vt(3);

    return 25.*pow(av,2)/28.*l0h7+(3.*av*cv+9.*pow(bv,2)/5.)*l0h5
      +(4.*bv*dv+3.*pow(cv,2)-2.*bv*C)*l0h3+(pow(C,2)-4.*C*dv+4.*pow(dv,2))*l0;
  }

  void Weight33RCM::intvvt() {
    Ivvt(1) = l0h5/80.;
    Ivvt(3) = l0h3/12.;
  }

  void Weight33RCM::intvxvt() {
    Ivxvt(0) = l0h5/16.;
    Ivxvt(2) = l0h3/4.;
  }

  void Weight33RCM::intxvvt() {
    Ixvvt(0) = l0h7/448.;
    Ixvvt(2) = l0h5/80.;
  }

  void Weight33RCM::intxvxvt() {
    Ixvxvt(1) = l0h5/20.;
    Ixvxvt(3) = l0h3/6.;
  }

  RowVec Weight33RCM::intvwwt(const Vec& vt) const {
    const double& av = vt(0); 
    const double& bv = vt(1); 
    const double& cv = vt(2); 
    const double& dv = vt(3);	

    RowVec Ivwwt(4);
    Ivwwt(0) = av/11264.*l0h11+cv/2304.*l0h9;
    Ivwwt(1) = bv/2304.*l0h9+dv/448.*l0h7;
    Ivwwt(2) = av/2304.*l0h9+cv/448.*l0h7;
    Ivwwt(3) = bv/448.*l0h7+dv/80.*l0h5;
    return Ivwwt.copy();
  }

  RowVec Weight33RCM::intvxwxwt(const Vec& vt) const {
    const double& av = vt(0); 
    const double& bv = vt(1); 
    const double& cv = vt(2); 
    const double& dv = vt(3);	

    RowVec Ivxwxwt(4);
    Ivxwxwt(0) = 25.*av/2304.*l0h9+15.*cv/448.*l0h7;
    Ivxwxwt(1) = bv/28.*l0h7+dv/10.*l0h5;
    Ivxwxwt(2) = 15.*av/448.*l0h7+9.*cv/80.*l0h5;
    Ivxwxwt(3) = bv/10.*l0h5+dv/3.*l0h3;		
    return Ivxwxwt.copy();
  }

  RowVec Weight33RCM::intvxxwxxwt(const Vec& vt,double C) const {
    const double& av = vt(0);
    const double& bv = vt(1);
    const double& cv = vt(2); 
    const double& dv = vt(3);

    RowVec Ivxxwxxwt(4);
    Ivxxwxxwt(0) = 25.*av/28.*l0h7+3.*cv/2.*l0h5;
    Ivxxwxxwt(1) = 9.*bv/5.*l0h5+(2.*dv-C)*l0h3;
    Ivxxwxxwt(2) = 3.*av/2.*l0h5+3.*cv*l0h3;
    Ivxxwxxwt(3) = 2.*bv*l0h3+(4.*dv-2.*C)*l0;		
    return Ivxxwxxwt.copy();
  }

  void Weight33RCM::intvvtwwt()  {
    Ivvtwwt(0,0) = l0h11/11264.;
    Ivvtwwt(0,2) = l0h9/2304.;
    Ivvtwwt(1,1) = Ivvtwwt(0,2);
    Ivvtwwt(1,3) = l0h7/448.;
    Ivvtwwt(2,2) = Ivvtwwt(1,3);
    Ivvtwwt(3,3) = l0h5/80.;
  }

  void Weight33RCM::intvxvtwxwt()  {
    Ivxvtwxwt(0,0) = 25.*l0h9/2304.;
    Ivxvtwxwt(0,2) = 15.*l0h7/448.;
    Ivxvtwxwt(1,1) = l0h7/28.;
    Ivxvtwxwt(1,3) = l0h5/10.;
    Ivxvtwxwt(2,2) = 9.*l0h5/80.;
    Ivxvtwxwt(3,3) = l0h3/3.;
  }

}

