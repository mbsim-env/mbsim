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

#include "mbsim/flexible_body/finite_elements/finite_element_1s_33_rcm/trafo33RCM.h"
#include "mbsim/flexible_body/finite_elements/finite_element_1s_33_rcm/revcardan.h"
#include "mbsim/utils/nonlinear_algebra.h"
#include "mbsim/utils/eps.h"
#include "mbsim/mbsim_event.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  /* Class PositionFunction */
  PositionFunction::PositionFunction(RevCardan *angle_,double l0_,const Vec& pL_,const Vec& pR_,double cL1_,double cR1_,double cL2_,double cR2_,const fmatvec::RowVec& rRrLmH_) : angle(angle_),l0(l0_),pL(pL_),pR(pR_),cL1(cL1_),cR1(cR1_),cL2(cL2_),cR2(cR2_),rRrLmH(rRrLmH_) {}

  PositionFunction::~PositionFunction() {}

  Vec PositionFunction::operator()(const Vec& pos, const void *) {			
    Vec pS = pos(0,2);

    Vec nS = angle->computen(pS);
    Vec bS = angle->computeb(pS);
    RowVec ntilSH = trans(angle->computentil(pS));
    RowVec btilSH = trans(angle->computebtil(pS));
    double xibtil = btilSH*nS; 
    double xintil = ntilSH*nS;
    double etabtil = btilSH*bS;
    double etantil = ntilSH*bS;

    Vec F(11);
    F(0) = pos(0)-0.5*(pR(0)+pL(0)-sin(pos(1))*(pos(9)+pos(10)));
    F(1) = pos(1)-0.5*(pR(1)+pL(1)-pos(5)-pos(6));
    F(2) = pos(2)-0.5*(pR(2)+pL(2)-pos(9)-pos(10));			
    F(3) = pos(6)-pos(5)-pR(1)+pL(1);	
    F(4) = pos(10)-pos(9)-pR(2)+pL(2);	
    F(5) = xintil*(pos(4)-pos(3))+xibtil*(pos(8)-pos(7))-rRrLmH*nS;
    F(6) = etantil*(pos(4)-pos(3))+etabtil*(pos(8)-pos(7))-rRrLmH*bS;	
    F(7) = 3.*l0*(pos(5)-pos(6))/64.+7.*(pos(3)+pos(4))/16.-cL1-cR1;
    F(8) = -3.*l0*(pos(5)+pos(6))/128.+17.*(pos(4)-pos(3))/64.+cL1-cR1;
    F(9) = 3.*l0*(pos(9)-pos(10))/64.+7.*(pos(7)+pos(8))/16.-cL2-cR2;		
    F(10) = -3.*l0*(pos(9)+pos(10))/128.+17.*(pos(8)-pos(7))/64.+cL2-cR2;		
    return F.copy();
  }
  /*******************************************************************/

  /* Class PositionJacobian */
  PositionJacobian::PositionJacobian(RevCardan *angle_,double l0_,const fmatvec::RowVec &rRrLmH_,const fmatvec::Mat &pSbE_) : angle(angle_),l0(l0_),rRrLmH(rRrLmH_),pSbE(pSbE_) {}

  PositionJacobian::~PositionJacobian() {}

  SqrMat PositionJacobian::operator()(const Vec& pos, const void *) {			
    Vec pS = pos(0,2);

    Vec nS = angle->computen(pS);
    Vec bS = angle->computeb(pS);
    RowVec ntilSH = trans(angle->computentil(pS));
    RowVec btilSH = trans(angle->computebtil(pS));
    double xibtil = btilSH*nS; 
    double xintil = ntilSH*nS;
    double etabtil = btilSH*bS;
    double etantil = ntilSH*bS;

    Mat nSbE = angle->computenq(pS)*pSbE; 
    Mat bSbE = angle->computebq(pS)*pSbE;
    Mat ntilSbEH = trans(angle->computentilq(pS)*pSbE); 
    Mat btilSbEH = trans(angle->computebtilq(pS)*pSbE);

    RowVec xibtilbE = trans(btilSbEH*nS)+btilSH*nSbE;
    RowVec xintilbE = trans(ntilSbEH*nS)+ntilSH*nSbE;	
    RowVec etabtilbE = trans(btilSbEH*bS)+btilSH*bSbE;
    RowVec etantilbE = trans(ntilSbEH*bS)+ntilSH*bSbE;

    SqrMat SMRHS_Jac(11,INIT,0.);
    SMRHS_Jac(0,0) = 1.;
    SMRHS_Jac(1,1) = 1.; SMRHS_Jac(1,5) = 0.5; SMRHS_Jac(1,6) = 0.5;
    SMRHS_Jac(2,2) = 1.; SMRHS_Jac(2,9) = 0.5; SMRHS_Jac(2,10) = 0.5;
    SMRHS_Jac(3,6) = 1.; SMRHS_Jac(3,5) = -1.;
    SMRHS_Jac(4,10) = 1.; SMRHS_Jac(4,9) = -1.;

    SMRHS_Jac(7,5) = 3.*l0/64.; SMRHS_Jac(7,6) = -3.*l0/64.; SMRHS_Jac(7,3) = 7./16.; SMRHS_Jac(7,4) = 7./16.;
    SMRHS_Jac(8,5) = -3.*l0/128.; SMRHS_Jac(8,6) = -3.*l0/128.; SMRHS_Jac(8,3) = -17./64.; SMRHS_Jac(8,4) = 17./64.;
    SMRHS_Jac(9,9) = 3.*l0/64.; SMRHS_Jac(9,10) = -3.*l0/64.; SMRHS_Jac(9,7) = 7./16.; SMRHS_Jac(9,8) = 7./16.;
    SMRHS_Jac(10,9) = -3.*l0/128.; SMRHS_Jac(10,10) = -3.*l0/128.; SMRHS_Jac(10,7) = -17./64.; SMRHS_Jac(10,8) = 17./64.;

    SMRHS_Jac(0,1) = 0.5*cos(pS(1))*(pos(9)+pos(10));
    SMRHS_Jac(0,9) = 0.5*sin(pS(1));
    SMRHS_Jac(0,10) = 0.5*sin(pS(1));
    SMRHS_Jac(5,0,5,10) = -rRrLmH*nSbE;
    SMRHS_Jac(5,0,5,10) +=xintilbE*(pos(4)-pos(3))+xibtilbE*(pos(8)-pos(7));
    SMRHS_Jac(5,4) +=xintil;
    SMRHS_Jac(5,3) -=xintil;
    SMRHS_Jac(5,8) +=xibtil;
    SMRHS_Jac(5,7) -=xibtil;

    SMRHS_Jac(6,0,6,10) = -rRrLmH*bSbE;
    SMRHS_Jac(6,0,6,10) +=etantilbE*(pos(4)-pos(3))+etabtilbE*(pos(8)-pos(7));
    SMRHS_Jac(6,4) +=etantil;
    SMRHS_Jac(6,3) -=etantil;
    SMRHS_Jac(6,8) +=etabtil;
    SMRHS_Jac(6,7) -=etabtil;
    return SMRHS_Jac.copy();
  }
  /*******************************************************************/

  /* CLASS TRAFO33RCM */
  Trafo33RCM::Trafo33RCM(RevCardan *angle_,double l0_) : angle(angle_),l0(l0_),l0h2(l0*l0),l0h3(l0h2*l0),l0h4(l0h3*l0),l0h5(l0h4*l0),xstar(0.25*l0),xstarh2(xstar*xstar),xstarh3(xstarh2*xstar),
  epstil(0.),k0(0.),rS(3,INIT,0.),pS(3,INIT,0.),rRrLp(3,INIT,0.),rRrLmH(3,INIT,0.),be(11,INIT,0.),tS(3,INIT,0.),nS(3,INIT,0.),bS(3,INIT,0.),ntilS(3,INIT,0.),btilS(3,INIT,0.),
  nSH(3,INIT,0.),bSH(3,INIT,0.),ntilSH(3,INIT,0.),btilSH(3,INIT,0.),tSpS(3,INIT,0.),nSpS(3,INIT,0.),bSpS(3,INIT,0.),ntilSpS(3,INIT,0.),btilSpS(3,INIT,0.),
  xibtil(0.),xintil(0.),etabtil(0.),etantil(0.),SMRHS_Jac(11,27,INIT,0.),V(4,INIT,0.),drRdrLp(3,16,INIT,0.),drRdrLm(3,16,INIT,0.),pSbE(3,11,INIT,0.),
  SMRHS(8,9,INIT,0.),nSbE(3,11,INIT,0.),bSbE(3,11,INIT,0.),ntilSbE(3,11,INIT,0.),btilSbE(3,11,INIT,0.),xibtilbE(11,INIT,0.),xintilbE(11,INIT,0.),etabtilbE(11,INIT,0.),etantilbE(11,INIT,0.),beqG(11,16,INIT,0.),
  tSqG(3,16,INIT,0.),nSqG(3,16,INIT,0.),bSqG(3,16,INIT,0.),ntilSqG(3,16,INIT,0.),btilSqG(3,16,INIT,0.),xintilqG(16,INIT,0.),xibtilqG(16,INIT,0.),etantilqG(16,INIT,0.),etabtilqG(16,INIT,0.),JIG(16,INIT,0.),JIGt(16,INIT,0.),
  k0t(0.),qIt(16,INIT,0.),rSt(3,INIT,0.),bet(11,INIT,0.),pSt(3,INIT,0.),tSt(3,INIT,0.),nSt(3,INIT,0.),bSt(3,INIT,0.),
  tStH(3,INIT,0.),nStH(3,INIT,0.),bStH(3,INIT,0.),ntilStH(3,INIT,0.),btilStH(3,INIT,0.),tSpSt(3,INIT,0.),nSpSt(3,INIT,0.),bSpSt(3,INIT,0.),
  ntilSpSt(3,INIT,0.),btilSpSt(3,INIT,0.),xibtilt(0.),xintilt(0.),etabtilt(0.),etantilt(0.)
  {	
    /* trafo initial value */
    SMRHS(0,2) = -1.; SMRHS(0,3) = 1.;
    SMRHS(1,6) = -1.; SMRHS(1,7) = 1.;

    SMRHS(4,0) = 7./16.; SMRHS(4,1) = 7./16.; SMRHS(4,2) = 3.*l0/64.; SMRHS(4,3) = -3.*l0/64.;
    SMRHS(5,0) = -17./64.; SMRHS(5,1) = 17./64.; SMRHS(5,2) = -3.*l0/128.; SMRHS(5,3) = -3.*l0/128.;
    SMRHS(6,4) = 7./16.; SMRHS(6,5) = 7./16.; SMRHS(6,6) = 3.*l0/64.; SMRHS(6,7) = -3.*l0/64.;
    SMRHS(7,4) = -17./64.; SMRHS(7,5) = 17./64.; SMRHS(7,6) = -3.*l0/128.; SMRHS(7,7) = -3.*l0/128.;

    /* trafo Jacobian */
    computeV();

    SMRHS_Jac(0,0) = 1.;
    SMRHS_Jac(1,1) = 1.; SMRHS_Jac(1,5) = 0.5; SMRHS_Jac(1,6) = 0.5;
    SMRHS_Jac(2,2) = 1.; SMRHS_Jac(2,9) = 0.5; SMRHS_Jac(2,10) = 0.5;
    SMRHS_Jac(3,6) = 1.; SMRHS_Jac(3,5) = -1.;
    SMRHS_Jac(4,10) = 1.; SMRHS_Jac(4,9) = -1.;
    SMRHS_Jac(7,3,7,6) = 2.*xstarh2*(V(1,0,1,3)*xstarh2+V(3,0,3,3));
    SMRHS_Jac(8,3,8,6) = 2.*xstarh3*(V(0,0,0,3)*xstarh2+V(2,0,2,3));
    SMRHS_Jac(9,7,9,10) = 2.*xstarh2*(V(1,0,1,3)*xstarh2+V(3,0,3,3));
    SMRHS_Jac(10,7,10,10) = 2.*xstarh3*(V(0,0,0,3)*xstarh2+V(2,0,2,3));

    SMRHS_Jac(0,14) = 0.5; SMRHS_Jac(0,24) = 0.5;
    SMRHS_Jac(1,15) = 0.5; SMRHS_Jac(1,25) = 0.5;
    SMRHS_Jac(2,16) = 0.5; SMRHS_Jac(2,26) = 0.5;		
    SMRHS_Jac(3,15) = -1.; SMRHS_Jac(3,25) = 1.;	
    SMRHS_Jac(4,16) = -1.; SMRHS_Jac(4,26) = 1.;	
    SMRHS_Jac(7,17) = 1.; SMRHS_Jac(7,18) = 1.;	
    SMRHS_Jac(8,17) = -1.; SMRHS_Jac(8,18) = 1.;
    SMRHS_Jac(9,19) = 1.; SMRHS_Jac(9,20) = 1.;	
    SMRHS_Jac(10,19) = -1.; SMRHS_Jac(10,20) = 1.;

    computedrRdrL();

    pSbE(0,0) = 1.;
    pSbE(1,1) = 1.;
    pSbE(2,2) = 1.;
  }

  Trafo33RCM::~Trafo33RCM() {}

  void Trafo33RCM::computeprelim(const Vec& qG) {
    rRrLmH = trans(qG(10,12)-qG(0,2)); 
  }

  Vec Trafo33RCM::computes0(const Vec& qG) {
    Vec pM = 0.5*(qG(3,5)+qG(13,15));
    Vec tM = angle->computet(pM);
    Vec nM = angle->computen(pM);
    Vec bM = angle->computeb(pM);
    Vec nMtil = angle->computentil(pM);
    Vec bMtil = angle->computebtil(pM);		
    SqrMat nMpM = angle->computenq(pM);
    SqrMat bMpM = angle->computebq(pM);

    double xinMtil = trans(nM)*nMtil;
    double xibMtil = trans(nM)*bMtil;
    double etanMtil = trans(bM)*nMtil;
    double etabMtil = trans(bM)*bMtil;

    RowVec rRrLmnMpM = 0.5*rRrLmH*nMpM;
    SMRHS(2,0) = -xinMtil; SMRHS(2,1) = xinMtil; SMRHS(2,4) = -xibMtil; SMRHS(2,5) = xibMtil;	
    SMRHS(2,2) = rRrLmnMpM(1); SMRHS(2,3) = rRrLmnMpM(1);
    SMRHS(2,6) = rRrLmnMpM(0)*sin(pM(1))+rRrLmnMpM(2); SMRHS(2,7) = rRrLmnMpM(0)*sin(pM(1))+rRrLmnMpM(2);	

    RowVec rRrLmbMpM = 0.5*rRrLmH*bMpM;
    SMRHS(3,0) = -etanMtil; SMRHS(3,1) = etanMtil; SMRHS(3,4) = -etabMtil; SMRHS(3,5) = etabMtil;
    SMRHS(3,2) = rRrLmbMpM(1); SMRHS(3,3) = rRrLmbMpM(1);
    SMRHS(3,6) = rRrLmbMpM(0)*sin(pM(1))+rRrLmbMpM(2); SMRHS(3,7) = rRrLmbMpM(0)*sin(pM(1))+rRrLmbMpM(2);

    SMRHS(0,8) = qG(14)-qG(4);
    SMRHS(1,8) = qG(15)-qG(5);
    SMRHS(2,8) = rRrLmH*nM;	
    SMRHS(3,8) = rRrLmH*bM;
    SMRHS(4,8) = qG(6)+qG(7);
    SMRHS(5,8) = qG(7)-qG(6);
    SMRHS(6,8) = qG(8)+qG(9);
    SMRHS(7,8) = qG(9)-qG(8);

    Vec sol = slvLU(static_cast<SqrMat>(SMRHS(0,0,7,7)),static_cast<Vec>(SMRHS(0,8,7,8)));	
    Vec s0(11);
    s0(3,10) = sol;

    s0(1) = 0.5*((qG(4)+qG(14))-(sol(2)+sol(3)));
    s0(2) = 0.5*((qG(5)+qG(15))-(sol(6)+sol(7)));
    s0(0) = 0.5*(qG(3)+qG(13)-sin(s0(1))*(sol(6)+sol(7)));

    return s0.copy();
  }

  void Trafo33RCM::computebe(const Vec& qG) {
    // REQUIRED	computeprelim()

    Vec pL = qG(3,5);	
    Vec pR = qG(13,15);

    PositionFunction fun(angle,l0,pL,pR,qG(6),qG(7),qG(8),qG(9),rRrLmH); // (object itself no reference)
    PositionJacobian jac (angle,l0,rRrLmH,pSbE);
    MultiDimNewtonMethod rf(&fun,&jac);
    rf.setMaximumNumberOfIterations(10);	

    if(nrm2(be)<epsroot()) {
      Vec s0 = computes0(qG); // initial value
      be = rf.solve(s0); // Newton method according to Deuflhard (nonlinear_algebra.h)
    }
    else be = rf.solve(be);

    if(rf.getInfo()!=0) {
      throw new MBSimError("ERROR (TRAFO33RCM:computebe): No convergence of Newton method during bending correction.");
    }
  }

  void Trafo33RCM::computeCOSY() {
    // REQUIRED	computebe()

    pS = be(0,2);
    tS = angle->computet(pS);
    nS = angle->computen(pS);
    bS = angle->computeb(pS);
    ntilS = angle->computentil(pS);
    btilS = angle->computebtil(pS); 
    nSH = trans(nS); 
    bSH = trans(bS);
    ntilSH = trans(ntilS);
    btilSH = trans(btilS);
    tSpS = angle->computetq(pS);
    nSpS = angle->computenq(pS);
    bSpS = angle->computebq(pS);
    ntilSpS = angle->computentilq(pS);
    btilSpS = angle->computebtilq(pS);

    xibtil = nSH*btilS;
    xintil = nSH*ntilS;
    etabtil = bSH*btilS;
    etantil = bSH*ntilS;
  }

  void Trafo33RCM::computerSepstk0(const Vec& qG) {
    // REQUIRED	computeCOSY()

    Vec rRrLp = qG(10,12)+qG(0,2);

    rS = 0.5*(rRrLp-(xintil*(be(3)+be(4))+xibtil*(be(7)+be(8)))*nS-(etantil*(be(3)+be(4))+etabtil*(be(7)+be(8)))*bS);
    epstil = rRrLmH*tS/l0-1.;
    k0 = (qG(13)-qG(3)-sin(pS(1))*(be(10)-be(9)))/l0;
  }

  void Trafo33RCM::computeqI(const Vec& qG) {
    computeprelim(qG);
    computebe(qG);
    computeCOSY();
    computerSepstk0(qG);
  }

  void Trafo33RCM::computedrRdrL() {
    drRdrLp(0,0) = 1.; drRdrLp(1,1) = 1.; drRdrLp(2,2) = 1.;
    drRdrLp(0,10) = 1.; drRdrLp(1,11) = 1.; drRdrLp(2,12) = 1.;	

    drRdrLm(0,0) = -1.; drRdrLm(1,1) = -1.; drRdrLm(2,2) = -1.;
    drRdrLm(0,10) = 1.; drRdrLm(1,11) = 1.; drRdrLm(2,12) = 1.;
  }

  void Trafo33RCM::computeV() {	 	
    V(0,0) = 24./l0h5;
    V(1,0) = -8./l0h4;
    V(2,0) = -10./l0h3;
    V(3,0) = 4./l0h2;		

    V(0,1) = -24./l0h5;
    V(1,1) = -8./l0h4;
    V(2,1) = 10./l0h3;
    V(3,1) = 4./l0h2;			

    V(0,2) = 4./l0h4;
    V(1,2) = -2./l0h3;
    V(2,2) = -1./l0h2;
    V(3,2) = 1./(2.*l0);

    V(0,3) = 4./l0h4;
    V(1,3) = 2./l0h3; 
    V(2,3) = -1./l0h2;
    V(3,3) = -1./(2.*l0);	
  }

  void Trafo33RCM::computebeqG() {
    // REQUIRED	computeqI()

    nSbE = nSpS*pSbE; 
    bSbE = bSpS*pSbE;
    ntilSbE = ntilSpS*pSbE; 
    btilSbE = btilSpS*pSbE;

    xibtilbE = nSH*btilSbE+btilSH*nSbE;
    xintilbE = nSH*ntilSbE+ntilSH*nSbE;	
    etabtilbE = bSH*btilSbE+btilSH*bSbE;
    etantilbE = bSH*ntilSbE+ntilSH*bSbE;

    SMRHS_Jac(0,1) = 0.5*cos(pS(1))*(be(9)+be(10));
    SMRHS_Jac(0,9) = 0.5*sin(pS(1));
    SMRHS_Jac(0,10) = 0.5*sin(pS(1));
    SMRHS_Jac(5,0,5,10) = -rRrLmH*nSbE;
    SMRHS_Jac(5,0,5,10) +=xintilbE*(be(4)-be(3))+xibtilbE*(be(8)-be(7));
    SMRHS_Jac(5,4) +=xintil;
    SMRHS_Jac(5,3) -=xintil;
    SMRHS_Jac(5,8) +=xibtil;
    SMRHS_Jac(5,7) -=xibtil;

    SMRHS_Jac(6,0,6,10) = -rRrLmH*bSbE;
    SMRHS_Jac(6,0,6,10) +=etantilbE*(be(4)-be(3))+etabtilbE*(be(8)-be(7));
    SMRHS_Jac(6,4) +=etantil;
    SMRHS_Jac(6,3) -=etantil;
    SMRHS_Jac(6,8) +=etabtil;
    SMRHS_Jac(6,7) -=etabtil;

    SMRHS_Jac(5,11,5,26) = nSH*drRdrLm;
    SMRHS_Jac(6,11,6,26) = bSH*drRdrLm;

    beqG = slvLU(static_cast<SqrMat>(SMRHS_Jac(0,0,10,10)),SMRHS_Jac(0,11,10,26));
  }

  void Trafo33RCM::computeCOSYqG() {
    // REQUIRED	computebeqG()

    tSqG = tSpS*beqG(0,0,2,15); 
    nSqG = nSpS*beqG(0,0,2,15); 
    bSqG = bSpS*beqG(0,0,2,15);
    ntilSqG = ntilSpS*beqG(0,0,2,15); 
    btilSqG = btilSpS*beqG(0,0,2,15);

    xintilqG = nSH*ntilSqG+ntilSH*nSqG;
    xibtilqG = nSH*btilSqG+btilSH*nSqG;
    etantilqG = bSH*ntilSqG+ntilSH*bSqG;
    etabtilqG = bSH*btilSqG+btilSH*bSqG;	
  }

  void Trafo33RCM::computeJIG(const Vec& qG) {
    computeqI(qG);
    computebeqG();
    computeCOSYqG();

    RowVec tSH = trans(tS);

    JIG(0,0,2,15) = drRdrLp;
    JIG(0,0,2,15) -= nS*(xintilqG*(be(3)+be(4))+xibtilqG*(be(7)+be(8))+xintil*(beqG(3,0,3,15)+beqG(4,0,4,15))+xibtil*(beqG(7,0,7,15)+beqG(8,0,8,15)));
    JIG(0,0,2,15) -= (xintil*(be(3)+be(4))+xibtil*(be(7)+be(8)))*nSqG;
    JIG(0,0,2,15) -= bS*(etantilqG*(be(3)+be(4))+etabtilqG*(be(7)+be(8))+etantil*(beqG(3,0,3,15)+beqG(4,0,4,15))+etabtil*(beqG(7,0,7,15)+beqG(8,0,8,15)));
    JIG(0,0,2,15) -= (etantil*(be(3)+be(4))+etabtil*(be(7)+be(8)))*bSqG;
    JIG(0,0,2,15) *= 0.5;		
    JIG(3,0,5,15) = beqG(0,0,2,15);
    JIG(6,0,6,15) = (tSH*drRdrLm+rRrLmH*tSqG)/l0;
    JIG(7,0,14,15) = beqG(3,0,10,15);
    JIG(15,0,15,15) = -sin(pS(1))*(beqG(10,0,10,15)-beqG(9,0,9,15))-(be(10)-be(9))*cos(pS(1))*beqG(1,0,1,15);
    JIG(15,3) -= 1.;
    JIG(15,13) += 1.;
    JIG(15,0,15,15) /= l0;
  }

  void Trafo33RCM::computezI(const Vec& qG,const Vec& qGt) {
    computeJIG(qG);

    qIt = JIG*qGt;
    rSt = qIt(0,2);
    pSt = qIt(3,5);
    epstilt = qIt(6);
    bet(0,2) = pSt;
    bet(3,10) = qIt(7,14);
    k0t = qIt(15);
  }

  void Trafo33RCM::computeCOSYt(const Vec& qG,const Vec& qGt) {
    computezI(qG,qGt);

    tSt = tSpS*pSt;
    nSt = nSpS*pSt;
    bSt = bSpS*pSt;
    Vec ntilSt = ntilSpS*pSt;
    Vec btilSt = btilSpS*pSt;

    nStH = trans(nSt); 
    bStH = trans(bSt);
    ntilStH = trans(ntilSt);
    btilStH = trans(btilSt);

    tSpSt = angle->computetqt(pS,pSt);
    nSpSt = angle->computenqt(pS,pSt);
    bSpSt = angle->computebqt(pS,pSt);
    ntilSpSt = angle->computentilqt(pS,pSt);
    btilSpSt = angle->computebtilqt(pS,pSt);

    xibtilt = nStH*btilS + btilStH*nS;
    xintilt = nStH*ntilS + ntilStH*nS;
    etabtilt = bStH*btilS + btilStH*bS;
    etantilt = bStH*ntilS + ntilStH*bS;
  }

  void Trafo33RCM::computeJIGt(const Vec& qGt) {
    // REQUIRED	computeCOSYt()

    RowVec rRrLtmH = trans(qGt(10,12)-qGt(0,2));		
    RowVec tStH = trans(tSt); 

    Mat nSbEt = nSpSt*pSbE;
    Mat bSbEt = bSpSt*pSbE;	 
    Mat ntilSbEt = ntilSpSt*pSbE;
    Mat btilSbEt = btilSpSt*pSbE;

    RowVec xibtilbEt = nStH*btilSbE+btilStH*nSbE + nSH*btilSbEt+btilSH*nSbEt;
    RowVec xintilbEt = nStH*ntilSbE+ntilStH*nSbE + nSH*ntilSbEt+ntilSH*nSbEt;
    RowVec etabtilbEt = bStH*btilSbE+btilStH*bSbE + bSH*btilSbEt+btilSH*bSbEt;
    RowVec etantilbEt = bStH*ntilSbE+ntilStH*bSbE + bSH*ntilSbEt+ntilSH*bSbEt;

    Mat tSqGt = tSpSt*beqG(0,0,2,15)+tSpS*JIGt(3,0,5,15);
    Mat nSqGt = nSpSt*beqG(0,0,2,15)+nSpS*JIGt(3,0,5,15);
    Mat bSqGt = bSpSt*beqG(0,0,2,15)+bSpS*JIGt(3,0,5,15);
    Mat ntilSqGt = ntilSpSt*beqG(0,0,2,15)+ntilSpS*JIGt(3,0,5,15);
    Mat btilSqGt = btilSpSt*beqG(0,0,2,15)+btilSpS*JIGt(3,0,5,15);

    RowVec xibtilqGt = nStH*btilSqG+btilStH*nSqG + nSH*btilSqGt+btilSH*nSqGt;
    RowVec xintilqGt = nStH*ntilSqG+ntilStH*nSqG + nSH*ntilSqGt+ntilSH*nSqGt;
    RowVec etabtilqGt = bStH*btilSqG+btilStH*bSqG + bSH*btilSqGt+btilSH*bSqGt;	
    RowVec etantilqGt = bStH*ntilSqG+ntilStH*bSqG + bSH*ntilSqGt+ntilSH*bSqGt;	

    SqrMat SMt(11,INIT,0.);
    SMt(0,1) = 0.5*(-sin(pS(1))*pSt(1)*(be(9)+be(10))+cos(pS(1))*(bet(9)+bet(10)));
    SMt(0,9) = 0.5*cos(pS(1))*pSt(1);
    SMt(0,10) = 0.5*cos(pS(1))*pSt(1);

    SMt(5,0,5,10) = -rRrLtmH*nSbE-rRrLmH*nSbEt;
    SMt(5,0,5,10) += xintilbEt*(be(4)-be(3))+xibtilbEt*(be(8)-be(7));
    SMt(5,0,5,10) += xintilbE*(bet(4)-bet(3))+xibtilbE*(bet(8)-bet(7));
    SMt(5,4) +=xintilt;
    SMt(5,3) -=xintilt;
    SMt(5,8) +=xibtilt;
    SMt(5,7) -=xibtilt;

    SMt(6,0,6,10) = -rRrLtmH*bSbE-rRrLmH*bSbEt;
    SMt(6,0,6,10) += etantilbEt*(be(4)-be(3))+etabtilbEt*(be(8)-be(7));
    SMt(6,0,6,10) += etantilbE*(bet(4)-bet(3))+etabtilbE*(bet(8)-bet(7));
    SMt(6,4) +=etantilt;
    SMt(6,3) -=etantilt;
    SMt(6,8) +=etabtilt;
    SMt(6,7) -=etabtilt;

    Mat RHS_JIGt = -SMt*beqG;
    RHS_JIGt(5,0,5,15) += nStH*drRdrLm;
    RHS_JIGt(6,0,6,15) += bStH*drRdrLm;

    Mat beqGt = slvLU(static_cast<SqrMat>(SMRHS_Jac(0,0,10,10)),RHS_JIGt);	

    JIGt(3,0,5,15) = beqGt(0,0,2,15);
    JIGt(7,0,14,15) = beqGt(3,0,10,15);

    JIGt(6,0,6,15) = (tStH*drRdrLm+rRrLtmH*tSqG+rRrLmH*tSqGt)/l0;

    JIGt(0,0,2,15) = -nSt*(xintilqG*(be(3)+be(4))+xibtilqG*(be(7)+be(8))+xintil*(beqG(3,0,3,15)+beqG(4,0,4,15))+xibtil*(beqG(7,0,7,15)+beqG(8,0,8,15)));
    JIGt(0,0,2,15) -= nS*(xintilqGt*(be(3)+be(4))+xibtilqGt*(be(7)+be(8))+xintilt*(beqG(3,0,3,15)+beqG(4,0,4,15))+xibtilt*(beqG(7,0,7,15)+beqG(8,0,8,15)));
    JIGt(0,0,2,15) -= nS*(xintilqG*(bet(3)+bet(4))+xibtilqG*(bet(7)+bet(8))+xintil*(beqGt(3,0,3,15)+beqGt(4,0,4,15))+xibtil*(beqGt(7,0,7,15)+beqGt(8,0,8,15)));

    JIGt(0,0,2,15) -= (xintil*(be(3)+be(4))+xibtil*(be(7)+be(8)))*nSqGt;
    JIGt(0,0,2,15) -= (xintilt*(be(3)+be(4))+xibtilt*(be(7)+be(8)))*nSqG;
    JIGt(0,0,2,15) -= (xintil*(bet(3)+bet(4))+xibtil*(bet(7)+bet(8)))*nSqG;

    JIGt(0,0,2,15) -= bSt*(etantilqG*(be(3)+be(4))+etabtilqG*(be(7)+be(8))+etantil*(beqG(3,0,3,15)+beqG(4,0,4,15))+etabtil*(beqG(7,0,7,15)+beqG(8,0,8,15)));
    JIGt(0,0,2,15) -= bS*(etantilqGt*(be(3)+be(4))+etabtilqGt*(be(7)+be(8))+etantilt*(beqG(3,0,3,15)+beqG(4,0,4,15))+etabtilt*(beqG(7,0,7,15)+beqG(8,0,8,15)));
    JIGt(0,0,2,15) -= bS*(etantilqG*(bet(3)+bet(4))+etabtilqG*(bet(7)+bet(8))+etantil*(beqGt(3,0,3,15)+beqGt(4,0,4,15))+etabtil*(beqGt(7,0,7,15)+beqGt(8,0,8,15)));

    JIGt(0,0,2,15) -= (etantil*(be(3)+be(4))+etabtil*(be(7)+be(8)))*bSqGt;
    JIGt(0,0,2,15) -= (etantilt*(be(3)+be(4))+etabtilt*(be(7)+be(8)))*bSqG;
    JIGt(0,0,2,15) -= (etantil*(bet(3)+bet(4))+etabtil*(bet(7)+bet(8)))*bSqG;

    JIGt(0,0,2,15) *= 0.5;

    JIGt(15,0,15,15) = -cos(pS(1))*pSt(1)*(beqG(10,0,10,15)-beqG(9,0,9,15))-((bet(10)-bet(9))*cos(pS(1))-(be(10)-be(9))*sin(pS(1))*pSt(1))*beqG(1,0,1,15);
    JIGt(15,0,15,15) += -sin(pS(1))*(beqGt(10,0,10,15)-beqGt(9,0,9,15))-(be(10)-be(9))*cos(pS(1))*beqGt(1,0,1,15);
    JIGt(15,0,15,15) /= l0;
  }

  void Trafo33RCM::computeTrafo(const Vec& qG,const Vec& qGt) {	
    computeCOSYt(qG,qGt);
    computeJIGt(qGt);
  }
  /*******************************************************************/

}

