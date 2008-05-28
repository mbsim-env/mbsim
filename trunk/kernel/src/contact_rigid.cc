/* Copyright (C) 2004-2006  Martin Förg
 
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
 * Contact:
 *   mfoerg@users.berlios.de
 *
 */
#include<config.h>
#define FMATVEC_NO_BOUNDS_CHECK
#include "contact_rigid.h"
#include "utils/nonsmooth_algebra.h"
#include "multi_body_system.h"
#include "function.h"

namespace MBSim {

  double sign(double x) {
    if(x>0) return 1.0;
    else if(x<0) return -1.0;
    else return 0;
  }

  ContactRigid::ContactRigid(const string &name) : Contact(name,true), argT(2) {}

  void ContactRigid::init() {
    Contact::init();
    for(int i=0; i<2; i++) {
      loadDir.push_back(Mat(6,laSize));
      fF[i] >> loadDir[i](Index(0,2),Index(0,laSize-1));
    }
  }

  void ContactRigid::updateKinetics(double t) {
    fF[0].col(0) = getContourPointData(0).Wn;
    fF[0](Index(0,2),iT) = getContourPointData(0).Wt;
    fF[1] = -fF[0];
  }

  void ContactRigid::checkActive() {
    bool active_old = active;
    Contact::checkActive();
    if(active != active_old) mbs->setActiveConstraintsChanged(true);
  }

  void ContactRigid::projectJ(double dt) {
    if(nFric==1) la(1) = proxCT2D(la(1)-rFactor(1)*s(1),((*mue_fun)(abs(s(1))))(0)*fabs(la(0)));
    else if(nFric == 2) la(1,2) = proxCT3D(la(1,2)-rFactor(1)*s(1,2),((*mue_fun)(nrm2(s(1,2))))(0)*fabs(la(0)));
    la(0) = proxCN(la(0)-rFactor(0)*s(0));
  }

  void ContactRigid::projectGS(double dt) {
    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    double *lap = mbs->getla()();
    gdn(0) = s(0);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++) gdn(0) += a[j]*lap[ja[j]];

    la(0) = proxCN(la(0)-rFactor(0)*gdn(0));

    for(int i=1; i<=nFric; i++) {
      gdn(i) = s(i);
      for(int j=ia[laInd+i]; j<ia[laInd+1+i]; j++) gdn(i) += a[j]*lap[ja[j]];
    }
    if(nFric == 1) la(1) = proxCT2D(la(1)-rFactor(1)*gdn(1),((*mue_fun)(abs(gdn(1))))(0)*la(0));
    else if(nFric == 2) la(1,2) = proxCT3D(la(1,2)-rFactor(1)*gdn(1,2),((*mue_fun)(nrm2(gdn(1,2))))(0)*la(0));
  }

  void ContactRigid::solveGS(double dt) {
    assert(nFric <= 1);

    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    gdn(0) = s(0);
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++) gdn(0) += a[j]*mbs->getla()(ja[j]);

    if(gdn(0) >= 0) la(0) = 0;
    else la(0) = -gdn(0)/a[ia[laInd]];

    if(nFric) {
      gdn(1) = s(1);
      for(int j=ia[laInd+1]+1; j<ia[laInd+2]; j++) gdn(1) += a[j]*mbs->getla()(ja[j]);
      double laNmue = fabs(la(0))*((*mue_fun)(abs(gdn(1))))(0);
      double sdG = -gdn(1)/a[ia[laInd+1]];

      if(fabs(sdG)<=laNmue) la(1) = sdG;
      else la(1) = (laNmue<=sdG) ? laNmue : -laNmue;
    }
  }

  void ContactRigid::updaterFactors() {
    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
//    int *ja = mbs->getGs().Jp(); // unused
    double sumN = 0;
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++) sumN += fabs(a[j]);
    double aN = a[ia[laInd]];
    if(aN > sumN) {
      rFactorUnsure(0) = 0;
      rFactor(0) = 1.0/aN;
    }
    else {
      rFactorUnsure(0) = 1;
      rFactor(0) = rMax/aN;
    }
    double sumT1 = 0;
    double sumT2 = 0;
    double aT1, aT2;
    if(nFric == 1) {
      for(int j=ia[laInd+1]+1; j<ia[laInd+2]; j++) sumT1 += fabs(a[j]);
      aT1 = a[ia[laInd+1]];
      if(aT1 > sumT1) {
		rFactorUnsure(1)=0;
		rFactor(1) = 1.0/aT1;
      }
      else {
		rFactorUnsure(1)=1;
		rFactor(1) = rMax/aT1;
      }
    }
    else if(nFric == 2) {
      for(int j=ia[laInd+1]+1; j<ia[laInd+2]; j++) sumT1 += fabs(a[j]);
      for(int j=ia[laInd+2]+1; j<ia[laInd+3]; j++) sumT2 += fabs(a[j]);
      aT1 = a[ia[laInd+1]];
      aT2 = a[ia[laInd+2]];

      // TODO rFactorUnsure
      if(aT1 - sumT1 >= aT2 - sumT2) 
		if(aT1 + sumT1 >= aT2 + sumT2) rFactor(1) = 2.0/(aT1+aT2+sumT1-sumT2);
		else rFactor(1) = 1.0/aT2;
      else 
		if(aT1 + sumT1 < aT2 + sumT2) rFactor(1) = 2.0/(aT1+aT2-sumT1+sumT2);
		else rFactor(1) = 1.0/aT1;
    }
  }

  void ContactRigid::residualProj(double dt) {
    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    for(int i=0; i < 1+nFric; i++) {
      gdn(i) = s(i);
      for(int j=ia[laInd+i]; j<ia[laInd+1+i]; j++) gdn(i) += a[j]*mbs->getla()(ja[j]);
    }

    argN = la(0) - rFactor(0)*gdn(0);
    res(0) = la(0) - proxCN(argN);

    if(nFric==1) {
      updateFrictionCoefficient(abs(gdn(1)));
      argT(0) = la(1) - rFactor(1)*gdn(1);
      res(1) = la(1) - proxCT2D(argT(0),mue*fabs(la(0)));

    }
    else if(nFric == 2) {
      updateFrictionCoefficient(nrm2(gdn(1,2)));
      argT = la(1,2) - rFactor(1)*gdn(1,2);
      res(1,2) = la(1,2) - proxCT3D(argT,mue*fabs(la(0)));
    }
  }

  void ContactRigid::residualProjJac(double dt) {
    SqrMat Jprox = mbs->getJprox();
    SymMat G = mbs->getG();

    double rfac0 = rFactor(0);
    RowVec jp1=Jprox.row(laInd);
    if(argN < 0.) {
      jp1.init(0);
      jp1(laInd) = 1;
    }
    else for(int i=0; i<G.size(); i++) jp1(i) = rfac0*G(laInd,i);

    if(nFric == 1) {
      RowVec jp2=Jprox.row(laInd+1);
      double rfac1 = rFactor(1);
      double laNmue = fabs(la(0))*mue;
      if(fabs(argT(0))<=laNmue) for(int i=0; i<G.size(); i++) jp2(i) = rfac1*G(laInd+1,i);
      else {
		jp2.init(0);
		jp2(laInd+1) = 1;
		jp2(laInd) = -sign(argT(0))*sign(la(0))*mue;
      }
    }
    else if(nFric == 2) {
      RowVec jp2=Jprox.row(laInd+1);
      RowVec jp3=Jprox.row(laInd+2);
      double LaT = pow(argT(0),2)+pow(argT(1),2);
      double fabsLaT = sqrt(LaT);
      double laNmue = fabs(la(0))*mue;
      double rFac1 = rFactor(1);
      if(fabsLaT <=  laNmue) {
		for(int i=0; i<G.size(); i++) {
		  jp2(i) = rFac1*G(laInd+1,i);
		  jp3(i) = rFac1*G(laInd+2,i);
		}
      }
      else {
		SymMat dfda(2,NONINIT);
		dfda(0,0) = 1-argT(0)*argT(0)/LaT;
		dfda(1,1) = 1-argT(1)*argT(1)/LaT;
		dfda(0,1) = -argT(0)*argT(1)/LaT;
	
		for(int i=0; i<G.size(); i++) {
		  double e1 = (i==laInd) ? sign(la(0))*mue : 0;
		  double e2 = (i==laInd+1?1.0:0.0);
		  double e3 = (i==laInd+2?1.0:0.0);
		  jp2(i) = e2 - ((dfda(0,0)*(e2 - rFac1*G(laInd+1,i)) + dfda(0,1)*(e3 - rFac1*G(laInd+2,i)))*laNmue + e1*argT(0))/fabsLaT;
		  jp3(i) = e3 - ((dfda(1,0)*(e2 - rFac1*G(laInd+1,i)) + dfda(1,1)*(e3 - rFac1*G(laInd+2,i)))*laNmue + e1*argT(1))/fabsLaT;
		}
      }
    }
  }

  void ContactRigid::checkForTermination(double dt) {
    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    for(int i=0; i < 1+nFric; i++) {
      gdn(i) = s(i);
      for(int j=ia[laInd+i]; j<ia[laInd+1+i]; j++) gdn(i) += a[j]*mbs->getla()(ja[j]);
    }

    if(gdn(0) >= -gdTol && fabs(la(0)) <= laTol*dt);
    else if(la(0) >= -laTol*dt && fabs(gdn(0)) <= gdTol);
    else {
      mbs->setTermination(false);
      return;
    }

    if(nFric==1) {
      if(fabs(la(1) + gdn(1)/fabs(gdn(1))*((*mue_fun)(abs(gdn(1))))(0)*fabs(la(0))) <= laTol*dt);
      else if(fabs(la(1)) <= ((*mue_fun)(abs(gdn(1))))(0)*fabs(la(0)) + laTol*dt && fabs(gdn(1)) <= gdTol);
      else {
		mbs->setTermination(false);
		return;
      }
    }
    else if(nFric==2) {
      if(nrm2(la(1,2) + gdn(1,2)/nrm2(gdn(1,2))*((*mue_fun)(nrm2(gdn(1,2))))(0)*fabs(la(0))) <= laTol*dt);
      else if(nrm2(la(1,2)) <= ((*mue_fun)(nrm2(gdn(1,2))))(0)*fabs(la(0))+laTol*dt && nrm2(gdn(1,2)) <= gdTol);
      else {
		mbs->setTermination(false);
		return;
      }
    }
  }

  std::string ContactRigid::getTerminationInfo(double dt) {
    std::string s= "ContactRigid " + getName();
    bool NormalDirectionFailed = false;
    
    if(gdn(0) >= -gdTol && fabs(la(0)) <= laTol*dt);
    else if(la(0) >= -laTol*dt && fabs(gdn(0)) <= gdTol);
    else {
      s= s+" (normal): no convergence gdn= " + numtostr(gdn(0)) + " (gdTol= " + numtostr(gdTol);
      s= s+ ")     la(0)= " + numtostr(la(0)/dt) + " (laTol= " + numtostr(laTol) + ")";
      NormalDirectionFailed = true;
    }

    if(nFric==1) {
      if(fabs(la(1) + gdn(1)/fabs(gdn(1))*((*mue_fun)(abs(gdn(1))))(0)*fabs(la(0))) <= laTol*dt); // Gleiten 
      else if(fabs(la(1)) <= ((*mue_fun)(abs(gdn(1))))(0)*fabs(la(0)) + laTol*dt && fabs(gdn(1)) <= gdTol); // Haften
      else {
		if (NormalDirectionFailed) s += "\n";
		s= s+" (1D tangential): no convergence gdT= " + numtostr(gdn(1)) + " (gdTol= "+ numtostr(gdTol);
		s= s+ ")\n    stick: abs(laT) - mue abs(laN) = " + numtostr(fabs(la(1)/dt)- ((*mue_fun)(abs(gdn(1))))(0)*fabs(la(0)/dt));
	    s= s+ "\n     slip: abs(laT - mue laN)       = " + numtostr(fabs(la(1) + gdn(1)/fabs(gdn(1))*((*mue_fun)(abs(gdn(1))))(0)*fabs(la(0)))/dt); 
	    s= s+ " (laTol= " + numtostr(laTol) + ")";
      }
    } 
    else if(nFric==2) {
      if(nrm2(la(1,2) + gdn(1,2)/nrm2(gdn(1,2))*((*mue_fun)(nrm2(gdn(1,2))))(0)*fabs(la(0))) <= laTol*dt);
      else if(nrm2(la(1,2)) <= ((*mue_fun)(nrm2(gdn(1,2))))(0)*fabs(la(0))+laTol*dt && nrm2(gdn(1,2)) <= gdTol);
      else {
		if (NormalDirectionFailed) s += "\n";
		s= s+" (2D tangential): no convergence gdT= " + numtostr(nrm2(gdn(1,2))) + " (gdTol= "+ numtostr(gdTol);
		s= s+ ")\n    stick: abs(laT) - mue abs(laN)  = " + numtostr(nrm2(la(1,2))/dt- ((*mue_fun)(nrm2(gdn(1,2))))(0)*fabs(la(0))/dt);
	    s= s+ " \n    slip: abs(laT -mue laN)         = " + numtostr(nrm2(la(1,2) + gdn(1,2)/nrm2(gdn(1,2))*((*mue_fun)(nrm2(gdn(1,2))))(0)*fabs(la(0)))/dt);
	    s= s+  "  (laTol= " + numtostr(laTol) + ")";
      }
    }
    return s;
  }
  
  void ContactRigid::updateFrictionCoefficient(double vel)
  {
  	if(mue_fun != 0) mue = ((*mue_fun)(vel))(0);
  }
}
