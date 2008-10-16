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
#include <config.h>
#define FMATVEC_NO_BOUNDS_CHECK
#include "rigid_contact.h"
#include "utils/nonsmooth_algebra.h"
#include "multi_body_system.h"
#include "function.h"
#include "class_factory.h"
#include "contact_kinematics.h"

namespace MBSim {

  double sign(double x) {
    if(x>0)
      return 1.0;
    else if(x<0)
      return -1.0;
    else 
      return 0;
    // return x>=0?1:-1;
  }

  RigidContact::RigidContact(const string &name) : Contact(name,true), argT(2), fcl(0), fdf(0), fnil(0), ftil(0) {
  }

  void RigidContact::init() {
    Contact::init();
  }

  void RigidContact::calcSize() {
    if(fdf)
      nFric = fdf->getFrictionDirections();
    else
      nFric = 0;
    Contact::calcSize();
  }

  void RigidContact::save(const string& path, ofstream &outputfile) {
    Contact::save(path, outputfile);

    fcl->save(path,outputfile);
    if(fdf)
      fdf->save(path,outputfile);
    else {
      outputfile << "# Type of friction law:" << endl << endl;
    }
    fnil->save(path,outputfile);
    if(ftil)
      ftil->save(path,outputfile);
    else {
      outputfile << "# Type of tangential impact law:" << endl << endl;
    }
  }

  void RigidContact::load(const string& path, ifstream &inputfile) {
    Contact::load(path,inputfile);
    string dummy;
    int s = inputfile.tellg();
    getline(inputfile,dummy); // # Type of contact law:
    getline(inputfile,dummy); // Type of contact law 
    inputfile.seekg(s,ios::beg);
    ClassFactory cf;
    setContactLaw(cf.getConstraintLaw(dummy));
    fcl->load(path, inputfile);

    s = inputfile.tellg();
    getline(inputfile,dummy); // # Type of friction law:
    getline(inputfile,dummy); // Type of friction law 
    inputfile.seekg(s,ios::beg);
    if(dummy.empty()) {
      getline(inputfile,dummy); // # Type of friction law
      getline(inputfile,dummy); // End of line
    } else {
      setFrictionLaw(cf.getFrictionLaw(dummy));
      fdf->load(path, inputfile);
    }

    s = inputfile.tellg();
    getline(inputfile,dummy); // # Type of normal impact law:
    getline(inputfile,dummy); // Type of normal impact law 
    inputfile.seekg(s,ios::beg);
    setNormalImpactLaw(cf.getNormalImpactLaw(dummy));
    fnil->load(path, inputfile);

    s = inputfile.tellg();
    getline(inputfile,dummy); // # Type of tangential impact law:
    getline(inputfile,dummy); // Type of tangential impact law 
    inputfile.seekg(s,ios::beg);
    if(dummy.empty()) {
      getline(inputfile,dummy); // # Type of friction law
      getline(inputfile,dummy); // End of line
    } else {
      setTangentialImpactLaw(cf.getTangentialImpactLaw(dummy));
      ftil->load(path, inputfile);
    }
  }

  void RigidContact::updateKinetics(double t) {
    fF[0].col(0) = getContourPointData(0).Wn;
    fF[0](Index(0,2),iT) = getContourPointData(0).Wt;
    fF[1] = -fF[0];
  }

  void RigidContact::updateb(double t) {
    Contact::updateb(t);
    contactKinematics->updateb(b,cpData);
  }

  void RigidContact::checkActive() {

    bool active_old = active;

    Contact::checkActive();

    if(active != active_old)
      parent->setActiveConstraintsChanged(true);
  }

  void RigidContact::projectGS(double dt) {
    double *a = parent->getGs()();
    int *ia = parent->getGs().Ip();
    int *ja = parent->getGs().Jp();
    int laInd = getlaIndMBS();
    Vec &laMBS = parent->getlaMBS();

    gdn(0) = s(0);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdn(0) += a[j]*laMBS(ja[j]);

    //la(0) = ((*fcl)(la, gdn, gd, rFactor))(0);
    //la(0) = (*fcl)(la(0), gdn(0), rFactor(0));
    la(0) = (*fnil)(la(0), gdn(0), gd(0), rFactor(0));

    for(int i=1; i<=nFric; i++) {
      gdn(i) = s(i);
      for(int j=ia[laInd+i]; j<ia[laInd+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);
    }

    if(ftil)
      la(1,nFric) = (*ftil)(la(1,nFric), gdn(1,nFric), gd(1,nFric), la(0), rFactor(1));
      //la(1,nFric) = (*fdf)(la(1,nFric), gdn(1,nFric), la(0), rFactor(1));
      //la(1,nFric) = (*fdf)(la, gdn, gd, rFactor);
      //la(1,nFric) = (*fdf)(la, gdn, rFactor(1));
  }

  void RigidContact::solveGS(double dt) {
    assert(nFric <= 1);

    double *a = parent->getGs()();
    int *ia = parent->getGs().Ip();
    int *ja = parent->getGs().Jp();
    int laInd = getlaIndMBS();
    Vec &laMBS = parent->getlaMBS();
    gdn(0) = s(0);
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      gdn(0) += a[j]*laMBS(ja[j]);

    double om = 1.0;
    //Vec buf = fcl->solve(parent->getG()(Index(laInd,laInd)), gdn, gd);
    //double buf = fcl->solve(a[ia[laInd]], gdn(0));
    double buf = fnil->solve(a[ia[laInd]], gdn(0), gd(0));
    la(0) += om*(buf - la(0));

    if(nFric) {
      gdn(1) = s(1);
      for(int j=ia[laInd+1]+1; j<ia[laInd+2]; j++)
	gdn(1) += a[j]*laMBS(ja[j]);

      if(ftil) {
	//Vec buf = fdf->solve(a[ia[laInd+1]], la(0), gdn(1));
	//Vec buf = fdf->solve(parent->getG()(Index(laInd,laInd+1)), la, gdn, gd);
	//Vec buf = fdf->solve(parent->getG()(Index(laInd+1,laInd+nFric)), gdn(1,nFric), la(0));
	Vec buf = ftil->solve(parent->getG()(Index(laInd+1,laInd+nFric)), gdn(1,nFric), gd(1,nFric), la(0));
	la(1,nFric) += om*(buf - la(1,nFric));
      }
    }
  }


  void RigidContact::residualProj(double dt) {
    double *a = parent->getGs()();
    int *ia = parent->getGs().Ip();
    int *ja = parent->getGs().Jp();
    int laInd = getlaIndMBS();
    Vec &laMBS = parent->getlaMBS();
    for(int i=0; i < 1+nFric; i++) {
      gdn(i) = s(i);
      for(int j=ia[laInd+i]; j<ia[laInd+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);
    }

    //res(0) = la(0) - ((*fcl)(la, gdn, gd, rFactor))(0);
    //res(0) = la(0) - (*fcl)(la(0), gdn(0), rFactor(0));
    res(0) = la(0) - (*fnil)(la(0), gdn(0), gd(0), rFactor(0));
    if(ftil) 
      //res(1,nFric) = la(1,nFric) - (*fdf)(la, gdn, rFactor(1));
      //res(1,nFric) = la(1,nFric) - (*fdf)(la, gdn, gd, rFactor);
      //res(1,nFric) = la(1,nFric) - (*fdf)(la(1,nFric), gdn(1,nFric), la(0), rFactor(1));
      res(1,nFric) = la(1,nFric) - (*ftil)(la(1,nFric), gdn(1,nFric), gd(1,nFric), la(0), rFactor(1));
  }


  void RigidContact::residualProjJac(double dt) {

    SqrMat Jprox = parent->getJprox();
    SymMat G = parent->getG();
    int laInd = getlaIndMBS();

    RowVec jp1=Jprox.row(laInd);
    RowVec e1(jp1.size());
    e1(laInd) = 1;
    //Vec diff = fcl->diff(la, gdn, gd, rFactor);
    //Vec diff = fcl->diff(la(0), gdn(0), rFactor(0));
    Vec diff = fnil->diff(la(0), gdn(0), gd(0), rFactor(0));

    jp1 = e1-diff(0)*e1; // -diff(1)*G.row(laInd)
    for(int i=0; i<G.size(); i++) 
      jp1(i) -= diff(1)*G(laInd,i);

    if(nFric == 1) {
      //Mat diff = fdf->diff(la, gdn, gd, rFactor);
      //Mat diff = fdf->diff(la(1,1), gdn(1,1), la(0), rFactor(1));
      Mat diff = ftil->diff(la(1,1), gdn(1,1), gd(1,1), la(0), rFactor(1));
      RowVec jp2=Jprox.row(laInd+1);
      RowVec e2(jp2.size());
      e2(laInd+1) = 1;
      Mat e(2,jp2.size());
      e(0,laInd) = 1;
      e(1,laInd+1) = 1;
      jp2 = e2-diff(0,2)*e1-diff(0,0)*e2; // -diff(1)*G.row(laInd)
      //jp2 = e2-diff.row(0)(0,1)*e; // -diff(1)*G.row(laInd)
      for(int i=0; i<G.size(); i++) 
	jp2(i) -= diff(0,1)*G(laInd+1,i);

    } else if(nFric == 2) {
      Mat diff = ftil->diff(la(1,2), gdn(1,2), gd(1,2), la(0), rFactor(1));
      //Mat diff = fdf->diff(la(1,2), gdn(1,2), la(0), rFactor(1));
      //Mat diff = fdf->diff(la, gdn, gd, rFactor);
      Mat jp2=Jprox(Index(laInd+1,laInd+2),Index(0,Jprox.cols()));
      Mat e2(2,jp2.cols());
      e2(0,laInd+1) = 1;
      e2(1,laInd+2) = 1;
      jp2 = e2-diff(Index(0,1),Index(4,4))*e1-diff(Index(0,1),Index(0,1))*e2; // -diff(Index(0,1),Index(4,5))*G(Index(laInd+1,laInd+2),Index(0,G.size()-1))
      for(int i=0; i<G.size(); i++) {
	jp2(0,i) = diff(0,2)*G(laInd+1,i)+diff(0,3)*G(laInd+2,i);
	jp2(1,i) = diff(1,2)*G(laInd+1,i)+diff(1,3)*G(laInd+2,i);
      }

//      RowVec jp2=Jprox.row(laInd+1);
//      RowVec jp3=Jprox.row(laInd+2);
//      double LaT = pow(argT(0),2)+pow(argT(1),2);
//      double fabsLaT = sqrt(LaT);
//      double laNmu0 = fabs(la(0))*mu0;
//      double rFac1 = rFactor(1);
//      if(fabsLaT <=  laNmu0) {
//	for(int i=0; i<G.size(); i++) {
//	  jp2(i) = rFac1*G(laInd+1,i);
//	  jp3(i) = rFac1*G(laInd+2,i);
//	}
//      } else {
//	SymMat dfda(2,NONINIT);
//	dfda(0,0) = 1-argT(0)*argT(0)/LaT;
//	dfda(1,1) = 1-argT(1)*argT(1)/LaT;
//	dfda(0,1) = -argT(0)*argT(1)/LaT;
//
//	for(int i=0; i<G.size(); i++) {
//	  double e1 = (i==laInd) ? sign(la(0))*mu0 : 0;
//	  double e2 = (i==laInd+1?1.0:0.0);
//	  double e3 = (i==laInd+2?1.0:0.0);
//	  jp2(i) = e2 - ((dfda(0,0)*(e2 - rFac1*G(laInd+1,i)) + dfda(0,1)*(e3 - rFac1*G(laInd+2,i)))*laNmu0 + e1*argT(0))/fabsLaT;
//	  jp3(i) = e3 - ((dfda(1,0)*(e2 - rFac1*G(laInd+1,i)) + dfda(1,1)*(e3 - rFac1*G(laInd+2,i)))*laNmu0 + e1*argT(1))/fabsLaT;
//	}
//    }
    }
  }

  void RigidContact::checkForTermination(double dt) {

    double *a = parent->getGs()();
    int *ia = parent->getGs().Ip();
    int *ja = parent->getGs().Jp();
    int laInd = getlaIndMBS();
    Vec &laMBS = parent->getlaMBS();
    for(int i=0; i < 1+nFric; i++) {
      gdn(i) = s(i);
      for(int j=ia[laInd+i]; j<ia[laInd+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);
    }

    //if(!fcl->isFullfield(la(0),gdn(0),laTol*dt,gdTol)) {
    //if(!fcl->isFullfield(la,gdn,gd,laTol*dt,gdTol)) {
    if(!fnil->isFullfield(la(0),gdn(0),gd(0),laTol*dt,gdTol)) {
      parent->setTermination(false);
      return;
    }
    if(ftil) 
      //if(!fdf->isFullfield(la,gdn,gd,laTol*dt,gdTol)) {
      //if(!fdf->isFullfield(la(1,nFric),gdn(1,nFric),la(0),laTol*dt,gdTol)) {
      if(!ftil->isFullfield(la(1,nFric),gdn(1,nFric),gd(1,nFric),la(0),laTol*dt,gdTol)) {
	parent->setTermination(false);
	return;
      }
  }

  void RigidContact::updaterFactors() {
    double *a = parent->getGs()();
    int *ia = parent->getGs().Ip();
    double sumN = 0;
    int laInd = getlaIndMBS();

    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      sumN += fabs(a[j]);
    double aN = a[ia[laInd]];
    if(aN > sumN) {
      rFactorUnsure(0) = 0;
      rFactor(0) = 1.0/aN;
    } else {
      rFactorUnsure(0) = 1;
      rFactor(0) = rMax/aN;
    }
    double sumT1 = 0;
    double sumT2 = 0;
    double aT1, aT2;
    if(nFric == 1) {
      for(int j=ia[laInd+1]+1; j<ia[laInd+2]; j++)
	sumT1 += fabs(a[j]);
      aT1 = a[ia[laInd+1]];
      if(aT1 > sumT1) {
	rFactorUnsure(1)=0;
	rFactor(1) = 1.0/aT1;
      } else {
	rFactorUnsure(1)=1;
	rFactor(1) = rMax/aT1;
      }
    } else if(nFric == 2) {
      for(int j=ia[laInd+1]+1; j<ia[laInd+2]; j++)
	sumT1 += fabs(a[j]);
      for(int j=ia[laInd+2]+1; j<ia[laInd+3]; j++)
	sumT2 += fabs(a[j]);
      aT1 = a[ia[laInd+1]];
      aT2 = a[ia[laInd+2]];

      // TODO rFactorUnsure
      if(aT1 - sumT1 >= aT2 - sumT2) 
	if(aT1 + sumT1 >= aT2 + sumT2) 
	  rFactor(1) = 2.0/(aT1+aT2+sumT1-sumT2);
	else 
	  rFactor(1) = 1.0/aT2;
      else 
	if(aT1 + sumT1 < aT2 + sumT2) 
	  rFactor(1) = 2.0/(aT1+aT2-sumT1+sumT2);
	else 
	  rFactor(1) = 1.0/aT1;
    }
  }

  std::string RigidContact::getTerminationInfo(double dt){
    std::string s= "RigidContact " + getName();
 //   bool NormalDirectionFailed= false;

 //   if(gdn(0) >= -gdTol && fabs(la(0)) <= laTol*dt)
 //     ;
 //   else if(la(0) >= -laTol*dt && fabs(gdn(0)) <= gdTol)
 //     ;
 //   else {
 //     s= s+" (normal): no convergence  gdn= " + numtostr(gdn(0)) + " (gdTol= " + numtostr(gdTol);
 //     s= s+ ")     la(0)= " + numtostr(la(0)/dt) + " (laTol= " + numtostr(laTol) + ")";
 //     NormalDirectionFailed= true;
 //   }

 //   if(nFric==1) {
 //     if(fabs(la(1) + gdn(1)/fabs(gdn(1))*mu0*fabs(la(0))) <= laTol*dt)  // Gleiten
 //       ; 
 //     else if(fabs(la(1)) <= mu0*fabs(la(0)) + laTol*dt && fabs(gdn(1)) <= gdTol)  // Haften
 //       ;
 //     else {
 //       if (NormalDirectionFailed) s += "\n";
 //       s= s+" (1D tangential): no convergence gdT= " + numtostr(gdn(1)) + " (gdTol= "+ numtostr(gdTol);
 //       s= s+ ")\n    stick: abs(laT) - mu0 abs(laN) = " + numtostr(fabs(la(1)/dt)- mu0*fabs(la(0)/dt));
 //       s= s+ "\n     slip: abs(laT - mu0 laN)       = " + numtostr(fabs(la(1) + gdn(1)/fabs(gdn(1))*mu0*fabs(la(0)))/dt); 
 //       s= s+ " (laTol= " + numtostr(laTol) + ")";
 //     }
 //   } else if(nFric==2) {
 //     if(nrm2(la(1,2) + gdn(1,2)/nrm2(gdn(1,2))*mu0*fabs(la(0))) <= laTol*dt)
 //       ;
 //     else if(nrm2(la(1,2)) <= mu0*fabs(la(0))+laTol*dt && nrm2(gdn(1,2)) <= gdTol)
 //       ;
 //     else {
 //       if (NormalDirectionFailed) s += "\n";
 //       s= s+" (2D tangential): no convergence gdT= " + numtostr(nrm2(gdn(1,2))) + " (gdTol= "+ numtostr(gdTol);
 //       s= s+ ")\n    stick: abs(laT) - mu0 abs(laN)  = " + numtostr(nrm2(la(1,2))/dt- mu0*fabs(la(0))/dt);
 //       s= s+ " \n    slip: abs(laT -mu0 laN)         = " + numtostr(nrm2(la(1,2) + gdn(1,2)/nrm2(gdn(1,2))*mu0*fabs(la(0)))/dt);
 //       s= s+  "  (laTol= " + numtostr(laTol) + ")";
 //     }
 //   }
    return s;
  }

}

