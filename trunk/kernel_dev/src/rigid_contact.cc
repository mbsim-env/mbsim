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
    gdd.resize(gdSize);
    Contact::init();
    // TODO: schöner lösen
    if(getFrictionDirections() == 0)
      gdActive[1] = false;
  }

  void RigidContact::calcsvSize() {
    Contact::calcsvSize();
    svSize = 1+getFrictionDirections();
  }

  void RigidContact::checkActiveg() { 
    gActive = fcl->isClosed(g(0),0) ? 1 : 0; 
  }

  void RigidContact::checkActivegd() { 
    gdActive[0] = gActive ? (fcl->remainsClosed(gd(0),gdTol) ? 1 : 0) : 0; 
    gdActive[1] = getFrictionDirections() && gdActive[0] ? (fdf->isSticking(gd(1,getFrictionDirections()),gdTol) ? 1 : 0) : 0; 
  }

  void RigidContact::checkAllgd() { 
    gdActive[0] = gActive ? 1 : 0; 
    gdActive[1] = getFrictionDirections() && gActive ? 1 : 0; 
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

  void RigidContact::updatewb(double t) {
    for(unsigned i=0; i<contour.size(); i++) {
      contour[i]->updateMovingFrame(t, cpData[i]);
      wb += trans(fF[i](Index(0,2),Index(0,laSize-1)))*contour[i]->getMovingFrame()->getGyroscopicAccelerationOfTranslation();
    }
    contactKinematics->updatewb(wb,g,cpData);
  }

  void RigidContact::updateW(double t) {
    //fF[0].resize(3,laSize,NONINIT);
    fF[0].col(0) = getContourPointData(0).Wn;
    fF[0](Index(0,2),iT) = getContourPointData(0).Wt;

    fF[1] = -fF[0];

    for(unsigned int i=0; i<contour.size(); i++) {
      contour[i]->updateMovingFrame(t, cpData[i]);
      W[i] += trans(contour[i]->getMovingFrame()->getJacobianOfTranslation())*fF[i](Index(0,2),Index(0,laSize-1));
    }
  }

  void RigidContact::updateV(double t) {
    if(getFrictionDirections() && !gdActive[1]) { 
      for(unsigned int i=0; i<contour.size(); i++) {
	V[i] += trans(contour[i]->getMovingFrame()->getJacobianOfTranslation())*fF[i](Index(0,2),iT)*fdf->dlaTdlaN(gd(1,getFrictionDirections()), la(0));
      }
    }
  }

  void RigidContact::checkActivegdn() { 
   // cout << name << endl;
   // cout <<"gdn = "<< gdn << endl;
   // cout <<"la = "<< la << endl;
    if(gActive) {
      if(gdn(0) <= gdTol) {
	gActive = true;
	gdActive[0] = true;
      } 
      else {
	gActive = false;
	gdActive[0] = false;
      }
    }
  //  else { // Ist wahrscheinlich unnötig
  //    gActive = false;
  //    gdActive[0] = false;
  //  }
    if(getFrictionDirections())
      if(gdActive[0])
	if(nrm2(gdn(1,getFrictionDirections())) <= gdTol)
	  gdActive[1] = true;
	else
	  gdActive[1] = false;
      else
	  gdActive[1] = false;
  }

  void RigidContact::checkActivegdd() { 
    //cout << name << endl;
    //cout <<"gdd = "<< gdd << endl;
    //cout <<"la = "<< la << endl;
    if(gdActive[0]) {
      if(gdd(0) <= gddTol) {
	gActive = true;
	gdActive[0] = true;
      }
      else {
	gActive = false;
	gdActive[0] = false;
      }
    }
    if(getFrictionDirections())
      if(gdActive[0]) 
	if(gdActive[1]) 
	  if(nrm2(gdd(1,getFrictionDirections())) <= gddTol)
	    gdActive[1] = true;
	  else
	    gdActive[1] = false;
	else 
	  gdActive[1] = false;
  }

  void RigidContact::updateCondition() {

    //cout<<"before " << "gA = " << gActive << " gd0 = " << gdActive[0] << " gd1 = " << gdActive[1] << endl;
    if(jsv(0)) {
      if(gActive) {
	gActive = false;
	gdActive[0] = false;
	if(getFrictionDirections())
	  gdActive[1] = false;
	return;
      }
      else {// if(gd(0)<=0) { // evtl. zur Abfrage zur Vermeidung von Schein-Kollisionen wegen Eindringen
	gActive = true;
	gdActive[0] = true;
	if(getFrictionDirections())
	  gdActive[1] = true;
	mbs->setImpact(true);
	return;
      }
    }
    if(getFrictionDirections())
      if(jsv(1)) {
	if(gdActive[1]) {
	  gdActive[1] = false;
	} 
	else {
	  gdActive[1] = true;
	  mbs->setSticking(true);
	}
      }
  //  cout<<"after " << "gA = " << gActive << " gd0 = " << gdActive[0] << " gd1 = " << gdActive[1] << endl;
  }

  void RigidContact::updateStopVector(double t) {
    if(gActive) {
      sv(0) = la(0);
      if(gdActive[1]) {
	sv(1) = nrm2(la(1,getFrictionDirections())) - fdf->getFrictionCoefficient(nrm2(gd(1,getFrictionDirections())))*la(0);
      } 
      else
	sv(1,getFrictionDirections()) = gd(1,getFrictionDirections());
    }
    else {
      sv(0) = g(0);
      sv(1,getFrictionDirections()).init(1);
    }
  }

  void RigidContact::solveContactFixpointSingle() {

    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();

    gdd(0) = b(laIndMBS);
    for(int j=ia[laIndMBS]; j<ia[laIndMBS+1]; j++)
      gdd(0) += a[j]*laMBS(ja[j]);

    la(0) = (*fcl)(la(0), gdd(0), rFactor(0));

    for(int i=1; i<=getFrictionDirections(); i++) {
      gdd(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdd(i) += a[j]*laMBS(ja[j]);
    }

    if(fdf)
      la(1,getFrictionDirections()) = (*fdf)(la(1,getFrictionDirections()), gdd(1,getFrictionDirections()), la(0), rFactor(1));
  } 
  
  void RigidContact::solveImpactFixpointSingle() {

    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();

    gdn(0) = b(laIndMBS);
    for(int j=ia[laIndMBS]; j<ia[laIndMBS+1]; j++)
      gdn(0) += a[j]*laMBS(ja[j]);

    la(0) = (*fnil)(la(0), gdn(0), gd(0), rFactor(0));

    for(int i=1; i<=getFrictionDirections(); i++) {
      gdn(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);
    }

    if(ftil)
      la(1,getFrictionDirections()) = (*ftil)(la(1,getFrictionDirections()), gdn(1,getFrictionDirections()), gd(1,getFrictionDirections()), la(0), rFactor(1));
  }


  void RigidContact::solveContactGaussSeidel() {
    assert(getFrictionDirections() <= 1);

    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    int laInd = laIndMBS;
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();

    gdd(0) = b(laIndMBS);
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      gdd(0) += a[j]*laMBS(ja[j]);

    double om = 1.0;
    double buf = fcl->solve(a[ia[laInd]], gdd(0));
    la(0) += om*(buf - la(0));

    if(getFrictionDirections() && gdActive[1]) {
      gdd(1) = b(laIndMBS+1);
      for(int j=ia[laInd+1]+1; j<ia[laInd+2]; j++)
	gdd(1) += a[j]*laMBS(ja[j]);

      if(fdf) {
	Vec buf = fdf->solve(mbs->getG()(Index(laInd+1,laInd+getFrictionDirections())), gdd(1,getFrictionDirections()), la(0));
	la(1,getFrictionDirections()) += om*(buf - la(1,getFrictionDirections()));
      }
    }
  }

  void RigidContact::solveImpactGaussSeidel() {
    assert(getFrictionDirections() <= 1);

    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    int laInd = laIndMBS;
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();

    gdn(0) = b(laIndMBS);
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      gdn(0) += a[j]*laMBS(ja[j]);

    double om = 1.0;
    double buf = fnil->solve(a[ia[laInd]], gdn(0), gd(0));
    la(0) += om*(buf - la(0));

    if(getFrictionDirections()) {
      gdn(1) = b(laIndMBS+1);
      for(int j=ia[laInd+1]+1; j<ia[laInd+2]; j++)
	gdn(1) += a[j]*laMBS(ja[j]);

      if(ftil) {
	Vec buf = ftil->solve(mbs->getG()(Index(laInd+1,laInd+getFrictionDirections())), gdn(1,getFrictionDirections()), gd(1,getFrictionDirections()), la(0));
	la(1,getFrictionDirections()) += om*(buf - la(1,getFrictionDirections()));
      }
    }
  }

  void RigidContact::solveContactRootFinding() {
    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();

    for(int i=0; i < 1+getFrictionDirections(); i++) {
      gdd(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdd(i) += a[j]*laMBS(ja[j]);
    }

    res(0) = la(0) - (*fcl)(la(0), gdd(0), rFactor(0));
    if(fdf) 
      res(1,getFrictionDirections()) = la(1,getFrictionDirections()) - (*fdf)(la(1,getFrictionDirections()), gdd(1,getFrictionDirections()), la(0), rFactor(1));
  }

  void RigidContact::solveImpactRootFinding() {
    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();

    for(int i=0; i < 1+getFrictionDirections(); i++) {
      gdn(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);
    }

    res(0) = la(0) - (*fnil)(la(0), gdn(0), gd(0), rFactor(0));
    if(ftil) 
      res(1,getFrictionDirections()) = la(1,getFrictionDirections()) - (*ftil)(la(1,getFrictionDirections()), gdn(1,getFrictionDirections()), gd(1,getFrictionDirections()), la(0), rFactor(1));
  }

  void RigidContact::jacobianContact() {

    SqrMat Jprox = mbs->getJprox();
    SqrMat G = mbs->getG();

    RowVec jp1=Jprox.row(laIndMBS);
    RowVec e1(jp1.size());
    e1(laIndMBS) = 1;
    Vec diff = fcl->diff(la(0), gdd(0), rFactor(0));

    jp1 = e1-diff(0)*e1; // -diff(1)*G.row(laIndMBS)
    for(int i=0; i<G.size(); i++) 
      jp1(i) -= diff(1)*G(laIndMBS,i);

    if(getFrictionDirections() == 1) {
      Mat diff = fdf->diff(la(1,1), gdd(1,1), la(0), rFactor(1));
      RowVec jp2=Jprox.row(laIndMBS+1);
      RowVec e2(jp2.size());
      e2(laIndMBS+1) = 1;
      Mat e(2,jp2.size());
      e(0,laIndMBS) = 1;
      e(1,laIndMBS+1) = 1;
      jp2 = e2-diff(0,2)*e1-diff(0,0)*e2; // -diff(1)*G.row(laIndMBS)
      //jp2 = e2-diff.row(0)(0,1)*e; // -diff(1)*G.row(laIndMBS)
      for(int i=0; i<G.size(); i++) 
	jp2(i) -= diff(0,1)*G(laIndMBS+1,i);

    } else if(getFrictionDirections() == 2) {
      Mat diff = ftil->diff(la(1,2), gdd(1,2), gd(1,2), la(0), rFactor(1));
      Mat jp2=Jprox(Index(laIndMBS+1,laIndMBS+2),Index(0,Jprox.cols()));
      Mat e2(2,jp2.cols());
      e2(0,laIndMBS+1) = 1;
      e2(1,laIndMBS+2) = 1;
      jp2 = e2-diff(Index(0,1),Index(4,4))*e1-diff(Index(0,1),Index(0,1))*e2; // -diff(Index(0,1),Index(4,5))*G(Index(laIndMBS+1,laIndMBS+2),Index(0,G.size()-1))
      for(int i=0; i<G.size(); i++) {
	jp2(0,i) = diff(0,2)*G(laIndMBS+1,i)+diff(0,3)*G(laIndMBS+2,i);
	jp2(1,i) = diff(1,2)*G(laIndMBS+1,i)+diff(1,3)*G(laIndMBS+2,i);
      }
    }
  }

  void RigidContact::jacobianImpact() {

    SqrMat Jprox = mbs->getJprox();
    SqrMat G = mbs->getG();

    RowVec jp1=Jprox.row(laIndMBS);
    RowVec e1(jp1.size());
    e1(laIndMBS) = 1;
    Vec diff = fnil->diff(la(0), gdn(0), gd(0), rFactor(0));

    jp1 = e1-diff(0)*e1; // -diff(1)*G.row(laIndMBS)
    for(int i=0; i<G.size(); i++) 
      jp1(i) -= diff(1)*G(laIndMBS,i);

    if(getFrictionDirections() == 1) {
      Mat diff = ftil->diff(la(1,1), gdn(1,1), gd(1,1), la(0), rFactor(1));
      RowVec jp2=Jprox.row(laIndMBS+1);
      RowVec e2(jp2.size());
      e2(laIndMBS+1) = 1;
      Mat e(2,jp2.size());
      e(0,laIndMBS) = 1;
      e(1,laIndMBS+1) = 1;
      jp2 = e2-diff(0,2)*e1-diff(0,0)*e2; // -diff(1)*G.row(laIndMBS)
      //jp2 = e2-diff.row(0)(0,1)*e; // -diff(1)*G.row(laIndMBS)
      for(int i=0; i<G.size(); i++) 
	jp2(i) -= diff(0,1)*G(laIndMBS+1,i);

    } else if(getFrictionDirections() == 2) {
      Mat diff = ftil->diff(la(1,2), gdn(1,2), gd(1,2), la(0), rFactor(1));
      Mat jp2=Jprox(Index(laIndMBS+1,laIndMBS+2),Index(0,Jprox.cols()));
      Mat e2(2,jp2.cols());
      e2(0,laIndMBS+1) = 1;
      e2(1,laIndMBS+2) = 1;
      jp2 = e2-diff(Index(0,1),Index(4,4))*e1-diff(Index(0,1),Index(0,1))*e2; // -diff(Index(0,1),Index(4,5))*G(Index(laIndMBS+1,laIndMBS+2),Index(0,G.size()-1))
      for(int i=0; i<G.size(); i++) {
	jp2(0,i) = diff(0,2)*G(laIndMBS+1,i)+diff(0,3)*G(laIndMBS+2,i);
	jp2(1,i) = diff(1,2)*G(laIndMBS+1,i)+diff(1,3)*G(laIndMBS+2,i);
      }
    }
  }

  void RigidContact::checkContactForTermination() {

    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();

    for(unsigned int i=0; i < 1+ gdActive[1]*getFrictionDirections(); i++) {
      gdd(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdd(i) += a[j]*laMBS(ja[j]);
    }

    if(!fcl->isFullfield(la(0),gdd(0),laTol,gddTol)) {
      mbs->setTermination(false);
      return;
    }
    if(fdf && gdActive[1]) 
      if(!fdf->isFullfield(la(1,getFrictionDirections()),gdd(1,getFrictionDirections()),la(0),laTol,gddTol)) {
	mbs->setTermination(false);
	return;
      }
  }

  void RigidContact::checkImpactForTermination() {

    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();

    for(int i=0; i < 1+getFrictionDirections(); i++) {
      gdn(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);
    }

    if(!fnil->isFullfield(la(0),gdn(0),gd(0),LaTol,gdTol)) {
      mbs->setTermination(false);
      return;
    }
    if(ftil) 
      if(!ftil->isFullfield(la(1,getFrictionDirections()),gdn(1,getFrictionDirections()),gd(1,getFrictionDirections()),la(0),LaTol,gdTol)) {
	mbs->setTermination(false);
	return;
      }
  }

  void RigidContact::updaterFactors() {
    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    double sumN = 0;

    for(int j=ia[laIndMBS]+1; j<ia[laIndMBS+1]; j++)
      sumN += fabs(a[j]);
    double aN = a[ia[laIndMBS]];
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
    if(getFrictionDirections() == 1) {
      for(int j=ia[laIndMBS+1]+1; j<ia[laIndMBS+2]; j++)
	sumT1 += fabs(a[j]);
      aT1 = a[ia[laIndMBS+1]];
      if(aT1 > sumT1) {
	rFactorUnsure(1)=0;
	rFactor(1) = 1.0/aT1;
      } else {
	rFactorUnsure(1)=1;
	rFactor(1) = rMax/aT1;
      }
    } else if(getFrictionDirections() == 2) {
      for(int j=ia[laIndMBS+1]+1; j<ia[laIndMBS+2]; j++)
	sumT1 += fabs(a[j]);
      for(int j=ia[laIndMBS+2]+1; j<ia[laIndMBS+3]; j++)
	sumT2 += fabs(a[j]);
      aT1 = a[ia[laIndMBS+1]];
      aT2 = a[ia[laIndMBS+2]];

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

    //   if(getFrictionDirections()==1) {
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
    //   } else if(getFrictionDirections()==2) {
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

