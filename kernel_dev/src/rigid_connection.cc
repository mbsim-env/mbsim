/* Copyright (C) 2004-2008  Martin Förg
 
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

#define FMATVEC_NO_BOUNDS_CHECK
#include <config.h>
#include "rigid_connection.h"
#include "coordinate_system.h"
#include "multi_body_system.h"
#include "function.h"

namespace MBSim {

  RigidConnection::RigidConnection(const string &name) : Connection(name,true), ffl(0), fml(0), fifl(0), fiml(0) {
  }

  void RigidConnection::init() {

    gdd.resize(gdSize);
    gdn.resize(gdSize);

    Connection::init();

    fF[0](Index(0,2),Index(0,Wf.cols()-1)) = -Wf;
    fF[1](Index(0,2),Index(0,Wf.cols()-1)) = Wf;
    fM[0](Index(0,2),Index(Wf.cols(),Wf.cols()+Wm.cols()-1)) = -Wm;
    fM[1](Index(0,2),Index(Wf.cols(),Wf.cols()+Wm.cols()-1)) = Wm;
  }

  void RigidConnection::updateW(double t) {
    fF[0](Index(0,2),Index(0,Wf.cols()-1)) = -Wf;
    fM[0](Index(0,2),Index(Wf.cols(),Wf.cols()+Wm.cols()-1)) = -Wm;
    fM[0](Index(0,2),Index(0,Wf.cols()-1)) = -tilde(WrP0P1)*Wf;
    fF[1](Index(0,2),Index(0,Wf.cols()-1)) = Wf;
    fM[1](Index(0,2),Index(Wf.cols(),Wf.cols()+Wm.cols()-1)) = Wm;

    for(unsigned int i=0; i<port.size(); i++) 
      W[i] += trans(port[i]->getJacobianOfTranslation())*fF[i] + trans(port[i]->getJacobianOfRotation())*fM[i];
  }

  void RigidConnection::updatewb(double t) {

    for(unsigned i=0; i<port.size(); i++) 
      wb += trans(fF[i])*port[i]->getGyroscopicAccelerationOfTranslation() + trans(fM[i])*port[i]->getGyroscopicAccelerationOfRotation();

    wb(0,Wf.cols()-1) += trans(Wf)*(crossProduct(port[0]->getAngularVelocity(),crossProduct(port[0]->getAngularVelocity(),WrP0P1)) - 2*crossProduct(port[0]->getAngularVelocity(),WvP0P1));
    wb(Wf.cols(),Wm.cols()+Wf.cols()-1) -=  trans(Wm)*crossProduct(port[0]->getAngularVelocity(),WomP0P1);
  }

  void RigidConnection::solveConstraintsFixpointSingle() {
    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();
  
    for(int i=0; i<forceDir.cols(); i++) {
      gdd(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdd(i) += a[j]*laMBS(ja[j]);

      la(i) = (*ffl)(la(i), gdd(i), rFactor(i));
    }
    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {
      gdd(i) = b(i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdd(i) += a[j]*laMBS(ja[j]);

      la(i) = (*fml)(la(i), gdd(i), rFactor(i));
    }
  }

  void RigidConnection::solveImpactsFixpointSingle() {
    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();
  
    for(int i=0; i<forceDir.cols(); i++) {
      gdn(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);

      la(i) = (*fifl)(la(i), gdn(i), gd(i), rFactor(i));
    }
    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = b(i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);

      la(i) = (*fiml)(la(i), gdn(i), gd(i), rFactor(i));
    }
  }

  void RigidConnection::solveConstraintsGaussSeidel() {
    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();
  
    for(int i=0; i<forceDir.cols(); i++) {
      gdd(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]+1; j<ia[laIndMBS+1+i]; j++)
	gdd(i) += a[j]*laMBS(ja[j]);

      la(i) = ffl->solve(a[ia[laIndMBS+i]], gdd(i));
    }
    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {
      gdd(i) = b(i);
      for(int j=ia[laIndMBS+i]+1; j<ia[laIndMBS+1+i]; j++)
	gdd(i) += a[j]*laMBS(ja[j]);

      la(i) = fml->solve(a[ia[laIndMBS+i]], gdd(i));
    }
  }

  void RigidConnection::solveImpactsGaussSeidel() {
    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();
  
    for(int i=0; i<forceDir.cols(); i++) {
      gdn(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]+1; j<ia[laIndMBS+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);

      la(i) = fifl->solve(a[ia[laIndMBS+i]], gdn(i), gd(i));
    }
    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = b(i);
      for(int j=ia[laIndMBS+i]+1; j<ia[laIndMBS+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);

      la(i) = fiml->solve(a[ia[laIndMBS+i]], gdn(i), gd(i));
    }
  }

  void RigidConnection::solveConstraintsRootFinding() {
    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();
  
    for(int i=0; i<forceDir.cols(); i++) {
      gdd(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdd(i) += a[j]*laMBS(ja[j]);

      res(i) = la(i) - (*ffl)(la(i), gdd(i), rFactor(i));
    }
    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {
      gdd(i) = b(i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdd(i) += a[j]*laMBS(ja[j]);

      res(i) = la(i) - (*fml)(la(i), gdd(i), rFactor(i));
    }
  }
  void RigidConnection::solveImpactsRootFinding() {
    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();
  
    for(int i=0; i<forceDir.cols(); i++) {
      gdn(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);

      res(i) = la(i) - (*fifl)(la(i), gdn(i), gd(i), rFactor(i));
    }
    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = b(i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);

      res(i) = la(i) - (*fiml)(la(i), gdn(i), gd(i), rFactor(i));
    }
  }

  void RigidConnection::jacobianConstraints() {
    SqrMat Jprox = mbs->getJprox();
    SqrMat G = mbs->getG();

    for(int i=0; i<forceDir.cols(); i++) {
      RowVec jp1=Jprox.row(laIndMBS+i);
      RowVec e1(jp1.size());
      e1(laIndMBS+i) = 1;
      Vec diff = ffl->diff(la(i), gdd(i), rFactor(i));

      jp1 = e1-diff(0)*e1; // -diff(1)*G.row(laIndMBS+i)
      for(int j=0; j<G.size(); j++) 
	jp1(j) -= diff(1)*G(laIndMBS+i,j);
    }

    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {

      RowVec jp1=Jprox.row(laIndMBS+i);
      RowVec e1(jp1.size());
      e1(laIndMBS+i) = 1;
      Vec diff = fml->diff(la(i), gdd(i), rFactor(i));

      jp1 = e1-diff(0)*e1; // -diff(1)*G.row(laIndMBS+i)
      for(int j=0; j<G.size(); j++) 
	jp1(j) -= diff(1)*G(laIndMBS+i,j);
    }
  }

  void RigidConnection::jacobianImpacts() {
    SqrMat Jprox = mbs->getJprox();
    SqrMat G = mbs->getG();

    for(int i=0; i<forceDir.cols(); i++) {
      RowVec jp1=Jprox.row(laIndMBS+i);
      RowVec e1(jp1.size());
      e1(laIndMBS+i) = 1;
      Vec diff = fifl->diff(la(i), gdn(i), gd(i), rFactor(i));

      jp1 = e1-diff(0)*e1; // -diff(1)*G.row(laIndMBS+i)
      for(int j=0; j<G.size(); j++) 
	jp1(j) -= diff(1)*G(laIndMBS+i,j);
    }

    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {

      RowVec jp1=Jprox.row(laIndMBS+i);
      RowVec e1(jp1.size());
      e1(laIndMBS+i) = 1;
      Vec diff = fiml->diff(la(i), gdn(i), gd(i), rFactor(i));

      jp1 = e1-diff(0)*e1; // -diff(1)*G.row(laIndMBS+i)
      for(int j=0; j<G.size(); j++) 
	jp1(j) -= diff(1)*G(laIndMBS+i,j);
    }
  }

  void RigidConnection::updaterFactors() {
    if(isActive()) {
      double *a = mbs->getGs()();
      int *ia = mbs->getGs().Ip();

      for(int i=0; i<rFactorSize; i++) {
	double sum = 0;
	for(int j=ia[laIndMBS+i]+1; j<ia[laIndMBS+i+1]; j++)
	  sum += fabs(a[j]);
	double ai = a[ia[laIndMBS+i]];
	if(ai > sum) {
	  rFactorUnsure(i) = 0;
	  rFactor(i) = 1.0/ai;
	} else {
	  rFactorUnsure(i) = 1;
	  rFactor(i) = 1.0/ai;
	}
      }
    }
  }

  void RigidConnection::checkConstraintsForTermination() {

    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();

    for(int i=0; i < forceDir.cols(); i++) {
      gdd(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdd(i) += a[j]*laMBS(ja[j]);

      if(!ffl->isFullfield(la(i),gdd(i),laTol,gddTol)) {
	mbs->setTermination(false);
	return;
      }
    }
    for(int i=forceDir.cols(); i < forceDir.cols() + momentDir.cols(); i++) {
      gdd(i) = b(i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdd(i) += a[j]*laMBS(ja[j]);

      if(!fml->isFullfield(la(i),gdd(i),laTol,gddTol)) {
	mbs->setTermination(false);
	return;
      }
    }
  }

  void RigidConnection::checkImpactsForTermination() {

    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();

    for(int i=0; i < forceDir.cols(); i++) {
      gdn(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);

      if(!fifl->isFullfield(la(i),gdn(i),gd(i),LaTol,gdTol)) {
	mbs->setTermination(false);
	return;
      }
    }
    for(int i=forceDir.cols(); i < forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = b(i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);

      if(!fiml->isFullfield(la(i),gdn(i),gd(i),LaTol,gdTol)) {
	mbs->setTermination(false);
	return;
      }
    }
  }

  std::string RigidConnection::getTerminationInfo(double dt) {
    std::string s;
    int j=-1;
    s= Link::getTerminationInfo(dt);
    for (int i=0; i<gdn.size(); i++) {
      if (fabs(gdn(i)) > gdTol) j=i;
    }
    if (j>0) s = s + " gdn(" + numtostr(j) + ")= " + numtostr(gdn(j));
      s= s + " (gdTol= " + numtostr(gdTol) + " )";
    return s;
  }

}
