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

  RigidConnection::RigidConnection(const string &name) : Connection(name,true) {
  }

  void RigidConnection::init() {
    Connection::init();

    for(int i=0; i<2; i++) {
      loadDir.push_back(Mat(6,laSize));
      fF[i] >> loadDir[i](Index(0,2),Index(0,laSize-1));
      fM[i] >> loadDir[i](Index(3,5),Index(0,laSize-1));
    }

    fF[0](Index(0,2),Index(0,Wf.cols()-1)) = -Wf;
    fF[1](Index(0,2),Index(0,Wf.cols()-1)) = Wf;
    fM[0](Index(0,2),Index(Wf.cols(),Wf.cols()+Wm.cols()-1)) = -Wm;
    fM[1](Index(0,2),Index(Wf.cols(),Wf.cols()+Wm.cols()-1)) = Wm;
  }

  void RigidConnection::updateKinetics(double dt) {
    fF[0](Index(0,2),Index(0,Wf.cols()-1)) = -Wf;
    fF[1](Index(0,2),Index(0,Wf.cols()-1)) = Wf;
    fM[0](Index(0,2),Index(Wf.cols(),Wf.cols()+Wm.cols()-1)) = -Wm;
    fM[1](Index(0,2),Index(Wf.cols(),Wf.cols()+Wm.cols()-1)) = Wm;
  }

  void RigidConnection::updateW(double t) {
    W[0] += trans(port[0]->getJacobianOfTranslation())*fF[0] + trans(port[0]->getJacobianOfRotation())*(fM[0]+tilde(WrP0P1)*fF[0]);
    W[1] += trans(port[1]->getJacobianOfTranslation())*fF[1] + trans(port[1]->getJacobianOfRotation())*fM[1];
  }

  void RigidConnection::updateb(double t) {
    b(0,Wf.cols()-1) += trans(Wf)*(port[1]->getGyroscopicAccelerationOfTranslation() - port[0]->getGyroscopicAccelerationOfTranslation() + crossProduct(WrP0P1,port[0]->getGyroscopicAccelerationOfRotation()) + crossProduct(WrP0P1,port[0]->getGyroscopicAccelerationOfRotation()) + crossProduct(port[0]->getAngularVelocity(),crossProduct(port[0]->getAngularVelocity(),WrP0P1)) - 2*crossProduct(port[0]->getAngularVelocity(),WvP0P1));
    b(Wf.cols(),Wm.cols()+Wf.cols()-1) += trans(Wm)*(port[1]->getGyroscopicAccelerationOfRotation()-port[0]->getGyroscopicAccelerationOfRotation() - crossProduct(port[0]->getAngularVelocity(),WomP0P1));
  }

  void RigidConnection::projectJ(double dt) {
    for(int i=0; i<forceDir.cols() + momentDir.cols(); i++) 
      la(i) -= rFactor(i)*s(i);
  }

  void RigidConnection::projectGS(double dt) {
    double *a = parent->getGs()();
    int *ia = parent->getGs().Ip();
    int *ja = parent->getGs().Jp();
    int laInd = getlaIndMBS();
    Vec &laMBS = parent->getlaMBS();
  
    for(int i=0; i<forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = s(i);
      for(int j=ia[laInd+i]; j<ia[laInd+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);

      la(i) -= rFactor(i)*gdn(i);
    }
  }

  void RigidConnection::solveGS(double dt) {
    double *a = parent->getGs()();
    int *ia = parent->getGs().Ip();
    int *ja = parent->getGs().Jp();
    int laInd = getlaIndMBS();
    Vec &laMBS = parent->getlaMBS();
  
    for(int i=0; i<forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = s(i);
      for(int j=ia[laInd+i]+1; j<ia[laInd+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);

      la(i) = -gdn(i)/a[ia[laInd+i]];
    }
  }

  void RigidConnection::updaterFactors() {
    if(active) {
      double *a = parent->getGs()();
      int *ia = parent->getGs().Ip();
      int laInd = getlaIndMBS();

      for(int i=0; i<rFactorSize; i++) {
	double sum = 0;
	for(int j=ia[laInd+i]+1; j<ia[laInd+i+1]; j++)
	  sum += fabs(a[j]);
	double ai = a[ia[laInd+i]];
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
  void RigidConnection::residualProj(double dt) {
    double *a = parent->getGs()();
    int *ia = parent->getGs().Ip();
    int *ja = parent->getGs().Jp();
    int laInd = getlaIndMBS();
    Vec &laMBS = parent->getlaMBS();
  
    for(int i=0; i<forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = s(i);
      for(int j=ia[laInd+i]; j<ia[laInd+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);

      res(i) = gdn(i);
    }
  }

  void RigidConnection::residualProjJac(double dt) {
    int laInd = getlaIndMBS();
    for(int i=laInd; i<laInd+forceDir.cols() + momentDir.cols(); i++) 
      for(int j=0; j<parent->getG().size(); j++)  
	parent->getJprox()(i,j) = parent->getG()(i,j);
  }

  void RigidConnection::checkForTermination(double dt) {

    double *a = parent->getGs()();
    int *ia = parent->getGs().Ip();
    int *ja = parent->getGs().Jp();
    int laInd = getlaIndMBS();
    Vec &laMBS = parent->getlaMBS();

    for(int i=0; i < forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = s(i);
      for(int j=ia[laInd+i]; j<ia[laInd+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);

      if(fabs(gdn(i)) <= gdTol)
	;
      else {
	parent->setTermination(false);
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
