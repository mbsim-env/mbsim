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
#include "connection_rigid.h"
#include "port.h"
#include "multi_body_system.h"
#include "function.h"

namespace MBSim {

  ConnectionRigid::ConnectionRigid(const string &name) : Connection(name,true) {
  }

  void ConnectionRigid::init() {
    Connection::init();

    for(int i=0; i<2; i++) {
      loadDir.push_back(Mat(6,laSize));
      fF[i] >> loadDir[i](Index(0,2),Index(0,Wf.cols()-1));
      fM[i] >> loadDir[i](Index(3,5),Index(Wf.cols(),Wf.cols()+Wm.cols()-1));
    }

    fF[0] = -Wf;
    fF[1] = Wf;
    fM[0] = -Wm;
    fM[1] = Wm;
  }

  void ConnectionRigid::updateKinetics(double dt) {
    if(KOSYID) {
      fF[0] = -Wf;
      fF[1] = Wf;
      fM[0] = -Wm;
      fM[1] = Wm;
      loadDir[0](Index(3,5),Index(0,Wf.cols()-1)) = tilde(WrP0P1)*fF[0];
    }
  }

  void ConnectionRigid::updatew(double t) {
    if(KOSYID) {
      w[0](0,Wf.cols()-1)=trans(Wf)*(crossProduct(port[0]->getWomegaP(),crossProduct(port[0]->getWomegaP(),WrP0P1)) - 2*crossProduct(port[0]->getWomegaP(),WvP0P1));
      w[0](Wf.cols(),Wm.cols()+Wf.cols()-1)=trans(Wm)*crossProduct(WomP0P1,port[1]->getWomegaP());
    }
  }

  void ConnectionRigid::projectJ(double dt) {
    for(int i=0; i<forceDir.cols() + momentDir.cols(); i++) 
      la(i) -= rFactor(i)*s(i);
  }

  void ConnectionRigid::projectGS(double dt) {
    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();

    for(int i=0; i<forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = s(i);
      for(int j=ia[laInd+i]; j<ia[laInd+1+i]; j++)
	gdn(i) += a[j]*mbs->getla()(ja[j]);

      la(i) -= rFactor(i)*gdn(i);
    }
  }

  void ConnectionRigid::solveGS(double dt) {
    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();

    for(int i=0; i<forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = s(i);
      for(int j=ia[laInd+i]+1; j<ia[laInd+1+i]; j++)
	gdn(i) += a[j]*mbs->getla()(ja[j]);

      la(i) = -gdn(i)/a[ia[laInd+i]];
    }
  }

  void ConnectionRigid::updaterFactors() {
    if(active) {
      double *a = mbs->getGs()();
      int *ia = mbs->getGs().Ip();
//      int *ja = mbs->getGs().Jp(); // unused
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
  void ConnectionRigid::residualProj(double dt) {
    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();

    for(int i=0; i<forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = s(i);
      for(int j=ia[laInd+i]; j<ia[laInd+1+i]; j++)
	gdn(i) += a[j]*mbs->getla()(ja[j]);

      res(i) = gdn(i);
    }
  }

  void ConnectionRigid::residualProjJac(double dt) {
    for(int i=laInd; i<laInd+forceDir.cols() + momentDir.cols(); i++) 
      for(int j=0; j<mbs->getG().size(); j++)  
	mbs->getJprox()(i,j) = mbs->getG()(i,j);
  }

  void ConnectionRigid::checkForTermination(double dt) {

    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    for(int i=0; i < forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = s(i);
      for(int j=ia[laInd+i]; j<ia[laInd+1+i]; j++)
	gdn(i) += a[j]*mbs->getla()(ja[j]);

      if(fabs(gdn(i)) <= gdTol)
	;
      else {
	mbs->setTermination(false);
	return;
      }
    }
  }

  std::string ConnectionRigid::getTerminationInfo(double dt) {
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
