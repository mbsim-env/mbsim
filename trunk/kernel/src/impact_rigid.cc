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
#include "impact_rigid.h"
#include "utils/nonsmooth_algebra.h"
#include "multi_body_system.h"

namespace MBSim {

  ImpactRigid::ImpactRigid(const string &name) : ContactRigid(name), epsilonN(0), gd_grenz(1e-2) {
  }

  void ImpactRigid::projectGS(double dt) {
    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    gdn(0) = s(0);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdn(0) += a[j]*mbs->getla()(ja[j]);

    if(fabs(gd(0)) > gd_grenz)
      gdn(0) += epsilonN*gd(0);

    la(0) = proxCN(la(0)-rFactor(0)*gdn(0));

    for(int i=1; i<=nFric; i++) {
      gdn(i) = s(i);
      for(int j=ia[laInd+i]; j<ia[laInd+1+i]; j++)
	gdn(i) += a[j]*mbs->getla()(ja[j]);
    }
    if(nFric==1) 
      la(1) = proxCT2D(la(1)-rFactor(1)*gdn(1),mue*fabs(la(0)));
    else if(nFric == 2) 
      la(1,2) = proxCT3D(la(1,2)-rFactor(1)*gdn(1,2),mue*fabs(la(0)));
  }

  void ImpactRigid::solveGS(double dt) {
    assert(nFric <= 1);

    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    gdn(0) = s(0);
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      gdn(0) += a[j]*mbs->getla()(ja[j]);

    if(fabs(gd(0)) > gd_grenz)
      gdn(0) += epsilonN*gd(0);

    double om = 1.0;
    double buf;
    if(gdn(0) >= 0)
      buf = 0;
    else {
      buf = -gdn(0)/a[ia[laInd]];
    }
    la(0) += om*(buf - la(0));;

    if(nFric) {
      gdn(1) = s(1);
      for(int j=ia[laInd+1]+1; j<ia[laInd+2]; j++)
	gdn(1) += a[j]*mbs->getla()(ja[j]);

      double laNmue = fabs(la(0))*mue;
      double sdG = -gdn(1)/a[ia[laInd+1]];

      if(fabs(sdG)<=laNmue) 
	buf = sdG;
      else 
	buf = (laNmue<=sdG) ? laNmue : -laNmue;
      la(1) += om*(buf - la(1));;
    }
  }


  void ImpactRigid::residualProj(double dt) {
    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    for(int i=0; i < 1+nFric; i++) {
      gdn(i) = s(i);
      for(int j=ia[laInd+i]; j<ia[laInd+1+i]; j++)
	gdn(i) += a[j]*mbs->getla()(ja[j]);
    }

    if(fabs(gd(0)) > gd_grenz)
      gdn(0) += epsilonN*gd(0);

    argN = la(0) - rFactor(0)*gdn(0);
    res(0) = la(0) - proxCN(argN);

    if(nFric==1) {
      argT(0) = la(1) - rFactor(1)*gdn(1);
      res(1) = la(1) - proxCT2D(argT(0),mue*fabs(la(0)));

    } else if(nFric == 2) {
      argT = la(1,2) - rFactor(1)*gdn(1,2);
      res(1,2) = la(1,2) - proxCT3D(argT,mue*fabs(la(0)));
    }
  }

  void ImpactRigid::checkForTermination(double dt) {

    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    for(int i=0; i < 1+nFric; i++) {
      gdn(i) = s(i);
      for(int j=ia[laInd+i]; j<ia[laInd+1+i]; j++)
	gdn(i) += a[j]*mbs->getla()(ja[j]);
    }

    if(fabs(gd(0)) > gd_grenz)
      gdn(0) += epsilonN*gd(0);

    if(gdn(0) >= -gdTol && fabs(la(0)) <= laTol*dt)
      ;
    else if(la(0) >= -laTol*dt && fabs(gdn(0)) <= gdTol)
      ;
    else {
      mbs->setTermination(false);
      return;
    }

    if(nFric==1) {
      if(fabs(la(1) + gdn(1)/fabs(gdn(1))*mue*fabs(la(0))) <= laTol*dt)
	;
      else if(fabs(la(1)) <= mue*fabs(la(0))+laTol*dt && fabs(gdn(1)) <= gdTol)
	;
      else {
	mbs->setTermination(false);
	return;
      }
    } else if(nFric==2) {
      if(nrm2(la(1,2) + gdn(1,2)/nrm2(gdn(1,2))*mue*fabs(la(0))) <= laTol*dt)
	;
      else if(nrm2(la(1,2)) <= mue*fabs(la(0))+laTol*dt && nrm2(gdn(1,2)) <= gdTol)
	;
      else {
	mbs->setTermination(false);
	return;
      }
    }

  }

}
