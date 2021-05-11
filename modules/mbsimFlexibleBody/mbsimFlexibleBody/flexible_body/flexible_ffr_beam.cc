/* Copyright (C) 2004-2020 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "flexible_ffr_beam.h"
#include "openmbvcppinterface/dynamicpointset.h"
#include "openmbvcppinterface/dynamicindexedfaceset.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMFLEX, FlexibleFfrBeam)

  void FlexibleFfrBeam::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      int nE = nN-1;
      ne = 8;
      int ng = nN*ne/2;
      int nr = 4;
      int n = ng-nr;
      vector<Mat3xV> rPdme(3,Mat3xV(ne));
      vector<Mat3xV> rPdmg(3,Mat3xV(ng));
      rPdm.resize(3,Mat3xV(n,NONINIT));
      vector<vector<SqrMatV>> PPdme(3,vector<SqrMatV>(3,SqrMatV(ne)));
      vector<vector<SqrMatV>> PPdmg(3,vector<SqrMatV>(3,SqrMatV(ng)));
      PPdm.resize(3,vector<SqrMatV>(3,SqrMatV(n,NONINIT)));
      Mat3xV Pdme(ne);
      Mat3xV Pdmg(ng);
      Pdm.resize(n,NONINIT);
      SymMatV Kee(ne);
      SqrMatV Keg(ng);
      Ke0.resize(n,NONINIT);
      KrKP.resize(nN,Vec3());
      Phi.resize(nN,Mat3xV(n));
      Psi.resize(nN,Mat3xV(n));

      const int vl = 0;
      const int wl = 1;
      const int bel = 2;
      const int gal = 3;
      const int vr = 4;
      const int wr = 5;
      const int ber = 6;
      const int gar = 7;

      double D = l/nE;

      m = rho*A*l;
      double me = m/nE;

      rdm(0) = m*l/2;

      rrdm(0,0) = m/3*pow(l,2);
      rrdm(1,1) = rho*l*Iz;
      rrdm(1,2) = -rho*l*Iyz;
      rrdm(2,2) = rho*l*Iy;

      Pdme(1,vl) = me/2;
      Pdme(1,gal) = D*me/12;
      Pdme(1,vr) = me/2;
      Pdme(1,gar) = -D*me/12;
      Pdme(2,wl) = Pdme(1,vl);
      Pdme(2,bel) = -Pdme(1,gal);
      Pdme(2,wr) = Pdme(1,vr);
      Pdme(2,ber) = -Pdme(1,gar);

      PPdme[0][0](vl,vl) = 6./5/D*rho*Iz;
      PPdme[0][0](vl,wl) = -6./5/D*rho*Iyz;
      PPdme[0][0](vl,bel) = 1./10*rho*Iyz;
      PPdme[0][0](vl,gal) = 1./10*rho*Iz;
      PPdme[0][0](vl,vr) = -6./5/D*rho*Iz;
      PPdme[0][0](vl,wr) = 6./5/D*rho*Iyz;
      PPdme[0][0](vl,ber) = 1./10*rho*Iyz;
      PPdme[0][0](vl,gar) = 1./10*rho*Iz;

      PPdme[0][0](wl,wl) = 6./5/D*rho*Iy;
      PPdme[0][0](wl,bel) = -1./10*rho*Iy;
      PPdme[0][0](wl,gal) = -1./10*rho*Iyz;
      PPdme[0][0](wl,vr) = 6./5/D*rho*Iyz;
      PPdme[0][0](wl,wr) = -6./5/D*rho*Iy;
      PPdme[0][0](wl,ber) = -1./10*rho*Iy;
      PPdme[0][0](wl,gar) = -1./10*rho*Iyz;

      PPdme[0][0](bel,bel) = 2*D/15*rho*Iy;
      PPdme[0][0](bel,gal) = 2*D/15*rho*Iyz;
      PPdme[0][0](bel,vr) = -1./10*rho*Iyz;
      PPdme[0][0](bel,wr) = 1./10*rho*Iy;
      PPdme[0][0](bel,ber) = -D/30*rho*Iy;
      PPdme[0][0](bel,gar) = -D/30*rho*Iyz;

      PPdme[0][0](gal,gal) = 2*D/15*rho*Iz;
      PPdme[0][0](gal,vr) = -1./10*rho*Iz;
      PPdme[0][0](gal,wr) = 1./10*rho*Iyz;
      PPdme[0][0](gal,ber) = -D/30*rho*Iyz;
      PPdme[0][0](gal,gar) = -D/30*rho*Iz;

      PPdme[0][0](vr,vr) = 6./5/D*rho*Iz;
      PPdme[0][0](vr,wr) = -6./5/D*rho*Iyz;
      PPdme[0][0](vr,ber) = -1./10*rho*Iyz;
      PPdme[0][0](vr,gar) = -1./10*rho*Iz;

      PPdme[0][0](wr,wr) = 6./5/D*rho*Iy;
      PPdme[0][0](wr,ber) = 1./10*rho*Iy;
      PPdme[0][0](wr,gar) = 1./10*rho*Iyz;

      PPdme[0][0](ber,ber) = 2*D/15*rho*Iy;
      PPdme[0][0](ber,gar) = 2*D/15*rho*Iyz;

      PPdme[0][0](gar,gar) = 2*D/15*rho*Iz;

      PPdme[1][1](vl,vl) = 13./35*me;
      PPdme[1][1](vl,gal) = 11./210*me*D;
      PPdme[1][1](vl,vr) = 9./70*me;
      PPdme[1][1](vl,gar) = -13./420*me*D;
      PPdme[1][1](gal,gal) = me*pow(D,2)/105;
      PPdme[1][1](gal,vr) = 13./420*me*D;
      PPdme[1][1](gal,gar) = -me*pow(D,2)/140;
      PPdme[1][1](vr,vr) = 13./35*me;
      PPdme[1][1](vr,gar) = -11./210*me*D;
      PPdme[1][1](gar,gar) = me*pow(D,2)/105;

      PPdme[2][2](wl,wl) = PPdme[1][1](vl,vl);
      PPdme[2][2](wl,bel) = -PPdme[1][1](vl,gal);
      PPdme[2][2](wl,wr) = PPdme[1][1](vl,vr);
      PPdme[2][2](wl,ber) = -PPdme[1][1](vl,gar);
      PPdme[2][2](bel,bel) = PPdme[1][1](gal,gal);
      PPdme[2][2](bel,wr) = -PPdme[1][1](gal,vr);
      PPdme[2][2](bel,ber) = PPdme[1][1](gal,gar);
      PPdme[2][2](wr,wr) = PPdme[1][1](vr,vr);
      PPdme[2][2](wr,ber) = -PPdme[1][1](vr,gar);
      PPdme[2][2](ber,ber) = PPdme[1][1](gar,gar);

      for(int k=0; k<8; k++) {
	for(int j=0; j<k; j++) {
	  PPdme[0][0](k,j) = PPdme[0][0](j,k);
	  PPdme[1][1](k,j) = PPdme[1][1](j,k);
	  PPdme[2][2](k,j) = PPdme[2][2](j,k);
	}
      }

      PPdme[1][2](vl,wl) = PPdme[1][1](vl,vl);
      PPdme[1][2](vl,bel) = -PPdme[1][1](vl,gal);
      PPdme[1][2](vl,wr) = PPdme[1][1](vl,vr);
      PPdme[1][2](vl,ber) = -PPdme[1][1](vl,gar);
      PPdme[1][2](gal,wl) = PPdme[1][1](gal,vl);
      PPdme[1][2](gal,bel) = -PPdme[1][1](gal,gal);
      PPdme[1][2](gal,wr) = PPdme[1][1](gal,vr);
      PPdme[1][2](gal,ber) = -PPdme[1][1](gal,gar);
      PPdme[1][2](vr,wl) = PPdme[1][1](vr,vl);
      PPdme[1][2](vr,bel) = -PPdme[1][1](vr,gal);
      PPdme[1][2](vr,wr) = PPdme[1][1](vr,vr);
      PPdme[1][2](vr,ber) = -PPdme[1][1](vr,gar);
      PPdme[1][2](gar,wl) = PPdme[1][1](gar,vl);
      PPdme[1][2](gar,bel) = -PPdme[1][1](gar,gal);
      PPdme[1][2](gar,wr) = PPdme[1][1](gar,vr);
      PPdme[1][2](gar,ber) = -PPdme[1][1](gar,gar);

      PPdme[2][1] = PPdme[1][2].T();

      rPdme[1](0,vl) = rho*Iz;
      rPdme[1](0,wl) = -rho*Iyz;
      rPdme[1](0,vr) = -rho*Iz;
      rPdme[1](0,wr) = rho*Iyz;
      rPdme[2](0,vl) = -rho*Iyz;
      rPdme[2](0,wl) = rho*Iy;
      rPdme[2](0,vr) = rho*Iyz;
      rPdme[2](0,wr) = -rho*Iy;

      Kee(vl,vl) = 12./pow(D,3)*E*Iz;
      Kee(vl,wl) = -12./pow(D,3)*E*Iyz;
      Kee(vl,bel) = 6./pow(D,2)*E*Iyz;
      Kee(vl,gal) = 6./pow(D,2)*E*Iz;
      Kee(vl,vr) = -12./pow(D,3)*E*Iz;
      Kee(vl,wr) = 12./pow(D,3)*E*Iyz;
      Kee(vl,ber) = 6./pow(D,2)*E*Iyz;
      Kee(vl,gar) = 6./pow(D,2)*E*Iz;

      Kee(wl,wl) = 12./pow(D,3)*E*Iy;
      Kee(wl,bel) = -6./pow(D,2)*E*Iy;
      Kee(wl,gal) = -6./pow(D,2)*E*Iyz;
      Kee(wl,vr) = 12./pow(D,3)*E*Iyz;
      Kee(wl,wr) = -12./pow(D,3)*E*Iy;
      Kee(wl,ber) = -6./pow(D,2)*E*Iy;
      Kee(wl,gar) = -6./pow(D,2)*E*Iyz;

      Kee(bel,bel) = 4./D*E*Iy;
      Kee(bel,gal) = 4./D*E*Iyz;
      Kee(bel,vr) = -6./pow(D,2)*E*Iyz;
      Kee(bel,wr) = 6./pow(D,2)*E*Iy;
      Kee(bel,ber) = 2./D*E*Iy;
      Kee(bel,gar) = 2./D*E*Iyz;

      Kee(gal,gal) = 4./D*E*Iz;
      Kee(gal,vr) = -6./pow(D,2)*E*Iz;
      Kee(gal,wr) = 6./pow(D,2)*E*Iyz;
      Kee(gal,ber) = 2./D*E*Iyz;
      Kee(gal,gar) = 2./D*E*Iz;

      Kee(vr,vr) = 12./pow(D,3)*E*Iz;
      Kee(vr,wr) = -12./pow(D,3)*E*Iyz;
      Kee(vr,ber) = -6./pow(D,2)*E*Iyz;
      Kee(vr,gar) = -6./pow(D,2)*E*Iz;

      Kee(wr,wr) = 12./pow(D,3)*E*Iy;
      Kee(wr,ber) = 6./pow(D,2)*E*Iy;
      Kee(wr,gar) = 6./pow(D,2)*E*Iyz;

      Kee(ber,ber) = 4./D*E*Iy;
      Kee(ber,gar) = 4./D*E*Iyz;

      Kee(gar,gar) = 4./D*E*Iz;

      RangeV I(0,2);
      for(int i=0; i<nE; i++) {
	RangeV J(i*ne/2,i*ne/2+ne-1);
	int j=i+1;
	rPdme[0](1,vl) = me*D*(j/2.-7./20);
	rPdme[0](1,gal) = me*D*(j*D/12.-D/20);
	rPdme[0](1,vr) = me*D*(j/2.-3./20);
	rPdme[0](1,gar) = me*D*(-j*D/12+D/30);
	rPdme[0](2,wl) = rPdme[0](1,vl);
	rPdme[0](2,bel) = -rPdme[0](1,gal);
	rPdme[0](2,wr) = rPdme[0](1,vr);
	rPdme[0](2,ber) = -rPdme[0](1,gar);
	Pdmg.add(I,J,Pdme);
	for(int j=0; j<3; j++) {
	  rPdmg[j].add(I,J,rPdme[j]);
	  for(int k=0; k<3; k++)
	    PPdmg[j][k].add(J,J,PPdme[j][k]);
	}
	Keg.add(J,J,Kee);
      }

      vector<int> c;
      for(int i=0; i<bc.rows(); i++) {
	for(int j=(int)bc(i,1); j<=(int)bc(i,2); j++)
	  c.push_back(bc(i,0)*ne/2+j);
      }

      int k=0, h=0;
      for(int i=0; i<ng; i++) {
	if(i==c[h])
	  h++;
	else {
	  Pdm.set(k,Pdmg.col(i));
	  for(int ii=0; ii<3; ii++)
	    rPdm[ii].set(k,rPdmg[ii].col(i));
	  int l=0, r=0;
	  for(int j=0; j<ng; j++) {
	    if(j==c[r])
	      r++;
	    else {
	      Ke0(k,l) = Keg(i,j);
	      for(int ii=0; ii<3; ii++) {
		for(int jj=0; jj<3; jj++)
		  PPdm[ii][jj](k,l) = PPdmg[ii][jj](i,j);
	      }
	      l++;
	    }
	  }
	  k++;
	}
      }
      for(int i=0; i<nN; i++) {
	KrKP[i](0) = i*D;
	k=0; h=0;
	for(int j=0; j<ng; j++) {
	  if(j==c[h])
	    h++;
	  else {
	    if(j==i*ne/2) Phi[i](1,k) = 1;
	    else if(j==i*ne/2+1) Phi[i](2,k) = 1;
	    else if(j==i*ne/2+2) Psi[i](1,k) = 1;
	    else if(j==i*ne/2+3) Psi[i](2,k) = 1;
	    k++;
	  }
	}
      }
    }
    else if(stage==plotting) {
      if(plotFeature[openMBV] and ombvBody) {
        std::shared_ptr<OpenMBV::FlexibleBody> flexbody = ombvBody->createOpenMBV();
        openMBVBody = flexbody;
        ombvColorRepresentation = static_cast<OpenMBVFlexibleBody::ColorRepresentation>(ombvBody->getColorRepresentation());
      }
    }
    GenericFlexibleFfrBody::init(stage, config);
  }

  void FlexibleFfrBeam::initializeUsingXML(DOMElement *element) {
    GenericFlexibleFfrBody::initializeUsingXML(element);
    DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"numberOfNodes");
    setNumberOfNodes(MBXMLUtils::E(e)->getText<int>());
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"length");
    setLength(MBXMLUtils::E(e)->getText<double>());
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"crossSectionArea");
    setCrossSectionArea(MBXMLUtils::E(e)->getText<double>());
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"momentOfInertia");
    setMomentOfInertia(MBXMLUtils::E(e)->getText<Vec3>());
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"youngsModulus");
    setYoungsModulus(MBXMLUtils::E(e)->getText<double>());
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"density");
    setDensity(MBXMLUtils::E(e)->getText<double>());
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"proportionalDamping");
    if(e) setProportionalDamping(MBXMLUtils::E(e)->getText<Vec>());
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"boundaryConditions");
    if(e) {
      setBoundaryConditions(MBXMLUtils::E(e)->getText<MatVx3>());
      for(int i=0; i<bc.rows(); i++) {
	for(int j=0; j<bc.cols(); j++)
	  bc(i,j)--;
      }
    }
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"enableOpenMBV");
    if(e) {
      ombvBody = shared_ptr<OpenMBVFlexibleFfrBeam>(new OpenMBVFlexibleFfrBeam);
      ombvBody->initializeUsingXML(e);
    }
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"plotNodeNumbers");
    if(e) setPlotNodeNumbers(MBXMLUtils::E(e)->getText<VecVI>());
  }

}
