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
      int nee = 0;
      const int x = 0;
      const int y = 1;
      const int z = 2;
      int ul=-1, vl=-1, wl=-1, all=-1, bel=-1, gal=-1, ur=-1, vr=-1, wr=-1, alr=-1, ber=-1, gar=-1;
      if(ten)
	ul = nee++;
      if(benz)
	vl = nee++;
      if(beny)
	wl = nee++;
      if(tor)
	all = nee++;
      if(beny)
	bel = nee++;
      if(benz)
	gal = nee++;
      if(ten)
	ur = nee++;
      if(benz)
	vr = nee++;
      if(beny)
	wr = nee++;
      if(tor)
	alr = nee++;
      if(beny)
	ber = nee++;
      if(benz)
	gar = nee++;
      int ng = nN*nee/2;
      int nr = 0;
      for(int i=0; i<bc.rows(); i++)
	nr += bc(i,2)-bc(i,1)+1;
      int n = ng-nr;
      vector<Mat3xV> rPdme(3,Mat3xV(nee));
      rPdm.resize(3,Mat3xV(ng));
      vector<vector<SqrMatV>> PPdme(3,vector<SqrMatV>(3,SqrMatV(nee)));
      PPdm.resize(3,vector<SqrMatV>(3,SqrMatV(ng)));
      Mat3xV Pdme(nee);
      Pdm.resize(ng);
      SymMatV Kee(nee);
      Ke0.resize(ng);
      KrKP.resize(nN,Vec3());
      Phi.resize(nN,Mat3xV(ng));
      Psi.resize(nN,Mat3xV(ng));
      sigmahel.resize(nN,Matrix<General,Fixed<6>,Var,double>(ng));

      double D = l/nE;

      m = rho*A*l;
      double me = m/nE;

      rdm(x) = m*l/2;

      rrdm(x,x) = m/3*pow(l,2);
      rrdm(y,y) = rho*l*Iz;
      rrdm(y,z) = -rho*l*Iyz;
      rrdm(z,z) = rho*l*Iy;

      if(ten) {
	Pdme(x,ul) = me/2;
	Pdme(x,ur) = me/2;
      }
      if(benz) {
	Pdme(y,vl) = me/2;
	Pdme(y,gal) = D*me/12;
	Pdme(y,vr) = me/2;
	Pdme(y,gar) = -D*me/12;
      }
      if(beny) {
	Pdme(z,wl) = me/2;
	Pdme(z,bel) = -D*me/12;
	Pdme(z,wr) = me/2;
	Pdme(z,ber) = D*me/12;
      }

      if(ten) {
	PPdme[x][x](ul,ul) = me/3;
	PPdme[x][x](ul,ur) = me/6;
	PPdme[x][x](ur,ul) = me/6;
	PPdme[x][x](ur,ur) = me/3;
      }

      if(benz) { // v and ga
	PPdme[x][x](vl,vl) = 6./5/D*rho*Iz;
	PPdme[x][x](vl,gal) = 1./10*rho*Iz;
	PPdme[x][x](vl,vr) = -6./5/D*rho*Iz;
	PPdme[x][x](vl,gar) = 1./10*rho*Iz;
	PPdme[x][x](gal,gal) = 2*D/15*rho*Iz;
	PPdme[x][x](gal,vr) = -1./10*rho*Iz;
	PPdme[x][x](gal,gar) = -D/30*rho*Iz;
	PPdme[x][x](vr,vr) = 6./5/D*rho*Iz;
	PPdme[x][x](vr,gar) = -1./10*rho*Iz;
	PPdme[x][x](gar,gar) = 2*D/15*rho*Iz;
	if(beny) { // w and be
	  PPdme[x][x](vl,wl) = -6./5/D*rho*Iyz;
	  PPdme[x][x](vl,bel) = 1./10*rho*Iyz;
	  PPdme[x][x](vl,wr) = 6./5/D*rho*Iyz;
	  PPdme[x][x](vl,ber) = 1./10*rho*Iyz;
	  PPdme[x][x](gal,wr) = 1./10*rho*Iyz;
	  PPdme[x][x](gal,ber) = -D/30*rho*Iyz;
	  PPdme[x][x](vr,wr) = -6./5/D*rho*Iyz;
	  PPdme[x][x](vr,ber) = -1./10*rho*Iyz;
	}
      }
      if(beny) { // w and be
	PPdme[x][x](wl,wl) = 6./5/D*rho*Iy;
	PPdme[x][x](wl,bel) = -1./10*rho*Iy;
	PPdme[x][x](wl,wr) = -6./5/D*rho*Iy;
	PPdme[x][x](wl,ber) = -1./10*rho*Iy;
	PPdme[x][x](bel,bel) = 2*D/15*rho*Iy;
	PPdme[x][x](bel,wr) = 1./10*rho*Iy;
	PPdme[x][x](bel,ber) = -D/30*rho*Iy;
	PPdme[x][x](wr,wr) = 6./5/D*rho*Iy;
	PPdme[x][x](wr,ber) = 1./10*rho*Iy;
	PPdme[x][x](ber,ber) = 2*D/15*rho*Iy;
	if(benz) { // v and ga
	  PPdme[x][x](wl,gal) = -1./10*rho*Iyz;
	  PPdme[x][x](wl,vr) = 6./5/D*rho*Iyz;
	  PPdme[x][x](wl,gar) = -1./10*rho*Iyz;
	  PPdme[x][x](bel,gal) = 2*D/15*rho*Iyz;
	  PPdme[x][x](bel,vr) = -1./10*rho*Iyz;
	  PPdme[x][x](bel,gar) = -D/30*rho*Iyz;
	  PPdme[x][x](wr,gar) = 1./10*rho*Iyz;
	  PPdme[x][x](ber,gar) = 2*D/15*rho*Iyz;
	}
      }
      if(benz) { // v and ga
	PPdme[y][y](vl,vl) = 13./35*me;
	PPdme[y][y](vl,gal) = 11./210*me*D;
	PPdme[y][y](vl,vr) = 9./70*me;
	PPdme[y][y](vl,gar) = -13./420*me*D;
	PPdme[y][y](gal,gal) = me*pow(D,2)/105;
	PPdme[y][y](gal,vr) = 13./420*me*D;
	PPdme[y][y](gal,gar) = -me*pow(D,2)/140;
	PPdme[y][y](vr,vr) = 13./35*me;
	PPdme[y][y](vr,gar) = -11./210*me*D;
	PPdme[y][y](gar,gar) = me*pow(D,2)/105;
      }
      if(beny) { // w and be
	PPdme[z][z](wl,wl) = 13./35*me;
	PPdme[z][z](wl,bel) = -11./210*me*D;
	PPdme[z][z](wl,wr) = 9./70*me;
	PPdme[z][z](wl,ber) = 13./420*me*D;
	PPdme[z][z](bel,bel) = me*pow(D,2)/105;
	PPdme[z][z](bel,wr) = -13./420*me*D;
	PPdme[z][z](bel,ber) = -me*pow(D,2)/140;
	PPdme[z][z](wr,wr) = 13./35*me;
	PPdme[z][z](wr,ber) = 11./210*me*D;
	PPdme[z][z](ber,ber) = me*pow(D,2)/105;
      }
      if(tor) {
	PPdme[y][y](all,all) = rho*Iy*D/3;
	PPdme[y][y](all,alr) = rho*Iy*D/6;
	PPdme[y][y](alr,alr) = rho*Iy*D/3;
	PPdme[z][z](all,all) = rho*Iz*D/3;
	PPdme[z][z](all,alr) = rho*Iz*D/6;
	PPdme[z][z](alr,alr) = rho*Iz*D/3;
	PPdme[y][z](all,all) = D/3*rho*Iyz;
	PPdme[y][z](all,alr) = D/6*rho*Iyz;
	PPdme[y][z](alr,all) = D/6*rho*Iyz;
	PPdme[y][z](alr,alr) = D/3*rho*Iyz;
      }
      if(ten and benz) { // u, v and ga
	PPdme[x][y](ul,vl) = 7./20*me;
	PPdme[x][y](ul,gal) = 1./20*me*D;
	PPdme[x][y](ul,vr) = 3./20*me;
	PPdme[x][y](ul,gar) = -1./30*me*D;
	PPdme[x][y](ur,vl) = 3./20*me;
	PPdme[x][y](ur,gal) = 1./30*me*D;
	PPdme[x][y](ur,vr) = 7./20*me;
	PPdme[x][y](ur,gar) = -1./20*me*D;
      }
      if(ten and beny) { // u, w and be
	PPdme[x][z](ul,wl) = 7./20*me;
	PPdme[x][z](ul,bel) = -1./20*me*D;
	PPdme[x][z](ul,wr) = 3./20*me;
	PPdme[x][z](ul,ber) = 1./30*me*D;
	PPdme[x][z](ur,wl) = 3./20*me;
	PPdme[x][z](ur,bel) = -1./30*me*D;
	PPdme[x][z](ur,wr) = 7./20*me;
	PPdme[x][z](ur,ber) = 1./20*me*D;
      }
      if(tor and benz) { // al, v and ga
	PPdme[x][y](vl,all) = 1./2*rho*Iyz;
	PPdme[x][y](vl,alr) = 1./2*rho*Iyz;
	PPdme[x][y](gal,all) = -D/12*rho*Iyz;
	PPdme[x][y](gal,alr) = D/12*rho*Iyz;
	PPdme[x][y](vr,all) = -1./2*rho*Iyz;
	PPdme[x][y](vr,alr) = -1./2*rho*Iyz;
	PPdme[x][y](gar,all) = D/12*rho*Iyz;
	PPdme[x][y](gar,alr) = -D/12*rho*Iyz;
	PPdme[x][z](vl,all) = 1./2*rho*Iz;
	PPdme[x][z](vl,alr) = 1./2*rho*Iz;
	PPdme[x][z](gal,all) = -D/12*rho*Iz;
	PPdme[x][z](gal,alr) = D/12*rho*Iz;
	PPdme[x][z](vr,all) = -1./2*rho*Iz;
	PPdme[x][z](vr,alr) = -1./2*rho*Iz;
	PPdme[x][z](gar,all) = D/12*rho*Iz;
	PPdme[x][z](gar,alr) = -D/12*rho*Iz;
      }
      if(tor and beny) { // al, w and be
	PPdme[x][y](wl,all) = -1./2*rho*Iy;
	PPdme[x][y](wl,alr) = -1./2*rho*Iy;
	PPdme[x][y](bel,all) = -D/12*rho*Iy;
	PPdme[x][y](bel,alr) = D/12*rho*Iy;
	PPdme[x][y](wr,all) = 1./2*rho*Iy;
	PPdme[x][y](wr,alr) = 1./2*rho*Iy;
	PPdme[x][y](ber,all) = D/12*rho*Iy;
	PPdme[x][y](ber,alr) = -D/12*rho*Iy;
	PPdme[x][z](wl,all) = -1./2*rho*Iyz;
	PPdme[x][z](wl,alr) = -1./2*rho*Iyz;
	PPdme[x][z](bel,all) = -D/12*rho*Iyz;
	PPdme[x][z](bel,alr) = D/12*rho*Iyz;
	PPdme[x][z](wr,all) = 1./2*rho*Iyz;
	PPdme[x][z](wr,alr) = 1./2*rho*Iyz;
	PPdme[x][z](ber,all) = D/12*rho*Iyz;
	PPdme[x][z](ber,alr) = -D/12*rho*Iyz;
      }
      if(beny and benz) {
	PPdme[y][z](vl,wl) = 13./35*me;
	PPdme[y][z](vl,bel) = -11./210*me*D;
	PPdme[y][z](vl,wr) = 9./70*me;
	PPdme[y][z](vl,ber) = 13./420*me*D;
	PPdme[y][z](gal,wl) = 11./210*me*D;
	PPdme[y][z](gal,bel) = -me*pow(D,2)/105;
	PPdme[y][z](gal,wr) = 13./420*me*D;
	PPdme[y][z](gal,ber) = me*pow(D,2)/140;
	PPdme[y][z](vr,wl) = 9./70*me;
	PPdme[y][z](vr,bel) = -13./420*me*D;
	PPdme[y][z](vr,wr) = 13./35*me;
	PPdme[y][z](vr,ber) = 11./210*me*D;
	PPdme[y][z](gar,wl) = -13./420*me*D;
	PPdme[y][z](gar,bel) = me*pow(D,2)/140;
	PPdme[y][z](gar,wr) = -11./210*me*D;
	PPdme[y][z](gar,ber) = -me*pow(D,2)/105;
      }
      for(int k=0; k<nee; k++) {
	for(int j=0; j<k; j++) {
	  PPdme[x][x](k,j) = PPdme[x][x](j,k);
	  PPdme[y][y](k,j) = PPdme[y][y](j,k);
	  PPdme[z][z](k,j) = PPdme[z][z](j,k);
	}
      }
      PPdme[y][x] = PPdme[x][y].T();
      PPdme[z][x] = PPdme[x][z].T();
      PPdme[z][y] = PPdme[y][z].T();

      if(benz) { // v and ga
	rPdme[y](x,vl) = rho*Iz;
	rPdme[y](x,vr) = -rho*Iz;
	rPdme[z](x,vl) = -rho*Iyz;
	rPdme[z](x,vr) = rho*Iyz;
      }
      if(beny) { // w and be
	rPdme[y](x,wl) = -rho*Iyz;
	rPdme[y](x,wr) = rho*Iyz;
	rPdme[z](x,wl) = rho*Iy;
	rPdme[z](x,wr) = -rho*Iy;
      }
      if(tor) {
	rPdme[y](y,all) = D/2*rho*Iyz;
	rPdme[y](y,alr) = D/2*rho*Iyz;
	rPdme[y](z,all) = D/2*rho*Iz;
	rPdme[y](z,alr) = D/2*rho*Iz;
	rPdme[z](y,all) = -D/2*rho*Iy;
	rPdme[z](y,alr) = -D/2*rho*Iy;
	rPdme[z](z,all) = -D/2*rho*Iyz;
	rPdme[z](z,alr) = -D/2*rho*Iyz;
      }

      if(ten) {
	Kee(ul,ul) = E*A/D;
	Kee(ul,ur) = -E*A/D;
	Kee(ur,ur) = E*A/D;
      }
      if(benz) { // v and ga
	Kee(vl,vl) = 12./pow(D,3)*E*Iz;
	Kee(vl,gal) = 6./pow(D,2)*E*Iz;
	Kee(vl,vr) = -12./pow(D,3)*E*Iz;
	Kee(vl,gar) = 6./pow(D,2)*E*Iz;
	Kee(gal,gal) = 4./D*E*Iz;
	Kee(gal,vr) = -6./pow(D,2)*E*Iz;
	Kee(gal,gar) = 2./D*E*Iz;
	Kee(vr,vr) = 12./pow(D,3)*E*Iz;
	Kee(vr,gar) = -6./pow(D,2)*E*Iz;
	Kee(gar,gar) = 4./D*E*Iz;
	if(beny) { // w and be
	  Kee(vl,wl) = -12./pow(D,3)*E*Iyz;
	  Kee(vl,bel) = 6./pow(D,2)*E*Iyz;
	  Kee(vl,wr) = 12./pow(D,3)*E*Iyz;
	  Kee(vl,ber) = 6./pow(D,2)*E*Iyz;
	  Kee(gal,wr) = 6./pow(D,2)*E*Iyz;
	  Kee(gal,ber) = 2./D*E*Iyz;
	  Kee(vr,wr) = -12./pow(D,3)*E*Iyz;
	  Kee(vr,ber) = -6./pow(D,2)*E*Iyz;
	}
      }
      if(beny) { // w and be
	Kee(wl,wl) = 12./pow(D,3)*E*Iy;
	Kee(wl,bel) = -6./pow(D,2)*E*Iy;
	Kee(wl,wr) = -12./pow(D,3)*E*Iy;
	Kee(wl,ber) = -6./pow(D,2)*E*Iy;
	Kee(bel,bel) = 4./D*E*Iy;
	Kee(bel,wr) = 6./pow(D,2)*E*Iy;
	Kee(bel,ber) = 2./D*E*Iy;
	Kee(wr,wr) = 12./pow(D,3)*E*Iy;
	Kee(wr,ber) = 6./pow(D,2)*E*Iy;
	Kee(ber,ber) = 4./D*E*Iy;
	if(benz) { // v and ga
	  Kee(wl,gal) = -6./pow(D,2)*E*Iyz;
	  Kee(wl,vr) = 12./pow(D,3)*E*Iyz;
	  Kee(wl,gar) = -6./pow(D,2)*E*Iyz;
	  Kee(bel,gal) = 4./D*E*Iyz;
	  Kee(bel,vr) = -6./pow(D,2)*E*Iyz;
	  Kee(bel,gar) = 2./D*E*Iyz;
	  Kee(wr,gar) = 6./pow(D,2)*E*Iyz;
	  Kee(ber,gar) = 4./D*E*Iyz;
	}
      }
      if(tor) {
	Kee(all,all) = E/2/D*(Iy+Iz);
	Kee(all,alr) = -E/2/D*(Iy+Iz);
	Kee(alr,alr) = E/2/D*(Iy+Iz);
      }

      RangeV I(0,2);
      for(int i=0; i<nE; i++) {
	RangeV J(i*nee/2,i*nee/2+nee-1);
	if(ten) {
	  rPdme[x](x,ul) = me*D*(i/2.+1./6);
	  rPdme[x](x,ur) = me*D*(i/2.+1./3);
	}
	if(benz) { // v and ga
	  rPdme[x](y,vl) = me*D*(i/2.+3./20);
	  rPdme[x](y,gal) = me*D*(i*D/12.+D/30);
	  rPdme[x](y,vr) = me*D*(i/2.+7./20);
	  rPdme[x](y,gar) = -me*D*(i*D/12.+D/20);
	}
	if(beny) { // w and be
	  rPdme[x](z,wl) = me*D*(i/2.+3./20);
	  rPdme[x](z,bel) = -me*D*(i*D/12.+D/30);
	  rPdme[x](z,wr) = me*D*(i/2.+7./20);
	  rPdme[x](z,ber) = me*D*(i*D/12.+D/20);
	}
	Pdm.add(I,J,Pdme);
	for(int j=0; j<3; j++) {
	  rPdm[j].add(I,J,rPdme[j]);
	  for(int k=0; k<3; k++)
	    PPdm[j][k].add(J,J,PPdme[j][k]);
	}
	Ke0.add(J,Kee);
      }

      vector<int> c;
      for(int i=0; i<bc.rows(); i++) {
	for(int j=(int)bc(i,1); j<=(int)bc(i,2); j++)
	  c.push_back(bc(i,0)*nee/2+j);
      }
      sort(c.begin(), c.end());

      size_t h=0;
      Indices IF;
      for(int i=0; i<ng; i++) {
	if(h<c.size() and i==c[h])
	  h++;
	else
	  IF.add(i);
      }
      Indices I3{0,1,2};
      Indices I6{0,1,2,3,4,5};
      Pdm <<= Pdm(I3,IF);
      for(size_t i=0; i<3; i++) {
	rPdm[i] <<= rPdm[i](I3,IF);
	for(size_t j=0; j<3; j++)
	  PPdm[i][j] <<= PPdm[i][j](IF,IF);
      }
      Ke0 <<= Ke0(IF);
      for(int i=0; i<nN; i++) {
	KrKP[i](0) = i*D;
	if(ten) {
	  Phi[i](x,i*nee/2+ul) = 1;
	  if(i>0 and i<nN-1) {
	    sigmahel[i](x,(i-1)*nee/2+ul) = -E/D/2;
	    sigmahel[i](x,(i+1)*nee/2+ul) = E/D/2;
	  }
	  else if(i<nN-1) { // i=0
	    sigmahel[i](x,i*nee/2+ul) = -E/D;
	    sigmahel[i](x,(i+1)*nee/2+ul) = E/D;
	  }
	  else { // i=nN-1
	    sigmahel[i](x,(i-1)*nee/2+ul) = -E/D;
	    sigmahel[i](x,i*nee/2+ul) = E/D;
	  }
	}
	if(benz) Phi[i](y,i*nee/2+vl) = 1;
	if(beny) Phi[i](z,i*nee/2+wl) = 1;
	if(tor) Psi[i](x,i*nee/2+all) = 1;
	if(beny) Psi[i](y,i*nee/2+bel) = 1;
	if(benz) Psi[i](z,i*nee/2+gal) = 1;
	Phi[i] <<= Phi[i](I3,IF);
	Psi[i] <<= Psi[i](I3,IF);
	sigmahel[i] <<= sigmahel[i](I6,IF);
      }

      c.clear();
      for(int i=0; i<inodes.size(); i++) {
	int j1=nee/2;
	int j2=-1;
	for(int k=0; k<bc.rows(); k++) {
	  if(inodes(i)==(int)bc(k,0)) {
	    j1=bc(k,1);
	    j2=bc(k,2);
	  }
	}
	for(int j=0; j<nee/2; j++)
	  if(j<j1 or j>j2) c.push_back(inodes(i)*nee/2+j);
      }
      sort(c.begin(), c.end());
      h=0;
      Indices IH, IN;
      for(int i=0; i<IF.size(); i++) {
	if(h<c.size() and IF[i]==c[h]) {
	  IH.add(i);
	  h++;
	}
	else
	  IN.add(i);
      }
      MatV Vsd(n,IH.size()+nmodes.size(),NONINIT);
      if(IH.size()) {
	Indices IJ;
	for(int i=0; i<IH.size(); i++)
	  IJ.add(i);
	MatV Vs(IF.size(),IH.size(),NONINIT);
	Vs.set(IN,IJ,-slvLL(Ke0(IN),Ke0(IN,IH)));
	Vs.set(IH,IJ,MatV(IH.size(),IH.size(),Eye()));
	Vsd.set(RangeV(0,n-1),RangeV(0,Vs.cols()-1),Vs);
      }

      SqrMat V;
      Vec w;
      if(nmodes.size()) {
	if(fixedBoundaryNormalModes) {
	  eigvec(Ke0(IN),SymMat(PPdm[0][0]+PPdm[1][1]+PPdm[2][2])(IN),V,w);
	  vector<int> imod;
	  for(int i=0; i<w.size(); i++) {
	    if(w(i)>pow(2*M_PI*0.1,2))
	      imod.push_back(i);
	  }
	  if(min(nmodes)<1 or max(nmodes)>(int)imod.size())
	    throwError(string("(FlexibleFfrBeam::init): node numbers do not match, must be within the range [1,") + to_string(imod.size()) + "]");
	  for(int i=0; i<nmodes.size(); i++) {
	    Vsd.set(IN,IH.size()+i,V.col(imod[nmodes(i)-1]));
	    Vsd.set(IH,IH.size()+i,Vec(IH.size()));
	  }
	}
	else {
	  eigvec(Ke0,SymMat(PPdm[0][0]+PPdm[1][1]+PPdm[2][2]),V,w);
	  vector<int> imod;
	  for(int i=0; i<w.size(); i++) {
	    if(w(i)>pow(2*M_PI*0.1,2))
	      imod.push_back(i);
	  }
	  if(min(nmodes)<1 or max(nmodes)>(int)imod.size())
	    throwError(string("(FlexibleFfrBeam::init): node numbers do not match, must be within the range [1,") + to_string(imod.size()) + "]");
	  for(int i=0; i<nmodes.size(); i++)
	    Vsd.set(IH.size()+i,V.col(imod[nmodes(i)-1]));
	}
      }

      if(IH.size()) {
	eigvec(JTMJ(Ke0,Vsd),JTMJ(SymMatV(PPdm[0][0]+PPdm[1][1]+PPdm[2][2]),Vsd),V,w);
	vector<int> imod;
	for(int i=0; i<w.size(); i++) {
	  if(w(i)>pow(2*M_PI*0.1,2))
	    imod.push_back(i);
	}
	MatV Vr(w.size(),imod.size(),NONINIT);
	for(size_t i=0; i<imod.size(); i++)
	  Vr.set(i,V.col(imod[i]));
	Vsd <<= Vsd*Vr;
      }

      if(Vsd.cols()) {
	Pdm <<= Pdm*Vsd;
	for(int i=0; i<3; i++) {
	  rPdm[i] <<= rPdm[i]*Vsd;
	  for(int j=0; j<3; j++)
	    PPdm[i][j] <<= Vsd.T()*PPdm[i][j]*Vsd;
	}
	Ke0 <<= JTMJ(Ke0,Vsd);
	for(int i=0; i<nN; i++) {
	  Phi[i] <<= Phi[i]*Vsd;
	  Psi[i] <<= Psi[i]*Vsd;
	  sigmahel[i] <<= sigmahel[i]*Vsd;
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
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"tension");
    setTension(MBXMLUtils::E(e)->getText<bool>());
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"bendingAboutYAxis");
    setBendingAboutYAxis(MBXMLUtils::E(e)->getText<bool>());
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"bendingAboutZAxis");
    setBendingAboutZAxis(MBXMLUtils::E(e)->getText<bool>());
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"torsion");
    setTorsion(MBXMLUtils::E(e)->getText<bool>());
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
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"interfaceNodeNumbers");
    if(e) {
      setInterfaceNodeNumbers(MBXMLUtils::E(e)->getText<VecVI>());
      for(int i=0; i<inodes.size(); i++)
	  inodes(i)--;
    }
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"normalModeNumbers");
    if(e) setNormalModeNumbers(MBXMLUtils::E(e)->getText<VecVI>());
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"fixedBoundaryNormalModes");
    if(e) setFixedBoundaryNormalModes(MBXMLUtils::E(e)->getText<bool>());
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"enableOpenMBV");
    if(e) {
      ombvBody = shared_ptr<OpenMBVFlexibleFfrBeam>(new OpenMBVFlexibleFfrBeam);
      ombvBody->initializeUsingXML(e);
    }
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"plotNodeNumbers");
    if(e) setPlotNodeNumbers(MBXMLUtils::E(e)->getText<VecVI>());
  }

}
