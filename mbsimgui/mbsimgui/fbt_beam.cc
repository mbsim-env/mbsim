/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2022 Martin Förg

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
*/

#include <config.h>
#include "wizards.h"
#include "variable_widgets.h"
#include "extended_widgets.h"

using namespace std;
using namespace fmatvec;

namespace MBSimGUI {

  void FlexibleBodyTool::beam() {
    int nN = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->n->getWidget())->getWidget())->getWidget()->getEvalMat()[0][0].toInt();
    auto l = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->l->getWidget())->getWidget())->getWidget()->getEvalMat()[0][0].toDouble();
    auto A = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->A->getWidget())->getWidget())->getWidget()->getEvalMat()[0][0].toDouble();
    auto I_ = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->I->getWidget())->getWidget())->getWidget()->getEvalMat();
    auto Iy = I_[0][0].toDouble();
    auto Iz = I_[1][0].toDouble();
    auto Iyz = I_[2][0].toDouble();
    auto E = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->E->getWidget())->getWidget())->getWidget()->getEvalMat()[0][0].toDouble();
    auto rho = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->rho->getWidget())->getWidget())->getWidget()->getEvalMat()[0][0].toDouble();
    auto ten = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->ten->getWidget())->getWidget())->getEvalMat()[0][0].toInt();
    auto benz = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->benz->getWidget())->getWidget())->getWidget()->getEvalMat()[0][0].toInt();
    auto beny = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->beny->getWidget())->getWidget())->getWidget()->getEvalMat()[0][0].toInt();
    auto tor = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->tor->getWidget())->getWidget())->getWidget()->getEvalMat()[0][0].toInt();
    nodeTable.resize(nN+1);
    for(int i=0; i<nN; i++)
      nodeTable[i+1] = i;
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
    net = nee;
    if(tor)
      all = nee++;
    if(beny)
      bel = nee++;
    if(benz)
      gal = nee++;
    int nen = nee;
    ner = nen - net;
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
    int ng = nN*nen;

    vector<Mat3xV> rPdme(3,Mat3xV(nee));
    vector<vector<SqrMatV>> PPdme(3,vector<SqrMatV>(3,SqrMatV(nee)));
    Mat3xV Pdme(nee);
    SymMatV Kee(nee);

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

    rPdm.resize(3,Mat3xV(ng));
    Pdm.resize(ng);
    Km.resize(ng);
    vector<vector<map<int,double>>> PPdmm(3,vector<map<int,double>>(ng));
    vector<vector<map<int,double>>> PPdm2m(3,vector<map<int,double>>(ng));
    RangeV I(0,2);
    for(int i=0; i<nE; i++) {
      RangeV J(i*nen,i*nen+nee-1);
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
      for(int j=0; j<3; j++)
	rPdm[j].add(I,J,rPdme[j]);
      for(int j=0; j<nee; j++) {
	for(int k=0; k<j; k++) {
	  PPdm2m[0][i*nen+j][i*nen+k] += PPdme[0][1].e(j,k);
	  PPdm2m[1][i*nen+j][i*nen+k] += PPdme[0][2].e(j,k);
	  PPdm2m[2][i*nen+j][i*nen+k] += PPdme[1][2].e(j,k);
	}
	for(int k=j; k<nee; k++) {
	  Km[i*nen+j][i*nen+k] += Kee.ej(j,k);
	  PPdmm[0][i*nen+j][i*nen+k] += PPdme[0][0].e(j,k);
	  PPdmm[1][i*nen+j][i*nen+k] += PPdme[1][1].e(j,k);
	  PPdmm[2][i*nen+j][i*nen+k] += PPdme[2][2].e(j,k);
	  PPdm2m[0][i*nen+j][i*nen+k] += PPdme[0][1].e(j,k);
	  PPdm2m[1][i*nen+j][i*nen+k] += PPdme[0][2].e(j,k);
	  PPdm2m[2][i*nen+j][i*nen+k] += PPdme[1][2].e(j,k);
	}
      }
    }

    r.resize(nN);
    vector<vector<map<int,double>>> Phim(nN,vector<map<int,double>>(3));
    vector<vector<map<int,double>>> Psim(nN,vector<map<int,double>>(3));
    vector<vector<map<int,double>>> sigm(nN,vector<map<int,double>>(6));
    for(int i=0; i<nN; i++) {
      r[i](0) = i*D;
      r[i](1) = 0;
      r[i](2) = 0;
      if(ten) {
	Phim[i][x][i*nee/2+ul] = 1;
	if(i>0 and i<nN-1) {
	  sigm[i][x][(i-1)*nee/2+ul] = -E/D/2;
	  sigm[i][x][(i+1)*nee/2+ul] = E/D/2;
	}
	else if(i<nN-1) { // i=0
	  sigm[i][x][i*nee/2+ul] = -E/D;
	  sigm[i][x][(i+1)*nee/2+ul] = E/D;
	}
	else { // i=nN-1
	  sigm[i][x][(i-1)*nee/2+ul] = -E/D;
	  sigm[i][x][i*nee/2+ul] = E/D;
	}
      }
      if(benz) {
	Phim[i][y][i*nee/2+vl] = 1;
	Psim[i][z][i*nee/2+gal] = 1;
      }
      if(beny) {
	Phim[i][z][i*nee/2+wl] = 1;
	Psim[i][y][i*nee/2+bel] = 1;
      }
      if(tor)
	Psim[i][x][i*nee/2+all] = 1;
    }

    indices.resize(3*(nN-1));
    int j = 0;
    for(int i=0; i<nN-1; i++) {
      indices[j++] = i;
      indices[j++] = i+1;
      indices[j++] = -1;
    }

    Ks <<= createSymSparseMat(Km);
    for(int i=0; i<3; i++) {
      PPdms[i] <<= createSymSparseMat(PPdmm[i]);
      PPdm2s[i] <<= createSparseMat(ng,PPdm2m[i]);
    }
    Phis.resize(nN);
    Psis.resize(nN);
    sigs.resize(nN);
    for(size_t i=0; i<nN; i++) {
      Phis[i] <<= createSparseMat(ng,Phim[i]);
      Psis[i] <<= createSparseMat(ng,Psim[i]);
      sigs[i] <<= createSparseMat(ng,sigm[i]);
    }

    links.resize(nN);
    for(int i=0; i<nN-1; i++)
      links[i][i+1] = 1;

  }

}
