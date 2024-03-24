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
#include "fe_type.h"
#include "variable_widgets.h"
#include "extended_widgets.h"
#include <iostream>

using namespace std;
using namespace fmatvec;

namespace MBSimGUI {

  void FlexibleBodyTool::fma() {
    int nN = r.size();
    int nM = U.cols();
    Phi.resize(nN,Mat3xV(nM,NONINIT));
    for(size_t i=0; i<nN; i++)
      Phi[i] = Phis[i]*U;

    createSingleInterfaceNodes();

    if(Psis.size()) {
      Psi.resize(nN,Mat3xV(nM,NONINIT));
      for(size_t i=0; i<nN; i++)
	Psi[i] = Psis[i]*U;
    }
    if(sigs.size()) {
      sigmahel.resize(nN,Matrix<General,Fixed<6>,Var,double>(nM,NONINIT));
      for(size_t i=0; i<nN; i++)
	sigmahel[i] = sigs[i]*U;
    }
    else {
      auto E = page<FiniteElementsPage>(PageFiniteElements)->E->getWidget<ChoiceWidget>()->getWidget<PhysicalVariableWidget>()->getWidget<VariableWidget>()->getEvalMat()[0][0].toDouble();
      auto nu = page<FiniteElementsPage>(PageFiniteElements)->nu->getWidget<ChoiceWidget>()->getWidget<PhysicalVariableWidget>()->getWidget<VariableWidget>()->getEvalMat()[0][0].toDouble();
      sigmahel.resize(nN,Matrix<General,Fixed<6>,Var,double>(nM));
      Vec3 dNi(NONINIT);
      SqrMat3 J(NONINIT), LUJ(NONINIT);
      Vec3 r0(NONINIT);
      double x, y, z, detJ, dNi0, dNi1, dNi2;
      double omnu = 1-nu;
      double om2nu = 1-2*nu;
      double nudb1m2nu = nu/om2nu;
      double omnudbom2nu = omnu/om2nu;
      double al = E/(1+nu);
      double alh = al/2;
      double alnudb1m2nu = al*nudb1m2nu;
      double alomnudbom2nu = al*omnudbom2nu;
      for(int k=0; k<ele.size(); k++) {
	FiniteElementType *typei = type[k];
	MatVI &elei = ele[k];
	int npe = typei->getNumberOfNodes();
	int nep = typei->getNumberOfExtrapolationPoints();
	int nip = typei->getNumberOfIntegrationPoints();
	vector<Matrix<General,Fixed<6>,Var,double>> sigmahel_(npe,Matrix<General,Fixed<6>,Var,double>(nM,NONINIT));
	Vec3 dN_[npe];
	int Ip[7];
	Ip[0] = 0;
	for(int i=1; i<4; i++)
	  Ip[i] = Ip[i-1]+3*npe;
	for(int i=4; i<7; i++)
	  Ip[i] = Ip[i-1]+2*npe;
	int Jp[15*npe];
	SparseMat sigmaheles(6,Ks.size(),15*npe,Ip,Jp);
	for(int ee=0; ee<elei.rows(); ee++) {
	  for(int i=0; i<npe; i++) {
	    int u = nodeTable[elei(ee,i)];
	    Jp[3*i] = u*3;
	    Jp[3*i+1] = u*3+1;
	    Jp[3*i+2] = u*3+2;
	    Jp[3*npe+3*i] = u*3;
	    Jp[3*npe+3*i+1] = u*3+1;
	    Jp[3*npe+3*i+2] = u*3+2;
	    Jp[6*npe+3*i] = u*3;
	    Jp[6*npe+3*i+1] = u*3+1;
	    Jp[6*npe+3*i+2] = u*3+2;
	    Jp[9*npe+2*i] = u*3;
	    Jp[9*npe+2*i+1] = u*3+1;
	    Jp[11*npe+2*i] = u*3+1;
	    Jp[11*npe+2*i+1] = u*3+2;
	    Jp[13*npe+2*i] = u*3;
	    Jp[13*npe+2*i+1] = u*3+2;
	  }
	  for(int i=0; i<nep; i++)
	    sigmahel_[i].init(0);
	  for(int ii=0; ii<nip; ii++) {
	    sigmaheles.init(0);
	    x = typei->getIntegrationPoint(ii)(0);
	    y = typei->getIntegrationPoint(ii)(1);
	    z = typei->getIntegrationPoint(ii)(2);
	    J.init(0);
	    for(int ll=0; ll<npe; ll++) {
	      r0 = r[nodeTable[elei(ee,ll)]];
	      for(int mm=0; mm<3; mm++)
		dN_[ll](mm) = typei->dNdq(ll,mm,x,y,z);
	      J += dN_[ll]*r0.T();
	    }
	    detJ = J(0,0)*J(1,1)*J(2,2)+J(0,1)*J(1,2)*J(2,0)+J(0,2)*J(1,0)*J(2,1)-J(2,0)*J(1,1)*J(0,2)-J(2,1)*J(1,2)*J(0,0)-J(2,2)*J(1,0)*J(0,1);
	    LUJ(0,0) = J(2,2)*J(1,1)-J(2,1)*J(1,2);
	    LUJ(0,1) = J(2,1)*J(0,2)-J(2,2)*J(0,1);
	    LUJ(0,2) = J(1,2)*J(0,1)-J(1,1)*J(0,2);
	    LUJ(1,0) = J(2,0)*J(1,2)-J(2,2)*J(1,0);
	    LUJ(1,1) = J(2,2)*J(0,0)-J(2,0)*J(0,2);
	    LUJ(1,2) = J(1,0)*J(0,2)-J(1,2)*J(0,0);
	    LUJ(2,0) = J(2,1)*J(1,0)-J(2,0)*J(1,1);
	    LUJ(2,1) = J(2,0)*J(0,1)-J(2,1)*J(0,0);
	    LUJ(2,2) = J(1,1)*J(0,0)-J(1,0)*J(0,1);
	    for(int i=0; i<npe; i++) {
	      dNi = LUJ*dN_[i];
	      dNi0 = dNi(0)/detJ;
	      dNi1 = dNi(1)/detJ;
	      dNi2 = dNi(2)/detJ;
	      sigmaheles()[3*i] += alomnudbom2nu*dNi0;
	      sigmaheles()[3*i+1] += alnudb1m2nu*dNi1;
	      sigmaheles()[3*i+2] += alnudb1m2nu*dNi2;
	      sigmaheles()[3*npe+3*i] += alnudb1m2nu*dNi0;
	      sigmaheles()[3*npe+3*i+1] += alomnudbom2nu*dNi1;
	      sigmaheles()[3*npe+3*i+2] += alnudb1m2nu*dNi2;
	      sigmaheles()[6*npe+3*i] += alnudb1m2nu*dNi0;
	      sigmaheles()[6*npe+3*i+1] += alnudb1m2nu*dNi1;
	      sigmaheles()[6*npe+3*i+2] += alomnudbom2nu*dNi2;
	      sigmaheles()[9*npe+2*i] += alh*dNi1;
	      sigmaheles()[9*npe+2*i+1] += alh*dNi0;
	      sigmaheles()[11*npe+2*i] += alh*dNi2;
	      sigmaheles()[11*npe+2*i+1] += alh*dNi1;
	      sigmaheles()[13*npe+2*i] += alh*dNi2;
	      sigmaheles()[13*npe+2*i+1] += alh*dNi0;
	    }
	    auto B = sigmaheles*U;
	    for(int j=0; j<nep; j++)
	      sigmahel_[j] += typei->getExtrapolationCoefficient(ii,j)*B;
	  }
	  for(int j=nep; j<npe; j++)
	    sigmahel_[j] = 0.5*(sigmahel_[typei->getExtrapolationIndex(j-nep,0)] + sigmahel_[typei->getExtrapolationIndex(j-nep,1)]);
	  for(int j=0; j<npe; j++)
	    sigmahel[nodeTable[elei(ee,j)]] += sigmahel_[j]/double(nodeCount[elei(ee,j)]);
	}
      }
    }

    PPdm.resize(3,vector<SqrMatV>(3));
    Pdm <<= Pdm*U;
    for(int i=0; i<3; i++) {
      rPdm[i] <<= rPdm[i]*U;
      PPdm[i][i] <<= U.T()*(PPdms[i]*U);
    }
    PPdm[0][1] <<= U.T()*(PPdm2s[0]*U);
    PPdm[0][2] <<= U.T()*(PPdm2s[1]*U);
    PPdm[1][2] <<= U.T()*(PPdm2s[2]*U);
    Ke0 <<= JTMJ(Ks,U);
  }

}
