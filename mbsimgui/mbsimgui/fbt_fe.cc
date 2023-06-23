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
#include "basic_widgets.h"
#include "variable_widgets.h"
#include "special_widgets.h"
#include "C3D10.h"
#include "C3D15.h"
#include "C3D20.h"
#include "C3D20R.h"

using namespace std;
using namespace fmatvec;

namespace MBSimGUI {

  void FlexibleBodyTool::fe() {
    MatV R;
    string str = static_cast<FileWidget*>(static_cast<FiniteElementsPage*>(page(PageFiniteElements))->nodes->getWidget())->getFile(true).toStdString();
    if(!str.empty())
      R <<= readMat(str);

    auto *list = static_cast<ListWidget*>(static_cast<FiniteElementsPage*>(page(PageFiniteElements))->elements->getWidget());
    for(int i=0; i<list->getSize(); i++) {
      auto type_ = static_cast<FiniteElementsDataWidget*>(list->getWidget(i))->getType().toStdString();
      type_ = type_.substr(1,type_.size()-2);
      int npe = stod(type_.substr(type_.find('D')+1,type_.size()-1));
      if(type_=="C3D10")
	type.emplace_back(new MBSimGUI::C3D10);
      else if(type_=="C3D15")
	type.emplace_back(new MBSimGUI::C3D15);
      else if(type_=="C3D20")
	type.emplace_back(new MBSimGUI::C3D20);
      else if(type_=="C3D20R")
	type.emplace_back(new MBSimGUI::C3D20R);
      str = static_cast<FiniteElementsDataWidget*>(list->getWidget(i))->getElementsFile().toStdString();
      if(!str.empty()) {
	auto ele_ = readMat(str);
	if(ele_.cols()==npe)
	  ele.emplace_back(ele_);
	else if(ele_.cols()==npe+1)
	  ele.emplace_back(ele_(RangeV(0,ele_.rows()-1),RangeV(1,ele_.cols()-1)));
	else
	  throw 5;
      }
    }

    auto E = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FiniteElementsPage*>(page(PageFiniteElements))->E->getWidget())->getWidget())->getWidget()->getEvalMat()[0][0].toDouble();

    auto nu = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FiniteElementsPage*>(page(PageFiniteElements))->nu->getWidget())->getWidget())->getWidget()->getEvalMat()[0][0].toDouble();

    auto rho = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FiniteElementsPage*>(page(PageFiniteElements))->rho->getWidget())->getWidget())->getWidget()->getEvalMat()[0][0].toDouble();

    int max = 0;
    if(R.cols()==3)
      max = R.rows();
    else if(R.cols()==4) {
      for(int i=0; i<R.rows(); i++) {
	if(R(i,0)>max)
	  max = R(i,0);
      }
    }

    int nE = 0;
    nodeCount.resize(max+1);
    for(size_t k=0; k<ele.size(); k++) {
      nE += ele[k].rows();
      for(int i=0; i<ele[k].rows(); i++) {
	for(int j=0; j<ele[k].cols(); j++)
	  nodeCount[ele[k](i,j)]++;
      }
    }

    nodeTable.resize(nodeCount.size());
    int nN = 0;
    for(size_t i=0; i<nodeCount.size(); i++) {
      if(nodeCount[i])
	nodeTable[i] = nN++;
    }

    links.resize(nN);
    for(size_t k=0; k<ele.size(); k++) {
      for(int ee=0; ee<ele[k].rows(); ee++) {
	int npe = type[k]->getNumberOfNodes();
	for(int i=0; i<npe; i++) {
	  int u = nodeTable[ele[k](ee,i)];
	  for(int j=i+1; j<npe; j++) {
	    int v = nodeTable[ele[k](ee,j)];
	    if(v>=u)
	      links[u][v] = 1;
	    else
	      links[v][u] = 1;
	  }
	}
      }
    }
    vector<map<int,int>> links2(nN);
    for(size_t k=0; k<ele.size(); k++) {
      for(int ee=0; ee<ele[k].rows(); ee++) {
	int npe = type[k]->getNumberOfNodes();
	for(int i=0; i<npe; i++) {
	  int u = nodeTable[ele[k](ee,i)];
	  for(int j=0; j<npe; j++) {
	    int v = nodeTable[ele[k](ee,j)];
	    links2[u][v] = 1;
	  }
	}
      }
    }

    r.resize(nN,Vec3(NONINIT));
    nodeNumbers.resize(nN);
    if(R.cols()==3) {
      int j=0;
      for(int i=0; i<R.rows(); i++) {
	if(nodeCount[i+1]) {
	  r[nodeTable[i+1]] = R.row(i).T();
	  nodeNumbers[j] = j+1;
	  j++;
	}
      }
    }
    else {
      for(int i=0; i<R.rows(); i++) {
	if(nodeCount[R(i,0)]) {
	  r[nodeTable[R(i,0)]] = R.row(i)(RangeV(1,3)).T();
	  nodeNumbers[nodeTable[R(i,0)]] = R(i,0);
	}
      }
    }

    int ng = nN*3;
    net = 3;
    ner = 0;

    int nze = 0;
    for(size_t i=0; i<nN; i++)
      nze += 9*links[i].size()+6;
    int nze2 = 0;
    for(size_t i=0; i<nN; i++)
      nze2 += 9*links2[i].size();
    int *Ip = new int[ng+1];
    int *Jp = new int[nze];
    int *Ip2 = new int[ng+1];
    int *Jp2 = new int[nze2];
    Ip[0] = 0;
    Ip2[0] = 0;
    nze = 0;
    nze2 = 0;
    ng = 0;
    for(size_t i=0; i<nN; i++) {
      for(int k=0; k<3; k++) {
	for(int l=k; l<3; l++)
	  Jp[nze++] = 3*i+l;
	for(const auto & j : links[i]) {
	  for(int l=0; l<3; l++)
	    Jp[nze++] = 3*j.first+l;
	}
	for(const auto & j : links2[i]) {
	  for(int l=0; l<3; l++)
	    Jp2[nze2++] = 3*j.first+l;
	}
	Ip[++ng] = nze;
	Ip2[ng] = nze2;
      }
    }
    SymSparseMat SM1(ng,nze,Ip,Jp);
    Ks &= SM1;
    SymSparseMat SM2(ng,nze,Ip,Jp);
    PPdms[0] &= SM2;
    SymSparseMat SM3(ng,nze,Ip,Jp);
    PPdms[1] &= SM3;
    SymSparseMat SM4(ng,nze,Ip,Jp);
    PPdms[2] &= SM4;
    SparseMat SM5(ng,ng,nze2,Ip2,Jp2);
    PPdm2s[0] &= SM5;
    SparseMat SM6(ng,ng,nze2,Ip2,Jp2);
    PPdm2s[1] &= SM6;
    SparseMat SM7(ng,ng,nze2,Ip2,Jp2);
    PPdm2s[2] &= SM7;

    rPdm.resize(3,Mat3xV(ng));
    Pdm.resize(ng);

    Phis.resize(nN,SparseMat(3,ng,3,NONINIT));
    for(int i=0; i<nN; i++) {
      for(int j=0; j<3; j++) {
	Phis[i].Ip()[j] = j;
	Phis[i].Jp()[j] = 3*i+j;
	Phis[i]()[j] = 1;
      }
      Phis[i].Ip()[3] = 3;
    }

    indices.resize(5*6*nE);
    int oj = 0;
    double P[20];
    double rP[20][3];
    double M[20][20];
    double K[60][60];
    double omnu = 1-nu;
    double om2nu = 1-2*nu;
    double nudb1m2nu = nu/om2nu;
    double omnudbom2nu = omnu/om2nu;
    double x, y, z, wijk, detJ, dm, dk, Ni_, Nj_, dNi0, dNi1, dNi2, dNj0, dNj1, dNj2, dNi0dNj0, dNi1dNj1, dNi2dNj2, dNi0dNj1, dNi0dNj2, dNi1dNj0, dNi1dNj2, dNi2dNj0, dNi2dNj1;
    Vec3 dNi(NONINIT), dNj(NONINIT);
    SqrMat3 J(NONINIT), LUJ(NONINIT);
    Vec3 r(NONINIT), r0(NONINIT);
    for(size_t k=0; k<ele.size(); k++) {
      FiniteElementType *typei = type[k];
      MatVI &elei = ele[k];
      int npe = typei->getNumberOfNodes();
      double N_[npe];
      Vec3 dN_[npe];
      for(int ee=0; ee<elei.rows(); ee++) {
	for(int i=0; i<npe; i++) {
	  P[i] = 0;
	  for(int j=0; j<3; j++)
	    rP[i][j] = 0;
	  for(int j=i; j<npe; j++)
	    M[i][j] = 0;
	}
	for(int i=0; i<3*npe; i++) {
	  for(int j=i; j<3*npe; j++)
	    K[i][j] = 0;
	}
	for(int ii=0; ii<typei->getNumberOfIntegrationPoints(); ii++) {
	  x = typei->getIntegrationPoint(ii)(0);
	  y = typei->getIntegrationPoint(ii)(1);
	  z = typei->getIntegrationPoint(ii)(2);
	  wijk = typei->getWeight(ii);
	  J.init(0);
	  r.init(0);
	  for(int ll=0; ll<npe; ll++) {
	    r0 = this->r[nodeTable[elei(ee,ll)]];
	    N_[ll] = typei->N(ll,x,y,z);
	    for(int mm=0; mm<3; mm++)
	      dN_[ll](mm) = typei->dNdq(ll,mm,x,y,z);
	    J += dN_[ll]*r0.T();
	    r += N_[ll]*r0;
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

	  dm = rho*wijk*detJ;
	  dk = E/(1+nu)*wijk*detJ;
	  m += dm;
	  rdm += dm*r;
	  rrdm += dm*JTJ(r.T());
	  for(int i=0; i<npe; i++) {
	    Ni_ = N_[i];
	    dNi = LUJ*dN_[i];
	    dNi0 = dNi(0)/detJ;
	    dNi1 = dNi(1)/detJ;
	    dNi2 = dNi(2)/detJ;
	    P[i] += dm*Ni_;
	    for(int j=0; j<3; j++)
	      rP[i][j] += dm*r(j)*Ni_;
	    for(int j=i; j<npe; j++) {
	      Nj_ = N_[j];
	      dNj = LUJ*dN_[j];
	      dNj0 = dNj(0)/detJ;
	      dNj1 = dNj(1)/detJ;
	      dNj2 = dNj(2)/detJ;
	      dNi0dNj0 = dNi0*dNj0;
	      dNi1dNj1 = dNi1*dNj1;
	      dNi2dNj2 = dNi2*dNj2;
	      dNi0dNj1 = dNi0*dNj1;
	      dNi0dNj2 = dNi0*dNj2;
	      dNi1dNj0 = dNi1*dNj0;
	      dNi1dNj2 = dNi1*dNj2;
	      dNi2dNj0 = dNi2*dNj0;
	      dNi2dNj1 = dNi2*dNj1;
	      M[i][j] += dm*Ni_*Nj_;
	      K[i*3][j*3] += dk*(omnudbom2nu*dNi0dNj0+0.5*(dNi1dNj1+dNi2dNj2));
	      K[i*3][j*3+1] += dk*(nudb1m2nu*dNi0dNj1+0.5*dNi1dNj0);
	      K[i*3][j*3+2] += dk*(nudb1m2nu*dNi0dNj2+0.5*dNi2dNj0);
	      K[i*3+1][j*3] += dk*(nudb1m2nu*dNi1dNj0+0.5*dNi0dNj1);
	      K[i*3+1][j*3+1] += dk*(omnudbom2nu*dNi1dNj1+0.5*(dNi0dNj0+dNi2dNj2));
	      K[i*3+1][j*3+2] += dk*(nudb1m2nu*dNi1dNj2+0.5*dNi2dNj1);
	      K[i*3+2][j*3] += dk*(nudb1m2nu*dNi2dNj0+0.5*dNi0dNj2);
	      K[i*3+2][j*3+1] += dk*(nudb1m2nu*dNi2dNj1+0.5*dNi1dNj2);
	      K[i*3+2][j*3+2] += dk*(omnudbom2nu*dNi2dNj2+0.5*(dNi0dNj0+dNi1dNj1));
	    }
	  }
	}

	const auto ind = typei->getOmbvIndices();
	for(int i=0; i<ind.size(); i++) {
	  for(int j=0; j<ind[i].size(); j++)
	    indices[oj++] = nodeTable[elei(ee,ind[i][j])];
	  indices[oj++] = -1;
	}

	for(int i=0; i<npe; i++) {
	  int u = nodeTable[elei(ee,i)];
	  for(int ii=0; ii<3; ii++) {
	    Pdm(ii,u*3+ii) += P[i];
	    for(int jj=0; jj<3; jj++)
	      rPdm[ii](jj,u*3+jj) += rP[i][ii];
	  }
	  for(int j=i; j<npe; j++) {
	    int v = nodeTable[elei(ee,j)];
	    if(v==u) {
	      for(int ii=0; ii<3; ii++) {
		int pos = Ks.pos(u*3+ii,v*3+ii);
		PPdms[ii]()[pos] += M[i][j];
		for(int jj=ii, kk=0; jj<3; jj++, kk++) {
		  Ks()[pos+kk] += K[i*3+ii][j*3+jj];
		}
	      }
	    }
	    else if(v>u) {
	      for(int ii=0; ii<3; ii++) {
		int pos = Ks.pos(u*3+ii,v*3);
		PPdms[ii]()[pos+ii] += M[i][j];
		for(int jj=0; jj<3; jj++)
		  Ks()[pos+jj] += K[i*3+ii][j*3+jj];
	      }
	    }
	    else {
	      for(int ii=0; ii<3; ii++) {
		int pos = Ks.pos(v*3+ii,u*3);
		PPdms[ii]()[pos+ii] += M[i][j];
		for(int jj=0; jj<3; jj++)
		  Ks()[pos+jj] += K[i*3+jj][j*3+ii];
	      }
	    }
	    int pos = PPdm2s[0].pos(u*3,v*3+1);
	    PPdm2s[0]()[pos] += M[i][j];
	    PPdm2s[1]()[pos+1] += M[i][j];
	    PPdm2s[2](u*3+1,v*3+2) += M[i][j];
	    if(u!=v) {
	      int pos = PPdm2s[0].pos(v*3,u*3+1);
	      PPdm2s[0]()[pos] += M[i][j];
	      PPdm2s[1]()[pos+1] += M[i][j];
	      PPdm2s[2](v*3+1,u*3+2) += M[i][j];
	    }
	  }
	}
      }
    }
  }
}
