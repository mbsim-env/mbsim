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
#include "C3D20.h"

using namespace std;
using namespace fmatvec;

namespace MBSimGUI {

  void FlexibleBodyTool::fe() {
    MatV R;
    string str = static_cast<FileWidget*>(static_cast<FiniteElementsPage*>(page(PageFiniteElements))->nodes->getWidget())->getFile(true).toStdString();
    if(!str.empty())
      R <<= readMat(str);

    std::vector<fmatvec::MatVI> ele;
    auto *list = static_cast<ListWidget*>(static_cast<FiniteElementsPage*>(page(PageFiniteElements))->elements->getWidget());
    vector<string> type;
    for(int i=0; i<list->getSize(); i++) {
      auto type_ = static_cast<FiniteElementsDataWidget*>(list->getWidget(i))->getType().toStdString();
      type_ = type_.substr(1,type_.size()-2);
      int npe = stod(type_.substr(type_.find('D')+1,type_.size()-1));
      type.emplace_back(type_);
      str = static_cast<FiniteElementsDataWidget*>(list->getWidget(i))->getElementsFile().toStdString();
      if(!str.empty()) {
	auto ele_ = readMat(str);
	if(ele_.cols()==npe)
	  ele.emplace_back(ele_);
	else if(ele_.cols()==npe+1)
	  ele.emplace_back(ele_(RangeV(0,ele_.rows()-1),RangeV(1,ele_.cols()-1)));
      }
    }

    auto E = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FiniteElementsPage*>(page(PageFiniteElements))->E->getWidget())->getWidget())->getEvalMat()[0][0].toDouble();

    auto nu = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FiniteElementsPage*>(page(PageFiniteElements))->nu->getWidget())->getWidget())->getEvalMat()[0][0].toDouble();

    auto rho = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FiniteElementsPage*>(page(PageFiniteElements))->rho->getWidget())->getWidget())->getEvalMat()[0][0].toDouble();

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
    vector<int> nodeCount(max+1);
    for(size_t k=0; k<ele.size(); k++) {
      nE += ele[k].rows();
      for(int i=0; i<ele[k].rows(); i++) {
	for(int j=0; j<ele[k].cols(); j++)
	  nodeCount[ele[k](i,j)]++;
      }
    }

    nodeTable.resize(nodeCount.size());
    nN = 0;
    for(size_t i=0; i<nodeCount.size(); i++) {
      if(nodeCount[i])
	nodeTable[i] = nN++;
    }

    KrKP.resize(nN,Vec3(NONINIT));
    if(R.cols()==3) {
      for(int i=0; i<R.rows(); i++) {
	if(nodeCount[i+1])
	  KrKP[nodeTable[i+1]] = R.row(i).T();
      }
    }
    else {
      nodeNumbers.resize(nN);
      for(int i=0; i<R.rows(); i++) {
	if(nodeCount[R(i,0)]) {
	  KrKP[nodeTable[R(i,0)]] = R.row(i)(RangeV(1,3)).T();
	  nodeNumbers[nodeTable[R(i,0)]] = R(i,0);
	}
      }
    }

    ng = nN*3;
    nen = 3;

    rPdm.resize(3,Mat3xV(ng));
    Pdm.resize(ng);

    Phim.resize(nN,vector<map<int,double>>(3));
    for(int i=0; i<nN; i++) {
      Phim[i][0][3*i] = 1;
      Phim[i][1][3*i+1] = 1;
      Phim[i][2][3*i+2] = 1;
    }

    sigm.resize(nN,vector<map<int,double>>(6));
    MKm.resize(ng);
    PPm.resize(ng);

    m = 0;
    indices.resize(5*6*nE);
    SqrMat3 LUJ(NONINIT);
    double dK[3][3];
    double dsig[9];
    int oj = 0;
    FiniteElementType *feType;
    for(size_t k=0; k<ele.size(); k++) {
      if(type[k]=="C3D10")
	feType = new MBSimGUI::C3D10;
      else if(type[k]=="C3D20")
	feType = new MBSimGUI::C3D20;
      else
	feType = nullptr;

      MatVI &elei = ele[k];

      int npe = feType->getNumberOfNodes();
      double N_[npe];
      Vec3 dN_[npe];
      for(int ee=0; ee<elei.rows(); ee++) {
	for(int ii=0; ii<feType->getNumberOfIntegrationPoints(); ii++) {
	  double x = feType->getIntegrationPoint(ii)(0);
	  double y = feType->getIntegrationPoint(ii)(1);
	  double z = feType->getIntegrationPoint(ii)(2);
	  double wijk = feType->getWeight(ii);
	  SqrMat3 J(3);
	  Vec3 r(3);
	  for(int ll=0; ll<npe; ll++) {
	    Vec3 r0 = KrKP[nodeTable[elei(ee,ll)]];
	    N_[ll] = feType->N(ll,x,y,z);
	    for(int mm=0; mm<3; mm++)
	      dN_[ll](mm) = feType->dNdq(ll,mm,x,y,z);
	    J += dN_[ll]*r0.T();
	    r += N_[ll]*r0;
	  }
	  Vector<Ref,int> ipiv(J.size(),NONINIT);
	  double detJ = J(0,0)*J(1,1)*J(2,2)+J(0,1)*J(1,2)*J(2,0)+J(0,2)*J(1,0)*J(2,1)-J(2,0)*J(1,1)*J(0,2)-J(2,1)*J(1,2)*J(0,0)-J(2,2)*J(1,0)*J(0,1);
	  LUJ(0,0) = J(2,2)*J(1,1)-J(2,1)*J(1,2);
	  LUJ(0,1) = J(2,1)*J(0,2)-J(2,2)*J(0,1);
	  LUJ(0,2) = J(1,2)*J(0,1)-J(1,1)*J(0,2);
	  LUJ(1,0) = J(2,0)*J(1,2)-J(2,2)*J(1,0);
	  LUJ(1,1) = J(2,2)*J(0,0)-J(2,0)*J(0,2);
	  LUJ(1,2) = J(1,0)*J(0,2)-J(1,2)*J(0,0);
	  LUJ(2,0) = J(2,1)*J(1,0)-J(2,0)*J(1,1);
	  LUJ(2,1) = J(2,0)*J(0,1)-J(2,1)*J(0,0);
	  LUJ(2,2) = J(1,1)*J(0,0)-J(1,0)*J(0,1);

	  double dm = rho*wijk*detJ;
	  double dk = E/(1+nu)*wijk*detJ;
	  m += dm;
	  rdm += dm*r;
	  rrdm += dm*JTJ(r.T());
	  for(int i=0; i<npe; i++) {
	    int u = nodeTable[elei(ee,i)];
	    double Ni_ = N_[i];
	    Vec3 dNi = LUJ*dN_[i]/detJ;
	    for(int i1=0; i1<3; i1++) {
	      Pdm(i1,u*3+i1) += dm*Ni_;
	      for(int j1=0; j1<3; j1++)
		rPdm[i1](j1,u*3+j1) += dm*r(i1)*Ni_;
	    }
	    for(int j=i; j<npe; j++) {
	      int v = nodeTable[elei(ee,j)];
	      double Nj_ = N_[j];
	      Vec3 dNj = LUJ*dN_[j]/detJ;
	      double dPPdm = dm*Ni_*Nj_;
	      dK[0][0] = dk*((1-nu)/(1-2*nu)*dNi(0)*dNj(0)+0.5*(dNi(1)*dNj(1)+dNi(2)*dNj(2)));
	      dK[0][1] = dk*(nu/(1-2*nu)*dNi(0)*dNj(1)+0.5*dNi(1)*dNj(0));
	      dK[0][2] = dk*(nu/(1-2*nu)*dNi(0)*dNj(2)+0.5*dNi(2)*dNj(0));
	      dK[1][0] = dk*(nu/(1-2*nu)*dNi(1)*dNj(0)+0.5*dNi(0)*dNj(1));
	      dK[1][1] = dk*((1-nu)/(1-2*nu)*dNi(1)*dNj(1)+0.5*(dNi(0)*dNj(0)+dNi(2)*dNj(2)));
	      dK[1][2] = dk*(nu/(1-2*nu)*dNi(1)*dNj(2)+0.5*dNi(2)*dNj(1));
	      dK[2][0] = dk*(nu/(1-2*nu)*dNi(2)*dNj(0)+0.5*dNi(0)*dNj(2));
	      dK[2][1] = dk*(nu/(1-2*nu)*dNi(2)*dNj(1)+0.5*dNi(1)*dNj(2));
	      dK[2][2] = dk*((1-nu)/(1-2*nu)*dNi(2)*dNj(2)+0.5*(dNi(0)*dNj(0)+dNi(1)*dNj(1)));
	      if(v>=u) {
		for(int iii=0; iii<3; iii++) {
		  auto d = MKm[u*3+iii][v*3+iii];
		  d[iii] += dPPdm;
		  d[3] += dK[iii][iii];
		}
		MKm[u*3][v*3+1][3] += dK[0][1];
		MKm[u*3][v*3+2][3] += dK[0][2];
		MKm[u*3+1][v*3+2][3] += dK[1][2];
		if(v!=u) {
		  MKm[u*3+1][v*3][3] += dK[1][0];
		  MKm[u*3+2][v*3][3] += dK[2][0];
		  MKm[u*3+2][v*3+1][3] += dK[2][1];
		}
	      }
	      else {
		for(int iii=0; iii<3; iii++) {
		  auto d = MKm[v*3+iii][u*3+iii];
		  d[iii] += dPPdm;
		  d[3] += dK[iii][iii];
		}
		MKm[v*3][u*3+1][3] += dK[1][0];
		MKm[v*3][u*3+2][3] += dK[2][0];
		MKm[v*3+1][u*3+2][3] += dK[2][1];
		MKm[v*3+1][u*3][3] += dK[0][1];
		MKm[v*3+2][u*3][3] += dK[0][2];
		MKm[v*3+2][u*3+1][3] += dK[1][2];
	      }
	      PPm[u*3][v*3+1][0] += dPPdm;
	      PPm[u*3][v*3+2][1] += dPPdm;
	      PPm[u*3+1][v*3+2][2] += dPPdm;
	      if(u!=v) {
		PPm[v*3][u*3+1][0] += dPPdm;
		PPm[v*3][u*3+2][1] += dPPdm;
		PPm[v*3+1][u*3+2][2] += dPPdm;
	      }
	    }
	  }
	}

	for(int k=0; k<npe; k++) {
	  double x = feType->getNaturalCoordinates(k)(0);
	  double y = feType->getNaturalCoordinates(k)(1);
	  double z = feType->getNaturalCoordinates(k)(2);
	  SqrMat3 J(3);
	  for(int ll=0; ll<npe; ll++) {
	    Vec3 r0 = KrKP[nodeTable[elei(ee,ll)]];
	    for(int mm=0; mm<3; mm++)
	      dN_[ll](mm) = feType->dNdq(ll,mm,x,y,z);
	    J += dN_[ll]*r0.T();
	  }
	  Vector<Ref,int> ipiv(J.size(),NONINIT);
	  double detJ = J(0,0)*J(1,1)*J(2,2)+J(0,1)*J(1,2)*J(2,0)+J(0,2)*J(1,0)*J(2,1)-J(2,0)*J(1,1)*J(0,2)-J(2,1)*J(1,2)*J(0,0)-J(2,2)*J(1,0)*J(0,1);
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
	    Vec3 dNi = LUJ*dN_[i]/detJ;
	    int ku = nodeTable[elei(ee,k)];
	    int u = nodeTable[elei(ee,i)];
	    double al = E/(1+nu)/nodeCount[elei(ee,k)];
	    dsig[0] = al*(1-nu)/(1-2*nu)*dNi(0);
	    dsig[1] = al*nu/(1-2*nu)*dNi(1);
	    dsig[2] = al*nu/(1-2*nu)*dNi(2);
	    dsig[3] = al*nu/(1-2*nu)*dNi(0);
	    dsig[4] = al*(1-nu)/(1-2*nu)*dNi(1);
	    dsig[5] = al*(1-nu)/(1-2*nu)*dNi(2);
	    dsig[6] = al*0.5*dNi(1);
	    dsig[7] = al*0.5*dNi(0);
	    dsig[8] = al*0.5*dNi(2);
	    sigm[ku][0][u*3] += dsig[0];
	    sigm[ku][0][u*3+1] += dsig[1];
	    sigm[ku][0][u*3+2] += dsig[2];
	    sigm[ku][1][u*3] += dsig[3];
	    sigm[ku][1][u*3+1] += dsig[4];
	    sigm[ku][1][u*3+2] += dsig[2];
	    sigm[ku][2][u*3] += dsig[3];
	    sigm[ku][2][u*3+1] += dsig[1];
	    sigm[ku][2][u*3+2] += dsig[5];
	    sigm[ku][3][u*3] += dsig[6];
	    sigm[ku][3][u*3+1] += dsig[7];
	    sigm[ku][4][u*3+1] += dsig[8];
	    sigm[ku][4][u*3+2] += dsig[6];
	    sigm[ku][5][u*3] += dsig[8];
	    sigm[ku][5][u*3+2] += dsig[7];
	  }
	}
	for(int i=0; i<feType->getNumberOfFaces(); i++) {
	  for(int j=0; j<feType->getNumberOfIndicesPerFace(); j++)
	    indices[oj++] = nodeTable[elei(ee,feType->getOmbvIndex(i,j))];
	  indices[oj++] = -1;
	}
      }
      delete feType;
    }
  }
}
