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

using namespace std;
using namespace fmatvec;

namespace MBSimGUI {

  vector<Vec3> rN;
  double (*Ni[20])(double x, double y, double z, int i);
  double (*dNidq[20][3])(double x, double y, double z, int i);

  double N1(double x, double y, double z, int i) {
    return 1./8*(1+rN[i](0)*x)*(1+rN[i](1)*y)*(1+rN[i](2)*z)*(rN[i](0)*x+rN[i](1)*y+rN[i](2)*z-2);
  }

  double N2(double x, double y, double z, int i) {
    return 1./4*(1-x*x)*(1+rN[i](1)*y)*(1+rN[i](2)*z);
  }

  double N3(double x, double y, double z, int i) {
    return 1./4*(1-y*y)*(1+rN[i](2)*z)*(1+rN[i](0)*x);
  }

  double N4(double x, double y, double z, int i) {
    return 1./4*(1-z*z)*(1+rN[i](0)*x)*(1+rN[i](1)*y);
  }

  double dN1dxq(double x, double y, double z, int i) {
    return 1./8*rN[i](0)*(1+rN[i](1)*y)*(1+rN[i](2)*z)*(2*rN[i](0)*x+rN[i](1)*y+rN[i](2)*z-1);
  }

  double dN1dyq(double x, double y, double z, int i) {
    return 1./8*rN[i](1)*(1+rN[i](2)*z)*(1+rN[i](0)*x)*(2*rN[i](1)*y+rN[i](0)*x+rN[i](2)*z-1);
  }

  double dN1dzq(double x, double y, double z, int i) {
    return 1./8*rN[i](2)*(1+rN[i](0)*x)*(1+rN[i](1)*y)*(2*rN[i](2)*z+rN[i](0)*x+rN[i](1)*y-1);
  }

  double dN2dxq(double x, double y, double z, int i) {
    return -1./2*x*(1+rN[i](1)*y)*(1+rN[i](2)*z);
  }

  double dN2dyq(double x, double y, double z, int i) {
    return 1./4*rN[i](1)*(1-x*x)*(1+rN[i](2)*z);
  }

  double dN2dzq(double x, double y, double z, int i) {
    return 1./4*rN[i](2)*(1-x*x)*(1+rN[i](1)*y);
  }

  double dN3dxq(double x, double y, double z, int i) {
    return 1./4*rN[i](0)*(1-y*y)*(1+rN[i](2)*z);
  }

  double dN3dyq(double x, double y, double z, int i) {
    return -1./2*y*(1+rN[i](2)*z)*(1+rN[i](0)*x);
  }

  double dN3dzq(double x, double y, double z, int i) {
    return 1./4*rN[i](2)*(1-y*y)*(1+rN[i](0)*x);
  }

  double dN4dxq(double x, double y, double z, int i) {
    return 1./4*rN[i](0)*(1-z*z)*(1+rN[i](1)*y);
  }

  double dN4dyq(double x, double y, double z, int i) {
    return 1./4*rN[i](1)*(1-z*z)*(1+rN[i](0)*x);
  }

  double dN4dzq(double x, double y, double z, int i) {
    return -1./2*z*(1+rN[i](0)*x)*(1+rN[i](1)*y);
  }

  void FlexibleBodyTool::fe() {
    string str = static_cast<FileWidget*>(static_cast<FiniteElementsPage*>(page(PageFiniteElements))->nodes->getWidget())->getFile(true).toStdString();
    if(!str.empty())
      r <<= readMat(str);

    vector<MatVI> ele;
    str = static_cast<FiniteElementsDataWidget*>(static_cast<ListWidget*>(static_cast<FiniteElementsPage*>(page(PageFiniteElements))->elements->getWidget())->getWidget(0))->getElementsFile().toStdString();
    if(!str.empty())
      ele.emplace_back(readMat(str));

    auto E = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FiniteElementsPage*>(page(PageFiniteElements))->E->getWidget())->getWidget())->getEvalMat()[0][0].toDouble();

    auto nu = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FiniteElementsPage*>(page(PageFiniteElements))->nu->getWidget())->getWidget())->getEvalMat()[0][0].toDouble();

    auto rho = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FiniteElementsPage*>(page(PageFiniteElements))->rho->getWidget())->getWidget())->getEvalMat()[0][0].toDouble();

    for(int i=0; i<8; i++) {
      Ni[i] = &N1;
      dNidq[i][0] = &dN1dxq;
      dNidq[i][1] = &dN1dyq;
      dNidq[i][2] = &dN1dzq;
    }
    for(int i=8; i<15; i+=2) {
      Ni[i] = &N2;
      dNidq[i][0] = &dN2dxq;
      dNidq[i][1] = &dN2dyq;
      dNidq[i][2] = &dN2dzq;
    }
    for(int i=9; i<16; i+=2) {
      Ni[i] = &N3;
      dNidq[i][0] = &dN3dxq;
      dNidq[i][1] = &dN3dyq;
      dNidq[i][2] = &dN3dzq;
    }
    for(int i=16; i<20; i++) {
      Ni[i] = &N4;
      dNidq[i][0] = &dN4dxq;
      dNidq[i][1] = &dN4dyq;
      dNidq[i][2] = &dN4dzq;
    }

    rN.resize(20,Vec3(NONINIT));
    rN[0](0)  = -1;   rN[0](1)  = -1;  rN[0](2)  = -1;
    rN[1](0)  =  1;   rN[1](1)  = -1;  rN[1](2)  = -1;
    rN[2](0)  =  1;   rN[2](1)  =  1;  rN[2](2)  = -1;
    rN[3](0)  = -1;   rN[3](1)  =  1;  rN[3](2)  = -1;
    rN[4](0)  = -1;   rN[4](1)  = -1;  rN[4](2)  =  1;
    rN[5](0)  =  1;   rN[5](1)  = -1;  rN[5](2)  =  1;
    rN[6](0)  =  1;   rN[6](1)  =  1;  rN[6](2)  =  1;
    rN[7](0)  = -1;   rN[7](1)  =  1;  rN[7](2)  =  1;
    rN[8](0)  =  0;   rN[8](1)  = -1;  rN[8](2)  = -1;
    rN[9](0)  =  1;   rN[9](1)  =  0;  rN[9](2)  = -1;
    rN[10](0) =  0;   rN[10](1) =  1;  rN[10](2) = -1;
    rN[11](0) = -1;   rN[11](1) =  0;  rN[11](2) = -1;
    rN[12](0) =  0;   rN[12](1) = -1;  rN[12](2) =  1;
    rN[13](0) =  1;   rN[13](1) =  0;  rN[13](2) =  1;
    rN[14](0) =  0;   rN[14](1) =  1;  rN[14](2) =  1;
    rN[15](0) = -1;   rN[15](1) =  0;  rN[15](2) =  1;
    rN[16](0) = -1;   rN[16](1) = -1;  rN[16](2) =  0;
    rN[17](0) =  1;   rN[17](1) = -1;  rN[17](2) =  0;
    rN[18](0) =  1;   rN[18](1) =  1;  rN[18](2) =  0;
    rN[19](0) = -1;   rN[19](1) =  1;  rN[19](2) =  0;

    Vec3 xi(NONINIT);
    xi(0) = -sqrt(3./5);
    xi(1) = 0;
    xi(2) = sqrt(3./5);

    Vec3 wi(NONINIT);
    wi(0) = 5./9;
    wi(1) = 8./9;
    wi(2) = 5./9;

    map<int,int> nodeCount;
    for(int i=0; i<ele[0].rows(); i++) {
      for(int j=0; j<ele[0].cols(); j++)
	nodeCount[ele[0](i,j)]++;
    }

    ng = 0;
    nN = 0;
    for(const auto & i : nodeCount) {
      if(i.second>0) {
	ng+=3;
	nodeMap[i.first] = nN++;
      }
    }

    nen = 3;

    rPdm.resize(3,Mat3xV(ng));
    Pdm.resize(ng);

    double dK[3][3];
    for(int ee=0; ee<ele[0].rows(); ee++) {
      for(int ii=0; ii<3; ii++) {
	double x = xi(ii);
	for(int jj=0; jj<3; jj++) {
	  double y = xi(jj);
	  for(int kk=0; kk<3; kk++) {
	    double z = xi(kk);
	    SqrMat J(3);
	    Vec r(3);
	    for(int ll=0; ll<20; ll++) {
	      Vec3 r0 = this->r.row(ele[0](ee,ll)-1).T();
	      for(int mm=0; mm<3; mm++) {
		double dN = (*dNidq[ll][mm])(x,y,z,ll);
		for(int nn=0; nn<3; nn++)
		  J(mm,nn) += dN*r0(nn);
	      }
	      r += (*Ni[ll])(x,y,z,ll)*r0;
	    }
	    Vector<Ref,int> ipiv(J.size(),NONINIT);
	    SqrMat LUJ = facLU(J,ipiv);
	    double detJ = J(0,0)*J(1,1)*J(2,2)+J(0,1)*J(1,2)*J(2,0)+J(0,2)*J(1,0)*J(2,1)-J(2,0)*J(1,1)*J(0,2)-J(2,1)*J(1,2)*J(0,0)-J(2,2)*J(1,0)*J(0,1);
	    double wijk = wi(ii)*wi(jj)*wi(kk);
	    double dm = rho*wijk*detJ;
	    double dk = E/(1+nu)*wijk*detJ;
	    m += dm;
	    rdm += dm*r;
	    rrdm += dm*JTJ(r.T());
	    for(int i=0; i<20; i++) {
	      //	      int u = nodeMap[elements(ee,i)];
	      int u = ele[0](ee,i)-1;
	      double Ni_ = (*Ni[i])(x,y,z,i);
	      Vec dN(3,NONINIT);
	      for(int rr=0; rr<3; rr++)
		dN(rr) = (*dNidq[i][rr])(x,y,z,i);
	      Vec dNi = slvLUFac(LUJ,dN,ipiv);
	      for(int i1=0; i1<3; i1++) {
		Pdm(i1,u*3+i1) += dm*Ni_;
		for(int j1=0; j1<3; j1++)
		  rPdm[i1](j1,u*3+j1) += dm*r(i1)*Ni_;
	      }
	      for(int j=i; j<20; j++) {
		//		int v = nodeMap[elements(ee,j)];
		int v = ele[0](ee,j)-1;
		double Nj_ = (*Ni[j])(x,y,z,j);
		Vec dN(3,NONINIT);
		for(int rr=0; rr<3; rr++)
		  dN(rr) = (*dNidq[j][rr])(x,y,z,j);
		Vec dNj = slvLUFac(LUJ,dN,ipiv);
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
	}
      }
    }

    Phis.resize(nN,SparseMat(3,ng,3,NONINIT));
    for(size_t i=0; i<nN; i++) {
      for(int j=0; j<3; j++) {
	Phis[i]()[j] = 1;
	Phis[i].Ip()[j] = j;
	Phis[i].Jp()[j] = 3*i+j;
      }
      Phis[i].Ip()[3] = 3;
    }

    //    for(int i=0; i<nN; i++) {
    //      Phi[i](0,3*i) = 1;
    //      Phi[i](1,3*i+1) = 1;
    //      Phi[i](2,3*i+2) = 1;
    //    }
    //    for(int ee=0; ee<nE; ee++) {
    //      for(int k=0; k<20; k++) {
    //	int ku = nodeMap[elements(ee,k)];
    //	SqrMat J(3);
    //	for(int ll=0; ll<20; ll++) {
    //	  J.add(0,(this->*dNidxq[ll])(rN[k](0),rN[k](1),rN[k](2),ll)*nodes.row(elements(ee,ll)-1));
    //	  J.add(1,(this->*dNidyq[ll])(rN[k](0),rN[k](1),rN[k](2),ll)*nodes.row(elements(ee,ll)-1));
    //	  J.add(2,(this->*dNidzq[ll])(rN[k](0),rN[k](1),rN[k](2),ll)*nodes.row(elements(ee,ll)-1));
    //	}
    //	Vector<Ref,int> ipiv(J.size(),NONINIT);
    //	SqrMat LUJ = facLU(J,ipiv);
    //	for(int i=0; i<20; i++) {
    //	  int u = nodeMap[elements(ee,i)];
    //	  Vec dN(3,NONINIT);
    //	  dN(0) = (this->*dNidxq[i])(rN[k](0),rN[k](1),rN[k](2),i);
    //	  dN(1) = (this->*dNidyq[i])(rN[k](0),rN[k](1),rN[k](2),i);
    //	  dN(2) = (this->*dNidzq[i])(rN[k](0),rN[k](1),rN[k](2),i);
    //	  Vec dNi = slvLUFac(LUJ,dN,ipiv);
    //	  double al = E/(1+nu)/nI(elements(ee,k)-1);
    //	  sigmahel[ku](0,u*3) += al*(1-nu)/(1-2*nu)*dNi(0);
    //	  sigmahel[ku](0,u*3+1) += al*nu/(1-2*nu)*dNi(1);
    //	  sigmahel[ku](0,u*3+2) += al*nu/(1-2*nu)*dNi(2);
    //	  sigmahel[ku](1,u*3) += al*nu/(1-2*nu)*dNi(0);
    //	  sigmahel[ku](1,u*3+1) += al*(1-nu)/(1-2*nu)*dNi(1);
    //	  sigmahel[ku](1,u*3+2) += al*nu/(1-2*nu)*dNi(2);
    //	  sigmahel[ku](2,u*3) += al*nu/(1-2*nu)*dNi(0);
    //	  sigmahel[ku](2,u*3+1) += al*nu/(1-2*nu)*dNi(1);
    //	  sigmahel[ku](2,u*3+2) += al*(1-nu)/(1-2*nu)*dNi(2);
    //	  sigmahel[ku](3,u*3) += al*0.5*dNi(1);
    //	  sigmahel[ku](3,u*3+1) += al*0.5*dNi(0);
    //	  sigmahel[ku](4,u*3+1) += al*0.5*dNi(2);
    //	  sigmahel[ku](4,u*3+2) += al*0.5*dNi(1);
    //	  sigmahel[ku](5,u*3) += al*0.5*dNi(2);
    //	  sigmahel[ku](5,u*3+2) += al*0.5*dNi(0);
    //	}
    //      }
    //    }
  }
}
