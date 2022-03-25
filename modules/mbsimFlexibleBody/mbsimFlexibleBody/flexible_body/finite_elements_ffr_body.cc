/* Copyright (C) 2004-2022 MBSim Development Team
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
#include "finite_elements_ffr_body.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMFLEX, FiniteElementsFfrBody)

  void FiniteElementsFfrBody::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      if(type==unknownElementType)
	throwError("(FiniteElementsFfrBody::init): element type unknown");
      for(int i=0; i<8; i++) {
	Ni[i] = &FiniteElementsFfrBody::N1;
	dNidxq[i] = &FiniteElementsFfrBody::dN1dxq;
	dNidyq[i] = &FiniteElementsFfrBody::dN1dyq;
	dNidzq[i] = &FiniteElementsFfrBody::dN1dzq;
      }
      for(int i=8; i<15; i+=2) {
	Ni[i] = &FiniteElementsFfrBody::N2;
	dNidxq[i] = &FiniteElementsFfrBody::dN2dxq;
	dNidyq[i] = &FiniteElementsFfrBody::dN2dyq;
	dNidzq[i] = &FiniteElementsFfrBody::dN2dzq;
      }
      for(int i=9; i<16; i+=2) {
	Ni[i] = &FiniteElementsFfrBody::N3;
	dNidxq[i] = &FiniteElementsFfrBody::dN3dxq;
	dNidyq[i] = &FiniteElementsFfrBody::dN3dyq;
	dNidzq[i] = &FiniteElementsFfrBody::dN3dzq;
      }
      for(int i=16; i<20; i++) {
	Ni[i] = &FiniteElementsFfrBody::N4;
	dNidxq[i] = &FiniteElementsFfrBody::dN4dxq;
	dNidyq[i] = &FiniteElementsFfrBody::dN4dyq;
	dNidzq[i] = &FiniteElementsFfrBody::dN4dzq;
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

      xi(0) = -sqrt(3./5);
      xi(2) = sqrt(3./5);
      wi(0) = 5./9;
      wi(1) = 8./9;
      wi(2) = 5./9;

      int nE = elements.rows();

      VecV nI(nodes.rows());
      for(int i=0; i<elements.rows(); i++) {
	for(int j=0; j<elements.cols(); j++)
	  nI(elements(i,j)-1) += 1;
      }

      int ng = 0;
      int nN = 0;
      for(int i=0; i<nI.size(); i++) {
	if(nI(i)>0) {
	  ng+=3;
          nodeMap[i+1] = nN++;
	}
      }

      int nr = 0;
      for(int i=0; i<bc.rows(); i++)
	nr += bc(i,2)-bc(i,1)+1;
      int n = ng-nr;

      rPdm.resize(3,Mat3xV(ng));
      PPdm.resize(3,vector<SqrMatV>(3,SqrMatV(ng)));
      Pdm.resize(ng);
      Ke0.resize(ng);
      KrKP.resize(nN,Vec3());
      Phi.resize(nN,Mat3xV(ng));
      Psi.resize(nN,Mat3xV(ng));
      sigmahel.resize(nN,Matrix<General,Fixed<6>,Var,double>(ng));

      KrKP.resize(nN);
      int j=0;
      for(const auto & i : nodeMap)
	KrKP[j++] = nodes.row(i.first-1).T();

      for(int ee=0; ee<nE; ee++) {
	for(int ii=0; ii<3; ii++) {
	  for(int jj=0; jj<3; jj++) {
	    for(int kk=0; kk<3; kk++) {
	      SqrMat J(3);
	      Vec r(3);
	      for(int ll=0; ll<20; ll++) {
		J.add(0,(this->*dNidxq[ll])(xi(ii),xi(jj),xi(kk),ll)*nodes.row(elements(ee,ll)-1));
		J.add(1,(this->*dNidyq[ll])(xi(ii),xi(jj),xi(kk),ll)*nodes.row(elements(ee,ll)-1));
		J.add(2,(this->*dNidzq[ll])(xi(ii),xi(jj),xi(kk),ll)*nodes.row(elements(ee,ll)-1));
		r += (this->*Ni[ll])(xi(ii),xi(jj),xi(kk),ll)*nodes.row(elements(ee,ll)-1).T();
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
		int u = nodeMap[elements(ee,i)];
		double Ni_ = (this->*Ni[i])(xi(ii),xi(jj),xi(kk),i);
		Vec dN(3,NONINIT);
		dN(0) = (this->*dNidxq[i])(xi(ii),xi(jj),xi(kk),i);
		dN(1) = (this->*dNidyq[i])(xi(ii),xi(jj),xi(kk),i);
		dN(2) = (this->*dNidzq[i])(xi(ii),xi(jj),xi(kk),i);
		Vec dNi = slvLUFac(LUJ,dN,ipiv);
		Pdm(0,u*3) += dm*Ni_;
		Pdm(1,u*3+1) = Pdm(0,u*3);
		Pdm(2,u*3+2) = Pdm(0,u*3);
		for(int j=0; j<3; j++) {
		  rPdm[j](0,u*3) += dm*r(j)*Ni_;
		  rPdm[j](1,u*3+1) = rPdm[j](0,u*3);
		  rPdm[j](2,u*3+2) = rPdm[j](0,u*3);
		}
		for(int j=i; j<20; j++) {
		  int v = nodeMap[elements(ee,j)];
		  double Nj_ = (this->*Ni[j])(xi(ii),xi(jj),xi(kk),j);
		  Vec dN(3,NONINIT);
		  dN(0) = (this->*dNidxq[j])(xi(ii),xi(jj),xi(kk),j);
		  dN(1) = (this->*dNidyq[j])(xi(ii),xi(jj),xi(kk),j);
		  dN(2) = (this->*dNidzq[j])(xi(ii),xi(jj),xi(kk),j);
		  Vec dNj = slvLUFac(LUJ,dN,ipiv);
		  double dPPdm = dm*Ni_*Nj_;
		  double dK1 = dk*((1-nu)/(1-2*nu)*dNi(0)*dNj(0)+0.5*(dNi(1)*dNj(1)+dNi(2)*dNj(2)));
		  double dK2 = dk*(nu/(1-2*nu)*dNi(1)*dNj(0)+0.5*dNi(0)*dNj(1));
		  double dK3 = dk*(nu/(1-2*nu)*dNi(2)*dNj(0)+0.5*dNi(0)*dNj(2));
		  double dK4 = dk*(nu/(1-2*nu)*dNi(0)*dNj(1)+0.5*dNi(1)*dNj(0));
		  double dK5 = dk*((1-nu)/(1-2*nu)*dNi(1)*dNj(1)+0.5*(dNi(0)*dNj(0)+dNi(2)*dNj(2)));
		  double dK6 = dk*(nu/(1-2*nu)*dNi(2)*dNj(1)+0.5*dNi(1)*dNj(2));
		  double dK7 = dk*(nu/(1-2*nu)*dNi(0)*dNj(2)+0.5*dNi(2)*dNj(0));
		  double dK8 = dk*(nu/(1-2*nu)*dNi(1)*dNj(2)+0.5*dNi(2)*dNj(1));
		  double dK9 = dk*((1-nu)/(1-2*nu)*dNi(2)*dNj(2)+0.5*(dNi(0)*dNj(0)+dNi(1)*dNj(1)));
		  if(v>=u) {
		    PPdm[0][0](u*3,v*3) += dPPdm;
		    PPdm[0][0](v*3,u*3) = PPdm[0][0](u*3,v*3);
		    PPdm[1][1](u*3+1,v*3+1) = PPdm[0][0](u*3,v*3);
		    PPdm[1][1](v*3+1,u*3+1) = PPdm[0][0](u*3,v*3);
		    PPdm[2][2](u*3+2,v*3+2) = PPdm[0][0](u*3,v*3);
		    PPdm[2][2](v*3+2,u*3+2) = PPdm[0][0](u*3,v*3);
		    PPdm[0][1](u*3,v*3+1) = PPdm[0][0](u*3,v*3);
		    PPdm[0][1](v*3,u*3+1) = PPdm[0][0](u*3,v*3);
		    PPdm[0][2](u*3,v*3+2) = PPdm[0][0](u*3,v*3);
		    PPdm[0][2](v*3,u*3+2) = PPdm[0][0](u*3,v*3);
		    PPdm[1][2](u*3+1,v*3+2) = PPdm[0][0](u*3,v*3);
		    PPdm[1][2](v*3+1,u*3+2) = PPdm[0][0](u*3,v*3);
		    Ke0(u*3,v*3) += dK1;
		    if(v!=u) Ke0(u*3+1,v*3) += dK2;
		    if(v!=u) Ke0(u*3+2,v*3) += dK3;
		    Ke0(u*3,v*3+1) += dK4;
		    Ke0(u*3+1,v*3+1) += dK5;
		    if(v!=u) Ke0(u*3+2,v*3+1) += dK6;
		    Ke0(u*3,v*3+2) += dK7;
		    Ke0(u*3+1,v*3+2) += dK8;
		    Ke0(u*3+2,v*3+2) += dK9;
		  }
		  else {
		    PPdm[0][0](v*3,u*3) += dPPdm;
		    PPdm[0][0](u*3,v*3) = PPdm[0][0](v*3,u*3);
		    PPdm[1][1](v*3+1,u*3+1) = PPdm[0][0](v*3,u*3);
		    PPdm[1][1](u*3+1,v*3+1) = PPdm[0][0](v*3,u*3);
		    PPdm[2][2](v*3+2,u*3+2) = PPdm[0][0](v*3,u*3);
		    PPdm[2][2](u*3+2,v*3+2) = PPdm[0][0](v*3,u*3);
		    PPdm[0][1](v*3+1,u*3) = PPdm[0][0](v*3,u*3);
		    PPdm[0][1](u*3+1,v*3) = PPdm[0][0](v*3,u*3);
		    PPdm[0][2](v*3+2,u*3) = PPdm[0][0](v*3,u*3);
		    PPdm[0][2](u*3+2,v*3) = PPdm[0][0](v*3,u*3);
		    PPdm[1][2](v*3+2,u*3+1) = PPdm[0][0](v*3,u*3);
		    PPdm[1][2](u*3+2,v*3+1) = PPdm[0][0](v*3,u*3);
		    Ke0(v*3,u*3) += dK1;
		    Ke0(v*3,u*3+1) += dK2;
		    Ke0(v*3,u*3+2) += dK3;
		    Ke0(v*3+1,u*3) += dK4;
		    Ke0(v*3+1,u*3+1) += dK5;
		    Ke0(v*3+1,u*3+2) += dK6;
		    Ke0(v*3+2,u*3) += dK7;
		    Ke0(v*3+2,u*3+1) += dK8;
		    Ke0(v*3+2,u*3+2) += dK9;
		  }
		}
	      }
	    }
	  }
	}
      }
      PPdm[1][0] = PPdm[0][1].T();
      PPdm[2][0] = PPdm[0][2].T();
      PPdm[2][1] = PPdm[1][2].T();

      vector<int> c;
      for(int i=0; i<bc.rows(); i++) {
	for(int j=(int)bc(i,1); j<=(int)bc(i,2); j++)
	  c.push_back(nodeMap[bc(i,0)]*3+j);
      }
      sort(c.begin(), c.end());

      size_t h=0;
      Indices IF;
      Indices IX;
      for(int i=0; i<ng; i++) {
	if(h<c.size() and i==c[h]) {
	  h++;
	  IX.add(i);
	}
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

      for(int ee=0; ee<nE; ee++) {
	for(int k=0; k<20; k++) {
	  int ku = nodeMap[elements(ee,k)];
	  SqrMat J(3);
	  for(int ll=0; ll<20; ll++) {
	    J.add(0,(this->*dNidxq[ll])(rN[k](0),rN[k](1),rN[k](2),ll)*nodes.row(elements(ee,ll)-1));
	    J.add(1,(this->*dNidyq[ll])(rN[k](0),rN[k](1),rN[k](2),ll)*nodes.row(elements(ee,ll)-1));
	    J.add(2,(this->*dNidzq[ll])(rN[k](0),rN[k](1),rN[k](2),ll)*nodes.row(elements(ee,ll)-1));
	  }
	  Vector<Ref,int> ipiv(J.size(),NONINIT);
	  SqrMat LUJ = facLU(J,ipiv);
	  for(int i=0; i<20; i++) {
	    int u = nodeMap[elements(ee,i)];
	    Vec dN(3,NONINIT);
	    dN(0) = (this->*dNidxq[i])(rN[k](0),rN[k](1),rN[k](2),i);
	    dN(1) = (this->*dNidyq[i])(rN[k](0),rN[k](1),rN[k](2),i);
	    dN(2) = (this->*dNidzq[i])(rN[k](0),rN[k](1),rN[k](2),i);
	    Vec dNi = slvLUFac(LUJ,dN,ipiv);
	    double al = E/(1+nu)/nI(elements(ee,k)-1);
	    sigmahel[ku](0,u*3) += al*(1-nu)/(1-2*nu)*dNi(0);
	    sigmahel[ku](0,u*3+1) += al*nu/(1-2*nu)*dNi(1);
	    sigmahel[ku](0,u*3+2) += al*nu/(1-2*nu)*dNi(2);
	    sigmahel[ku](1,u*3) += al*nu/(1-2*nu)*dNi(0);
	    sigmahel[ku](1,u*3+1) += al*(1-nu)/(1-2*nu)*dNi(1);
	    sigmahel[ku](1,u*3+2) += al*nu/(1-2*nu)*dNi(2);
	    sigmahel[ku](2,u*3) += al*nu/(1-2*nu)*dNi(0);
	    sigmahel[ku](2,u*3+1) += al*nu/(1-2*nu)*dNi(1);
	    sigmahel[ku](2,u*3+2) += al*(1-nu)/(1-2*nu)*dNi(2);
	    sigmahel[ku](3,u*3) += al*0.5*dNi(1);
	    sigmahel[ku](3,u*3+1) += al*0.5*dNi(0);
	    sigmahel[ku](4,u*3+1) += al*0.5*dNi(2);
	    sigmahel[ku](4,u*3+2) += al*0.5*dNi(1);
	    sigmahel[ku](5,u*3) += al*0.5*dNi(2);
	    sigmahel[ku](5,u*3+2) += al*0.5*dNi(0);
	  }
	}
      }
      for(int i=0; i<nN; i++) {
	Phi[i](0,3*i) = 1;
	Phi[i](1,3*i+1) = 1;
	Phi[i](2,3*i+2) = 1;
	Phi[i] <<= Phi[i](I3,IF);
	Psi[i] <<= Psi[i](I3,IF);
	sigmahel[i] <<= sigmahel[i](I6,IF);
      }
      c.clear();
      for(int i=0; i<inodes.size(); i++) {
	int j1=3;
	int j2=-1;
	for(int k=0; k<bc.rows(); k++) {
	  if(inodes(i)==(int)bc(k,0)) {
	    j1=bc(k,1);
	    j2=bc(k,2);
	  }
	}
	for(int j=0; j<3; j++)
	  if(j<j1 or j>j2) c.push_back(nodeMap[inodes(i)]*3+j);
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
	    throwError(string("(FiniteElementsFfrBody::init): node numbers do not match, must be within the range [1,") + to_string(imod.size()) + "]");
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
	    throwError(string("(FiniteElementsFfrBody::init): node numbers do not match, must be within the range [1,") + to_string(imod.size()) + "]");
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
	if(ombvBody->getVisualization()==OpenMBVFiniteElementsBody::faces) {
	  // visualization
	  vector<int> ombvIndices(5*6*elements.rows());
	  int j = 0;
	  for(int i=0; i<elements.rows(); i++) {
	    ombvIndices[j++] = nodeMap[elements(i,3)];
	    ombvIndices[j++] = nodeMap[elements(i,2)];
	    ombvIndices[j++] = nodeMap[elements(i,1)];
	    ombvIndices[j++] = nodeMap[elements(i,0)];
	    ombvIndices[j++] = -1;
	    ombvIndices[j++] = nodeMap[elements(i,4)];
	    ombvIndices[j++] = nodeMap[elements(i,5)];
	    ombvIndices[j++] = nodeMap[elements(i,6)];
	    ombvIndices[j++] = nodeMap[elements(i,7)];
	    ombvIndices[j++] = -1;
	    ombvIndices[j++] = nodeMap[elements(i,1)];
	    ombvIndices[j++] = nodeMap[elements(i,2)];
	    ombvIndices[j++] = nodeMap[elements(i,6)];
	    ombvIndices[j++] = nodeMap[elements(i,5)];
	    ombvIndices[j++] = -1;
	    ombvIndices[j++] = nodeMap[elements(i,2)];
	    ombvIndices[j++] = nodeMap[elements(i,3)];
	    ombvIndices[j++] = nodeMap[elements(i,7)];
	    ombvIndices[j++] = nodeMap[elements(i,6)];
	    ombvIndices[j++] = -1;
	    ombvIndices[j++] = nodeMap[elements(i,4)];
	    ombvIndices[j++] = nodeMap[elements(i,7)];
	    ombvIndices[j++] = nodeMap[elements(i,3)];
	    ombvIndices[j++] = nodeMap[elements(i,0)];
	    ombvIndices[j++] = -1;
	    ombvIndices[j++] = nodeMap[elements(i,0)];
	    ombvIndices[j++] = nodeMap[elements(i,1)];
	    ombvIndices[j++] = nodeMap[elements(i,5)];
	    ombvIndices[j++] = nodeMap[elements(i,4)];
	    ombvIndices[j++] = -1;
	  }
	  static_pointer_cast<OpenMBV::DynamicIndexedFaceSet>(flexbody)->setIndices(ombvIndices);
	}
	ombvColorRepresentation = static_cast<OpenMBVFlexibleBody::ColorRepresentation>(ombvBody->getColorRepresentation());
      }
    }
    GenericFlexibleFfrBody::init(stage, config);
  }

  void FiniteElementsFfrBody::initializeUsingXML(DOMElement *element) {
    GenericFlexibleFfrBody::initializeUsingXML(element);
    DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"youngsModulus");
    setYoungsModulus(MBXMLUtils::E(e)->getText<double>());
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"poissonsRatio");
    setPoissonsRatio(MBXMLUtils::E(e)->getText<double>());
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"density");
    setDensity(MBXMLUtils::E(e)->getText<double>());
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodes");
    setNodes(MBXMLUtils::E(e)->getText<MatVx3>());
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"elements");
    setElements(MBXMLUtils::E(e)->getText<MatVI>());
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"elementType");
    string typeStr=string(X()%MBXMLUtils::E(e)->getFirstTextChild()->getData()).substr(1,string(X()%MBXMLUtils::E(e)->getFirstTextChild()->getData()).length()-2);
    if(typeStr=="C3D20") type=C3D20;
    else type=unknownElementType;
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"proportionalDamping");
    if(e) setProportionalDamping(MBXMLUtils::E(e)->getText<Vec>());
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"boundaryConditions");
    if(e) {
      setBoundaryConditions(MBXMLUtils::E(e)->getText<MatVx3>());
      for(int i=0; i<bc.rows(); i++) {
	for(int j=1; j<bc.cols(); j++)
	  bc(i,j)--;
      }
    }
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"interfaceNodeNumbers");
    if(e) setInterfaceNodeNumbers(MBXMLUtils::E(e)->getText<VecVI>());
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"normalModeNumbers");
    if(e) setNormalModeNumbers(MBXMLUtils::E(e)->getText<VecVI>());
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"fixedBoundaryNormalModes");
    if(e) setFixedBoundaryNormalModes(MBXMLUtils::E(e)->getText<bool>());
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"enableOpenMBV");
    if(e) {
      ombvBody = shared_ptr<OpenMBVFiniteElementsBody>(new OpenMBVFiniteElementsBody);
      ombvBody->initializeUsingXML(e);
    }
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"plotNodeNumbers");
    if(e) setPlotNodeNumbers(MBXMLUtils::E(e)->getText<VecVI>());
  }

  double FiniteElementsFfrBody::N1(double x, double y, double z, int i) {
    return 1./8*(1+rN[i](0)*x)*(1+rN[i](1)*y)*(1+rN[i](2)*z)*(rN[i](0)*x+rN[i](1)*y+rN[i](2)*z-2);
  }

  double FiniteElementsFfrBody::N2(double x, double y, double z, int i) {
    return 1./4*(1-x*x)*(1+rN[i](1)*y)*(1+rN[i](2)*z);
  }

  double FiniteElementsFfrBody::N3(double x, double y, double z, int i) {
    return 1./4*(1-y*y)*(1+rN[i](2)*z)*(1+rN[i](0)*x); 
  }

  double FiniteElementsFfrBody::N4(double x, double y, double z, int i) {
    return 1./4*(1-z*z)*(1+rN[i](0)*x)*(1+rN[i](1)*y);
  }

  double FiniteElementsFfrBody::dN1dxq(double x, double y, double z, int i) {
    return 1./8*rN[i](0)*(1+rN[i](1)*y)*(1+rN[i](2)*z)*(2*rN[i](0)*x+rN[i](1)*y+rN[i](2)*z-1);
  }

  double FiniteElementsFfrBody::dN1dyq(double x, double y, double z, int i) {
    return 1./8*rN[i](1)*(1+rN[i](2)*z)*(1+rN[i](0)*x)*(2*rN[i](1)*y+rN[i](0)*x+rN[i](2)*z-1);
  }

  double FiniteElementsFfrBody::dN1dzq(double x, double y, double z, int i) {
    return 1./8*rN[i](2)*(1+rN[i](0)*x)*(1+rN[i](1)*y)*(2*rN[i](2)*z+rN[i](0)*x+rN[i](1)*y-1);
  }

  double FiniteElementsFfrBody::dN2dxq(double x, double y, double z, int i) {
    return -1./2*x*(1+rN[i](1)*y)*(1+rN[i](2)*z);
  }

  double FiniteElementsFfrBody::dN2dyq(double x, double y, double z, int i) {
    return 1./4*rN[i](1)*(1-x*x)*(1+rN[i](2)*z);
  }

  double FiniteElementsFfrBody::dN2dzq(double x, double y, double z, int i) {
    return 1./4*rN[i](2)*(1-x*x)*(1+rN[i](1)*y);
  }

  double FiniteElementsFfrBody::dN3dxq(double x, double y, double z, int i) {
    return 1./4*rN[i](0)*(1-y*y)*(1+rN[i](2)*z);
  }

  double FiniteElementsFfrBody::dN3dyq(double x, double y, double z, int i) {
    return -1./2*y*(1+rN[i](2)*z)*(1+rN[i](0)*x);
  }

  double FiniteElementsFfrBody::dN3dzq(double x, double y, double z, int i) {
    return 1./4*rN[i](2)*(1-y*y)*(1+rN[i](0)*x);
  }

  double FiniteElementsFfrBody::dN4dxq(double x, double y, double z, int i) {
    return 1./4*rN[i](0)*(1-z*z)*(1+rN[i](1)*y);
  }

  double FiniteElementsFfrBody::dN4dyq(double x, double y, double z, int i) {
    return 1./4*rN[i](1)*(1-z*z)*(1+rN[i](0)*x);
  }

  double FiniteElementsFfrBody::dN4dzq(double x, double y, double z, int i) {
    return -1./2*z*(1+rN[i](0)*x)*(1+rN[i](1)*y);
  }

}
