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
      for(size_t i=0; i<types.size(); i++) {
	if(types[i]==unknownElementType)
	  throwError("(FiniteElementsFfrBody::init): element type number " + to_string(i) + " unknown");
      }

      for(int i=0; i<8; i++) {
	Ni[i] = &FiniteElementsFfrBody::N1;
	dNidq[i][0] = &FiniteElementsFfrBody::dN1dxq;
	dNidq[i][1] = &FiniteElementsFfrBody::dN1dyq;
	dNidq[i][2] = &FiniteElementsFfrBody::dN1dzq;
      }
      for(int i=8; i<15; i+=2) {
	Ni[i] = &FiniteElementsFfrBody::N2;
	dNidq[i][0] = &FiniteElementsFfrBody::dN2dxq;
	dNidq[i][1] = &FiniteElementsFfrBody::dN2dyq;
	dNidq[i][2] = &FiniteElementsFfrBody::dN2dzq;
      }
      for(int i=9; i<16; i+=2) {
	Ni[i] = &FiniteElementsFfrBody::N3;
	dNidq[i][0] = &FiniteElementsFfrBody::dN3dxq;
	dNidq[i][1] = &FiniteElementsFfrBody::dN3dyq;
	dNidq[i][2] = &FiniteElementsFfrBody::dN3dzq;
      }
      for(int i=16; i<20; i++) {
	Ni[i] = &FiniteElementsFfrBody::N4;
	dNidq[i][0] = &FiniteElementsFfrBody::dN4dxq;
	dNidq[i][1] = &FiniteElementsFfrBody::dN4dyq;
	dNidq[i][2] = &FiniteElementsFfrBody::dN4dzq;
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
      xi(1) = 0;
      xi(2) = sqrt(3./5);

      wi(0) = 5./9;
      wi(1) = 8./9;
      wi(2) = 5./9;

      if(nodes.cols()==4) {
	for(int i=0; i<nodes.rows(); i++)
	  nodalPos[nodes(i,0)] = nodes.row(i)(RangeV(1,3)).T();
      }
      else if(nodes.cols()==3) {
	for(int i=0; i<nodes.rows(); i++)
	  nodalPos[i+1] = nodes.row(i).T();
      }
      else
	throwError("(FiniteElementsFfrBody::init): number of columns in nodes does not match, must be 3 or 4");

      if(types.size() != elements.size())
	throwError("(FiniteElementsFfrBody::init): number of element types (" + to_string(types.size()) + ") must equal number of element data sets (" + to_string(elements.size()) + ")");
      for(size_t e=0, j=0; e<elements.size(); e++) {
	if(types[e] != C3D20)
	  throwError("(FiniteElementsFfrBody::init): only elements of type C3D20 are currently supported");
	if(elements[e].cols()==21) {
	  for(int i=0; i<elements[e].rows(); i++) {
	    type[elements[e](i,0)] = types[e];
	    ele[elements[e](i,0)] <<= elements[e].row(i)(RangeV(1,20)).T();
	  }
	}
	else if(elements[e].cols()==20) {
	  for(int i=0; i<elements[e].rows(); i++, j++) {
	    type[j+1] = types[e];
	    ele[j+1] <<= elements[e].row(i).T();
	  }
	}
	else
	  throwError("(FiniteElementsFfrBody::init): number of columns in elements does not match, must be " + to_string(20) + " or " + to_string(21));
      }

      map<int,int> nodeCount;
      for(const auto & i : ele) {
	for(int j=0; j<i.second.size(); j++)
	  nodeCount[i.second(j)]++;
      }

      int ng = 0;
      int nN = 0;
      for(const auto & i : nodeCount) {
	if(i.second>0) {
	  ng+=3;
          nodeMap[i.first] = nN++;
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
      for(const auto & i : nodeMap)
	KrKP[i.second] = nodalPos[i.first];

      for(const auto & ee : ele) {
	for(int ii=0; ii<3; ii++) {
	  double x = xi(ii);
	  for(int jj=0; jj<3; jj++) {
	    double y = xi(jj);
	    for(int kk=0; kk<3; kk++) {
	      double z = xi(kk);
	      SqrMat J(3);
	      Vec r(3);
	      for(int ll=0; ll<20; ll++) {
		Vec3 r0 = nodalPos[ee.second(ll)];
		for(int rr=0; rr<3; rr++)
		  J.add(rr,(this->*dNidq[ll][rr])(x,y,z,ll)*r0);
		r += (this->*Ni[ll])(x,y,z,ll)*r0;
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
		int u = nodeMap[ee.second(i)];
		double Ni_ = (this->*Ni[i])(x,y,z,i);
		Vec dN(3,NONINIT);
		for(int rr=0; rr<3; rr++)
		  dN(rr) = (this->*dNidq[i][rr])(x,y,z,i);
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
		  int v = nodeMap[ee.second(j)];
		  double Nj_ = (this->*Ni[j])(x,y,z,j);
		  Vec dN(3,NONINIT);
		  for(int rr=0; rr<3; rr++)
		    dN(rr) = (this->*dNidq[j][rr])(x,y,z,j);
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

      for(const auto & ee : ele) {
	for(int k=0; k<20; k++) {
	  double x = rN[k](0);
	  double y = rN[k](1);
	  double z = rN[k](2);
	  int ku = nodeMap[ee.second(k)];
	  SqrMat J(3);
	  for(int ll=0; ll<20; ll++) {
	    Vec3 r0 = nodalPos[ee.second(ll)];
	    for(int rr=0; rr<3; rr++)
	      J.add(rr,(this->*dNidq[ll][rr])(x,y,z,ll)*r0);
	  }
	  Vector<Ref,int> ipiv(J.size(),NONINIT);
	  SqrMat LUJ = facLU(J,ipiv);
	  for(int i=0; i<20; i++) {
	    int u = nodeMap[ee.second(i)];
	    Vec dN(3,NONINIT);
	    for(int rr=0; rr<3; rr++)
	      dN(rr) = (this->*dNidq[i][rr])(x,y,z,i);
	    Vec dNi = slvLUFac(LUJ,dN,ipiv);
	    double al = E/(1+nu)/nodeCount[ee.second(k)];
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
	  vector<int> ombvIndices(5*6*ele.size());
	  int j = 0;
	  for(const auto & i : ele) {
	    ombvIndices[j++] = nodeMap[i.second(3)];
	    ombvIndices[j++] = nodeMap[i.second(2)];
	    ombvIndices[j++] = nodeMap[i.second(1)];
	    ombvIndices[j++] = nodeMap[i.second(0)];
	    ombvIndices[j++] = -1;
	    ombvIndices[j++] = nodeMap[i.second(4)];
	    ombvIndices[j++] = nodeMap[i.second(5)];
	    ombvIndices[j++] = nodeMap[i.second(6)];
	    ombvIndices[j++] = nodeMap[i.second(7)];
	    ombvIndices[j++] = -1;
	    ombvIndices[j++] = nodeMap[i.second(1)];
	    ombvIndices[j++] = nodeMap[i.second(2)];
	    ombvIndices[j++] = nodeMap[i.second(6)];
	    ombvIndices[j++] = nodeMap[i.second(5)];
	    ombvIndices[j++] = -1;
	    ombvIndices[j++] = nodeMap[i.second(2)];
	    ombvIndices[j++] = nodeMap[i.second(3)];
	    ombvIndices[j++] = nodeMap[i.second(7)];
	    ombvIndices[j++] = nodeMap[i.second(6)];
	    ombvIndices[j++] = -1;
	    ombvIndices[j++] = nodeMap[i.second(4)];
	    ombvIndices[j++] = nodeMap[i.second(7)];
	    ombvIndices[j++] = nodeMap[i.second(3)];
	    ombvIndices[j++] = nodeMap[i.second(0)];
	    ombvIndices[j++] = -1;
	    ombvIndices[j++] = nodeMap[i.second(0)];
	    ombvIndices[j++] = nodeMap[i.second(1)];
	    ombvIndices[j++] = nodeMap[i.second(5)];
	    ombvIndices[j++] = nodeMap[i.second(4)];
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
    setNodes(MBXMLUtils::E(e)->getText<MatV>());
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"elementType");
    while(e && MBXMLUtils::E(e)->getTagName()==MBSIMFLEX%"elementType") {
      ElementType type;
      string typeStr=string(X()%MBXMLUtils::E(e)->getFirstTextChild()->getData()).substr(1,string(X()%MBXMLUtils::E(e)->getFirstTextChild()->getData()).length()-2);
      if(typeStr=="C3D20") type=C3D20;
      else type=unknownElementType;
      addElementType(type);
      e=e->getNextElementSibling();
      if(MBXMLUtils::E(e)->getTagName()==MBSIMFLEX%"elements") addElements(MBXMLUtils::E(e)->getText<MatVI>());
      e=e->getNextElementSibling();
    }
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

  map<int,double> FiniteElementsFfrBody::getWeightingFactors(const VecVI &elesel, int faceNum) {
    map<int,double> weights;
    double A = 0;
    for(int e=0; e<elesel.size(); e++) {
      Vec4 val(NONINIT);
      val(2) = -1;
      val(3) = 1;
      vector<Indices> faceInd(6,Indices(8));
      vector<vector<int>> ind1(6,vector<int>(3)), ind2(6,vector<int>(2));
      faceInd[0] = {0,1,2,3,8,9,10,11};   ind1[0] = {0,1,2}; ind2[0] = {0,1};
      faceInd[1] = {4,5,6,7,12,13,14,15}; ind1[1] = {0,1,3}; ind2[1] = {0,1};
      faceInd[2] = {0,1,4,5,8,12,16,17};  ind1[2] = {0,2,1}; ind2[2] = {0,2};
      faceInd[3] = {1,2,5,6,9,13,17,18};  ind1[3] = {3,0,1}; ind2[3] = {1,2};
      faceInd[4] = {2,3,6,7,10,14,18,19}; ind1[4] = {0,3,1}; ind2[4] = {0,2};
      faceInd[5] = {0,3,4,7,11,15,16,19}; ind1[5] = {2,0,1}; ind2[5] = {1,2};
      int eleNum = elesel(e);
      VecVI eleNodes = ele[eleNum];
      VecVI faceNodes = eleNodes(faceInd[faceNum-1]);
      for(int ii=0; ii<3; ii++) {
	val(0) = xi(ii);
	for(int jj=0; jj<3; jj++) {
	  val(1) = xi(jj);
	  double x = val(ind1[faceNum-1][0]);
	  double y = val(ind1[faceNum-1][1]);
	  double z = val(ind1[faceNum-1][2]);
	  SqrMat J(2);
	  for(int ll=0; ll<faceNodes.size(); ll++) {
	    int fn = faceNodes(ll);
	    int ni = faceInd[faceNum-1][ll];
	    Vec3 r0 = nodalPos[fn];
	    int i1 = ind2[faceNum-1][0];
	    int i2 = ind2[faceNum-1][1];
	    J(0,0) += (this->*dNidq[ni][i1])(x,y,z,ni)*r0(i1);
	    J(0,1) += (this->*dNidq[ni][i2])(x,y,z,ni)*r0(i1);
	    J(1,0) += (this->*dNidq[ni][i1])(x,y,z,ni)*r0(i2);
	    J(1,1) += (this->*dNidq[ni][i2])(x,y,z,ni)*r0(i2);
	  }
	  double detJ = J(0,0)*J(1,1)-J(0,1)*J(1,0);
	  double wij = wi(ii)*wi(jj);
	  double dA = wij*detJ;
	  A += dA;
	  for(int i=0; i<faceNodes.size(); i++) {
	    int fn = faceNodes(i);
	    int ni = faceInd[faceNum-1][i];
	    double Ni_ = (this->*Ni[ni])(x,y,z,ni);
	    weights[fn] += dA*Ni_;
	  }
	}
      }
    }

    for(auto & i : weights)
      i.second /= A;

    return weights;
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
