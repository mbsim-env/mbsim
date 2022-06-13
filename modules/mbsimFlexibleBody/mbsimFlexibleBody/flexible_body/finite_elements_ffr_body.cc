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
#include <fmatvec/sparse_linear_algebra_double.h>

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMFLEX, FiniteElementsFfrBody)

  SymSparseMat sparseSub(const SymSparseMat &K, map<int,VecVI> &dofN, map<int,map<int,Matrix<General,Fixed<3>,Fixed<3>,int>>> &linksK, const VecVI &inodes) {
    int nzs = 0;
    int nrows = 0;
    for(auto & i : linksK) {
      VecVI &ai = dofN[i.first];
      for(int ii=0; ii<3; ii++) {
	if(ai(ii)>-1) {
	  for(auto & j : i.second) {
	    VecVI &aj = dofN[j.first];
	    int js=0;
	    if(i.first==j.first)
	      js=ii;
	    for(int jj=js; jj<3; jj++) {
	      if(aj(jj)>-1)
		nzs++;
	    }
	  }
	  nrows++;
	}
      }
    }
    SymSparseMat B(nrows,nzs,NONINIT);
    nzs = 0;
    nrows = 0;
    int kk = 0;
    B.Ip()[0]=0;
    for(auto & i : linksK) {
      VecVI &ai = dofN[i.first];
      for(int ii=0; ii<3; ii++) {
	if(ai(ii)>-1) {
	  for(auto & j : i.second) {
	    Matrix<General,Fixed<3>,Fixed<3>,int> &A = j.second;
	    VecVI &aj = dofN[j.first];
	    int js=0;
	    if(i.first==j.first)
	      js=ii;
	    for(int jj=js; jj<3; jj++) {
	      if(aj(jj)>-1) {
		kk++;
		B.Jp()[nzs] = aj.e(jj);
		B()[nzs++] = K()[A(ii,jj)];
	      }
	    }
	  }
	  B.Ip()[++nrows]=kk;
	}
      }
    }
    return B;
  }

  Mat denseSub(const SymSparseMat &K, map<int,VecVI> &dofN, map<int,VecVI> &dofH, map<int,map<int,Matrix<General,Fixed<3>,Fixed<3>,int>>> &linksK, const VecVI &inodes, const VecVI &jnodes) {
    int nrows = 0;
    int ncols = 0;
    for(int i=0; i<inodes.size(); i++) {
      VecVI &ai = dofN[inodes(i)];
      for(int ii=0; ii<3; ii++) {
	if(ai(ii)>-1)
	  nrows++;
      }
    }
    for(int i=0; i<jnodes.size(); i++) {
      VecVI &ai = dofH[jnodes(i)];
      for(int ii=0; ii<3; ii++) {
	if(ai(ii)>-1)
	  ncols++;
      }
    }
    Mat B(nrows,ncols);
    for(int i=0; i<inodes.size(); i++) {
      VecVI &ai = dofN[inodes(i)];
      for(int ii=0; ii<3; ii++) {
	if(ai(ii)>-1) {
	  for(int j=0; j<jnodes.size(); j++) {
	    VecVI &aj = dofH[jnodes(j)];
	    if(jnodes(j)>=inodes(i)) {
	      auto it = linksK[inodes(i)].find(jnodes(j));
	      if(it!=linksK[inodes(i)].end()) {
		Matrix<General,Fixed<3>,Fixed<3>,int> &A = linksK[inodes(i)][jnodes(j)];
		for(int jj=0; jj<3; jj++) {
		  if(aj(jj)>-1)
		    B(ai(ii),aj(jj)) = K()[A(ii,jj)];
		}
	      }
	    }
	    else {
	      auto it = linksK[jnodes(j)].find(inodes(i));
	      if(it!=linksK[jnodes(j)].end()) {
	      Matrix<General,Fixed<3>,Fixed<3>,int> &A = linksK[jnodes(j)][inodes(i)];
	      for(int jj=0; jj<3; jj++) {
		if(aj(jj)>-1)
		  B(ai(ii),aj(jj)) = K()[A(jj,ii)];
	      }
	    }
	    }
	  }
	}
      }
    }
    return B;
  }

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

      rN.resize(20,Vec3());
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
      if(bnodes.size() != dof.size())
	throwError("(FiniteElementsFfrBody::init): number of boundary nodes (" + to_string(bnodes.size()) + ") must equal number of degrees of freedom (" + to_string(dof.size()) + ")");
      for(size_t i=0; i<bnodes.size(); i++) {
	for(int j=0; j<bnodes[i].size(); j++) {
	  int node = bnodes[i](j);
	  auto it = nodeMap.find(node);
	  if(it!=nodeMap.end()) {
	    bc[node].resize(3);
	    for(int k=0; k<dof[i].size(); k++) {
	      if(dof[i](k)<0 or dof[i](k)>2)
		throwError("(FiniteElementsFfrBody::init): degrees of freedom of boundary node number (" + to_string(i) + ") must be within the range [0,3]");
	      bc[node](dof[i](k)) = 1;
	    }
	  }
	}
      }
      for(const auto & i : bc) {
	for(int j=0; j<i.second.size(); j++)
	  nr += i.second(j);
      }
      int n = ng-nr;

      VecVI nnodes;
      if(inodes.size()) {
	sort(inodes.begin(),inodes.end());
	nnodes.resize(nN-inodes.size(),NONINIT);
	int k=0, kk=0;
	for(const auto & i : nodeMap) {
	  if(k<inodes.size() and i.first==inodes(k))
	    k++;
	  else
	    nnodes(kk++) = i.first;
	}
      }

      map<int,VecVI> dofA;
      for(const auto & i : nodeMap) {
	VecVI d = bc[i.first];
	if(d.size())
	  dofA[i.first] <<= d;
	else
	  dofA[i.first] <<= VecVI(3,INIT,0);
      }
      int k=0;
      for(auto & i : dofA) {
	for(int j=0; j<3; j++) {
	  if(i.second(j)<1)
	    i.second(j) = k++;
	  else
	    i.second(j) = -1;
	}
      }

      map<int,VecVI> dofN = dofA;
      for(int i=0; i<inodes.size(); i++)
	dofN[inodes(i)].init(-1);
      k=0;
      for(auto & i : dofN) {
	for(int j=0; j<3; j++) {
	  if(i.second(j)!=-1)
	    i.second(j) = k++;
	}
      }

      map<int,VecVI> dofH = dofA;
      for(int i=0; i<nnodes.size(); i++)
	dofH[nnodes(i)].init(-1);
      k=0;
      for(auto & i : dofH) {
	for(int j=0; j<3; j++) {
	  if(i.second(j)!=-1)
	    i.second(j) = k++;
	}
      }

      map<int,map<int,Matrix<General,Fixed<3>,Fixed<3>,int>>> linksK;
      map<int,map<int,Matrix<General,Fixed<3>,Fixed<3>,int>>> linksP;
      map<int,map<int,RowVector<Fixed<3>,int>>> linksS;
      for(const auto & ee : ele) {
	for(int i=0; i<20; i++) {
	  int nri = ee.second(i);
	  for(int j=0; j<i; j++) {
	    int nrj = ee.second(j);
	    linksP[nri][nrj].init(-1);
	    linksS[nri][nrj].init(-1);
	  }
	  for(int j=i; j<20; j++) {
	    int nrj = ee.second(j);
	    linksP[nri][nrj].init(-1);
	    linksS[nri][nrj].init(-1);
	    if(nrj>=nri)
	      linksK[nri][nrj].init(-1);
	    else
	      linksK[nrj][nri].init(-1);
	  }
	}
      }
      int nzsK = 0;
      for(auto & i : linksK) {
	VecVI &ai = dofA[i.first];
	for(int ii=0; ii<3; ii++) {
	  if(ai(ii)>-1) {
	    for(auto & j : i.second) {
	      VecVI &aj = dofA[j.first];
	      int js=0;
	      if(i.first==j.first)
		js=ii;
	      for(int jj=js; jj<3; jj++) {
		if(aj(jj)>-1)
		  j.second.e(ii,jj) = nzsK++;
	      }
	    }
	  }
	}
      }
      int nzsP = 0;
      for(auto & i : linksP) {
	VecVI &ai = dofA[i.first];
	for(int ii=0; ii<3; ii++) {
	  if(ai(ii)>-1) {
	    for(auto & j : i.second) {
	      VecVI &aj = dofA[j.first];
	      for(int jj=0; jj<3; jj++) {
		if(aj(jj)>-1)
		  j.second.e(ii,jj) = nzsP++;
	      }
	    }
	  }
	}
      }

      vector<int> nzsS(nN);
      for(auto & i : linksS) {
	int u = nodeMap[i.first];
	nzsS[u] = 0;
	for(auto & j : i.second) {
	  VecVI &aj = dofA[j.first];
	  for(int jj=0; jj<3; jj++) {
	    if(aj(jj)>-1)
	      j.second.e(jj) = nzsS[u]++;
	  }
	}
      }

      int *JpK = new int[nzsK];
      int *IpK = new int[n+1];
      k=0;
      int kk=0;
      IpK[0]=0;
      for(auto & i : linksK) {
	VecVI &ai = dofA[i.first];
	for(int ii=0; ii<3; ii++) {
	  if(ai(ii)>-1) {
	    for(auto & j : i.second) {
	      VecVI &aj = dofA[j.first];
	      int js=0;
	      if(i.first==j.first)
		js=ii;
	      for(int jj=js; jj<3; jj++) {
		if(aj(jj)>-1) {
		  kk++;
		  JpK[j.second.e(ii,jj)] = aj.e(jj);
		}
	      }
	    }
	    IpK[++k]=kk;
	  }
	}
      }

      k=0;
      kk=0;
      int *JpP = new int[nzsP];
      int *IpP = new int[n+1];
      IpP[0]=0;
      for(auto & i : linksP) {
	VecVI &ai = dofA[i.first];
	for(int ii=0; ii<3; ii++) {
	  if(ai(ii)>-1) {
	    for(auto & j : i.second) {
	      VecVI &aj = dofA[j.first];
	      for(int jj=0; jj<3; jj++) {
		if(aj(jj)>-1) {
		  kk++;
		  JpP[j.second.e(ii,jj)] = aj.e(jj);
		}
	      }
	    }
	    IpP[++k]=kk;
	  }
	}
      }

      k=0;
      kk=0;
      vector<int*> JpS(nN);
      vector<int*> IpS(nN);
      for(auto & i : linksS) {
	int u = nodeMap[i.first];
	JpS[u] = new int[nzsS[u]];
	IpS[u] = new int[2];
	IpS[u][0]=0;
	for(auto & j : i.second) {
	  VecVI &aj = dofA[j.first];
	  for(int jj=0; jj<3; jj++) {
	    if(aj(jj)>-1)
	      JpS[u][j.second.e(jj)] = aj.e(jj);
	  }
	}
	IpS[u][1]=nzsS[u];
      }

      SymSparseMat Ke0s(n,nzsK,IpK,JpK);
      vector<SymSparseMat> PPdms1(3,SymSparseMat(n,nzsK,IpK,JpK));
      vector<SparseMat> PPdms2(3,SparseMat(n,n,nzsP,IpP,JpP));
      map<int,vector<SparseMat>> sigmahels;
      for(auto & i : nodeMap)
	sigmahels[i.first] = vector<SparseMat>(6,SparseMat(1,n,nzsS[i.second],IpS[i.second],JpS[i.second]));

      rPdm.resize(3,Mat3xV(n));
      PPdm.resize(3,vector<SqrMatV>(3));
      Pdm.resize(n);

      KrKP.resize(nN,Vec3(NONINIT));
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
		for(int mm=0; mm<3; mm++) {
		  double dN = (this->*dNidq[ll][mm])(x,y,z,ll);
		  for(int nn=0; nn<3; nn++)
		    J(mm,nn) += dN*r0(nn);
		}
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
		double Ni_ = (this->*Ni[i])(x,y,z,i);
		Vec dN(3,NONINIT);
		for(int rr=0; rr<3; rr++)
		  dN(rr) = (this->*dNidq[i][rr])(x,y,z,i);
		Vec dNi = slvLUFac(LUJ,dN,ipiv);
		VecVI &ai = dofA[ee.second(i)];
		for(int i1=0; i1<3; i1++) {
		  if(ai(i1)>-1)
		    Pdm(i1,ai(i1)) += dm*Ni_;
		  for(int j1=0; j1<3; j1++) {
		    if(ai(j1)>-1)
		      rPdm[i1](j1,ai(j1)) += dm*r(i1)*Ni_;
		  }
		}
		for(int j=i; j<20; j++) {
		  double Nj_ = (this->*Ni[j])(x,y,z,j);
		  Vec dN(3,NONINIT);
		  for(int rr=0; rr<3; rr++)
		    dN(rr) = (this->*dNidq[j][rr])(x,y,z,j);
		  Vec dNj = slvLUFac(LUJ,dN,ipiv);
		  double dPPdm = dm*Ni_*Nj_;
		  double dK[3][3];
		  dK[0][0] = dk*((1-nu)/(1-2*nu)*dNi(0)*dNj(0)+0.5*(dNi(1)*dNj(1)+dNi(2)*dNj(2)));
		  dK[1][0] = dk*(nu/(1-2*nu)*dNi(1)*dNj(0)+0.5*dNi(0)*dNj(1));
		  dK[2][0] = dk*(nu/(1-2*nu)*dNi(2)*dNj(0)+0.5*dNi(0)*dNj(2));
		  dK[0][1] = dk*(nu/(1-2*nu)*dNi(0)*dNj(1)+0.5*dNi(1)*dNj(0));
		  dK[1][1] = dk*((1-nu)/(1-2*nu)*dNi(1)*dNj(1)+0.5*(dNi(0)*dNj(0)+dNi(2)*dNj(2)));
		  dK[2][1] = dk*(nu/(1-2*nu)*dNi(2)*dNj(1)+0.5*dNi(1)*dNj(2));
		  dK[0][2] = dk*(nu/(1-2*nu)*dNi(0)*dNj(2)+0.5*dNi(2)*dNj(0));
		  dK[1][2] = dk*(nu/(1-2*nu)*dNi(1)*dNj(2)+0.5*dNi(2)*dNj(1));
		  dK[2][2] = dk*((1-nu)/(1-2*nu)*dNi(2)*dNj(2)+0.5*(dNi(0)*dNj(0)+dNi(1)*dNj(1)));
		  if(ee.second(j)>=ee.second(i)) {
		    Matrix<General,Fixed<3>,Fixed<3>,int> &Kij = linksK[ee.second(i)][ee.second(j)];
		    for(int i1=0; i1<3; i1++) {
		      if(Kij.e(i1,i1)>-1)
			PPdms1[i1]()[Kij.e(i1,i1)] += dPPdm;
		      int j1s = 0;
		      if(ee.second(j)==ee.second(i)) j1s=i1;
		      for(int j1=j1s; j1<3; j1++) {
			if(Kij.e(i1,j1)>-1)
			  Ke0s()[Kij.e(i1,j1)] += dK[i1][j1];
		      }
		    }
		    Matrix<General,Fixed<3>,Fixed<3>,int> &Pij = linksP[ee.second(i)][ee.second(j)];
		    Matrix<General,Fixed<3>,Fixed<3>,int> &Pji = linksP[ee.second(j)][ee.second(i)];
		    if(Pij.e(0,1)>-1)
		      PPdms2[0]()[Pij.e(0,1)] += dPPdm;
		    if(Pij.e(0,2)>-1)
		      PPdms2[1]()[Pij.e(0,2)] += dPPdm;
		    if(Pij.e(1,2)>-1)
		      PPdms2[2]()[Pij.e(1,2)] += dPPdm;
		    if(ee.second(i)!=ee.second(j)) {
		      if(Pji.e(0,1)>-1)
			PPdms2[0]()[Pji.e(0,1)] += dPPdm;
		      if(Pji.e(0,2)>-1)
			PPdms2[1]()[Pji.e(0,2)] += dPPdm;
		      if(Pji.e(1,2)>-1)
			PPdms2[2]()[Pji.e(1,2)] += dPPdm;
		    }
		  }
		  else {
		    Matrix<General,Fixed<3>,Fixed<3>,int> &Kji = linksK[ee.second(j)][ee.second(i)];
		    for(int i1=0; i1<3; i1++) {
		      if(Kji.e(i1,i1)>-1)
			PPdms1[i1]()[Kji.e(i1,i1)] += dPPdm;
		      for(int j1=0; j1<3; j1++) {
			if(Kji.e(i1,j1)>-1)
			  Ke0s()[Kji.e(i1,j1)] += dK[j1][i1];
		      }
		    }
		    Matrix<General,Fixed<3>,Fixed<3>,int> &Pij = linksP[ee.second(i)][ee.second(j)];
		    Matrix<General,Fixed<3>,Fixed<3>,int> &Pji = linksP[ee.second(j)][ee.second(i)];
		    if(Pij.e(1,0)>-1)
		      PPdms2[0]()[Pij.e(1,0)] += dPPdm;
		    if(Pij.e(2,0)>-1)
		      PPdms2[1]()[Pij.e(2,0)] += dPPdm;
		    if(Pij.e(2,1)>-1)
		      PPdms2[2]()[Pij.e(2,1)] += dPPdm;
		    if(Pji.e(1,0)>-1)
		      PPdms2[0]()[Pji.e(1,0)] += dPPdm;
		    if(Pji.e(2,0)>-1)
		      PPdms2[1]()[Pji.e(2,0)] += dPPdm;
		    if(Pji.e(2,1)>-1)
		      PPdms2[2]()[Pji.e(2,1)] += dPPdm;
		  }
		}
	      }
	    }
	  }
	}
      }

      for(const auto & ee : ele) {
	for(int k=0; k<20; k++) {
	  double x = rN[k](0);
	  double y = rN[k](1);
	  double z = rN[k](2);
	  SqrMat J(3);
	  for(int ll=0; ll<20; ll++) {
	    Vec3 r0 = nodalPos[ee.second(ll)];
	    for(int mm=0; mm<3; mm++) {
	      double dN = (this->*dNidq[ll][mm])(x,y,z,ll);
	      for(int nn=0; nn<3; nn++)
		J(mm,nn) += dN*r0(nn);
	    }
	  }
	  Vector<Ref,int> ipiv(J.size(),NONINIT);
	  SqrMat LUJ = facLU(J,ipiv);
	  for(int i=0; i<20; i++) {
	    Vec dN(3,NONINIT);
	    for(int rr=0; rr<3; rr++)
	      dN(rr) = (this->*dNidq[i][rr])(x,y,z,i);
	    Vec dNi = slvLUFac(LUJ,dN,ipiv);
	    double al = E/(1+nu)/nodeCount[ee.second(k)];
	    RowVector<Fixed<3>,int> &sij = linksS[ee.second(k)][ee.second(i)];
	    if(sij(0)>-1) {
	      sigmahels[ee.second(k)][0]()[sij(0)] += al*(1-nu)/(1-2*nu)*dNi(0);
	      sigmahels[ee.second(k)][1]()[sij(0)] += al*nu/(1-2*nu)*dNi(0);
	      sigmahels[ee.second(k)][2]()[sij(0)] += al*nu/(1-2*nu)*dNi(0);
	      sigmahels[ee.second(k)][3]()[sij(0)] += al*0.5*dNi(1);
	      sigmahels[ee.second(k)][5]()[sij(0)] += al*0.5*dNi(2);
	    }
	    if(sij(1)>-1) {
	      sigmahels[ee.second(k)][0]()[sij(1)] += al*nu/(1-2*nu)*dNi(1);
	      sigmahels[ee.second(k)][1]()[sij(1)] += al*(1-nu)/(1-2*nu)*dNi(1);
	      sigmahels[ee.second(k)][2]()[sij(1)] += al*nu/(1-2*nu)*dNi(1);
	      sigmahels[ee.second(k)][3]()[sij(1)] += al*0.5*dNi(0);
	      sigmahels[ee.second(k)][4]()[sij(1)] += al*0.5*dNi(2);
	    }
	    if(sij(2)>-1) {
	      sigmahels[ee.second(k)][0]()[sij(2)] += al*nu/(1-2*nu)*dNi(2);
	      sigmahels[ee.second(k)][1]()[sij(2)] += al*nu/(1-2*nu)*dNi(2);
	      sigmahels[ee.second(k)][2]()[sij(2)] += al*(1-nu)/(1-2*nu)*dNi(2);
	      sigmahels[ee.second(k)][4]()[sij(2)] += al*0.5*dNi(1);
	      sigmahels[ee.second(k)][5]()[sij(2)] += al*0.5*dNi(0);
	    }
	  }
	}
      }

      Indices IH, IN;
      for(int i=0; i<inodes.size(); i++) {
	VecVI &ai = dofA[inodes(i)];
	for(int ii=0; ii<3; ii++) {
	  if(ai(ii)>-1)
	    IH.add(ai(ii));
	}
      }
      for(int i=0; i<nnodes.size(); i++) {
	VecVI &ai = dofA[nnodes(i)];
	for(int ii=0; ii<3; ii++) {
	  if(ai(ii)>-1)
	    IN.add(ai(ii));
	}
      }

      MatV Vsd(n,IH.size()+nmodes.size(),NONINIT);
      if(IH.size()) {
	Indices IJ;
	for(int i=0; i<IH.size(); i++)
	  IJ.add(i);
	MatV Vs(n,IH.size(),NONINIT);
	Vs.set(IN,IJ,-slvLU(sparseSub(Ke0s,dofN,linksK,nnodes),denseSub(Ke0s,dofN,dofH,linksK,nnodes,inodes)));
	Vs.set(IH,IJ,MatV(IH.size(),IH.size(),Eye()));
	Vsd.set(RangeV(0,n-1),RangeV(0,Vs.cols()-1),Vs);
      }

      if(nmodes.size()) {
	if(fixedBoundaryNormalModes) {
	  throwError("(FiniteElementsFfrBody::init): option \"fixed boundary normal modes\" is currently not available");
	  SqrMat V; Vec w;
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
	  SymSparseMat M = PPdms1[0];
	  for(int i=0; i<M.nonZeroElements(); i++)
	    M()[i] += PPdms1[1]()[i]+PPdms1[2]()[i];
	  Mat V; Vec w;
	  eigvec(Ke0s,M,6+nmodes.size(),1,V,w);
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
	SymSparseMat MS = PPdms1[0];
	for(int i=0; i<MS.nonZeroElements(); i++)
	  MS()[i] += PPdms1[1]()[i]+PPdms1[2]()[i];
	SqrMat V; Vec w;
	eigvec(JTMJ(Ke0s,Vsd),JTMJ(MS,Vsd),V,w);
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
	  PPdm[i][i] <<= Vsd.T()*(PPdms1[i]*Vsd);
	}
	PPdm[0][1] <<= Vsd.T()*(PPdms2[0]*Vsd);
	PPdm[0][2] <<= Vsd.T()*(PPdms2[1]*Vsd);
	PPdm[1][2] <<= Vsd.T()*(PPdms2[2]*Vsd);
	PPdm[1][0] <<= PPdm[0][1].T();
	PPdm[2][0] <<= PPdm[0][2].T();
	PPdm[2][1] <<= PPdm[1][2].T();
	Ke0 <<= JTMJ(Ke0s,Vsd);
	sigmahel.resize(nN,Matrix<General,Fixed<6>,Var,double>(Vsd.cols(),NONINIT));
	Phi.resize(nN,Mat3xV(Vsd.cols(),NONINIT));
	Psi.resize(nN,Mat3xV(Vsd.cols()));
	for(const auto & i : nodeMap) {
	  VecVI &ai = dofA[i.first];
	  for(int j=0; j<3; j++) {
	    if(ai(j)>-1)
	      Phi[i.second].set(j,Vsd.row(ai(j)));
	    else
	      Phi[i.second].set(j,RowVecV(Vsd.cols()));
	  }
	  for(int j=0; j<6; j++)
	    sigmahel[i.second].set(RangeV(j,j),RangeV(0,Vsd.cols()-1),sigmahels[i.first][j]*Vsd);
	}
      }
      else
	throwError("(FiniteElementsFfrBody::init): at least one interface node or normal mode must be given.");

      delete [] JpK;
      delete [] IpK;
      delete [] JpP;
      delete [] IpP;
      for(size_t i=0; i<nN; i++) {
	delete [] JpS[i];
	delete [] IpS[i];
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
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"boundaryNodeNumbers");
    while(e && MBXMLUtils::E(e)->getTagName()==MBSIMFLEX%"boundaryNodeNumbers") {
      addBoundaryNodes(MBXMLUtils::E(e)->getText<VecVI>());
      e=e->getNextElementSibling();
      if(MBXMLUtils::E(e)->getTagName()==MBSIMFLEX%"degreesOfFreedom") {
	VecVI dof = MBXMLUtils::E(e)->getText<VecVI>();
	for(int i=0; i<dof.size(); i++)
	  dof(i)--;
	addDegreesOfFreedom(dof);
      }
      e=e->getNextElementSibling();
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
