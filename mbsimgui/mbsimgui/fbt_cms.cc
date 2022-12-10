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
#include "extended_widgets.h"
#include "special_widgets.h"
#include <fmatvec/sparse_linear_algebra_double.h>

using namespace std;
using namespace fmatvec;

namespace MBSimGUI {

  template <class AT>
  AT max(const vector<AT> &x) {
    AT maximum = x[0];
    for(size_t i=1; i<x.size(); i++) {
      if(x[i] > maximum)
        maximum = x[i];
    }
    return maximum;
  }

  template <class AT>
  AT min(const vector<AT> &x) {
    AT minimum = x[0];
    for(size_t i=1; i<x.size(); i++) {
      if(x[i] < minimum)
        minimum = x[i];
    }
    return minimum;
  }

  void FlexibleBodyTool::cms() {
    bool fixedBoundaryNormalModes = false;
    auto *list = static_cast<ListWidget*>(static_cast<BoundaryConditionsPage*>(page(PageBC))->bc->getWidget());
    vector<vector<int>> dof(list->getSize());;
    vector<vector<int>> bnodes(list->getSize());
    for(int i=0; i<list->getSize(); i++) {
      auto *bcw = static_cast<BoundaryConditionWidget*>(list->getWidget(i));
      auto mat = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(bcw->getNodes()->getWidget())->getWidget())->getWidget()->getEvalMat();
      bnodes[i].resize(mat.size());
      for(size_t j=0; j<mat.size(); j++)
	bnodes[i][j] = mat[j][0].toInt();
      mat = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(bcw->getDof()->getWidget())->getWidget())->getWidget()->getEvalMat();
      dof[i].resize(mat.size());
      for(size_t j=0; j<mat.size(); j++)
	dof[i][j] = mat[j][0].toInt()-1;
    }

    vector<int> inodes;
    if(static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->inodes->isActive()) {
      auto mat = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->inodes->getWidget())->getWidget())->getWidget()->getEvalMat();
      inodes.resize(mat.size());
      for(size_t i=0; i<mat.size(); i++)
	inodes[i] = mat[i][0].toInt();
    }

    vector<int> nmodes;
    if(static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->nmodes->isActive()) {
      auto mat = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->nmodes->getWidget())->getWidget())->getWidget()->getEvalMat();
      nmodes.resize(mat.size());
      for(size_t i=0; i<mat.size(); i++)
	nmodes[i] = mat[i][0].toInt();
    }

    vector<vector<int>> bc(nN,vector<int>(nen));
    if(bnodes.size() != dof.size())
      runtime_error("(FlexibleBodyTool::init): number of boundary nodes (" + to_string(bnodes.size()) + ") must equal number of degrees of freedom (" + to_string(dof.size()) + ")");
    for(size_t i=0; i<bnodes.size(); i++) {
      for(int j=0; j<bnodes[i].size(); j++) {
	for(int k=0; k<dof[i].size(); k++) {
	  if(dof[i][k]<0 or dof[i][k]>nen-1)
	    runtime_error("(FlexibleBodyTool::init): degrees of freedom of boundary node number (" + to_string(i) + ") must be within range [0,3]");
	  bc[nodeTable[bnodes[i][j]]][dof[i][k]] = 1;
	}
      }
    }

    int nr = 0;
    for(int i=0; i<bc.size(); i++) {
      for(int j=0; j<bc[i].size(); j++)
	nr += bc[i][j];
    }

    int n = ng-nr;

    SymSparseMat Ms = PPdms[0];
    for(int i=0; i<Ms.nonZeroElements(); i++)
      Ms()[i] += PPdms[1]()[i]+PPdms[2]()[i];

    vector<double[3]> activeDof(nN);
    for(int i=0; i<nN; i++) {
      for(int j=0; j<3; j++)
	activeDof[i][j] = 1;
    }
    for(size_t i=0; i<bnodes.size(); i++) {
      for(size_t j=0; j<bnodes[i].size(); j++) {
	for(size_t k=0; k<dof[i].size(); k++)
	  activeDof[nodeTable[bnodes[i][j]]][dof[i][k]] = 0;
      }
    }
    vector<int> dofMapF(3*nN);
    for(size_t i=0, l=0, k=0; i<nN; i++) {
      for(size_t j=0; j<3; j++) {
	if(activeDof[i][j])
	  dofMapF[l] = k++;
	l++;
      }
    }
    int nzer = 0;
    for(size_t i=0; i<nN; i++) {
      for(int k=0; k<3; k++) {
	if(activeDof[i][k]) {
	  for(int l=k; l<3; l++) {
	    if(activeDof[i][l])
	      nzer++;
	  }
	  for(const auto & j : links[i]) {
	    for(int l=0; l<3; l++) {
	      if(activeDof[j.first][l])
		nzer++;
	    }
	  }
	}
      }
    }
    int *Ipr = new int[n+1];
    int *Jpr = new int[nzer];
    Ipr[0] = 0;
    SymSparseMat Krs(n,nzer,Ipr,Jpr);
    SymSparseMat Mrs(n,nzer,Ipr,Jpr);
    int ii = 0, kk = 0, ll = 0;
    for(size_t i=0; i<nN; i++) {
      for(int k=0; k<3; k++) {
	for(int l=k; l<3; l++) {
	  if(activeDof[i][k] and activeDof[i][l]) {
	    Krs()[ll] = Ks()[kk];
	    Mrs()[ll] = Ms()[kk];
	    Jpr[ll++] = dofMapF[3*i+l];
	  }
	  kk++;
	}
	for(const auto & j : links[i]) {
	  for(int l=0; l<3; l++) {
	    if(activeDof[i][k] and activeDof[j.first][l]) {
	      Krs()[ll] = Ks()[kk];
	      Mrs()[ll] = Ms()[kk];
	      Jpr[ll++] = dofMapF[3*j.first+l];
	    }
	    kk++;
	  }
	}
	if(activeDof[i][k])
	  Ipr[++ii] = ll;
      }
    }

    Indices iF, iX;
    for(int i=0; i<nN; i++) {
      for(int j=0; j<3; j++) {
	if(activeDof[i][j])
	  iF.add(3*i+j);
	else
	  iX.add(3*i+j);
      }
    }

    for(size_t i=0; i<inodes.size(); i++) {
      for(size_t j=0; j<3; j++)
	activeDof[nodeTable[inodes[i]]][j] *= 2;
    }
    Indices iH, iN;
    for(size_t i=0; i<nN; i++) {
      for(size_t j=0; j<3; j++) {
	if(activeDof[i][j]==2)
	  iH.add(dofMapF[3*i+j]);
	else if(activeDof[i][j]==1)
	  iN.add(dofMapF[3*i+j]);
      }
    }

    MatV Vsd(n,iH.size()+nmodes.size(),NONINIT);
    if(iH.size()) {
      vector<int> dofMapN(3*nN);
      for(size_t i=0, l=0, k=0; i<nN; i++) {
	for(size_t j=0; j<3; j++) {
	  if(activeDof[i][j]==1)
	    dofMapN[l] = k++;
	  l++;
	}
      }
      int nzern = 0;
      for(size_t i=0; i<nN; i++) {
	for(int k=0; k<3; k++) {
	  if(activeDof[i][k]==1) {
	    for(int l=k; l<3; l++) {
	      if(activeDof[i][l]==1)
		nzern++;
	    }
	    for(const auto & j : links[i]) {
	      for(int l=0; l<3; l++) {
		if(activeDof[j.first][l]==1)
		  nzern++;
	      }
	    }
	  }
	}
      }
      int *Iprn = new int[iN.size()+1];
      int *Jprn = new int[nzern];
      Iprn[0] = 0;
      SymSparseMat Mrns(iN.size(),nzern,Iprn,Jprn);
      SymSparseMat Krns(iN.size(),nzern,Iprn,Jprn);
      int ii = 0, kk = 0, ll = 0;
      for(size_t i=0; i<nN; i++) {
	for(int k=0; k<3; k++) {
	  for(int l=k; l<3; l++) {
	    if(activeDof[i][k]==1 and activeDof[i][l]==1) {
	      Mrns()[ll] = Ms()[kk];
	      Krns()[ll] = Ks()[kk];
	      Jprn[ll++] = dofMapN[3*i+l];
	    }
	    kk++;
	  }
	  for(const auto & j : links[i]) {
	    for(int l=0; l<3; l++) {
	      if(activeDof[i][k]==1 and activeDof[j.first][l]==1) {
		Mrns()[ll] = Ms()[kk];
		Krns()[ll] = Ks()[kk];
		Jprn[ll++] = dofMapN[3*j.first+l];
	      }
	      kk++;
	    }
	  }
	  if(activeDof[i][k]==1)
	    Iprn[++ii] = ll;
	}
      }
      vector<int> dofMapH(3*nN);
      for(size_t i=0, l=0, k=0; i<nN; i++) {
	for(size_t j=0; j<3; j++) {
	  if(activeDof[i][j]==2)
	    dofMapH[l] = k++;
	  l++;
	}
      }
      MatV Krnh(iN.size(),iH.size());
      kk = 0;
      for(size_t i=0; i<nN; i++) {
	for(int k=0; k<3; k++) {
	  for(int l=k; l<3; l++) {
	    if(activeDof[i][k]==1 and activeDof[i][l]==2)
	      Krnh(dofMapN[3*i+k],dofMapH[3*i+l]) = Ks()[kk];
	    else if(activeDof[i][l]==1 and activeDof[i][k]==2)
	      Krnh(dofMapN[3*i+l],dofMapH[3*i+k]) = Ks()[kk];
	    kk++;
	  }
	  for(const auto & j : links[i]) {
	    for(int l=0; l<3; l++) {
	      if(activeDof[i][k]==1 and activeDof[j.first][l]==2)
		Krnh(dofMapN[3*i+k],dofMapH[3*j.first+l]) = Ks()[kk];
	      if(activeDof[j.first][l]==1 and activeDof[i][k]==2)
		Krnh(dofMapN[3*j.first+l],dofMapH[3*i+k]) = Ks()[kk];
	      kk++;
	    }
	  }
	}
      }
      Indices IJ;
      for(int i=0; i<iH.size(); i++)
	IJ.add(i);
      MatV Vs(iF.size(),iH.size(),NONINIT);
      Vs.set(iN,IJ,-slvLU(Krns,Krnh));
      Vs.set(iH,IJ,MatV(iH.size(),iH.size(),Eye()));
      Vsd.set(RangeV(0,n-1),RangeV(0,Vs.cols()-1),Vs);
      if(nmodes.size() and fixedBoundaryNormalModes) {
	Mat V;
	Vec w;
	eigvec(Krns,Mrns,6+nmodes.size(),1,V,w);
	vector<int> imod;
	for(int i=0; i<w.size(); i++) {
	  if(w(i)>pow(2*M_PI*0.1,2))
	    imod.push_back(i);
	}
	if(min(nmodes)<1 or max(nmodes)>(int)imod.size())
	  runtime_error(string("(FlexibleBodyTool::init): node numbers do not match, must be within the range [1,") + to_string(imod.size()) + "]");
	for(int i=0; i<nmodes.size(); i++) {
	  Vsd.set(iN,iH.size()+i,V.col(imod[nmodes[i]-1]));
	  Vsd.set(iH,iH.size()+i,Vec(iH.size()));
	}
      }
    }

    if(nmodes.size() and not fixedBoundaryNormalModes) {
      Mat V;
      Vec w;
      eigvec(Krs,Mrs,6+nmodes.size(),1,V,w);
      vector<int> imod;
      for(int i=0; i<w.size(); i++) {
	if(w(i)>pow(2*M_PI*0.1,2))
	  imod.push_back(i);
      }
      if(min(nmodes)<1 or max(nmodes)>(int)imod.size())
	runtime_error(string("(FlexibleBodyTool::init): node numbers do not match, must be within the range [1,") + to_string(imod.size()) + "]");
      for(int i=0; i<nmodes.size(); i++)
	Vsd.set(iH.size()+i,V.col(imod[nmodes[i]-1]));
    }

    if(iH.size()) {
      SqrMat V;
      Vec w;
      eigvec(JTMJ(Krs,Vsd),JTMJ(Mrs,Vsd),V,w);
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
    nM = Vsd.cols();
    U.resize(ng,nM,NONINIT);
    Indices IJ;
    for(int i=0; i<nM; i++)
      IJ.add(i);
    U.set(iF,IJ,Vsd);
    if(iX.size()) U.set(iX,IJ,Mat(iX.size(),IJ.size()));
  }

}
