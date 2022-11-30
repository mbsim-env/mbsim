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

  void FlexibleBodyTool::cms() {
    std::map<int,VecVI> bc;
    bool fixedBoundaryNormalModes = false;
    int nr = 0;
    auto *list = static_cast<ListWidget*>(static_cast<BoundaryConditionsPage*>(page(PageBC))->bc->getWidget());
    std::vector<VecVI> dof(list->getSize());;
    std::vector<VecVI> bnodes(list->getSize());
    for(int i=0; i<list->getSize(); i++) {
      auto *bcw = static_cast<BoundaryConditionWidget*>(list->getWidget(i));
      auto mat = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(bcw->getNodes()->getWidget())->getWidget())->getWidget()->getEvalMat();
      bnodes[i].resize(mat.size(),NONINIT);
      for(size_t j=0; j<mat.size(); j++)
	bnodes[i](j) = mat[j][0].toInt();
      mat = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(bcw->getDof()->getWidget())->getWidget())->getWidget()->getEvalMat();
      dof[i].resize(mat.size(),NONINIT);
      for(size_t j=0; j<mat.size(); j++)
	dof[i](j) = mat[j][0].toInt()-1;
    }

    VecVI inodes;
    if(static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->inodes->isActive()) {
      auto mat = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->inodes->getWidget())->getWidget())->getWidget()->getEvalMat();
      inodes.resize(mat.size(),NONINIT);
      for(size_t i=0; i<mat.size(); i++)
	inodes(i) = mat[i][0].toInt();
    }

    VecVI nmodes;
    if(static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->nmodes->isActive()) {
      auto mat = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->nmodes->getWidget())->getWidget())->getWidget()->getEvalMat();
      nmodes.resize(mat.size(),NONINIT);
      for(size_t i=0; i<mat.size(); i++)
	nmodes(i) = mat[i][0].toInt();
    }

    if(bnodes.size() != dof.size())
      runtime_error("(FlexibleBodyTool::init): number of boundary nodes (" + to_string(bnodes.size()) + ") must equal number of degrees of freedom (" + to_string(dof.size()) + ")");
    for(size_t i=0; i<bnodes.size(); i++) {
      for(int j=0; j<bnodes[i].size(); j++) {
	bc[bnodes[i](j)].resize(nen);
	for(int k=0; k<dof[i].size(); k++) {
	  if(dof[i](k)<0 or dof[i](k)>nen-1)
	    runtime_error("(FlexibleBodyTool::init): degrees of freedom of boundary node number (" + to_string(i) + ") must be within range [0,3]");
	  bc[bnodes[i](j)](dof[i](k)) = 1;
	}
      }
    }

    for(const auto & i : bc)
      for(int j=0; j<i.second.size(); j++)
	nr += i.second(j);

    int n = ng-nr;

    vector<int> c;
    for(const auto & i : bc) {
      for(int j=0; j<i.second.size(); j++)
	if(i.second(j)) c.push_back(nodeTable[i.first]*nen+j);
    }
    sort(c.begin(), c.end());

    size_t h=0;
    Indices iF, iX;
    for(int i=0; i<ng; i++) {
      if(h<c.size() and i==c[h]) {
	h++;
	iX.add(i);
      }
      else
	iF.add(i);
    }

    c.clear();
    for(int i=0; i<inodes.size(); i++) {
      VecVI bci = bc[inodes(i)];
      for(int j=0; j<nen; j++) {
	if((not bci.size()) or (not bci(j)))
	  c.push_back(nodeTable[inodes(i)]*nen+j);
      }
    }
    sort(c.begin(), c.end());
    h=0;
    Indices iH, iN;
    for(int i=0; i<iF.size(); i++) {
      if(h<c.size() and iF[i]==c[h]) {
	iH.add(i);
	h++;
      }
      else
	iN.add(i);
    }

    auto MKmr = reduceMat(MKm,iF);
    auto MKmrn = reduceMat(MKmr,iN);
    auto MKsr = createMKs(MKmr);
    auto MKsrn = createMKs(MKmrn);

    MatV Vsd(n,iH.size()+nmodes.size(),NONINIT);
    if(iH.size()) {
      Indices IJ;
      for(int i=0; i<iH.size(); i++)
	IJ.add(i);
      MatV Vs(iF.size(),iH.size(),NONINIT);
      Vs.set(iN,IJ,-slvLU(MKsrn.second,reduceMat(MKmr,iN,iH)));
      Vs.set(iH,IJ,MatV(iH.size(),iH.size(),Eye()));
      Vsd.set(RangeV(0,n-1),RangeV(0,Vs.cols()-1),Vs);
    }

    if(nmodes.size()) {
      Mat V;
      Vec w;
      if(fixedBoundaryNormalModes) {
	eigvec(MKsrn.second,MKsrn.first,6+nmodes.size(),1,V,w);
	vector<int> imod;
	for(int i=0; i<w.size(); i++) {
	  if(w(i)>pow(2*M_PI*0.1,2))
	    imod.push_back(i);
	}
	if(min(nmodes)<1 or max(nmodes)>(int)imod.size())
	  runtime_error(string("(FlexibleBodyTool::init): node numbers do not match, must be within the range [1,") + to_string(imod.size()) + "]");
	for(int i=0; i<nmodes.size(); i++) {
	  Vsd.set(iN,iH.size()+i,V.col(imod[nmodes(i)-1]));
	  Vsd.set(iH,iH.size()+i,Vec(iH.size()));
	}
      }
      else {
	eigvec(MKsr.second,MKsr.first,6+nmodes.size(),1,V,w);
	vector<int> imod;
	for(int i=0; i<w.size(); i++) {
	  if(w(i)>pow(2*M_PI*0.1,2))
	    imod.push_back(i);
	}
	if(min(nmodes)<1 or max(nmodes)>(int)imod.size())
	  runtime_error(string("(FlexibleBodyTool::init): node numbers do not match, must be within the range [1,") + to_string(imod.size()) + "]");
	for(int i=0; i<nmodes.size(); i++)
	  Vsd.set(iH.size()+i,V.col(imod[nmodes(i)-1]));
      }
    }

    if(iH.size()) {
      SqrMat V;
      Vec w;
      eigvec(JTMJ(MKsr.second,Vsd),JTMJ(MKsr.first,Vsd),V,w);
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
