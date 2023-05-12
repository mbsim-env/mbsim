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

    list = static_cast<ListWidget*>(static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->idata->getWidget());
    vector<vector<int>> inodes(list->getSize());
    vector<bool> reduceToNode(list->getSize());
    vector<Indices> idof(list->getSize());
    vector<int> snn(list->getSize());
    for(int i=0; i<list->getSize(); i++) {
      inodes[i] = static_cast<CMSDataWidget*>(list->getWidget(i))->getNodes();
      reduceToNode[i] = static_cast<CMSDataWidget*>(list->getWidget(i))->getReduceToSingleNode();
      idof[i] = static_cast<CMSDataWidget*>(list->getWidget(i))->getDof();
      for(size_t j=0; j<idof[i].size(); j++)
	idof[i].set(j,idof[i][j]-1);
      snn[i] = static_cast<CMSDataWidget*>(list->getWidget(i))->getSingleNodeNumber();
    }

    vector<int> nmodes;
    if(static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->nmodes->isActive()) {
      auto mat = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->nmodes->getWidget())->getWidget())->getWidget()->getEvalMat();
      nmodes.resize(mat.size());
      for(size_t i=0; i<mat.size(); i++)
	nmodes[i] = mat[i][0].toInt();
    }

    bool fixedBoundaryNormalModes = false;
    if(static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->fbnm->isActive()) {
      fixedBoundaryNormalModes = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->fbnm->getWidget())->getWidget())->getEvalMat()[0][0].toInt();
    }

    if(bnodes.size() != dof.size())
      runtime_error("(FlexibleBodyTool::init): number of boundary nodes (" + to_string(bnodes.size()) + ") must equal number of degrees of freedom (" + to_string(dof.size()) + ")");

    int nN = r.size();
    int nen = net + ner;

    vector<vector<int>> activeDof(nN,vector<int>(nen));
    for(int i=0; i<nN; i++) {
      for(int j=0; j<nen; j++)
	activeDof[i][j] = 1;
    }
    for(size_t i=0; i<bnodes.size(); i++) {
      for(size_t j=0; j<bnodes[i].size(); j++) {
	for(size_t k=0; k<dof[i].size(); k++)
	  activeDof[nodeTable[bnodes[i][j]]][dof[i][k]] = 0;
      }
    }
    vector<int> dofMapF(nen*nN);
    for(size_t i=0, l=0, k=0; i<nN; i++) {
      for(size_t j=0; j<nen; j++) {
	if(activeDof[i][j])
	  dofMapF[l] = k++;
	l++;
      }
    }

    Indices iF, iX;
    for(int i=0; i<nN; i++) {
      for(int j=0; j<nen; j++) {
	if(activeDof[i][j])
	  iF.add(nen*i+j);
	else
	  iX.add(nen*i+j);
      }
    }

    SymSparseMat Ms, Mrs, Krs;
    vector<map<int,double>> Krm, Mrm;
    if(Mm.size()) {
      Mrm = reduceMat(Mm,iF);
      Krm = reduceMat(Km,iF);
      SymSparseMat SM1 = createSymSparseMat(Mrm);
      SymSparseMat SM2 = createSymSparseMat(Krm);
      Mrs &= SM1;
      Krs &= SM2;
    }
    else {
      Ms <<= PPdms[0];
      for(int i=0; i<Ms.nonZeroElements(); i++)
	Ms()[i] += PPdms[1]()[i]+PPdms[2]()[i];
      reduceMat(Ms,Ks,Mrs,Krs,iF.size(),activeDof,dofMapF);
    }

    for(size_t i=0; i<inodes.size(); i++) {
      for(size_t j=0; j<inodes[i].size(); j++) {
	for(size_t k=0; k<nen; k++)
	  activeDof[nodeTable[inodes[i][j]]][k] *= 2;
      }
    }
    Indices iH, iN;
    for(size_t i=0; i<nN; i++) {
      for(size_t j=0; j<nen; j++) {
	if(activeDof[i][j]==2)
	  iH.add(dofMapF[nen*i+j]);
	else if(activeDof[i][j]==1)
	  iN.add(dofMapF[nen*i+j]);
      }
    }

    MatV Ui, Un, Ui_(iF.size(),iH.size(),NONINIT);
    if(iH.size()) {
      vector<int> dofMapH(nen*nN);
      for(size_t i=0, l=0, k=0; i<nN; i++) {
	for(size_t j=0; j<nen; j++) {
	  if(activeDof[i][j]==2)
	    dofMapH[l] = k++;
	  l++;
	}
      }
      vector<int> dofMapN(nen*nN);
      for(size_t i=0, l=0, k=0; i<nN; i++) {
	for(size_t j=0; j<nen; j++) {
	  if(activeDof[i][j]==1)
	    dofMapN[l] = k++;
	  l++;
	}
      }

      SymSparseMat Krns, Mrns;
      MatV Krnh;
      if(Mm.size()) {
	SymSparseMat SM1 = createSymSparseMat(reduceMat(Krm,iN));
	Krns &= SM1;
	SymSparseMat SM2 = createSymSparseMat(reduceMat(Mrm,iN));
	Mrns &= SM2;
	Krnh <<= reduceMat(Krm,iN,iH);
      }
      else {
	reduceMat(Ms,Ks,Mrns,Krns,iN.size(),activeDof,dofMapN);
	Krnh <<= reduceMat(Ks,iN,iH,activeDof,dofMapN,dofMapH);
      }
      Indices IJ;
      for(int i=0; i<iH.size(); i++)
	IJ.add(i);
      Ui_.set(iN,IJ,-slvLU(Krns,Krnh));
      Ui_.set(iH,IJ,MatV(iH.size(),iH.size(),Eye()));
      VecV weights;

      int ni = 0;
      bool rdn = false;
      for(size_t i=0; i<inodes.size(); i++) {
	ni += idof[i].size();
	if(reduceToNode[i]) rdn = true;
      }

      if(rdn) {
	SymMatV Ki = JTMJ(Krs,Ui_);
	Ui.resize(iF.size(),ni,NONINIT);
	ni = 0;
	for(size_t i=0; i<inodes.size(); i++) {
	  if(reduceToNode[i]) {
	    singleNodeNumbers.push_back(snn[i]>-1?snn[i]:nodeNumbers.size()+1+singleNodeNumbers.size());
	    Indices iHi;
	    vector<Matrix<General,Fixed<6>,Var,double>> Jrr(inodes.size(),Matrix<General,Fixed<6>,Var,double>(6*inodes.size()));
	    if(weights.size()==0)
	      weights.resize(inodes[i].size(),INIT,1.0);
	    double sum = 0;
	    for(size_t j=0; j<inodes[i].size(); j++) {
	      sum += weights(j);
	      for(int k=0; k<nen; k++)
		iHi.add(dofMapH[3*nodeTable[inodes[i][j]]+k]);
	    }
	    Vec3 rr;
	    SymMat3 A;
	    Indices iI(Ui_.rows());
	    for(int k=0; k<iI.size(); k++)
	      iI.set(k,k);
	    MatV Ur = Ui_(iI,iHi);
	    MatV Ur_(nen*nN,Ur.cols());
	    Indices iJ(Ur.cols());
	    for(int k=0; k<iJ.size(); k++)
	      iJ.set(k,k);
	    Ur_.set(iF,iJ,Ur);
	    Mat3xV B(Ur.cols());
	    Mat3xV C(Ur.cols());
	    Matrix<General,Fixed<6>,Var,double> Jr(Ur.cols());
	    for(size_t j=0; j<inodes[i].size(); j++) {
	      rr += (weights(j)/sum)*r[nodeTable[inodes[i][j]]];
	      C += (weights(j)/sum)*(Phis[nodeTable[inodes[i][j]]]*Ur_);
	    }
	    for(size_t j=0; j<inodes[i].size(); j++) {
	      SqrMat3 tr = tilde(r[nodeTable[inodes[i][j]]]-rr);
	      A += (weights(j)/sum)*JTJ(tr);
	      B += (weights(j)/sum)*tr*(Phis[nodeTable[inodes[i][j]]]*Ur_);
	    }
	    Jr.set(RangeV(0,2),RangeV(0,Jr.cols()-1), C);
	    Jr.set(RangeV(3,5),RangeV(0,Jr.cols()-1), slvLL(A,B));
	    Indices iHj(iHi.size());
	    for(size_t j=0; j<iHj.size(); j++)
	      iHj.set(j,j);
	    MatV Vi = slvLL(Ki(iHi),Jr(idof[i],iHj).T());
	    Ui.set(RangeV(0,Ui.rows()-1),RangeV(ni,ni+idof[i].size()-1), Ur*Vi);
	    ni += idof[i].size();
	  }
	}
      } else
	Ui <<= Ui_;

      if(nmodes.size() and fixedBoundaryNormalModes) {
	Mat V;
	Vec w;
	eigvec(Krns,Mrns,max(nmodes),1,V,w,0.01);
	vector<int> imod;
	for(size_t i=0; i<nmodes.size(); i++) {
	  if(w(nmodes[i]-1)>pow(2*M_PI*0.1,2))
	    imod.emplace_back(nmodes[i]-1);
	}
	Un.resize(iF.size(),imod.size(),NONINIT);
	for(size_t i=0; i<imod.size(); i++) {
	  Un.set(iN,i,V.col(imod[i]));
	  Un.set(iH,i,VecV(iH.size()));
	}
      }

      if(not Mm.size()) {
	delete Krns.Ip();
	delete Krns.Jp();
      }
    }

    if(nmodes.size() and not fixedBoundaryNormalModes) {
      Mat V;
      Vec w;
      eigvec(Krs,Mrs,max(nmodes),1,V,w,0.01);
      vector<int> imod;
      for(size_t i=0; i<nmodes.size(); i++) {
	if(w(nmodes[i]-1)>pow(2*M_PI*0.1,2))
	  imod.emplace_back(nmodes[i]-1);
      }
      Un.resize(iF.size(),imod.size(),NONINIT);
      for(size_t i=0; i<imod.size(); i++)
	Un.set(i,V.col(imod[i]));

      if(not Mm.size()) {
	delete Krs.Ip();
	delete Krs.Jp();
      }
    }

    U.resize(Ks.size(),Ui.cols() + Un.cols(),NONINIT);
    Indices IJ;
    for(int i=0; i<Ui.cols(); i++) {
      U.set(iF,i,Ui.col(i));
      U.set(iX,i,VecV(iX.size()));
    }
    for(int i=Ui.cols(), j=0; i<Ui.cols()+Un.cols(); i++, j++) {
      U.set(iF,i,Un.col(j));
      U.set(iX,i,VecV(iX.size()));
    }
  }

}
