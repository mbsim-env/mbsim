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

  int nonZeroElements(const MatV &A) {
    int n = 0;
    for(int i=0; i<A.rows(); i++) {
      for(int j=0; j<A.cols(); j++)
	if(abs(A.e(i,j))>1e-16)
	  n++;
    }
    return n;
  }

  int nonZeroElements(const SymMatV &A) {
    int n = 0;
    for(int i=0; i<A.rows(); i++) {
      for(int j=i; j<A.cols(); j++)
	if(abs(A.e(i,j))>1e-16)
	  n++;
    }
    return n;
  }

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
    auto *list = page<BoundaryConditionsPage>(PageBC)->bc->getWidget<ListWidget>();
    vector<vector<int>> dof(list->getSize());;
    vector<vector<int>> bnodes(list->getSize());
    for(int i=0; i<list->getSize(); i++) {
      auto *bcw = list->getWidget<BoundaryConditionWidget>(i);
      auto mat = bcw->getNodes()->getFirstWidget<VariableWidget>()->getEvalMat();
      bnodes[i].resize(mat.size());
      for(size_t j=0; j<mat.size(); j++)
	bnodes[i][j] = mat[j][0].toInt();
      mat = bcw->getDof()->getFirstWidget<VariableWidget>()->getEvalMat();
      dof[i].resize(mat.size());
      for(size_t j=0; j<mat.size(); j++)
	dof[i][j] = mat[j][0].toInt()-1;
    }

    FlexibleBodyTool::TypeOfConstraint typeOfConstraint = FlexibleBodyTool::distributing;
    if(page<ComponentModeSynthesisPage>(PageCMS)->typeOfConstraint->isActive()) {
      auto str = page<ComponentModeSynthesisPage>(PageCMS)->typeOfConstraint->getWidget<TextChoiceWidget>()->getText();
      if(str=="\"distributing\"") typeOfConstraint=FlexibleBodyTool::distributing;
      else if(str=="\"kinematic\"") typeOfConstraint=FlexibleBodyTool::kinematic;
    }

    list = page<ComponentModeSynthesisPage>(PageCMS)->idata->getWidget<ListWidget>();
    vector<vector<int>> inodes(list->getSize());
    vector<vector<double>> weights(list->getSize());
    vector<bool> reduceToNode(list->getSize());
    vector<Indices> idof(list->getSize());
    vector<int> snn(list->getSize());
    vector<vector<double>> prf(list->getSize());
    for(int i=0; i<list->getSize(); i++) {
      inodes[i] = list->getWidget<CMSDataWidget>(i)->getNodes();
      weights[i] = list->getWidget<CMSDataWidget>(i)->getWeights();
      if(weights[i].size()==0) {
	weights[i].resize(inodes[i].size());
	for(size_t j=0; j<weights[i].size(); j++)
	  weights[i][j] = 1;
      }
      reduceToNode[i] = list->getWidget<CMSDataWidget>(i)->getReduceToSingleNode();
      idof[i] = list->getWidget<CMSDataWidget>(i)->getDof();
      for(size_t j=0; j<idof[i].size(); j++)
	idof[i].set(j,idof[i][j]-1);
      snn[i] = list->getWidget<CMSDataWidget>(i)->getSingleNodeNumber();
      prf[i] = list->getWidget<CMSDataWidget>(i)->getPositionOfReferenceNode();
    }

    vector<int> nmodes;
    if(page<ComponentModeSynthesisPage>(PageCMS)->nmodes->isActive()) {
      auto mat = page<ComponentModeSynthesisPage>(PageCMS)->nmodes->getFirstWidget<VariableWidget>()->getEvalMat();
      nmodes.resize(mat.size());
      for(size_t i=0; i<mat.size(); i++)
	nmodes[i] = mat[i][0].toInt();
    }

    FlexibleBodyTool::NormalModes normalModes = FlexibleBodyTool::freeBoundaryNormalModes;
    if(page<ComponentModeSynthesisPage>(PageCMS)->normalModes->isActive()) {
      auto str = page<ComponentModeSynthesisPage>(PageCMS)->normalModes->getWidget<TextChoiceWidget>()->getText();
      if(str=="\"freeBoundaryNormalModes\"") normalModes=FlexibleBodyTool::freeBoundaryNormalModes;
      else if(str=="\"fixedBoundaryNormalModes\"") normalModes=FlexibleBodyTool::fixedBoundaryNormalModes;
      else if(str=="\"constrainedBoundaryNormalModes\"") normalModes=FlexibleBodyTool::constrainedBoundaryNormalModes;
    }

    if(bnodes.size() != dof.size())
      runtime_error("(FlexibleBodyTool::init): number of boundary nodes (" + to_string(bnodes.size()) + ") must equal number of degrees of freedom (" + to_string(dof.size()) + ")");

    int nN = r.size();
    int nen = net + ner;

    MatVI activeDof(nN,nen,NONINIT);
    for(int i=0; i<nN; i++) {
      for(int j=0; j<nen; j++)
	activeDof(i,j) = 1;
    }
    for(size_t i=0; i<bnodes.size(); i++) {
      for(size_t j=0; j<bnodes[i].size(); j++) {
	for(size_t k=0; k<dof[i].size(); k++)
	  activeDof(nodeTable[bnodes[i][j]],dof[i][k]) = 0;
      }
    }
    vector<int> dofMapF(nen*nN);
    for(size_t i=0, l=0, k=0; i<nN; i++) {
      for(size_t j=0; j<nen; j++) {
	if(activeDof(i,j))
	  dofMapF[l] = k++;
	l++;
      }
    }

    Indices iF, iX;
    for(int i=0; i<nN; i++) {
      for(int j=0; j<nen; j++) {
	if(activeDof(i,j))
	  iF.add(nen*i+j);
	else
	  iX.add(nen*i+j);
      }
    }

    MatV activeDof0 = activeDof;
    for(size_t i=0; i<inodes.size(); i++) {
      for(size_t j=0; j<inodes[i].size(); j++) {
	for(size_t k=0; k<nen; k++)
	  activeDof(nodeTable[inodes[i][j]],k) *= 2;
      }
    }
    Indices iH, iN;
    for(size_t i=0; i<nN; i++) {
      for(size_t j=0; j<nen; j++) {
	if(activeDof(i,j)==2)
	  iH.add(nen*i+j);
	else if(activeDof(i,j)==1)
	  iN.add(nen*i+j);
      }
    }

    MatV Ui, Un, D;
    SymSparseMat Ms, Mrcs, Krcs, Krns, Mrns;
    if(not Mm.size()) {
      Ms <<= PPdms[0];
      for(int i=0; i<Ms.nonZeroElements(); i++)
	Ms()[i] += PPdms[1]()[i]+PPdms[2]()[i];
    }
    if(iH.size()) {
      vector<int> dofMapH(nen*nN);
      for(size_t i=0, l=0, k=0; i<nN; i++) {
	for(size_t j=0; j<nen; j++) {
	  if(activeDof(i,j)==2)
	    dofMapH[l] = k++;
	  l++;
	}
      }
      vector<int> dofMapN(nen*nN);
      for(size_t i=0, l=0, k=0; i<nN; i++) {
	for(size_t j=0; j<nen; j++) {
	  if(activeDof(i,j)==1)
	    dofMapN[l] = k++;
	  l++;
	}
      }

      MatV Krnh;
      if(Mm.size()) {
	SymSparseMat SM1 = createSymSparseMat(reduceMat(Km,iN.size(),activeDof,dofMapN));
	Krns &= SM1;
	SymSparseMat SM2 = createSymSparseMat(reduceMat(Mm,iN.size(),activeDof,dofMapN));
	Mrns &= SM2;
	Krnh <<= reduceMat(Km,iN.size(),iH.size(),activeDof,dofMapN,dofMapH);
      }
      else {
	reduceMat(Ms,Ks,Mrns,Krns,iN.size(),activeDof,dofMapN);
	Krnh <<= reduceMat(Ks,iN.size(),iH.size(),activeDof,dofMapN,dofMapH);
      }

      int ni = 0;
      bool rdn = false;
      for(size_t i=0; i<inodes.size(); i++) {
	ni += idof[i].size();
	if(reduceToNode[i]) rdn = true;
      }

      if(rdn) {
	if(typeOfConstraint==kinematic) {
	  D.resize(iH.size(),ni);
	  ni = 0;
	  for(size_t i=0; i<inodes.size(); i++) {
	    singleNodeNumbers.push_back(snn[i]>-1?snn[i]:nodeNumbers.size()+1+singleNodeNumbers.size());
	    Vec3 rr;
	    if(prf[i].size())
	      rr = Vec3(prf[i]);
	    else {
	      for(size_t j=0; j<inodes[i].size(); j++)
		rr += (1./inodes[i].size())*r[nodeTable[inodes[i][j]]];
	    }
	    rif.push_back(rr);
	    for(size_t j=0; j<inodes[i].size(); j++) {
	      Mat3xV T(6,NONINIT);
	      T.set(RangeV(0,2),RangeV(0,2),SqrMat3(Eye()));
	      T.set(RangeV(0,2),RangeV(3,5),-tilde(r[nodeTable[inodes[i][j]]]-rr));
	      D.set(RangeV(dofMapH[3*nodeTable[inodes[i][j]]],dofMapH[3*nodeTable[inodes[i][j]]]+2),RangeV(ni,ni+idof[i].size()-1),T(RangeV(0,2),idof[i]));
	    }
	    ni += idof[i].size();
	  }
	  SymSparseMat Mrhs, Krhs;
	  MatV Mrnh;
	  if(Mm.size()) {
	    SymSparseMat SM1 = createSymSparseMat(reduceMat(Km,iH.size(),activeDof,dofMapH));
	    Krhs &= SM1;
	    SymSparseMat SM2 = createSymSparseMat(reduceMat(Mm,iH.size(),activeDof,dofMapH));
	    Mrhs &= SM2;
	    Mrnh <<= reduceMat(Mm,iN.size(),iH.size(),activeDof,dofMapN,dofMapH);
	  }
	  else {
	    reduceMat(Ms,Ks,Mrhs,Krhs,iH.size(),activeDof,dofMapH,2);
	    Mrnh <<= reduceMat(Ms,iN.size(),iH.size(),activeDof,dofMapN,dofMapH);
	  }
	  SymMatV Mrcc = JTMJ(Mrhs,D);
	  SymMatV Krcc = JTMJ(Krhs,D);
	  MatV Mrnc = Mrnh*D;
	  MatV Krnc = Krnh*D;
	  int nze = Krns.nonZeroElements();
	  nze += nonZeroElements(Krcc);
	  nze += nonZeroElements(Krnc);
	  int *Ip = new int[Krns.rows()+D.cols()+1];
	  int *Jp = new int[nze];
	  SymSparseMat Mrcs_(iN.size()+D.cols(),nze,Ip,Jp);
	  SymSparseMat Krcs_(iN.size()+D.cols(),nze,Ip,Jp);
	  Mrcs &= Mrcs_;
	  Krcs &= Krcs_;
	  int k = 0;
	  Krcs.Ip()[0] = 0;
	  for(int i=0; i<Krns.rows(); i++) {
	    for(int j=Krns.Ip()[i], h=Krcs.Ip()[i]; j<Krns.Ip()[i+1]; j++, h++) {
	      Mrcs()[h] = Mrns()[j];
	      Mrcs.Jp()[h] = Mrns.Jp()[j];
	      Krcs()[h] = Krns()[j];
	      Krcs.Jp()[h] = Krns.Jp()[j];
	    }
	    k += Krns.Ip()[i+1]-Krns.Ip()[i];
	    for(int j=0, h=Krns.cols(); j<Krnc.cols(); j++, h++) {
	      if(abs(Krnc.e(i,j))>1e-16) {
		Mrcs()[k] = Mrnc(i,j);
		Mrcs.Jp()[k] = h;
		Krcs()[k] = Krnc(i,j);
		Krcs.Jp()[k++] = h;
	      }
	    }
	    Mrcs.Ip()[i+1] = k;
	    Krcs.Ip()[i+1] = k;
	  }
	  for(int i=0, ii = Krns.rows(); i<Krcc.rows(); i++, ii++) {
	    for(int j=i, h=Krns.cols()+i; j<Krcc.cols(); j++, h++) {
	      if(abs(Krcc.e(i,j))>1e-16) {
		Mrcs()[k] = Mrcc(i,j);
		Mrcs.Jp()[k] = h;
		Krcs()[k] = Krcc(i,j);
		Krcs.Jp()[k++] = h;
	      }
	    }
	    Mrcs.Ip()[ii+1] = k;
	    Krcs.Ip()[ii+1] = k;
	  }
	  Ui.resize(Ks.size(),ni,NONINIT);
	  MatV Q = -slvLU(Krns,Krnc);
	  RangeV IJ(0,ni-1);
	  Ui.set(iN,IJ,Q);
	  Ui.set(iH,IJ,D);
	  Ui.set(iX,IJ,MatV(iX.size(),ni));
	}
	else {
	  Ui.resize(Ks.size(),ni,NONINIT);
	  ni = 0;
	  for(size_t i=0; i<inodes.size(); i++) {
	    MatV activeDofi = activeDof0;
	    for(size_t j=0; j<inodes.size(); j++) {
	      if(j!=i) {
		for(size_t k=0; k<inodes[j].size(); k++) {
		  for(size_t h=0; h<nen; h++)
		    activeDofi(nodeTable[inodes[j][k]],h) = 0;
		}
	      }
	    }
	    vector<int> dofMapHi(nen*nN);
	    for(size_t i=0, l=0, k=0; i<nN; i++) {
	      for(size_t j=0; j<nen; j++) {
		if(activeDofi(i,j)==1)
		  dofMapHi[l] = k++;
		l++;
	      }
	    }
	    Indices iHi;
	    for(size_t i=0; i<nN; i++) {
	      for(size_t j=0; j<nen; j++) {
		if(activeDofi(i,j)==1)
		  iHi.add(nen*i+j);
	      }
	    }
	    SymSparseMat Mris, Kris;
	    reduceMat(Ms,Ks,Mris,Kris,iHi.size(),activeDofi,dofMapHi,1);
	    singleNodeNumbers.push_back(snn[i]>-1?snn[i]:nodeNumbers.size()+1+singleNodeNumbers.size());
	    double sum = 0;
	    Vec3 rr;
	    SymMat3 A;
	    MatV fri(Kris.size(),6);
	    for(size_t j=0; j<inodes[i].size(); j++)
	      sum += weights[i][j];
	    for(size_t j=0; j<inodes[i].size(); j++)
	      rr += (weights[i][j]/sum)*r[nodeTable[inodes[i][j]]];
	    for(size_t j=0; j<inodes[i].size(); j++)
	      A += (weights[i][j]/sum)*JTJ(tilde(r[nodeTable[inodes[i][j]]]-rr));
	    for(size_t j=0; j<inodes[i].size(); j++) {
	      int ii = dofMapHi[3*nodeTable[inodes[i][j]]];
	      SymMatV B = (weights[i][j]/sum)*SymMatV(3,Eye());
	      fri.set(RangeV(ii,ii+2),RangeV(0,2),B);
	      fri.set(RangeV(ii,ii+2),RangeV(3,5),slvLL(A,tilde(r[nodeTable[inodes[i][j]]]-rr).T()*B));
	    }
	    Ui.set(iHi,RangeV(ni,ni+idof[i].size()-1), slvLU(Kris,fri(RangeV(0,fri.rows()-1),idof[i])));
	    ni += idof[i].size();
	  }
	}
      } else {
	Ui.resize(Ks.size(),iH.size(),NONINIT);
	RangeV IJ(0,iH.size()-1);
	Ui.set(iN,IJ,-slvLU(Krns,Krnh));
	Ui.set(iH,IJ,MatV(iH.size(),iH.size(),Eye()));
	Ui.set(iX,IJ,MatV(iX.size(),iH.size()));
      }
    }

    Mat V;
    Vec w;
    if(nmodes.size()) {
      if(normalModes==freeBoundaryNormalModes) {
	SymSparseMat Mrs, Krs;
	if(Mm.size()) {
	  auto Mrm = reduceMat(Mm,iF.size(),activeDof0,dofMapF);
	  auto Krm = reduceMat(Km,iF.size(),activeDof0,dofMapF);
	  SymSparseMat SM1 = createSymSparseMat(Mrm);
	  SymSparseMat SM2 = createSymSparseMat(Krm);
	  Mrs &= SM1;
	  Krs &= SM2;
	}
	else
	  reduceMat(Ms,Ks,Mrs,Krs,iF.size(),activeDof0,dofMapF);
	eigvec(Krs,Mrs,max(nmodes),1,V,w,0.01);
	Un.resize(Ks.size(),nmodes.size(),NONINIT);
	for(size_t i=0; i<nmodes.size(); i++) {
	  Un.set(iF,i,V.col(nmodes[i]-1));
	  Un.set(iX,i,VecV(iX.size()));
	}
	if(not Mm.size()) {
	  delete Krs.Ip();
	  delete Krs.Jp();
	}
      }
      else if(normalModes==fixedBoundaryNormalModes) {
	eigvec(Krns,Mrns,max(nmodes),1,V,w,0.01);
	Un.resize(Ks.size(),nmodes.size(),NONINIT);
	for(size_t i=0; i<nmodes.size(); i++) {
	  Un.set(iN,i,V.col(nmodes[i]-1));
	  Un.set(iH,i,VecV(iH.size()));
	  Un.set(iX,i,VecV(iX.size()));
	}
      }
      else if(normalModes==constrainedBoundaryNormalModes) {
	eigvec(Krcs,Mrcs,max(nmodes),1,V,w,0.01);
	Un.resize(Ks.size(),nmodes.size(),NONINIT);
	for(size_t i=0; i<nmodes.size(); i++) {
	  Un.set(iN,i,V.col(nmodes[i]-1)(RangeV(0,iN.size()-1)));
	  Un.set(iH,i,D*V.col(nmodes[i]-1)(RangeV(iN.size(),V.rows()-1)));
	  Un.set(iX,i,VecV(iX.size()));
	}
      }
    }

    if(typeOfConstraint==kinematic) {
      int ni = 0;
      SymMatV I(6,Eye());
      for(size_t i=0; i<inodes.size(); i++) {
	MatV PhiPsii(6,Ui.cols()+Un.cols());
	Matrix<General,Fixed<6>,Var,double> sigmaheli(Ui.cols()+Un.cols());
	PhiPsii.set(idof[i],RangeV(ni,ni+idof[i].size()-1),I(idof[i]));
	if(normalModes==constrainedBoundaryNormalModes) {
	  for(size_t j=0; j<nmodes.size(); j++)
	    PhiPsii.set(idof[i],Ui.cols()+j,V.col(nmodes[j]-1)(RangeV(iN.size()+ni,iN.size()+ni+idof[i].size()-1)));
	}
	ni += idof[i].size();
	Phiif.push_back(PhiPsii(RangeV(0,2),RangeV(0,PhiPsii.cols()-1)));
	Psiif.push_back(PhiPsii(RangeV(3,5),RangeV(0,PhiPsii.cols()-1)));
	sigmahelif.push_back(sigmaheli);
      }
    }

    if(not Mm.size()) {
      delete Krns.Ip();
      delete Krns.Jp();
    }

    if(Ui.cols()+Un.cols()) {
      U.resize(Ks.size(),Ui.cols()+Un.cols(),NONINIT);
      for(int i=0; i<Ui.cols(); i++)
	U.set(i,Ui.col(i));
      for(int i=Ui.cols(), j=0; i<Ui.cols()+Un.cols(); i++, j++)
	U.set(i,Un.col(j));
    }
    else {
      U.resize(Ks.size(),iF.size());
      int k = 0;
      for(int i=0; i<nN; i++) {
	for(int j=0; j<nen; j++) {
	  if(activeDof0(i,j))
	    U(i*nen+j,k++) = 1;
	}
      }
    }
  }

}
