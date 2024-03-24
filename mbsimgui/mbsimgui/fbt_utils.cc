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
#include <iostream>

using namespace std;
using namespace fmatvec;

namespace MBSimGUI {

  MatV FlexibleBodyTool::readMat(const string &file) {
    ifstream is(file);
    string buf, buf2;
    getline(is,buf);
    int m=0;
    while(!is.eof()) {
      getline(is,buf2);
      m++;
    }
    is.close();

    istringstream iss(buf);
    double val;
    char s;
    int n=0;
    while(!iss.eof()) {
      iss >> val;
      int c = iss.peek();
      if(c==44 or c==59 or c==13)
	iss >> s;
      n++;
    }

    MatV A(m,n);
    is.open(file);
    for(int i=0; i<m; i++) {
      for(int j=0; j<n; j++) {
	is >> A(i,j);
	int c = is.peek();
	if(c==44 or c==59)
	  is >> s;
      }
    }
    is.close();

    return A;
  }

  SymSparseMat FlexibleBodyTool::createSymSparseMat(const vector<map<int,double>> &Am) {
    int nze=0;
    for(const auto & i : Am)
      nze+=i.size();
    SymSparseMat As(Am.size(),nze,NONINIT);
    int k=0, l=0;
    As.Ip()[0] = 0;
    for(const auto & i : Am) {
      for(const auto & j : i) {
	As.Jp()[l] = j.first;
	As()[l] = j.second;
	l++;
      }
      k++;
      As.Ip()[k] = l;
    }
    return As;
  }

  SparseMat FlexibleBodyTool::createSparseMat(int n, const vector<map<int,double>> &Am) {
    int nze=0;
    for(const auto & i : Am)
      nze+=i.size();
    SparseMat As(Am.size(),n,nze,NONINIT);
    int k=0, l=0;
    As.Ip()[0] = 0;
    for(const auto & i : Am) {
      for(const auto & j : i) {
	As.Jp()[l] = j.first;
	As()[l] = j.second;
	l++;
      }
      k++;
      As.Ip()[k] = l;
    }
    return As;
  }

  vector<map<int,double>> FlexibleBodyTool::reduceMat(const vector<map<int,double>> &Am, int n, const MatVI &activeDof, const vector<int> &dofMap, int val) {
    vector<map<int,double>> Amr(n);
    int ii = 0;
    for(const auto & i : Am) {
      if(activeDof.e(ii)==val) {
	for(const auto & j : i) {
	  if(activeDof.e(j.first)==val)
	    Amr[dofMap[ii]][dofMap[j.first]] = j.second;
	}
      }
      ii++;
    }
    return Amr;
  }

  MatV FlexibleBodyTool::reduceMat(const vector<map<int,double>> &Am, int m, int n, const MatVI &activeDof, const vector<int> &dofMapN, const vector<int> &dofMapH) {
    MatV Krnh(m,n);
    int ii = 0;
    for(const auto & i : Am) {
      if(activeDof.e(ii)==1) {
	for(const auto & j : i) {
	  if(activeDof.e(j.first)==2)
	    Krnh(dofMapN[ii],dofMapH[j.first]) = j.second;
	}
      }
      else if(activeDof.e(ii)==2) {
	for(const auto & j : i) {
	  if(activeDof.e(j.first)==1)
	    Krnh(dofMapN[j.first],dofMapH[ii]) = j.second;
	}
      }
      ii++;
    }
    return Krnh;
  }

  void FlexibleBodyTool::reduceMat(const SymSparseMat &Ms, const SymSparseMat &Ks, SymSparseMat &Mrs, SymSparseMat &Krs, int n, const MatVI &activeDof, const vector<int> &dofMap, int val) {
    int nen = net + ner;
    int nzer = 0;
    for(size_t i=0; i<r.size(); i++) {
      for(int k=0; k<nen; k++) {
	if(activeDof(i,k)==val) {
	  for(int l=k; l<nen; l++) {
	    if(activeDof(i,l)==val)
	      nzer++;
	  }
	  for(const auto & j : links[i]) {
	    for(int l=0; l<nen; l++) {
	      if(activeDof(j.first,l)==val)
		nzer++;
	    }
	  }
	}
      }
    }
    int *Ipr = new int[n+1];
    int *Jpr = new int[nzer];
    Ipr[0] = 0;
    SymSparseMat SM1 = SymSparseMat(n,nzer,Ipr,Jpr);
    SymSparseMat SM2 = SymSparseMat(n,nzer,Ipr,Jpr);
    Mrs &= SM1;
    Krs &= SM2;
    int ii = 0, kk = 0, ll = 0;
    for(size_t i=0; i<r.size(); i++) {
      for(int k=0; k<nen; k++) {
	if(activeDof(i,k)==val) {
	  for(int l=k; l<nen; l++) {
	    if(activeDof(i,l)==val) {
	      Mrs()[ll] = Ms()[kk];
	      Krs()[ll] = Ks()[kk];
	      Jpr[ll++] = dofMap[nen*i+l];
	    }
	    kk++;
	  }
	  for(const auto & j : links[i]) {
	    for(int l=0; l<nen; l++) {
	      if(activeDof(j.first,l)==val) {
		Mrs()[ll] = Ms()[kk];
		Krs()[ll] = Ks()[kk];
		Jpr[ll++] = dofMap[nen*j.first+l];
	      }
	      kk++;
	    }
	  }
	  Ipr[++ii] = ll;
	}
	else
	  kk += nen-k+links[i].size()*nen;
      }
    }
  }

  MatV FlexibleBodyTool::reduceMat(const SymSparseMat &Ks, int m, int n, const MatVI &activeDof, const vector<int> &dofMapN, const vector<int> &dofMapH) {
    MatV Krnh(m,n);
    int nen = net + ner;
    int kk = 0;
    for(size_t i=0; i<r.size(); i++) {
      for(int k=0; k<nen; k++) {
	if(activeDof(i,k)==1) {
	  for(int l=k; l<nen; l++) {
	    if(activeDof(i,l)==2)
	      Krnh(dofMapN[nen*i+k],dofMapH[nen*i+l]) = Ks()[kk];
	    kk++;
	  }
	  for(const auto & j : links[i]) {
	    for(int l=0; l<nen; l++) {
	      if(activeDof(j.first,l)==2)
		Krnh(dofMapN[nen*i+k],dofMapH[nen*j.first+l]) = Ks()[kk];
	      kk++;
	    }
	  }
	}
	else if(activeDof(i,k)==2) {
	  for(int l=k; l<nen; l++) {
	    if(activeDof(i,l)==1)
	      Krnh(dofMapN[nen*i+l],dofMapH[nen*i+k]) = Ks()[kk];
	    kk++;
	  }
	  for(const auto & j : links[i]) {
	    for(int l=0; l<nen; l++) {
	      if(activeDof(j.first,l)==1)
		Krnh(dofMapN[nen*j.first+l],dofMapH[nen*i+k]) = Ks()[kk];
	      kk++;
	    }
	  }
	}
	else
	  kk += nen-k+links[i].size()*nen;
      }
    }
    return Krnh;
  }

  void FlexibleBodyTool::createSingleInterfaceNodes() {
    int nM = U.cols();
    auto *list = page<ComponentModeSynthesisPage>(PageCMS)->idata->getWidget<ListWidget>();
    FlexibleBodyTool::TypeOfConstraint typeOfConstraint = FlexibleBodyTool::distributing;
    if(page<ComponentModeSynthesisPage>(PageCMS)->typeOfConstraint->isActive()) {
      auto str = page<ComponentModeSynthesisPage>(PageCMS)->typeOfConstraint->getWidget<TextChoiceWidget>()->getText();
      if(str=="\"distributing\"") typeOfConstraint=FlexibleBodyTool::distributing;
      else if(str=="\"kinematic\"") typeOfConstraint=FlexibleBodyTool::kinematic;
    }
    if(typeOfConstraint==distributing) {
      for(int i=0; i<list->getSize(); i++) {
	auto reduceToNode = list->getWidget<CMSDataWidget>(i)->getReduceToSingleNode();
	if(reduceToNode) {
	  auto inodes = list->getWidget<CMSDataWidget>(i)->getNodes();
	  auto weights = list->getWidget<CMSDataWidget>(i)->getWeights();
	  if(weights.size()==0) {
	    weights.resize(inodes.size());
	    for(size_t j=0; j<weights.size(); j++)
	      weights[j] = 1;
	  }
	  double sum = 0;
	  for(size_t j=0; j<inodes.size(); j++)
	    sum += weights[j];
	  Vec3 ri;
	  Mat3xV Phii(nM);
	  Mat3xV Psii(nM,NONINIT);
	  Matrix<General,Fixed<6>,Var,double> sigmaheli(nM);
	  for(size_t j=0; j<inodes.size(); j++) {
	    ri += (weights[j]/sum)*r[nodeTable[inodes[j]]];
	    Phii += (weights[j]/sum)*Phi[nodeTable[inodes[j]]];
	  }
	  SymMat3 A;
	  Mat3xV B(nM);
	  for(size_t j=0; j<inodes.size(); j++) {
	    SqrMat3 tr = tilde(r[nodeTable[inodes[j]]]-ri);
	    A += (weights[j]/sum)*JTJ(tr);
	    B += (weights[j]/sum)*tr*(Phi[nodeTable[inodes[j]]]);
	  }
	  Psii = slvLL(A,B);
	  rif.push_back(ri);
	  Phiif.push_back(Phii);
	  Psiif.push_back(Psii);
	  sigmahelif.push_back(sigmaheli);
	}
      }
    }
  }

}
