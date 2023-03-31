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

using namespace std;
using namespace fmatvec;

namespace MBSimGUI {

  void FlexibleBodyTool::calculix() {
    net = 3;
    ner = 0;
    string resultFileName = static_cast<FileWidget*>(static_cast<CalculixPage*>(page(PageCalculix))->file->getWidget())->getFile(true).toStdString();
    string jobname = resultFileName.substr(0,resultFileName.length()-4);

    ifstream isDOF(jobname+".dof");
    // dof
    std::vector<std::pair<size_t,size_t>> dof;
    double d;
    while(true) {
      isDOF >> d;
      if(isDOF.eof()) break;
      dof.push_back(make_pair(size_t(d)-1,int(d*10)-size_t(d)*10-1));
    }
    isDOF.close();

    ifstream isRes(jobname+".frd");
    // nodes
    string str;
    while(isRes) {
      getline(isRes,str);
      if(str.length()>6 and str.substr(4,2)=="2C")
	break;
    }
    int nN;
    stringstream sN(str);
    sN >> str >> nN;
    nodeTable.resize(nN+1);
    nodeNumbers.resize(nN);
    r.resize(nN);
    for(size_t i=0; i<nN; i++) {
      isRes >> d >> d;
      nodeTable[d] = i;
      nodeNumbers[i] = d;
      for(size_t k=0; k<3; k++)
	isRes >> r[i](k);
    }
    // elements
    while(isRes) {
      getline(isRes,str);
      if(str.length()>6 and str.substr(4,2)=="3C")
	break;
    }
    int nE;
    stringstream sE(str);
    sE >> str >> nE;
    Matrix<General,Var,Var,int> eles(nE,20,NONINIT);
    size_t type, nNpE;
    for(size_t i=0; i<nE; i++) {
      isRes >> str >> str >> type;
      if(type==4)
	nNpE = 20;
      else
	runtime_error("Unknown element type.");
      getline(isRes,str);
      for(size_t j=0; j<nNpE;) {
	isRes >> d;
	if(d>0) {
	  eles.e(i,j) = d;
	  j++;
	}
      }
      getline(isRes,str);
    }
    std::vector<VecV> disp;
    std::vector<VecV> stress;
    size_t i, nN_;
    while(isRes) {
      getline(isRes,str);
      if(str.length()>6 and str.substr(2,4)=="100C") {
	//        cout << str[57] << endl;
	stringstream s(str);
	s >> str >> str >> str >> nN_;
	if(nN != nN_) runtime_error("Number of nodes does not match.");
	isRes >> i >> str;
	if(str=="DISP") {
	  VecV dispi(3*nN,NONINIT);
	  double d;
	  string str;
	  for(size_t i=0; i<5; i++)
	    getline(isRes,str);
	  for(size_t i=0; i<nN; i++) {
	    isRes >> d >> d;
	    for(size_t k=0; k<3; k++)
	      isRes >> dispi.e(3*i+k);
	  }
	  disp.push_back(dispi);
	}
	else if(str=="STRESS") {
	  VecV stressi(6*nN,NONINIT);
	  double d;
	  string str;
	  for(size_t i=0; i<7; i++)
	    getline(isRes,str);
	  for(size_t i=0; i<nN; i++) {
	    isRes >> d >> d;
	    for(size_t k=0; k<6; k++)
	      isRes >> stressi.e(6*i+k);
	  }
	  stress.push_back(stressi);
	}
      }
    }
    int nM = disp.size();
    isRes.close();

    M <<= readMat(jobname+".mas");
    K <<= readMat(jobname+".sti");

    int ng = K(K.rows()-1,1);

    Km.resize(ng);
    Mm.resize(ng);
    for(int i=0; i<K.rows(); i++) {
      Mm[K(i,0)-1][K(i,1)-1] = M(i,2);
      Km[K(i,0)-1][K(i,1)-1] = K(i,2);
    }

    Ks <<= createSymSparseMat(Km);
    PPdms[0] <<= createSymSparseMat(Mm);

    U.resize(3*nN,nM,NONINIT);
    S.resize(6*nN,nM,NONINIT);
    for(int j=0; j<U.cols(); j++) {
      for(int i=0; i<U.rows(); i++)
	U.e(i,j) = disp[j].e(i);
      for(int i=0; i<S.rows(); i++)
	S.e(i,j) = stress[j].e(i);
    }

    indices.resize(5*6*eles.rows());
    int j = 0;
    for(int i=0; i<eles.rows(); i++) {
      indices[j++] = nodeTable[eles(i,3)];
      indices[j++] = nodeTable[eles(i,2)];
      indices[j++] = nodeTable[eles(i,1)];
      indices[j++] = nodeTable[eles(i,0)];
      indices[j++] = -1;
      indices[j++] = nodeTable[eles(i,4)];
      indices[j++] = nodeTable[eles(i,5)];
      indices[j++] = nodeTable[eles(i,6)];
      indices[j++] = nodeTable[eles(i,7)];
      indices[j++] = -1;
      indices[j++] = nodeTable[eles(i,1)];
      indices[j++] = nodeTable[eles(i,2)];
      indices[j++] = nodeTable[eles(i,6)];
      indices[j++] = nodeTable[eles(i,5)];
      indices[j++] = -1;
      indices[j++] = nodeTable[eles(i,2)];
      indices[j++] = nodeTable[eles(i,3)];
      indices[j++] = nodeTable[eles(i,7)];
      indices[j++] = nodeTable[eles(i,6)];
      indices[j++] = -1;
      indices[j++] = nodeTable[eles(i,4)];
      indices[j++] = nodeTable[eles(i,7)];
      indices[j++] = nodeTable[eles(i,3)];
      indices[j++] = nodeTable[eles(i,0)];
      indices[j++] = -1;
      indices[j++] = nodeTable[eles(i,0)];
      indices[j++] = nodeTable[eles(i,1)];
      indices[j++] = nodeTable[eles(i,5)];
      indices[j++] = nodeTable[eles(i,4)];
      indices[j++] = -1;
    }
  }

}