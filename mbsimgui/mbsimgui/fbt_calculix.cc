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
#include <QMessageBox>

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
    nodeNumbers.resize(nN);
    r.resize(nN);
    int dmax = 0;
    for(size_t i=0; i<nN; i++) {
      isRes >> d >> d;
      nodeNumbers[i] = d;
      if(d>dmax) dmax = d;
      for(size_t k=0; k<3; k++)
	isRes >> r[i](k);
    }
    nodeTable.resize(dmax+1);
    for(size_t i=0; i<nN; i++)
      nodeTable[nodeNumbers[i]] = i;
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
    size_t type, nNpE = 0;
    for(size_t i=0; i<nE; i++) {
      isRes >> str >> str >> type;
      if(type==4)
	nNpE = 20;
      else if(type==5)
	nNpE = 15;
      else if(type==6)
	nNpE = 10;
      else {
	QMessageBox::warning(this, "Flexible body tool", "Unknown element type.");
	return;
      }
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
	if(nN != nN_) {
	  QMessageBox::warning(this, "Flexible body tool", "Number of nodes does not match.");
	  return;
	}
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

    vector<vector<int>> ind;
    if(type==4) {
      ind.resize(6,vector<int>(4));
      ind[0][0] = 3; ind[0][1] = 2; ind[0][2] = 1; ind[0][3] = 0;
      ind[1][0] = 4; ind[1][1] = 5; ind[1][2] = 6; ind[1][3] = 7;
      ind[2][0] = 1; ind[2][1] = 2; ind[2][2] = 6; ind[2][3] = 5;
      ind[3][0] = 2; ind[3][1] = 3; ind[3][2] = 7; ind[3][3] = 6;
      ind[4][0] = 4; ind[4][1] = 7; ind[4][2] = 3; ind[4][3] = 0;
      ind[5][0] = 0; ind[5][1] = 1; ind[5][2] = 5; ind[5][3] = 4;
    }
    else if(type==5) {
      ind.resize(5);
      ind[0].resize(4);
      ind[1].resize(4);
      ind[2].resize(4);
      ind[3].resize(3);
      ind[4].resize(3);
      ind[0][0] = 0; ind[0][1] = 1; ind[0][2] = 4; ind[0][3] = 3;
      ind[1][0] = 0; ind[1][1] = 3; ind[1][2] = 5; ind[1][3] = 2;
      ind[2][0] = 2; ind[2][1] = 5; ind[2][2] = 4; ind[2][3] = 1;
      ind[3][0] = 0; ind[3][1] = 2; ind[3][2] = 1;
      ind[4][0] = 3; ind[4][1] = 4; ind[4][2] = 5;
    }
    else if(type==6) {
      ind.resize(4,vector<int>(3));
      ind[0][0] = 1; ind[0][1] = 2; ind[0][2] = 3;
      ind[1][0] = 0; ind[1][1] = 1; ind[1][2] = 3;
      ind[2][0] = 2; ind[2][1] = 0; ind[2][2] = 3;
      ind[3][0] = 2; ind[3][1] = 1; ind[3][2] = 0;
    }
    indices.resize(5*6*eles.rows());
    int oj = 0;
    for(int ee=0; ee<eles.rows(); ee++) {
      for(int i=0; i<ind.size(); i++) {
	for(int j=0; j<ind[i].size(); j++)
	  indices[oj++] = nodeTable[eles(ee,ind[i][j])];
	indices[oj++] = -1;
      }
    }
  }

}
