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

  MatV FlexibleBodyTool::reduceMat(const vector<map<int,double[4]>> &Km, const Indices &iN, const Indices &iH) {
    vector<int> mapR(Km.size()), mapC(Km.size());
    size_t hN=0, kN=0, hH=0, kH=0;
    for(size_t i=0; i<mapR.size(); i++) {
      if(hN<iN.size() and i==iN[hN]) {
	hN++;
	mapR[i] = kN++;
      }
      else
	mapR[i] = -1;
      if(hH<iH.size() and i==iH[hH]) {
	hH++;
	mapC[i] = kH++;
      }
      else
	mapC[i] = -1;
    }
    MatV Kmr(iN.size(),iH.size());
    int k = 0;
    for(const auto & i : Km) {
      for(const auto & j : i) {
	if(mapR[k]>=0 and mapC[j.first]>=0)
	  Kmr(mapR[k],mapC[j.first]) = j.second[3];
	else if(mapR[j.first]>=0 and mapC[k]>=0)
	  Kmr(mapR[j.first],mapC[k]) = j.second[3];
      }
      k++;
    }
    return Kmr;
  }

  vector<map<int,double[4]>> FlexibleBodyTool::reduceMat(const vector<map<int,double[4]>> &MKm, const Indices &iF) {
    vector<map<int,double[4]>> MKmr;
    vector<int> map(MKm.size());
    size_t h=0, k=0;
    for(size_t i=0; i<map.size(); i++) {
      if(h<iF.size() and i==iF[h]) {
	h++;
	map[i] = k++;
      }
      else
	map[i] = -1;
    }
    MKmr.resize(k);
    k=0;
    for(const auto & i : MKm) {
      if(map[k]>=0) {
	for(const auto & j : i) {
	  if(map[j.first]>=0) {
	    auto d = MKmr[map[k]][map[j.first]];
	    for(int l=0; l<4; l++)
	      d[l] = j.second[l];
	  }
	}
      }
      k++;
    }
    return MKmr;
  }

  vector<SymSparseMat> FlexibleBodyTool::createPPKs(const vector<map<int,double[4]>> &PPKm) {
    int nze=0;
    for(const auto & i : PPKm)
      nze+=i.size();
    vector<SymSparseMat> PPKs(4,SymSparseMat(PPKm.size(),nze,NONINIT));
    int k=0, l=0;
    for(int h=0; h<4; h++)
      PPKs[h].Ip()[0] = 0;
    for(const auto & i : PPKm) {
      for(const auto & j : i) {
	for(int h=0; h<4; h++) {
	  PPKs[h].Jp()[l] = j.first;
	  PPKs[h]()[l] = j.second[h];
	}
	l++;
      }
      k++;
      for(int h=0; h<4; h++)
	PPKs[h].Ip()[k] = l;
    }
    return PPKs;
  }

  pair<SymSparseMat,SymSparseMat> FlexibleBodyTool::createMKs(const vector<map<int,double[4]>> &MKm) {
    int nze=0;
    for(const auto & i : MKm)
      nze+=i.size();
    SymSparseMat Ks(MKm.size(),nze,NONINIT);;
    SymSparseMat Ms(MKm.size(),nze,NONINIT);;
    int k=0, l=0;
    Ms.Ip()[0] = 0;
    Ks.Ip()[0] = 0;
    for(const auto & i : MKm) {
      for(const auto & j : i) {
	Ms.Jp()[l] = j.first;
	Ms()[l] = j.second[0]+j.second[1]+j.second[2];
	Ks.Jp()[l] = j.first;
	Ks()[l] = j.second[3];
	l++;
      }
      k++;
      Ms.Ip()[k] = l;
      Ks.Ip()[k] = l;
    }
    return make_pair(Ms,Ks);
  }

  vector<SparseMat> FlexibleBodyTool::createPPs(const vector<map<int,double[3]>> &PPm) {
    int nze=0;
    for(const auto & i : PPm)
      nze+=i.size();
    vector<SparseMat> PPs(3,SparseMat(PPm.size(),PPm.size(),nze,NONINIT));
    int k=0, l=0;
    for(int h=0; h<3; h++)
      PPs[h].Ip()[0] = 0;
    for(const auto & i : PPm) {
      for(const auto & j : i) {
	for(int h=0; h<3; h++) {
	  PPs[h].Jp()[l] = j.first;
	  PPs[h]()[l] = j.second[h];
	}
	l++;
      }
      k++;
      for(int h=0; h<3; h++)
	PPs[h].Ip()[k] = l;
    }
    return PPs;
  }

  SparseMat FlexibleBodyTool::createSparseMat(int n, const vector<map<int,double>> &Phim) {
    int nze=0;
    for(const auto & i : Phim)
      nze+=i.size();
    SparseMat Phis(Phim.size(),n,nze,NONINIT);
    int k=0, l=0;
    Phis.Ip()[0] = 0;
    for(const auto & i : Phim) {
      for(const auto & j : i) {
	Phis.Jp()[l] = j.first;
	Phis()[l] = j.second;
	l++;
      }
      k++;
      Phis.Ip()[k] = l;
    }
    return Phis;
  }

}
