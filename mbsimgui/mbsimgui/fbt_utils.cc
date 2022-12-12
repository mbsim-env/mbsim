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

}
