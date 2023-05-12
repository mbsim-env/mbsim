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

  void FlexibleBodyTool::extfe() {
    MatV R;
    string str = static_cast<FileWidget*>(static_cast<ExternalFiniteElementsPage*>(page(PageExtFE))->nodes->getWidget())->getFile(true).toStdString();
    if(!str.empty())
      R <<= readMat(str);
    str = static_cast<FileWidget*>(static_cast<ExternalFiniteElementsPage*>(page(PageExtFE))->mass->getWidget())->getFile(true).toStdString();
    if(!str.empty())
      M <<= readMat(str);
    str = static_cast<FileWidget*>(static_cast<ExternalFiniteElementsPage*>(page(PageExtFE))->stiff->getWidget())->getFile(true).toStdString();
    if(!str.empty())
      K <<= readMat(str);

    int max=0;
    if(R.cols()==3)
      max = R.rows();
    else if(R.cols()==4) {
      for(int i=0; i<R.rows(); i++) {
	if(R(i,0)>max)
	  max = R(i,0);
      }
    }

    nodeTable.resize(max+1);
    nodeNumbers.resize(R.rows());
    r.resize(R.rows(),Vec3(NONINIT));
    if(R.cols()==3) {
      for(int i=0; i<R.rows(); i++) {
	nodeTable[i+1] = i;
	r[nodeTable[i+1]] = R.row(i).T();
	nodeNumbers[i] = i+1;
      }
    }
    else {
      for(int i=0; i<R.rows(); i++) {
	nodeTable[R(i,0)] = i;
	r[nodeTable[R(i,0)]] = R.row(i)(RangeV(1,3)).T();
	nodeNumbers[nodeTable[R(i,0)]] = R(i,0);
      }
    }

    int ng = K.cols()==3?K(K.rows()-1,1):3*(K(K.rows()-1,2)-1)+K(K.rows()-1,3);
    Km.resize(ng);
    Mm.resize(ng);
    if(K.cols()==3) {
      if(M.cols()==K.cols()) {
	for(int i=0; i<K.rows(); i++) {
	  Mm[K(i,0)-1][K(i,1)-1] = M(i,2);
	  Km[K(i,0)-1][K(i,1)-1] = K(i,2);
	}
      }
      else {
	for(int i=0; i<K.rows(); i++)
	  Km[K(i,0)-1][K(i,1)-1] = K(i,2);
      }
    } else {
      for(int i=0; i<K.rows(); i++)
	Km[3*(K(i,2)-1)+K(i,3)-1][3*(K(i,0)-1)+K(i,1)-1] = K(i,4);
    }

    Ks <<= createSymSparseMat(Km);
    PPdms[0] <<= createSymSparseMat(Mm);

    net = 3;
    ner = 0;
  }

}
