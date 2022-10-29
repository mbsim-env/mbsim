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
    string str = static_cast<FileWidget*>(static_cast<ExternalFiniteElementsPage*>(page(PageExtFE))->nodes->getWidget())->getFile(true).toStdString();
    if(!str.empty())
      r <<= readMat(str);
    str = static_cast<FileWidget*>(static_cast<ExternalFiniteElementsPage*>(page(PageExtFE))->mass->getWidget())->getFile(true).toStdString();
    if(!str.empty())
      M <<= readMat(str);
    str = static_cast<FileWidget*>(static_cast<ExternalFiniteElementsPage*>(page(PageExtFE))->stiff->getWidget())->getFile(true).toStdString();
    if(!str.empty())
      K <<= readMat(str);

    if(K.cols()==3) {
      if(M.cols()==K.cols()) {
	for(int i=0; i<K.rows(); i++) {
	  auto d = MKm[K(i,0)-1][K(i,1)-1];
	  d[0] = M(i,2);
	  d[3] = K(i,2);
	}
      }
      else {
	for(int i=0; i<K.rows(); i++) {
	  auto d = MKm[K(i,0)-1][K(i,1)-1];
	  d[3] = K(i,2);
	}
      }
    } else {
      for(int i=0; i<K.rows(); i++) {
	auto d = MKm[3*(K(i,2)-1)+K(i,3)-1][3*(K(i,0)-1)+K(i,1)-1];
	d[3] = K(i,4);
      }
    }
    ng = MKm.size();

    if(nodeMap.empty()) {
      for(int i=0; i<r.rows(); i++)
	nodeMap[i+1] = i;
    }
    nN = nodeMap.size();

    net = 3;
    ner = 0;
    nen = net + ner;
  }

}
