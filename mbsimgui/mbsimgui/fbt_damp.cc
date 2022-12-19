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
#include "variable_widgets.h"
#include "extended_widgets.h"

using namespace std;
using namespace fmatvec;

namespace MBSimGUI {

  void FlexibleBodyTool::damp() {
    if(static_cast<DampingPage*>(page(PageDamp))->mDamp->isActive()) {
      auto mat = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<DampingPage*>(page(PageDamp))->mDamp->getWidget())->getWidget())->getWidget()->getEvalMat();
      mDamp.resize(mat.size(),NONINIT);
      for(size_t i=0; i<mat.size(); i++)
	mDamp(i) = mat[i][0].toDouble();
    }

    if(static_cast<DampingPage*>(page(PageDamp))->pDamp->isActive()) {
      auto mat = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<DampingPage*>(page(PageDamp))->pDamp->getWidget())->getWidget())->getWidget()->getEvalMat();
      for(size_t i=0; i<mat.size(); i++)
	pDamp(i) = mat[i][0].toDouble();
    }
    if(mDamp.size()) {
      SquareMatrix<Ref,double> V;
      Vector<Ref,double> w;
      eigvec(Ke0,SymMatV(PPdm[0][0]+PPdm[1][1]+PPdm[2][2]),V,w);
      Pdm <<= Pdm*V;
      for(int i=0; i<3; i++) {
	rPdm[i] <<= rPdm[i]*V;
	for(int j=i; j<3; j++)
	  PPdm[i][j] <<= V.T()*PPdm[i][j]*V;
      }
      Ke0 <<= JTMJ(Ke0,V);
      for(size_t i=0; i<Phi.size(); i++)
	Phi[i] <<= Phi[i]*V;
      for(size_t i=0; i<sigmahel.size(); i++)
	sigmahel[i] <<= sigmahel[i]*V;
      De0.resize(V.cols(),INIT,0);
      for(int i=0; i<De0.size(); i++)
	De0(i,i) = 2*sqrt((PPdm[0][0](i,i)+PPdm[1][1](i,i)+PPdm[2][2](i,i))*Ke0(i,i))*(i>=mDamp.size()?mDamp(mDamp.size()-1):mDamp(i));
    }
    else if(pDamp.e(0)>0 or pDamp.e(1)>0)
      De0 <<= pDamp.e(0)*SymMatV(PPdm[0][0]+PPdm[1][1]+PPdm[2][2]) + pDamp.e(1)*Ke0;
  }

}
