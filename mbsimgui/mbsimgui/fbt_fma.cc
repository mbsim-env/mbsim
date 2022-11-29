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
#include "fe_type.h"
#include <iostream>

using namespace std;
using namespace fmatvec;

namespace MBSimGUI {

  void FlexibleBodyTool::fma() {
    Phi.resize(nN,Mat3xV(nM,NONINIT));
    for(size_t i=0; i<nN; i++)
      Phi[i] = createSparseMat(ng,Phim[i])*U;

    if(Psim.size()) {
      Psi.resize(nN,Mat3xV(nM,NONINIT));
      for(size_t i=0; i<nN; i++)
	Psi[i] = createSparseMat(ng,Psim[i])*U;
    }

    if(sigm.size()) {
      sigmahel.resize(nN,Matrix<General,Fixed<6>,Var,double>(nM,NONINIT));
      for(size_t i=0; i<nN; i++)
	sigmahel[i] = createSparseMat(ng,sigm[i])*U;
    }
    else if(sigem.size()) {
      sigmahel.resize(nN,Matrix<General,Fixed<6>,Var,double>(nM));
      vector<vector<vector<Matrix<General,Fixed<6>,Var,double>>>> sigmahel_(ele.size());
      for(int k=0; k<ele.size(); k++) {
	sigmahel_[k].resize(ele[k].rows(),vector<Matrix<General,Fixed<6>,Var,double>>(type[0]->getNumberOfNodes(),Matrix<General,Fixed<6>,Var,double>(nM)));
	for(int ee=0; ee<ele[k].rows(); ee++) {
	  for(int i=0; i<type[k]->getNumberOfIntegrationPoints(); i++) {
	    auto B = createSparseMat(ng,sigem[k][ee][i])*U;
	    for(int j=0; j<type[k]->getNumberOfExtrapolationPoints(); j++)
	      sigmahel_[k][ee][j] += type[k]->getExtrapolationCoefficient(i,j)*B;
	  }
	  for(int j=type[k]->getNumberOfExtrapolationPoints(); j<type[k]->getNumberOfNodes(); j++)
	    sigmahel_[k][ee][j] = 0.5*(sigmahel_[k][ee][type[k]->getExtrapolationIndex(j-type[k]->getNumberOfExtrapolationPoints(),0)] + sigmahel_[k][ee][type[k]->getExtrapolationIndex(j-type[k]->getNumberOfExtrapolationPoints(),1)]);
	  for(int j=0; j<type[k]->getNumberOfNodes(); j++)
	    sigmahel[nodeTable[ele[k](ee,j)]] += sigmahel_[k][ee][j]/double(nodeCount[ele[k](ee,j)]);
	}
      }
    }

    PPdm.resize(3,vector<SqrMatV>(3));
    auto PPKs = createPPKs(MKm);
    auto PPs = createPPs(PPm);
    Pdm <<= Pdm*U;
    for(int i=0; i<3; i++) {
      rPdm[i] <<= rPdm[i]*U;
      for(int j=0; j<3; j++)
	PPdm[j][j] <<= U.T()*(PPKs[j]*U);
      PPdm[0][1] <<= U.T()*(PPs[0]*U);
      PPdm[0][2] <<= U.T()*(PPs[1]*U);
      PPdm[1][2] <<= U.T()*(PPs[2]*U);
    }
    Ke0 <<= JTMJ(PPKs[3],U);
  }

}
