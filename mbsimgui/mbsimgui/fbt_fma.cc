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

  void FlexibleBodyTool::fma() {
    Phi.resize(nN,Mat3xV(nM,NONINIT));
    for(size_t i=0; i<nN; i++)
      Phi[i] = createPhis(ng,Phim[i])*U;

    if(Psim.size()) {
      Psi.resize(nN,Mat3xV(nM,NONINIT));
      for(size_t i=0; i<nN; i++)
	Psi[i] = createPhis(ng,Psim[i])*U;
    }

    if(sigm.size()) {
      sigmahel.resize(nN,Matrix<General,Fixed<6>,Var,double>(nM,NONINIT));
      for(size_t i=0; i<nN; i++)
	sigmahel[i] = createsigs(ng,sigm[i])*U;
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
