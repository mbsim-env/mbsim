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
    KrKP.resize(nN,Vec3(NONINIT));
    for(int i=0; i<nN; i++)
      KrKP[i] = r.row(i).T();

    Phi.resize(nN,Mat3xV(nM,NONINIT));
    for(size_t i=0; i<nN; i++)
      Phi[i] = Phis[i]*Phi_;

    if(Psis.size()) {
      Psi.resize(nN,Mat3xV(nM,NONINIT));
      for(size_t i=0; i<nN; i++)
	Psi[i] = Psis[i]*Phi_;
    }

    if(sigmahels.size()) {
      sigmahel.resize(nN,Matrix<General,Fixed<6>,Var,double>(nM,NONINIT));
      for(size_t i=0; i<nN; i++)
	sigmahel[i] = sigmahels[i]*Phi_;
    }

    PPdm.resize(3,vector<SqrMatV>(3));
    auto PPKs = createPPKs(MKm);
    auto PPs = createPPs(ng,PPm);
    Pdm <<= Pdm*Phi_;
    for(int i=0; i<3; i++) {
      rPdm[i] <<= rPdm[i]*Phi_;
      for(int j=0; j<3; j++)
	PPdm[j][j] <<= Phi_.T()*(PPKs[j]*Phi_);
      PPdm[0][1] <<= Phi_.T()*(PPs[0]*Phi_);
      PPdm[0][2] <<= Phi_.T()*(PPs[1]*Phi_);
      PPdm[1][2] <<= Phi_.T()*(PPs[2]*Phi_);
    }
    Ke0 <<= JTMJ(PPKs[3],Phi_);
  }

}
