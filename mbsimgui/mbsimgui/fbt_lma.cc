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

using namespace std;
using namespace fmatvec;

namespace MBSimGUI {

  void FlexibleBodyTool::lma() {
    int nN = r.size();
    int nM = U.cols();
    int nen = net + ner;
    Phi.resize(nN,Mat3xV(nM,NONINIT));
    for(size_t i=0; i<nN; i++)
      Phi[i] = U(RangeV(nen*i,nen*i+net-1),RangeV(0,nM-1));

    createSingleInterfaceNodes();

    if(S.rows()) {
      sigmahel.resize(nN,Matrix<General,Fixed<6>,Var,double>(nM,NONINIT));
      for(size_t i=0; i<nN; i++)
	sigmahel[i] = S(RangeV(6*i,6*i+6-1),RangeV(0,nM-1));
    }

    Ke0 <<= JTMJ(Ks,U);

    bool lumpedMass = true;
    // compute mass and lumped mass matrix
    vector<double> mi(nN);
    if(M.cols()==3) {
      double ds = 0;
      for(int i=0; i<nN; i++) {
	int r = PPdms[0].Ip()[i*nen];
	ds += PPdms[0]()[r];
	m += PPdms[0]()[r];
	for(int c=r+1; c<PPdms[0].Ip()[i*nen+1]; c++)
	  m += 2*PPdms[0]()[c];
      }
      for(int i=0; i<nN; i++) {
	int r = PPdms[0].Ip()[i*nen];
	mi[i] = PPdms[0]()[r]/ds*m;
      }
    } else if(M.cols()==1) {
      for(int i=0; i<M.rows(); i++) {
	mi[i] = M(i,0);
	m += mi[i];
      }
    }
    else
      runtime_error("lumped mass approach not available");

    Pdm.resize(nM);
    rPdm.resize(3,Mat3xV(nM));
    PPdm.resize(3,vector<SqrMatV>(3,SqrMatV(nM)));
    if(lumpedMass) {
      // compute integrals
      for(int i=0; i<nN; i++) {
	rdm += mi[i]*r[i];
	rrdm += mi[i]*JTJ(r[i].T());
	Pdm += mi[i]*Phi[i];
	for(int k=0; k<3; k++) {
	  rPdm[k] += mi[i]*r[i](k)*Phi[i];
	  for(int l=k; l<3; l++) {
	    PPdm[k][l] += mi[i]*Phi[i].row(k).T()*Phi[i].row(l);
	  }
	}
      }
    }
    else {
      // compute reduced mass matrix
      if(not PPdms[0].rows())
	runtime_error("full mass approach not available");
      PPdm[0][0] = JTMJ(PPdms[0],U);
    }
  }

}
