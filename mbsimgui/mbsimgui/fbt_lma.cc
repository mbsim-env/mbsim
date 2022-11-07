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
    Phi.resize(nN,Mat3xV(nM,NONINIT));
    for(size_t i=0; i<nN; i++)
      Phi[i] = Phi_(RangeV(nen*i,nen*i+net-1),RangeV(0,nM-1));

    if(Sr.rows()) {
      sigmahel.resize(nN,Matrix<General,Fixed<6>,Var,double>(nM,NONINIT));
      for(size_t i=0; i<nN; i++)
	sigmahel[i] = Sr(RangeV(6*i,6*i+6-1),RangeV(0,nM-1));
    }

    auto MKs = createMKs(MKm);
    Ke0 <<= JTMJ(MKs.second,Phi_);

    std::map<int,Vec3> nodalPos;
    if(r.cols()==4) {
      for(int i=0; i<r.rows(); i++)
	nodalPos[r(i,0)] = r.row(i)(RangeV(1,3)).T();
    }
    else if(r.cols()==3) {
      for(int i=0; i<r.rows(); i++)
	nodalPos[i+1] = r.row(i).T();
    }
    else
      runtime_error("(FlexibleBodyTool::init): number of columns in nodes does not match, must be 3 or 4");

    KrKP.resize(nN,Vec3(NONINIT));
    for(const auto & i : nodeMap)
      KrKP[i.second] = nodalPos[i.first];

    bool lumpedMass = true;
    // compute mass and lumped mass matrix
    vector<double> mi(nN);
    m = 0;
    if(M.cols()==3) {
      double ds = 0;
      for(int i=0; i<nN; i++) {
	int r = MKs.first.Ip()[i*nen];
	ds += MKs.first()[r];
	m += MKs.first()[r];
	for(int c=r+1; c<MKs.first.Ip()[i*nen+1]; c++)
	  m += 2*MKs.first()[c];
      }
      for(int i=0; i<nN; i++) {
	int r = MKs.first.Ip()[i*nen];
	mi[i] = MKs.first()[r]/ds*m;
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
	rdm += mi[i]*KrKP[i];
	rrdm += mi[i]*JTJ(KrKP[i].T());
	Pdm += mi[i]*Phi[i];
	for(int k=0; k<3; k++) {
	  rPdm[k] += mi[i]*KrKP[i](k)*Phi[i];
	  for(int l=k; l<3; l++) {
	    PPdm[k][l] += mi[i]*Phi[i].row(k).T()*Phi[i].row(l);
	  }
	}
      }
    }
    else {
      // compute reduced mass matrix
      if(not MKs.first.size())
	runtime_error("full mass approach not available");
      PPdm[0][0] = JTMJ(MKs.first,Phi_);
    }
  }

}
