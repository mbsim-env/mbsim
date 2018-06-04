/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <config.h>
#include "import.h"
#include <iostream>

using namespace std;
using namespace fmatvec;

namespace MBSimGUI {

  ImportFEMData::ImportFEMData(const string &jobname) {
    isRes.open(jobname+".frd");
    isStiff.open(jobname+".sti");
    isMass.open(jobname+".mas");
    isDof.open(jobname+".dof");
  }

  ImportFEMData::~ImportFEMData() {
    isRes.close();
    isStiff.close();
    isMass.close();
    isDof.close();
  }

  void ImportFEMData::read() {
    if(not isRes.is_open()) throw runtime_error("Result file does not exist.");
    if(not isStiff.is_open()) throw runtime_error("Stiffness matrix file does not exist.");
    if(not isMass.is_open()) throw runtime_error("Mass matrix file does not exist.");
    if(not isDof.is_open()) throw runtime_error("DOF file does not exist.");

    // count nodes
    string str;
    for(int i=0; i<12; i++)
      getline(isRes,str);
    isRes >> str >> nn;

    // read nodes
    getline(isRes,str);
    u0.resize(3*nn,NONINIT);
    double d;
    for(int i=0; i<nn; i++)
      isRes >> d >> d >> u0.e(3*i) >> u0.e(3*i+1) >> u0.e(3*i+2);

    // count elements
    for(int i=0; i<2; i++)
      getline(isRes,str);
    isRes >> str >> ne;

    // read elements
    eles.resize(ne,20,NONINIT);
    for(int i=0; i<ne; i++) {
      for(int j=0; j<2; j++)
        getline(isRes,str);
      isRes >> d;
      for(int j=0; j<10; j++)
        isRes >> eles.e(i,j);
      isRes >> d;
      for(int j=10; j<20; j++)
        isRes >> eles.e(i,j);
    }

    // read modes and stresses
    Phi.resize(3*nn,nm,NONINIT);
    Sr.resize(6*nn,nm,NONINIT);
    for(int j=0; j<nm; j++) {
      for(int i=0; i<14; i++)
        getline(isRes,str);
      for(int i=0; i<nn; i++) {
        isRes >> d >> d;
        for(int k=0; k<3; k++)
          isRes >> Phi.e(3*i+k,j);
      }
      for(int i=0; i<16; i++)
        getline(isRes,str);
      for(int i=0; i<nn; i++) {
        isRes >> d >> d;
        for(int k=0; k<6; k++)
          isRes >> Sr.e(6*i+k,j);
      }
    }

    // read dof table
    while(true) {
      isDof >> d;
      if(isDof.eof()) break;
      dof.push_back(make_pair(int(d)-1,int(d*10)-int(d)*10-1));
    }

    // read mass and stiffness matrix
    M.resize(dof.size());
    K.resize(dof.size());
    int i, j;
    while(true) {
      isMass >> i >> j >> d;
      if(isMass.eof()) break;
      M.e(i-1,j-1) = d;
    }
    while(true) {
      isStiff >> i >> j >> d;
      if(isStiff.eof()) break;
      K.e(i-1,j-1) = d;
    }

    // compute mass and lumped mass matrix
    mij.resize(M.size(),NONINIT);
    m.resize(3);
    for(size_t r=0; r<3; r++) {
      double ds = 0;
      for(size_t i=0; i<dof.size(); i++) {
        if(dof[i].second==r) {
          ds += M.e(i,i);
          m.e(r) += M.e(i,i);
          for(size_t j=i+1; j<dof.size(); j++) {
            if(dof[j].second==r)
              m.e(r) += 2*M.e(i,j);
          }
        }
      }
      for(size_t i=0; i<dof.size(); i++) {
        if(dof[i].second==r)
          mij(i) = M.e(i,i)/ds*m.e(r);
      }
    }

    // compute integrals
    Pdm.resize(nm);
    RangeV J = RangeV(0,nm-1);
    for(size_t i=0; i<nn; i++) {
      RangeV I = RangeV(3*i,3*i+2);
      Vec3 u0i = u0(I);
      Mat3xV Phii = Phi(I,J);
      double mi = mij.e(3*i);
      rdm += mi*u0i;
      rrdm += mi*JTJ(u0i.T());
      Pdm += mi*Phii;
    }
    rPdm.resize(9,nm);
    PPdm.resize(9*nm,nm);
    for(size_t k=0, h=0; k<3; k++) {
      RangeV K = RangeV(3*k,3*k+2);
      for(size_t i=0; i<nn; i++) {
        RangeV I = RangeV(3*i,3*i+2);
        double mi = mij.e(3*i);
        Mat3xV Phii = Phi(I,J);
        rPdm.add(K,J,mi*u0.e(3*i+k)*Phii);
      }
      for(size_t l=0; l<3; l++) {
        RangeV H = RangeV(nm*h,nm*h+nm-1);
        h++;
        for(size_t i=0; i<nn; i++) {
          double mi = mij.e(3*i);
          RowVecV Phik = Phi.row(3*i+k);
          RowVecV Phil = Phi.row(3*i+l);
          PPdm.add(H,J,mi*Phik.T()*Phil);
        }
      }
    }

    // compute reduced stiffness matrix
    Ke = JTMJ(K,Phi);

    // visualisation
    nodes.resize(nn,NONINIT);
    for(int i=0; i<nn; i++)
      nodes(i) = i+1;
    indices.resize(5*6*ne,NONINIT);
    j = 0;
    for(size_t i=0; i<eles.rows(); i++) {
      indices(j++) = eles(i,3);
      indices(j++) = eles(i,2);
      indices(j++) = eles(i,1);
      indices(j++) = eles(i,0);
      indices(j++) = -1;
      indices(j++) = eles(i,4);
      indices(j++) = eles(i,5);
      indices(j++) = eles(i,6);
      indices(j++) = eles(i,7);
      indices(j++) = -1;
      indices(j++) = eles(i,1);
      indices(j++) = eles(i,2);
      indices(j++) = eles(i,6);
      indices(j++) = eles(i,5);
      indices(j++) = -1;
      indices(j++) = eles(i,2);
      indices(j++) = eles(i,3);
      indices(j++) = eles(i,7);
      indices(j++) = eles(i,6);
      indices(j++) = -1;
      indices(j++) = eles(i,4);
      indices(j++) = eles(i,7);
      indices(j++) = eles(i,3);
      indices(j++) = eles(i,0);
      indices(j++) = -1;
      indices(j++) = eles(i,0);
      indices(j++) = eles(i,1);
      indices(j++) = eles(i,5);
      indices(j++) = eles(i,4);
      indices(j++) = -1;
    }

  }

}
