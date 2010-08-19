/* Copyright (C) 2004-2010 MBSim Development Team
 *
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 *  
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * Lesser General Public License for more details. 
 *  
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: thschindler@users.berlios.de
 */

#include<config.h>

#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_2s_13_mfr_mindlin.h"

#include<fstream>

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  FiniteElement2s13MFRMindlin::FiniteElement2s13MFRMindlin(double E_,double nu_,double rho_,double d0_,double d1_,double d2_,const fmatvec::Vec &NodeCoordinates) : DiscretizationInterface(),E(E_),nu(nu_),G(E/(2.*(1.+nu))),d0(d0_),d1(d1_),d2(d2_),rho(rho_),RefDofs(6),NodeDofs(3),Nodes(4),K(0),M_RR(0),N_compl(0),R_compl(0),R_ij(0) {
    for(int i=0;i<3;i++) 
      for(int j=0;j<3;j++) {
        N_ij[i][j]=0;
        NR_ij[i][j]=0;
      }

    this->NodeCoordinates = NodeCoordinates.copy();

    const double &r1 = NodeCoordinates(0);
    const double &phi1 = NodeCoordinates(1);
    const double &r2 = NodeCoordinates(2);
    const double &phi2 = NodeCoordinates(3);
    double beta = 0.2;
    double t_average = d0+d1*(r2+r1)/2.+d2*(pow(r2,3)-pow(r1,3))/((r2-r1)*3.);
    double l_e = (2.*(r2-r1)+(r1+r2)*(phi2-phi1))/M_PI;
    alphaS = 5./6.;
    if(t_average/l_e<beta) {
      alphaS *= pow(t_average/(l_e*beta),2);
    }
  }

  FiniteElement2s13MFRMindlin::~FiniteElement2s13MFRMindlin() {
    if(K!=0) delete K;
    if(M_RR!=0) delete M_RR;
    for(int i=0;i<3;i++) 
      for(int j=0;j<3;j++) {
        if(N_ij[i][j]!=0) delete N_ij[i][j];
        if(NR_ij[i][j]!=0) delete NR_ij[i][j];
      }
    if(N_compl!=0) delete N_compl;
    if(R_compl!=0) delete R_compl;
    if(R_ij!=0) delete R_ij;
  }    

  void FiniteElement2s13MFRMindlin::computeN_ij(int i, int j) {
    // mark: N_ij(1,3) = N_ij(2,3) = N_ij(3,1) = N_ij(3,2) = 0

    N_ij[i][j] = new SqrMat(NodeDofs*Nodes,INIT,0.);
    if(i==0){
      if(j==0) computeN_11();
      if(j==1) computeN_12();
    }
    else if(i==1){
      if(j==0) computeN_21();
      if(j==1) computeN_22();
    }
    else if(i==2){
      if(j==2) computeN_33();
    }
  }

  void FiniteElement2s13MFRMindlin::computeNR_ij(int i, int j) {
    // mark: NR_ij(1,1) = NR_ij(1,2) = NR_ij(2,1) = NR_ij(2,2) = NR_ij(3,1) = NR_ij(3,2) = NR_ij(3,3) = 0  

    NR_ij[i][j] = new RowVec(NodeDofs*Nodes,INIT,0.);
    if(i==0) {
      if(j==2) computeNR_13();
    }
    else if(i==1){
      if(j==2) computeNR_23();
    }
  }

}

