/* Copyright (C) 2004-2009 MBSim Development Team
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

#include "mbsim/flexible_body/finite_elements/finite_element_2s_13_mfr_mindlin.h"

#include<fstream>

using namespace std;
using namespace fmatvec;

namespace MBSim {

  inline double Power(const double base, const int exponent) {
    return pow(base,exponent);
  }

  FiniteElement2s13MFRMindlin::FiniteElement2s13MFRMindlin(double E_,double nu_,double rho_, double d0_, double d1_, double d2_) : DiscretizationInterface(),E(E_),nu(nu_),G(E/(2.*(1.+nu))),E1(E_*(1.-nu_)/((1.+nu_)*(1.-2.*nu_))),E2(E_*nu_/((1.+nu_)*(1.-2.*nu_))),d0(d0_),d1(d1_),d2(d2_),rho(rho_),alphaS(5./6.),RefDofs(6),NodeDofs(3),Nodes(4),K(NodeDofs*Nodes,INIT,0.) {
    M_RR    = SymMat(3,INIT,0.);
    N_compl = Mat(3,NodeDofs*Nodes,INIT,0.);
    for(int i=0; i<3; i++)
      for(int j=0; j<3; j++) {
        N_ij[i][j]  = SqrMat(NodeDofs*Nodes,INIT,0.);
        NR_ij[i][j] = RowVec(NodeDofs*Nodes,INIT,0.);
      }
    R_compl = Vec(3,INIT,0.);
    R_ij    = SymMat(3,INIT,0.);
  }

  void FiniteElement2s13MFRMindlin::computeConstantSystemMatrices(const Vec &NodeCoordinates) {

    computeStiffnesMatrix(NodeCoordinates);
    computeM_RR(NodeCoordinates);
    computeN_compl(NodeCoordinates);
    computeN_ij(NodeCoordinates);
    computeNR_ij(NodeCoordinates);
    computeR_compl(NodeCoordinates);
    computeR_ij(NodeCoordinates);
  }

  void FiniteElement2s13MFRMindlin::computeN_ij(const fmatvec::Vec &NodeCoordinates) {
    //Mark: N_ij(1,3) = N_ij(2,3) = N_ij(3,1) = N_ij(3,2) = 0

	  computeN_11(NodeCoordinates);
	  computeN_12(NodeCoordinates);
	  computeN_21(NodeCoordinates);
	  computeN_22(NodeCoordinates);
	  computeN_33(NodeCoordinates);
  }

  void FiniteElement2s13MFRMindlin::computeNR_ij(const fmatvec::Vec &NodeCoordinates) {
    //Mark: 0 = NR_ij(1,1) = NR_ij(1,2) = NR_ij(2,1) = NR_ij(2,2) = NR_ij(3,1) = NR_ij(3,2) = NR_ij(3,3) = 0  
  
	  computeNR_13(NodeCoordinates);
	  computeNR_23(NodeCoordinates);
  }


  Vec FiniteElement2s13MFRMindlin::computeState(const Vec &NodeCoordinates,const Vec &qElement,const Vec &qpElement,const Vec &s,double d1,double d2) {
    // state vector
    Vec X(12);

    const double &r1 = NodeCoordinates(0);
    const double &j1 = NodeCoordinates(1);
    const double &r2 = NodeCoordinates(2);
    const double &j2 = NodeCoordinates(3);

    const double &r = s(0);
    const double &j = s(1);

    const double &zS = qElement(0);       const double &zSp = qpElement(0);
    const double &alpha = qElement(1);    const double &alphap = qpElement(1);

    const double &w1 = qElement(2);  const double &w1p = qpElement(2);
    const double &a1 = qElement(3);  const double &a1p = qpElement(3);
    const double &b1 = qElement(4);  const double &b1p = qpElement(4);
    const double &w2 = qElement(5);  const double &w2p = qpElement(5);
    const double &a2 = qElement(6);  const double &a2p = qpElement(6);
    const double &b2 = qElement(7);  const double &b2p = qpElement(7);
    const double &w3 = qElement(8);  const double &w3p = qpElement(8);
    const double &a3 = qElement(9);  const double &a3p = qpElement(9);
    const double &b3 = qElement(10); const double &b3p = qpElement(10);
    const double &w4 = qElement(10); const double &w4p = qpElement(11);
    const double &a4 = qElement(12); const double &a4p = qpElement(12);
    const double &b4 = qElement(13); const double &b4p = qpElement(13);

    // WrOP position
    X(0) = cos(alpha)*(((d2*(r - r1) + d1*(-r + r2))*((b4*(j - j1)*(r - r1))/((-j1 + j2)*(-r1 + r2)) + (b3*(j - j2)*(-r + r1))/((-j1 + j2)*(-r1 + r2)) + (b2*(-j + j1)*(r - r2))/((-j1 + j2)*(-r1 + r2)) + (b1*(-j + j2)*(-r + r2))/((-j1 + j2)*(-r1 + r2))))/(2.*(-r1 + r2)) + r*cos(j)) - sin(alpha)*(((d2*(r - r1) + d1*(-r + r2))*((a4*(j - j1)*(r - r1))/((-j1 + j2)*(-r1 + r2)) + (a3*(j - j2)*(-r + r1))/((-j1 + j2)*(-r1 + r2)) + (a2*(-j + j1)*(r - r2))/((-j1 + j2)*(-r1 + r2)) + (a1*(-j + j2)*(-r + r2))/((-j1 + j2)*(-r1 + r2))))/(2.*(-r1 + r2)) + r*sin(j));
    X(1) = (((d2*(r - r1) + d1*(-r + r2))*((b4*(j - j1)*(r - r1))/((-j1 + j2)*(-r1 + r2)) + (b3*(j - j2)*(-r + r1))/((-j1 + j2)*(-r1 + r2)) + (b2*(-j + j1)*(r - r2))/((-j1 + j2)*(-r1 + r2)) + (b1*(-j + j2)*(-r + r2))/((-j1 + j2)*(-r1 + r2))))/(2.*(-r1 + r2)) + r*cos(j))*sin(alpha) + cos(alpha)*(((d2*(r - r1) + d1*(-r + r2))*((a4*(j - j1)*(r - r1))/((-j1 + j2)*(-r1 + r2)) + (a3*(j - j2)*(-r + r1))/((-j1 + j2)*(-r1 + r2)) + (a2*(-j + j1)*(r - r2))/((-j1 + j2)*(-r1 + r2)) + (a1*(-j + j2)*(-r + r2))/((-j1 + j2)*(-r1 + r2))))/(2.*(-r1 + r2)) + r*sin(j));
    X(2) = (d2*(r - r1) + d1*(-r + r2))/(2.*(-r1 + r2)) + ((-j + j2)*(-r + r2)*w1)/((-j1 + j2)*(-r1 + r2)) + ((-j + j1)*(r - r2)*w2)/((-j1 + j2)*(-r1 + r2)) + ((j - j2)*(-r + r1)*w3)/((-j1 + j2)*(-r1 + r2)) + ((j - j1)*(r - r1)*w4)/((-j1 + j2)*(-r1 + r2)) + zS;

    // phiP angles
    X(3) = 0;
    X(4) = 0;
    X(5) = 0;

    // WvP translational velocities
    X(6) = ((d2*(r - r1) + d1*(-r + r2))*((b4p*(j - j1)*(r - r1))/((-j1 + j2)*(-r1 + r2)) + (b3p*(j - j2)*(-r + r1))/((-j1 + j2)*(-r1 + r2)) + (b2p*(-j + j1)*(r - r2))/((-j1 + j2)*(-r1 + r2)) + (b1p*(-j + j2)*(-r + r2))/((-j1 + j2)*(-r1 + r2)))*cos(alpha))/(2.*(-r1 + r2)) - ((d2*(r - r1) + d1*(-r + r2))*((a4p*(j - j1)*(r - r1))/((-j1 + j2)*(-r1 + r2)) + (a3p*(j - j2)*(-r + r1))/((-j1 + j2)*(-r1 + r2)) + (a2p*(-j + j1)*(r - r2))/((-j1 + j2)*(-r1 + r2)) + (a1p*(-j + j2)*(-r + r2))/((-j1 + j2)*(-r1 + r2)))*sin(alpha))/(2.*(-r1 + r2)) - alphap*(((d2*(r - r1) + d1*(-r + r2))*((b4*(j - j1)*(r - r1))/((-j1 + j2)*(-r1 + r2)) + (b3*(j - j2)*(-r + r1))/((-j1 + j2)*(-r1 + r2)) + (b2*(-j + j1)*(r - r2))/((-j1 + j2)*(-r1 + r2)) + (b1*(-j + j2)*(-r + r2))/((-j1 + j2)*(-r1 + r2))))/(2.*(-r1 + r2)) + r*cos(j))*sin(alpha) - alphap*cos(alpha)*(((d2*(r - r1) + d1*(-r + r2))*((a4*(j - j1)*(r - r1))/((-j1 + j2)*(-r1 + r2)) + (a3*(j - j2)*(-r + r1))/((-j1 + j2)*(-r1 + r2)) + (a2*(-j + j1)*(r - r2))/((-j1 + j2)*(-r1 + r2)) + (a1*(-j + j2)*(-r + r2))/((-j1 + j2)*(-r1 + r2))))/(2.*(-r1 + r2)) + r*sin(j));
    X(7) = ((d2*(r - r1) + d1*(-r + r2))*((a4p*(j - j1)*(r - r1))/((-j1 + j2)*(-r1 + r2)) + (a3p*(j - j2)*(-r + r1))/((-j1 + j2)*(-r1 + r2)) + (a2p*(-j + j1)*(r - r2))/((-j1 + j2)*(-r1 + r2)) + (a1p*(-j + j2)*(-r + r2))/((-j1 + j2)*(-r1 + r2)))*cos(alpha))/(2.*(-r1 + r2)) + alphap*cos(alpha)*(((d2*(r - r1) + d1*(-r + r2))*((b4*(j - j1)*(r - r1))/((-j1 + j2)*(-r1 + r2)) + (b3*(j - j2)*(-r + r1))/((-j1 + j2)*(-r1 + r2)) + (b2*(-j + j1)*(r - r2))/((-j1 + j2)*(-r1 + r2)) + (b1*(-j + j2)*(-r + r2))/((-j1 + j2)*(-r1 + r2))))/(2.*(-r1 + r2)) + r*cos(j)) + ((d2*(r - r1) + d1*(-r + r2))*((b4p*(j - j1)*(r - r1))/((-j1 + j2)*(-r1 + r2)) + (b3p*(j - j2)*(-r + r1))/((-j1 + j2)*(-r1 + r2)) + (b2p*(-j + j1)*(r - r2))/((-j1 + j2)*(-r1 + r2)) + (b1p*(-j + j2)*(-r + r2))/((-j1 + j2)*(-r1 + r2)))*sin(alpha))/(2.*(-r1 + r2)) - alphap*sin(alpha)*(((d2*(r - r1) + d1*(-r + r2))*((a4*(j - j1)*(r - r1))/((-j1 + j2)*(-r1 + r2)) + (a3*(j - j2)*(-r + r1))/((-j1 + j2)*(-r1 + r2)) + (a2*(-j + j1)*(r - r2))/((-j1 + j2)*(-r1 + r2)) + (a1*(-j + j2)*(-r + r2))/((-j1 + j2)*(-r1 + r2))))/(2.*(-r1 + r2)) + r*sin(j));
    X(8) = ((-j + j2)*(-r + r2)*w1p)/((-j1 + j2)*(-r1 + r2)) + ((-j + j1)*(r - r2)*w2p)/((-j1 + j2)*(-r1 + r2)) + ((j - j2)*(-r + r1)*w3p)/((-j1 + j2)*(-r1 + r2)) + ((j - j1)*(r - r1)*w4p)/((-j1 + j2)*(-r1 + r2)) + zSp;

    // WomegaP angular velocities
    X(9) = cos(alpha)*(((b4p*(j - j1)*(r - r1))/((-j1 + j2)*(-r1 + r2)) + (b3p*(j - j2)*(-r + r1))/((-j1 + j2)*(-r1 + r2)) + (b2p*(-j + j1)*(r - r2))/((-j1 + j2)*(-r1 + r2)) + (b1p*(-j + j2)*(-r + r2))/((-j1 + j2)*(-r1 + r2)))*cos(j) - ((a4p*(j - j1)*(r - r1))/((-j1 + j2)*(-r1 + r2)) + (a3p*(j - j2)*(-r + r1))/((-j1 + j2)*(-r1 + r2)) + (a2p*(-j + j1)*(r - r2))/((-j1 + j2)*(-r1 + r2)) + (a1p*(-j + j2)*(-r + r2))/((-j1 + j2)*(-r1 + r2)))*sin(j)) - sin(alpha)*(((a4p*(j - j1)*(r - r1))/((-j1 + j2)*(-r1 + r2)) + (a3p*(j - j2)*(-r + r1))/((-j1 + j2)*(-r1 + r2)) + (a2p*(-j + j1)*(r - r2))/((-j1 + j2)*(-r1 + r2)) + (a1p*(-j + j2)*(-r + r2))/((-j1 + j2)*(-r1 + r2)))*cos(j) + ((b4p*(j - j1)*(r - r1))/((-j1 + j2)*(-r1 + r2)) + (b3p*(j - j2)*(-r + r1))/((-j1 + j2)*(-r1 + r2)) + (b2p*(-j + j1)*(r - r2))/((-j1 + j2)*(-r1 + r2)) + (b1p*(-j + j2)*(-r + r2))/((-j1 + j2)*(-r1 + r2)))*sin(j));
    X(10) = sin(alpha)*(((b4p*(j - j1)*(r - r1))/((-j1 + j2)*(-r1 + r2)) + (b3p*(j - j2)*(-r + r1))/((-j1 + j2)*(-r1 + r2)) + (b2p*(-j + j1)*(r - r2))/((-j1 + j2)*(-r1 + r2)) + (b1p*(-j + j2)*(-r + r2))/((-j1 + j2)*(-r1 + r2)))*cos(j) - ((a4p*(j - j1)*(r - r1))/((-j1 + j2)*(-r1 + r2)) + (a3p*(j - j2)*(-r + r1))/((-j1 + j2)*(-r1 + r2)) + (a2p*(-j + j1)*(r - r2))/((-j1 + j2)*(-r1 + r2)) + (a1p*(-j + j2)*(-r + r2))/((-j1 + j2)*(-r1 + r2)))*sin(j)) + cos(alpha)*(((a4p*(j - j1)*(r - r1))/((-j1 + j2)*(-r1 + r2)) + (a3p*(j - j2)*(-r + r1))/((-j1 + j2)*(-r1 + r2)) + (a2p*(-j + j1)*(r - r2))/((-j1 + j2)*(-r1 + r2)) + (a1p*(-j + j2)*(-r + r2))/((-j1 + j2)*(-r1 + r2)))*cos(j) + ((b4p*(j - j1)*(r - r1))/((-j1 + j2)*(-r1 + r2)) + (b3p*(j - j2)*(-r + r1))/((-j1 + j2)*(-r1 + r2)) + (b2p*(-j + j1)*(r - r2))/((-j1 + j2)*(-r1 + r2)) + (b1p*(-j + j2)*(-r + r2))/((-j1 + j2)*(-r1 + r2)))*sin(j));
    X(11) = alphap;

    return X;
  }

  Mat FiniteElement2s13MFRMindlin::JGeneralized(const Vec &NodeCoordinates,const Vec &s) {

    // node coordinates
    const double &r1 = NodeCoordinates(0);
    const double &j1 = NodeCoordinates(1);
    const double &r2 = NodeCoordinates(2);
    const double &j2 = NodeCoordinates(3); 

    // coordinates of force application point
    const double &r = s(0);
    const double &j = s(1);

    Mat J(RefDofs+4*NodeDofs,4);

    /* full Jacobian matrix */
    // translation (only one column -> translation along z-axis)
    J(0,0) = 1;
    J(1,0) = 0;
    J(2,0) = ((j - j2)*(r - r2))/((j1 - j2)*(r1 - r2));
    J(3,0) = 0;
    J(4,0) = 0;
    J(5,0) = ((-j + j1)*(r - r2))/((j1 - j2)*(r1 - r2));
    J(6,0) = 0;
    J(7,0) = 0;
    J(8,0) = ((j - j2)*(-r + r1))/((j1 - j2)*(r1 - r2));
    J(9,0) = 0;
    J(10,0) = 0;
    J(11,0) = ((j - j1)*(r - r1))/((j1 - j2)*(r1 - r2));
    J(12,0) = 0;
    J(13,0) = 0;

    // rotation about all three possible axes
    J(0,1) = 0;
    J(0,2) = 0;
    J(0,3) = 0;
    J(1,1) = 0;
    J(1,2) = 0;
    J(1,3) = 1;
    J(2,1) = 0;
    J(2,2) = 0;
    J(2,3) = 0;
    J(3,1) = -(((j - j2)*(r - r2)*sin(j))/((j1 - j2)*(r1 - r2)));
    J(3,2) = ((j - j2)*(r - r2)*cos(j))/((j1 - j2)*(r1 - r2));
    J(3,3) = 0;
    J(4,1) = ((j - j2)*(r - r2)*cos(j))/((j1 - j2)*(r1 - r2));
    J(4,2) = ((j - j2)*(r - r2)*sin(j))/((j1 - j2)*(r1 - r2));
    J(4,3) = 0;
    J(5,1) = 0;
    J(5,2) = 0;
    J(5,3) = 0;
    J(6,1) = -(((j - j1)*(r - r2)*sin(j))/((j1 - j2)*(-r1 + r2)));
    J(6,2) = ((j - j1)*(r - r2)*cos(j))/((j1 - j2)*(-r1 + r2));
    J(6,3) = 0;
    J(7,1) = ((j - j1)*(r - r2)*cos(j))/((j1 - j2)*(-r1 + r2));
    J(7,2) = ((j - j1)*(r - r2)*sin(j))/((j1 - j2)*(-r1 + r2));
    J(7,3) = 0;
    J(8,1) = 0;
    J(8,2) = 0;
    J(8,3) = 0;
    J(9,1) = -(((j - j2)*(r - r1)*sin(j))/((-j1 + j2)*(r1 - r2)));
    J(9,2) = ((j - j2)*(r - r1)*cos(j))/((-j1 + j2)*(r1 - r2));
    J(9,3) = 0;
    J(10,1) = ((j - j2)*(r - r1)*cos(j))/((-j1 + j2)*(r1 - r2));
    J(10,2) = ((j - j2)*(r - r1)*sin(j))/((-j1 + j2)*(r1 - r2));
    J(10,3) = 0;
    J(11,1) = 0;
    J(11,2) = 0;
    J(11,3) = 0;
    J(12,1) = -(((j - j1)*(r - r1)*sin(j))/((j1 - j2)*(r1 - r2)));
    J(12,2) = ((j - j1)*(r - r1)*cos(j))/((j1 - j2)*(r1 - r2));
    J(12,3) = 0;
    J(13,1) = ((j - j1)*(r - r1)*cos(j))/((j1 - j2)*(r1 - r2));
    J(13,2) = ((j - j1)*(r - r1)*sin(j))/((j1 - j2)*(r1 - r2));
    J(13,3) = 0;

    return J;
  }

}

