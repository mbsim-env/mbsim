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
#include<fstream>
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_2s_13_disk.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  inline double Power(const double base, const int exponent) {
    return pow(base,exponent);
  }

  FiniteElement2s13Disk::FiniteElement2s13Disk(double E_,double nu_,double rho_) : DiscretizationInterface(),E(E_),nu(nu_),G(E/(2.*(1.+nu))),rho(rho_),alphaS(5./6.),RefDofs(2),NodeDofs(3),Nodes(4),M(14,INIT,0.),K(14,INIT,0.) {}

  void FiniteElement2s13Disk::computeConstantSystemMatrices(const Vec &NodeCoordinates,double d1,double d2) {

    // node coordinates
    const double &r1 = NodeCoordinates(0);
    const double &j1 = NodeCoordinates(1);
    const double &r2 = NodeCoordinates(2);
    const double &j2 = NodeCoordinates(3);

    double dr = r2 - r1;
    double dj = j2 - j1;

    // mass matrix
    // reference components
    M(0,0) = ((j1 - j2)*(r1 - r2)*(d1*(2*r1 + r2) + d2*(r1 + 2*r2))*rho)/6.;
    //M(0,1) = 0;
    M(0,2) = ((j1 - j2)*(r1 - r2)*(d2*(r1 + r2) + d1*(3*r1 + r2))*rho)/24.;
    //M(0,3) = 0;
    //M(0,4) = 0;
    M(0,5) = ((j1 - j2)*(r1 - r2)*(d2*(r1 + r2) + d1*(3*r1 + r2))*rho)/24.;
    //M(0,6) = 0;
    //M(0,7) = 0;
    M(0,8) = ((j1 - j2)*(r1 - r2)*(d1*(r1 + r2) + d2*(r1 + 3*r2))*rho)/24.;
    //M(0,9) = 0;
    //M(0,10) = 0;
    M(0,11) = ((j1 - j2)*(r1 - r2)*(d1*(r1 + r2) + d2*(r1 + 3*r2))*rho)/24.;
    //M(0,12) = 0;
    //M(0,13) = 0;
    M(1,1) = ((j1 - j2)*(r1 - r2)*(d1*(4*Power(r1,3) + 3*Power(r1,2)*r2 + 2*r1*Power(r2,2) + Power(r2,3)) + d2*(Power(r1,3) + 2*Power(r1,2)*r2 + 3*r1*Power(r2,2) + 4*Power(r2,3)))*rho)/20.;
    //M(1,2) = 0;
    //M(1,3) = 0;
    //M(1,4) = 0;
    //M(1,5) = 0;
    //M(1,6) = 0;
    //M(1,7) = 0;
    //M(1,8) = 0;
    //M(1,9) = 0;
    //M(1,10) = 0;
    //M(1,11) = 0;
    //M(1,12) = 0;
    //M(1,13) = 0;

    // elastic components
    M(2,2) = ((j1 - j2)*(r1 - r2)*(3*d1*(4*r1 + r2) + d2*(3*r1 + 2*r2))*rho)/180.;
    //M(2,3) = 0;
    //M(2,4) = 0;
    M(2,5) = ((j1 - j2)*(r1 - r2)*(3*d1*(4*r1 + r2) + d2*(3*r1 + 2*r2))*rho)/360.;
    //M(2,6) = 0;
    //M(2,7) = 0;
    M(2,8) = ((j1 - j2)*(r1 - r2)*(3*d1*r1 + 2*d2*r1 + 2*d1*r2 + 3*d2*r2)*rho)/180.;
    //M(2,9) = 0;
    //M(2,10) = 0;
    M(2,11) = ((j1 - j2)*(r1 - r2)*(3*d1*r1 + 2*d2*r1 + 2*d1*r2 + 3*d2*r2)*rho)/360.;
    //M(2,12) = 0;
    //M(2,13) = 0;
    M(3,3) = ((j1 - j2)*(r1 - r2)*(10*Power(d1,3)*(6*r1 + r2) + 6*Power(d1,2)*d2*(5*r1 + 2*r2) + 3*d1*Power(d2,2)*(4*r1 + 3*r2) + Power(d2,3)*(3*r1 + 4*r2))*rho)/15120.;
    //M(3,4) = 0;
    //M(3,5) = 0;
    M(3,6) = ((j1 - j2)*(r1 - r2)*(10*Power(d1,3)*(6*r1 + r2) + 6*Power(d1,2)*d2*(5*r1 + 2*r2) + 3*d1*Power(d2,2)*(4*r1 + 3*r2) + Power(d2,3)*(3*r1 + 4*r2))*rho)/30240.;
    //M(3,7) = 0;
    //M(3,8) = 0;
    M(3,9) = ((j1 - j2)*(r1 - r2)*(2*Power(d1,3)*(5*r1 + 2*r2) + 3*Power(d1,2)*d2*(4*r1 + 3*r2) + 3*d1*Power(d2,2)*(3*r1 + 4*r2) + 2*Power(d2,3)*(2*r1 + 5*r2))*rho)/15120.;
    //M(3,10) = 0;
    //M(3,11) = 0;
    M(3,12) = ((j1 - j2)*(r1 - r2)*(2*Power(d1,3)*(5*r1 + 2*r2) + 3*Power(d1,2)*d2*(4*r1 + 3*r2) + 3*d1*Power(d2,2)*(3*r1 + 4*r2) + 2*Power(d2,3)*(2*r1 + 5*r2))*rho)/30240.;
    //M(3,13) = 0;
    M(4,4) = ((j1 - j2)*(r1 - r2)*(10*Power(d1,3)*(6*r1 + r2) + 6*Power(d1,2)*d2*(5*r1 + 2*r2) + 3*d1*Power(d2,2)*(4*r1 + 3*r2) + Power(d2,3)*(3*r1 + 4*r2))*rho)/15120.;
    //M(4,5) = 0;
    //M(4,6) = 0;
    M(4,7) = ((j1 - j2)*(r1 - r2)*(10*Power(d1,3)*(6*r1 + r2) + 6*Power(d1,2)*d2*(5*r1 + 2*r2) + 3*d1*Power(d2,2)*(4*r1 + 3*r2) + Power(d2,3)*(3*r1 + 4*r2))*rho)/30240.;
    //M(4,8) = 0;
    //M(4,9) = 0;
    M(4,10) = ((j1 - j2)*(r1 - r2)*(2*Power(d1,3)*(5*r1 + 2*r2) + 3*Power(d1,2)*d2*(4*r1 + 3*r2) + 3*d1*Power(d2,2)*(3*r1 + 4*r2) + 2*Power(d2,3)*(2*r1 + 5*r2))*rho)/15120.;
    //M(4,11) = 0;
    //M(4,12) = 0;
    M(4,13) = ((j1 - j2)*(r1 - r2)*(2*Power(d1,3)*(5*r1 + 2*r2) + 3*Power(d1,2)*d2*(4*r1 + 3*r2) + 3*d1*Power(d2,2)*(3*r1 + 4*r2) + 2*Power(d2,3)*(2*r1 + 5*r2))*rho)/30240.;
    M(5,5) = ((j1 - j2)*(r1 - r2)*(3*d1*(4*r1 + r2) + d2*(3*r1 + 2*r2))*rho)/180.;
    //M(5,6) = 0;
    //M(5,7) = 0;
    M(5,8) = ((j1 - j2)*(r1 - r2)*(3*d1*r1 + 2*d2*r1 + 2*d1*r2 + 3*d2*r2)*rho)/360.;
    //M(5,9) = 0;
    //M(5,10) = 0;
    M(5,11) = ((j1 - j2)*(r1 - r2)*(3*d1*r1 + 2*d2*r1 + 2*d1*r2 + 3*d2*r2)*rho)/180.;
    //M(5,12) = 0;
    //M(5,13) = 0;
    M(6,6) = ((j1 - j2)*(r1 - r2)*(10*Power(d1,3)*(6*r1 + r2) + 6*Power(d1,2)*d2*(5*r1 + 2*r2) + 3*d1*Power(d2,2)*(4*r1 + 3*r2) + Power(d2,3)*(3*r1 + 4*r2))*rho)/15120.;
    //M(6,7) = 0;
    //M(6,8) = 0;
    M(6,9) = ((j1 - j2)*(r1 - r2)*(2*Power(d1,3)*(5*r1 + 2*r2) + 3*Power(d1,2)*d2*(4*r1 + 3*r2) + 3*d1*Power(d2,2)*(3*r1 + 4*r2) + 2*Power(d2,3)*(2*r1 + 5*r2))*rho)/30240.;
    //M(6,10) = 0;
    //M(6,11) = 0;
    M(6,12) = ((j1 - j2)*(r1 - r2)*(2*Power(d1,3)*(5*r1 + 2*r2) + 3*Power(d1,2)*d2*(4*r1 + 3*r2) + 3*d1*Power(d2,2)*(3*r1 + 4*r2) + 2*Power(d2,3)*(2*r1 + 5*r2))*rho)/15120.;
    //M(6,13) = 0;
    M(7,7) = ((j1 - j2)*(r1 - r2)*(10*Power(d1,3)*(6*r1 + r2) + 6*Power(d1,2)*d2*(5*r1 + 2*r2) + 3*d1*Power(d2,2)*(4*r1 + 3*r2) + Power(d2,3)*(3*r1 + 4*r2))*rho)/15120.;
    //M(7,8) = 0;
    //M(7,9) = 0;
    M(7,10) = ((j1 - j2)*(r1 - r2)*(2*Power(d1,3)*(5*r1 + 2*r2) + 3*Power(d1,2)*d2*(4*r1 + 3*r2) + 3*d1*Power(d2,2)*(3*r1 + 4*r2) + 2*Power(d2,3)*(2*r1 + 5*r2))*rho)/30240.;
    //M(7,11) = 0;
    //M(7,12) = 0;
    M(7,13) = ((j1 - j2)*(r1 - r2)*(2*Power(d1,3)*(5*r1 + 2*r2) + 3*Power(d1,2)*d2*(4*r1 + 3*r2) + 3*d1*Power(d2,2)*(3*r1 + 4*r2) + 2*Power(d2,3)*(2*r1 + 5*r2))*rho)/15120.;
    M(8,8) = ((j1 - j2)*(r1 - r2)*(d1*(2*r1 + 3*r2) + 3*d2*(r1 + 4*r2))*rho)/180.;
    //M(8,9) = 0;
    //M(8,10) = 0;
    M(8,11) = ((j1 - j2)*(r1 - r2)*(d1*(2*r1 + 3*r2) + 3*d2*(r1 + 4*r2))*rho)/360.;
    //M(8,12) = 0;
    //M(8,13) = 0;
    M(9,9) = ((j1 - j2)*(r1 - r2)*(Power(d1,3)*(4*r1 + 3*r2) + 3*Power(d1,2)*d2*(3*r1 + 4*r2) + 6*d1*Power(d2,2)*(2*r1 + 5*r2) + 10*Power(d2,3)*(r1 + 6*r2))*rho)/15120.;
    //M(9,10) = 0;
    //M(9,11) = 0;
    M(9,12) = ((j1 - j2)*(r1 - r2)*(Power(d1,3)*(4*r1 + 3*r2) + 3*Power(d1,2)*d2*(3*r1 + 4*r2) + 6*d1*Power(d2,2)*(2*r1 + 5*r2) + 10*Power(d2,3)*(r1 + 6*r2))*rho)/30240.;
    //M(9,13) = 0;
    M(10,10) = ((j1 - j2)*(r1 - r2)*(Power(d1,3)*(4*r1 + 3*r2) + 3*Power(d1,2)*d2*(3*r1 + 4*r2) + 6*d1*Power(d2,2)*(2*r1 + 5*r2) + 10*Power(d2,3)*(r1 + 6*r2))*rho)/15120.;
    //M(10,11) = 0;
    //M(10,12) = 0;
    M(10,13) = ((j1 - j2)*(r1 - r2)*(Power(d1,3)*(4*r1 + 3*r2) + 3*Power(d1,2)*d2*(3*r1 + 4*r2) + 6*d1*Power(d2,2)*(2*r1 + 5*r2) + 10*Power(d2,3)*(r1 + 6*r2))*rho)/30240.;
    M(11,11) = ((j1 - j2)*(r1 - r2)*(d1*(2*r1 + 3*r2) + 3*d2*(r1 + 4*r2))*rho)/180.;
    //M(11,12) = 0;
    //M(11,13) = 0;
    M(12,12) = ((j1 - j2)*(r1 - r2)*(Power(d1,3)*(4*r1 + 3*r2) + 3*Power(d1,2)*d2*(3*r1 + 4*r2) + 6*d1*Power(d2,2)*(2*r1 + 5*r2) + 10*Power(d2,3)*(r1 + 6*r2))*rho)/15120.;
    //M(12,13) = 0;
    M(13,13) = ((j1 - j2)*(r1 - r2)*(Power(d1,3)*(4*r1 + 3*r2) + 3*Power(d1,2)*d2*(3*r1 + 4*r2) + 6*d1*Power(d2,2)*(2*r1 + 5*r2) + 10*Power(d2,3)*(r1 + 6*r2))*rho)/15120.;

    // stiffness matrix
    SymMat KEl = K(Index(2,13));
    KEl(0,0) = (alphaS*G*(dr*(d1*(-2*(3 + Power(j1 - j2,2))*Power(r1,2) + (21 + Power(j1 - j2,2))*r1*r2 + (-33 + Power(j1 - j2,2))*Power(r2,2)) - d2*((3 + Power(j1 - j2,2))*Power(r1,2) + (-15 + Power(j1 - j2,2))*r1*r2 - 2*(3 + Power(j1 - j2,2))*Power(r2,2))) + 18*Power(r2,2)*(d2*r1 - d1*r2)*log(r1/r2)))/(18.*dj*Power(dr,3));
    KEl(0,1) = -(alphaS*G*Power(j1 - j2,2)*Power(r1 - r2,2)*(d2*(r1 + r2) + d1*(3*r1 + r2)))/(36.*dj*Power(dr,2));
    KEl(0,2) = -(alphaS*(3*d1 + d2)*dr*G)/24.;
    KEl(0,3) = (alphaS*G*(dr*(-(d2*((-6 + Power(j1 - j2,2))*Power(r1,2) + (30 + Power(j1 - j2,2))*r1*r2 - 2*(-6 + Power(j1 - j2,2))*Power(r2,2))) + d1*(-2*(-6 + Power(j1 - j2,2))*Power(r1,2) + (-42 + Power(j1 - j2,2))*r1*r2 + (66 + Power(j1 - j2,2))*Power(r2,2))) + 36*Power(r2,2)*(-(d2*r1) + d1*r2)*log(r1/r2)))/(36.*dj*Power(dr,3));
    KEl(0,4) = -(alphaS*G*Power(j1 - j2,2)*Power(r1 - r2,2)*(d2*(r1 + r2) + d1*(3*r1 + r2)))/(72.*dj*Power(dr,2));
    KEl(0,5) = -(alphaS*(3*d1 + d2)*dr*G)/24.;
    KEl(0,6) = (alphaS*G*(dr*((d2*(-6 + Power(j1 - j2,2)) + d1*(-3 + 2*Power(j1 - j2,2)))*Power(r1,2) - (d1 - d2)*(-15 + Power(j1 - j2,2))*r1*r2 - (d1*(-6 + Power(j1 - j2,2)) + d2*(-3 + 2*Power(j1 - j2,2)))*Power(r2,2)) + 18*r1*r2*(d2*r1 - d1*r2)*log(r2/r1)))/(18.*dj*Power(dr,3));
    KEl(0,7) = -(alphaS*G*Power(j1 - j2,2)*Power(r1 - r2,2)*(d1*(r1 + r2) + d2*(r1 + 3*r2)))/(36.*dj*Power(dr,2));
    KEl(0,8) = -(alphaS*(d1 + d2)*G*(j1 - j2)*Power(r1 - r2,3))/(24.*dj*Power(dr,2));
    KEl(0,9) = (alphaS*G*(dr*(d2*((12 + Power(j1 - j2,2))*Power(r1,2) + (30 + Power(j1 - j2,2))*r1*r2 - 2*(3 + Power(j1 - j2,2))*Power(r2,2)) + d1*(2*(3 + Power(j1 - j2,2))*Power(r1,2) - (30 + Power(j1 - j2,2))*r1*r2 - (12 + Power(j1 - j2,2))*Power(r2,2))) + 36*r1*r2*(d2*r1 - d1*r2)*log(r1/r2)))/(36.*dj*Power(dr,3));
    KEl(0,10) = -(alphaS*G*Power(j1 - j2,2)*Power(r1 - r2,2)*(d1*(r1 + r2) + d2*(r1 + 3*r2)))/(72.*dj*Power(dr,2));
    KEl(0,11) = -(alphaS*(d1 + d2)*G*(j1 - j2)*Power(r1 - r2,3))/(24.*dj*Power(dr,2));
    KEl(1,1) = (alphaS*dj*dr*G*(3*d1*(4*r1 + r2) + d2*(3*r1 + 2*r2)))/180. + (E*(dr*(3*d1*Power(d2,2)*(2*(3 - 3*nu + 8*Power(j1 - j2,2)*(1 + nu))*Power(r1,4) - (-39*(-1 + nu) + 64*Power(j1 - j2,2)*(1 + nu))*Power(r1,3)*r2 + (-141*(-1 + nu) + 16*Power(j1 - j2,2)*(11 + 6*nu))*Power(r1,2)*Power(r2,2) + (8*Power(j1 - j2,2)*(17 - 8*nu) - 81*(-1 + nu))*r1*Power(r2,3) + (9*(-1 + nu) + 8*Power(j1 - j2,2)*(-3 + 2*nu))*Power(r2,4)) + 3*Power(d1,2)*d2*(3*(3 - 3*nu + 8*Power(j1 - j2,2)*(1 + nu))*Power(r1,4) - 3*(-17*(-1 + nu) + 32*Power(j1 - j2,2)*(1 + nu))*Power(r1,3)*r2 + (-129*(-1 + nu) + 8*Power(j1 - j2,2)*(23 + 18*nu))*Power(r1,2)*Power(r2,2) - (-231*(-1 + nu) + 8*Power(j1 - j2,2)*(37 + 12*nu))*r1*Power(r2,3) + 4*(9*(-1 + nu) + 2*Power(j1 - j2,2)*(-7 + 3*nu))*Power(r2,4)) + Power(d2,3)*(3*(3 - 3*nu + 8*Power(j1 - j2,2)*(1 + nu))*Power(r1,4) - 3*(-27*(-1 + nu) + 32*Power(j1 - j2,2)*(1 + nu))*Power(r1,3)*r2 + (141*(-1 + nu) + 8*Power(j1 - j2,2)*(-37 + 18*nu))*Power(r1,2)*Power(r2,2) + (8*Power(j1 - j2,2)*(23 - 12*nu) - 39*(-1 + nu))*r1*Power(r2,3) + 2*(3*(-1 + nu) + 4*Power(j1 - j2,2)*(-7 + 3*nu))*Power(r2,4)) + Power(d1,3)*(12*(3 - 3*nu + 8*Power(j1 - j2,2)*(1 + nu))*Power(r1,4) - 3*(-63*(-1 + nu) + 128*Power(j1 - j2,2)*(1 + nu))*Power(r1,3)*r2 + (-411*(-1 + nu) + 16*Power(j1 - j2,2)*(41 + 36*nu))*Power(r1,2)*Power(r2,2) - (-489*(-1 + nu) + 8*Power(j1 - j2,2)*(83 + 48*nu))*r1*Power(r2,3) + (-411*(-1 + nu) + 8*Power(j1 - j2,2)*(67 + 12*nu))*Power(r2,4))) + 60*(3 + 4*Power(j1 - j2,2) - 3*nu)*Power(r2,2)*Power(-(d2*r1) + d1*r2,3)*log(r1/r2)))/(8640.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(1,2) = (E*(j1 - j2)*(dr*(3*Power(d1,2)*d2*(12*(1 + nu)*Power(r1,4) - (63 + 53*nu)*Power(r1,3)*r2 + (147 + 97*nu)*Power(r1,2)*Power(r2,2) - (243 + 113*nu)*r1*Power(r2,3) - 3*(11 + nu)*Power(r2,4)) + 3*d1*Power(d2,2)*(8*(1 + nu)*Power(r1,4) - (47 + 37*nu)*Power(r1,3)*r2 + (153 + 83*nu)*Power(r1,2)*Power(r2,2) + (73 + 3*nu)*r1*Power(r2,3) + (-7 + 3*nu)*Power(r2,4)) + Power(d2,3)*(12*(1 + nu)*Power(r1,4) - 3*(31 + 21*nu)*Power(r1,3)*r2 + (-123 + 7*nu)*Power(r1,2)*Power(r2,2) + (27 - 23*nu)*r1*Power(r2,3) + (-3 + 7*nu)*Power(r2,4)) + Power(d1,3)*(48*(1 + nu)*Power(r1,4) - 3*(79 + 69*nu)*Power(r1,3)*r2 + (483 + 353*nu)*Power(r1,2)*Power(r2,2) - (537 + 307*nu)*r1*Power(r2,3) + (423 + 173*nu)*Power(r2,4))) + 60*(3 + nu)*Power(r2,2)*Power(-(d2*r1) + d1*r2,3)*log(r1/r2)))/(5760.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(1,3) = -(alphaS*dj*G*(d2*(r1 + r2) + d1*(3*r1 + r2)))/72.;
    KEl(1,4) = (alphaS*dj*dr*G*(3*d1*(4*r1 + r2) + d2*(3*r1 + 2*r2)))/360. + (E*(dr*(3*d1*Power(d2,2)*(2*(3*(-1 + nu) + 4*Power(j1 - j2,2)*(1 + nu))*Power(r1,4) - (39*(-1 + nu) + 32*Power(j1 - j2,2)*(1 + nu))*Power(r1,3)*r2 + (141*(-1 + nu) + 8*Power(j1 - j2,2)*(11 + 6*nu))*Power(r1,2)*Power(r2,2) + (4*Power(j1 - j2,2)*(17 - 8*nu) + 81*(-1 + nu))*r1*Power(r2,3) + (9 - 9*nu + 4*Power(j1 - j2,2)*(-3 + 2*nu))*Power(r2,4)) + 3*Power(d1,2)*d2*(3*(3*(-1 + nu) + 4*Power(j1 - j2,2)*(1 + nu))*Power(r1,4) - 3*(17*(-1 + nu) + 16*Power(j1 - j2,2)*(1 + nu))*Power(r1,3)*r2 + (129*(-1 + nu) + 4*Power(j1 - j2,2)*(23 + 18*nu))*Power(r1,2)*Power(r2,2) - (231*(-1 + nu) + 4*Power(j1 - j2,2)*(37 + 12*nu))*r1*Power(r2,3) + 4*(9 - 9*nu + Power(j1 - j2,2)*(-7 + 3*nu))*Power(r2,4)) + Power(d2,3)*(3*(3*(-1 + nu) + 4*Power(j1 - j2,2)*(1 + nu))*Power(r1,4) - 3*(27*(-1 + nu) + 16*Power(j1 - j2,2)*(1 + nu))*Power(r1,3)*r2 + (-141*(-1 + nu) + 4*Power(j1 - j2,2)*(-37 + 18*nu))*Power(r1,2)*Power(r2,2) + (39*(-1 + nu) - 4*Power(j1 - j2,2)*(-23 + 12*nu))*r1*Power(r2,3) + 2*(3 - 3*nu + 2*Power(j1 - j2,2)*(-7 + 3*nu))*Power(r2,4)) + Power(d1,3)*(12*(3*(-1 + nu) + 4*Power(j1 - j2,2)*(1 + nu))*Power(r1,4) - 3*(63*(-1 + nu) + 64*Power(j1 - j2,2)*(1 + nu))*Power(r1,3)*r2 + (411*(-1 + nu) + 8*Power(j1 - j2,2)*(41 + 36*nu))*Power(r1,2)*Power(r2,2) - (489*(-1 + nu) + 4*Power(j1 - j2,2)*(83 + 48*nu))*r1*Power(r2,3) + (411*(-1 + nu) + 4*Power(j1 - j2,2)*(67 + 12*nu))*Power(r2,4))) + 60*(2*Power(j1 - j2,2) + 3*(-1 + nu))*Power(r2,2)*Power(-(d2*r1) + d1*r2,3)*log(r1/r2)))/(8640.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(1,5) = -(E*(j1 - j2)*(dr*(Power(d1,3)*(48*(1 + nu)*Power(r1,4) - 3*(89 + 59*nu)*Power(r1,3)*r2 + (613 + 223*nu)*Power(r1,2)*Power(r2,2) - (767 + 77*nu)*r1*Power(r2,3) + (673 - 77*nu)*Power(r2,4)) + 3*Power(d1,2)*d2*(12*(1 + nu)*Power(r1,4) - (73 + 43*nu)*Power(r1,3)*r2 + (197 + 47*nu)*Power(r1,2)*Power(r2,2) + (-373 + 17*nu)*r1*Power(r2,3) + 9*(-7 + 3*nu)*Power(r2,4)) + 3*d1*Power(d2,2)*(8*(1 + nu)*Power(r1,4) - 3*(19 + 9*nu)*Power(r1,3)*r2 + (223 + 13*nu)*Power(r1,2)*Power(r2,2) + (143 - 67*nu)*r1*Power(r2,3) + (-17 + 13*nu)*Power(r2,4)) + Power(d2,3)*(12*(1 + nu)*Power(r1,4) - 3*(41 + 11*nu)*Power(r1,3)*r2 + (-253 + 137*nu)*Power(r1,2)*Power(r2,2) + (77 - 73*nu)*r1*Power(r2,3) + (-13 + 17*nu)*Power(r2,4))) + 60*(-5 + nu)*Power(r2,2)*Power(d2*r1 - d1*r2,3)*log(r1/r2)))/(5760.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(1,6) = (alphaS*dj*G*(d2*(r1 + r2) + d1*(3*r1 + r2)))/36.;
    KEl(1,7) = -(alphaS*dj*G*(r1 - r2)*(3*d1*r1 + 2*d2*r1 + 2*d1*r2 + 3*d2*r2))/180. + (E*(dr*(-(Power(d1,3)*(9*(-1 + nu + 4*Power(j1 - j2,2)*(1 + nu))*Power(r1,4) - (51*(-1 + nu) + 16*Power(j1 - j2,2)*(4 + 9*nu))*Power(r1,3)*r2 + (129*(-1 + nu) + 8*Power(j1 - j2,2)*(-8 + 27*nu))*Power(r1,2)*Power(r2,2) + (8*Power(j1 - j2,2)*(37 - 18*nu) - 231*(-1 + nu))*r1*Power(r2,3) + 36*(1 - nu + Power(j1 - j2,2)*(1 + nu))*Power(r2,4))) + 3*d1*Power(d2,2)*((9 - 9*nu + 4*Power(j1 - j2,2)*(1 + nu))*Power(r1,4) - (-81*(-1 + nu) + 16*Power(j1 - j2,2)*(6 + nu))*Power(r1,3)*r2 + (141*(-1 + nu) + 8*Power(j1 - j2,2)*(-22 + 3*nu))*Power(r1,2)*Power(r2,2) + (8*Power(j1 - j2,2)*(3 - 2*nu) - 39*(-1 + nu))*r1*Power(r2,3) + 2*(3*(-1 + nu) + 2*Power(j1 - j2,2)*(1 + nu))*Power(r2,4)) - 3*Power(d1,2)*d2*((6*(-1 + nu) + 4*Power(j1 - j2,2)*(1 + nu))*Power(r1,4) + (8*Power(j1 - j2,2)*(3 - 2*nu) - 39*(-1 + nu))*Power(r1,3)*r2 + (141*(-1 + nu) + 8*Power(j1 - j2,2)*(-22 + 3*nu))*Power(r1,2)*Power(r2,2) - (-81*(-1 + nu) + 16*Power(j1 - j2,2)*(6 + nu))*r1*Power(r2,3) + (9 - 9*nu + 4*Power(j1 - j2,2)*(1 + nu))*Power(r2,4)) + Power(d2,3)*(36*(1 - nu + Power(j1 - j2,2)*(1 + nu))*Power(r1,4) + (8*Power(j1 - j2,2)*(37 - 18*nu) - 231*(-1 + nu))*Power(r1,3)*r2 + (129*(-1 + nu) + 8*Power(j1 - j2,2)*(-8 + 27*nu))*Power(r1,2)*Power(r2,2) - (51*(-1 + nu) + 16*Power(j1 - j2,2)*(4 + 9*nu))*r1*Power(r2,3) + 9*(-1 + nu + 4*Power(j1 - j2,2)*(1 + nu))*Power(r2,4))) + 60*(3 + 4*Power(j1 - j2,2) - 3*nu)*r1*r2*Power(d2*r1 - d1*r2,3)*log(r1/r2)))/(8640.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(1,8) = (E*(j1 - j2)*(dr*(Power(d1,3)*(3*(-1 + 9*nu)*Power(r1,4) - (3 + 113*nu)*Power(r1,3)*r2 + (57 + 187*nu)*Power(r1,2)*Power(r2,2) - (183 + 173*nu)*r1*Power(r2,3) + 12*(-4 + nu)*Power(r2,4)) + 3*Power(d1,2)*d2*((3 + 13*nu)*Power(r1,4) - 3*(9 + 19*nu)*Power(r1,3)*r2 + (123 + 113*nu)*Power(r1,2)*Power(r2,2) + (93 - 17*nu)*r1*Power(r2,3) + 4*(-3 + 2*nu)*Power(r2,4)) + 3*d1*Power(d2,2)*((7 + 17*nu)*Power(r1,4) - (73 + 83*nu)*Power(r1,3)*r2 + (-153 + 37*nu)*Power(r1,2)*Power(r2,2) + (47 - 43*nu)*r1*Power(r2,3) + 4*(-2 + 3*nu)*Power(r2,4)) + Power(d2,3)*((33 + 63*nu)*Power(r1,4) + (243 - 127*nu)*Power(r1,3)*r2 + (-147 + 263*nu)*Power(r1,2)*Power(r2,2) + (63 - 187*nu)*r1*Power(r2,3) + 12*(-1 + 4*nu)*Power(r2,4))) + 60*(3 + nu)*r1*r2*Power(d2*r1 - d1*r2,3)*log(r1/r2)))/(5760.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(1,9) = (alphaS*dj*G*(d2*(r1 + r2) + d1*(3*r1 + r2)))/72.;
    KEl(1,10) = -(alphaS*dj*G*(r1 - r2)*(3*d1*r1 + 2*d2*r1 + 2*d1*r2 + 3*d2*r2))/360. + (E*(dr*(-(Power(d1,3)*(9*(1 - nu + 2*Power(j1 - j2,2)*(1 + nu))*Power(r1,4) - (-51*(-1 + nu) + 8*Power(j1 - j2,2)*(4 + 9*nu))*Power(r1,3)*r2 + (-129*(-1 + nu) + 4*Power(j1 - j2,2)*(-8 + 27*nu))*Power(r1,2)*Power(r2,2) + (4*Power(j1 - j2,2)*(37 - 18*nu) + 231*(-1 + nu))*r1*Power(r2,3) + 18*(2*(-1 + nu) + Power(j1 - j2,2)*(1 + nu))*Power(r2,4))) + 3*d1*Power(d2,2)*((9*(-1 + nu) + 2*Power(j1 - j2,2)*(1 + nu))*Power(r1,4) - (81*(-1 + nu) + 8*Power(j1 - j2,2)*(6 + nu))*Power(r1,3)*r2 + (-141*(-1 + nu) + 4*Power(j1 - j2,2)*(-22 + 3*nu))*Power(r1,2)*Power(r2,2) + (4*Power(j1 - j2,2)*(3 - 2*nu) + 39*(-1 + nu))*r1*Power(r2,3) + 2*(3 - 3*nu + Power(j1 - j2,2)*(1 + nu))*Power(r2,4)) - 3*Power(d1,2)*d2*(2*(3 - 3*nu + Power(j1 - j2,2)*(1 + nu))*Power(r1,4) + (4*Power(j1 - j2,2)*(3 - 2*nu) + 39*(-1 + nu))*Power(r1,3)*r2 + (-141*(-1 + nu) + 4*Power(j1 - j2,2)*(-22 + 3*nu))*Power(r1,2)*Power(r2,2) - (81*(-1 + nu) + 8*Power(j1 - j2,2)*(6 + nu))*r1*Power(r2,3) + (9*(-1 + nu) + 2*Power(j1 - j2,2)*(1 + nu))*Power(r2,4)) + Power(d2,3)*(18*(2*(-1 + nu) + Power(j1 - j2,2)*(1 + nu))*Power(r1,4) + (4*Power(j1 - j2,2)*(37 - 18*nu) + 231*(-1 + nu))*Power(r1,3)*r2 + (-129*(-1 + nu) + 4*Power(j1 - j2,2)*(-8 + 27*nu))*Power(r1,2)*Power(r2,2) - (-51*(-1 + nu) + 8*Power(j1 - j2,2)*(4 + 9*nu))*r1*Power(r2,3) + 9*(1 - nu + 2*Power(j1 - j2,2)*(1 + nu))*Power(r2,4))) + 60*(2*Power(j1 - j2,2) + 3*(-1 + nu))*r1*r2*Power(d2*r1 - d1*r2,3)*log(r1/r2)))/(8640.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(1,11) = (E*(j1 - j2)*(dr*(Power(d1,3)*(3*(-9 + nu)*Power(r1,4) + (133 - 17*nu)*Power(r1,3)*r2 + (-287 + 43*nu)*Power(r1,2)*Power(r2,2) + (433 - 77*nu)*r1*Power(r2,3) - 12*(-4 + nu)*Power(r2,4)) - 3*Power(d1,2)*d2*((13 + 3*nu)*Power(r1,4) - 7*(11 + nu)*Power(r1,3)*r2 + (253 - 17*nu)*Power(r1,2)*Power(r2,2) + (123 - 47*nu)*r1*Power(r2,3) + 4*(-3 + 2*nu)*Power(r2,4)) - 3*d1*Power(d2,2)*((17 + 7*nu)*Power(r1,4) - 13*(11 + nu)*Power(r1,3)*r2 + (-223 + 107*nu)*Power(r1,2)*Power(r2,2) + (57 - 53*nu)*r1*Power(r2,3) + 4*(-2 + 3*nu)*Power(r2,4)) - Power(d2,3)*((63 + 33*nu)*Power(r1,4) + (373 - 257*nu)*Power(r1,3)*r2 + (-197 + 313*nu)*Power(r1,2)*Power(r2,2) + (73 - 197*nu)*r1*Power(r2,3) + 12*(-1 + 4*nu)*Power(r2,4))) + 60*(-5 + nu)*r1*r2*Power(d2*r1 - d1*r2,3)*log(r1/r2)))/(5760.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(2,2) = (alphaS*dj*dr*G*(3*d1*(4*r1 + r2) + d2*(3*r1 + 2*r2)))/180. + (E*(dr*(Power(d1,3)*(72*Power(r1,4) - 378*Power(r1,3)*r2 + 2*(411 - 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) + (-978 + 35*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) + (822 - 55*Power(j1 - j2,2)*(-1 + nu))*Power(r2,4)) - 3*d1*Power(d2,2)*(-12*Power(r1,4) + 78*Power(r1,3)*r2 + 2*(-141 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) + (-162 + 25*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) + (18 - 5*Power(j1 - j2,2)*(-1 + nu))*Power(r2,4)) + 3*Power(d1,2)*d2*(18*Power(r1,4) - 102*Power(r1,3)*r2 + (258 - 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) + (-462 + 25*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) + 2*(-36 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r2,4)) + Power(d2,3)*(18*Power(r1,4) - 162*Power(r1,3)*r2 + (-282 + 55*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) + (78 - 35*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) + 2*(-6 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r2,4))) + 30*(-12 + Power(j1 - j2,2)*(-1 + nu))*Power(r2,2)*Power(d2*r1 - d1*r2,3)*log(r1/r2)))/(4320.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(2,3) = (alphaS*(3*d1 + d2)*dr*G)/24.;
    KEl(2,4) = -(E*(dj*dr*(Power(d1,3)*(48*(1 + nu)*Power(r1,4) - 3*(89 + 59*nu)*Power(r1,3)*r2 + (613 + 223*nu)*Power(r1,2)*Power(r2,2) - (767 + 77*nu)*r1*Power(r2,3) + (673 - 77*nu)*Power(r2,4)) + 3*Power(d1,2)*d2*(12*(1 + nu)*Power(r1,4) - (73 + 43*nu)*Power(r1,3)*r2 + (197 + 47*nu)*Power(r1,2)*Power(r2,2) + (-373 + 17*nu)*r1*Power(r2,3) + 9*(-7 + 3*nu)*Power(r2,4)) + 3*d1*Power(d2,2)*(8*(1 + nu)*Power(r1,4) - 3*(19 + 9*nu)*Power(r1,3)*r2 + (223 + 13*nu)*Power(r1,2)*Power(r2,2) + (143 - 67*nu)*r1*Power(r2,3) + (-17 + 13*nu)*Power(r2,4)) + Power(d2,3)*(12*(1 + nu)*Power(r1,4) - 3*(41 + 11*nu)*Power(r1,3)*r2 + (-253 + 137*nu)*Power(r1,2)*Power(r2,2) + (77 - 73*nu)*r1*Power(r2,3) + (-13 + 17*nu)*Power(r2,4))) + 60*(j1 - j2)*(-5 + nu)*Power(r2,2)*Power(-(d2*r1) + d1*r2,3)*log(r1/r2)))/(5760.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(2,5) = (alphaS*dj*dr*G*(3*d1*(4*r1 + r2) + d2*(3*r1 + 2*r2)))/360. + (E*(dr*(Power(d1,3)*(-144*Power(r1,4) + 756*Power(r1,3)*r2 - 2*(822 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) + (1956 + 35*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) + (-1644 - 55*Power(j1 - j2,2)*(-1 + nu))*Power(r2,4)) - 3*d1*Power(d2,2)*(24*Power(r1,4) - 156*Power(r1,3)*r2 + 2*(282 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) + (324 + 25*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) + (-36 - 5*Power(j1 - j2,2)*(-1 + nu))*Power(r2,4)) + Power(d2,3)*(-36*Power(r1,4) + 324*Power(r1,3)*r2 + (564 + 55*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) + (-156 - 35*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) + 2*(12 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r2,4)) - 3*Power(d1,2)*d2*(36*Power(r1,4) - 204*Power(r1,3)*r2 + (516 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) + (-924 - 25*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) - 2*(72 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r2,4))) + 30*(24 + Power(j1 - j2,2)*(-1 + nu))*Power(r2,2)*Power(d2*r1 - d1*r2,3)*log(r1/r2)))/(8640.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(2,6) = -(alphaS*(d1 + d2)*dr*G)/24.;
    KEl(2,7) = -(E*(j1 - j2)*(dr*(Power(d2,3)*(12*(-4 + nu)*Power(r1,4) - (183 + 173*nu)*Power(r1,3)*r2 + (57 + 187*nu)*Power(r1,2)*Power(r2,2) - (3 + 113*nu)*r1*Power(r2,3) + 3*(-1 + 9*nu)*Power(r2,4)) + 3*d1*Power(d2,2)*(4*(-3 + 2*nu)*Power(r1,4) + (93 - 17*nu)*Power(r1,3)*r2 + (123 + 113*nu)*Power(r1,2)*Power(r2,2) - 3*(9 + 19*nu)*r1*Power(r2,3) + (3 + 13*nu)*Power(r2,4)) + 3*Power(d1,2)*d2*(4*(-2 + 3*nu)*Power(r1,4) + (47 - 43*nu)*Power(r1,3)*r2 + (-153 + 37*nu)*Power(r1,2)*Power(r2,2) - (73 + 83*nu)*r1*Power(r2,3) + (7 + 17*nu)*Power(r2,4)) + Power(d1,3)*(12*(-1 + 4*nu)*Power(r1,4) + (63 - 187*nu)*Power(r1,3)*r2 + (-147 + 263*nu)*Power(r1,2)*Power(r2,2) + (243 - 127*nu)*r1*Power(r2,3) + 3*(11 + 21*nu)*Power(r2,4))) + 60*(3 + nu)*r1*r2*Power(d2*r1 - d1*r2,3)*log(r2/r1)))/(5760.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(2,8) = -(alphaS*dr*G*(j1 - j2)*(3*d1*r1 + 2*d2*r1 + 2*d1*r2 + 3*d2*r2))/180. + (E*(dr*(Power(d1,3)*(18*Power(r1,4) + 2*(-51 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 + (258 - 35*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) + 11*(-42 + 5*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) - 72*Power(r2,4)) + 3*Power(d1,2)*d2*(12*Power(r1,4) + (-78 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 + (282 - 25*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) - 2*(-81 + 5*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) - 18*Power(r2,4)) + Power(d2,3)*(72*Power(r1,4) - 11*(-42 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 + (-258 + 35*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) - 2*(-51 + 5*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) - 18*Power(r2,4)) + 3*d1*Power(d2,2)*(18*Power(r1,4) + 2*(-81 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 + (-282 + 25*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) + (78 - 5*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) - 12*Power(r2,4))) - 30*(-12 + Power(j1 - j2,2)*(-1 + nu))*r1*r2*Power(d2*r1 - d1*r2,3)*log(r1/r2)))/(4320.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(2,9) = (alphaS*(d1 + d2)*dr*G)/24.;
    KEl(2,10) = -(E*(dr*(j1 - j2)*(Power(d2,3)*(12*(-4 + nu)*Power(r1,4) + (-433 + 77*nu)*Power(r1,3)*r2 + (287 - 43*nu)*Power(r1,2)*Power(r2,2) + (-133 + 17*nu)*r1*Power(r2,3) - 3*(-9 + nu)*Power(r2,4)) + 3*d1*Power(d2,2)*(4*(-3 + 2*nu)*Power(r1,4) + (123 - 47*nu)*Power(r1,3)*r2 + (253 - 17*nu)*Power(r1,2)*Power(r2,2) - 7*(11 + nu)*r1*Power(r2,3) + (13 + 3*nu)*Power(r2,4)) + 3*Power(d1,2)*d2*(4*(-2 + 3*nu)*Power(r1,4) + (57 - 53*nu)*Power(r1,3)*r2 + (-223 + 107*nu)*Power(r1,2)*Power(r2,2) - 13*(11 + nu)*r1*Power(r2,3) + (17 + 7*nu)*Power(r2,4)) + Power(d1,3)*(12*(-1 + 4*nu)*Power(r1,4) + (73 - 197*nu)*Power(r1,3)*r2 + (-197 + 313*nu)*Power(r1,2)*Power(r2,2) + (373 - 257*nu)*r1*Power(r2,3) + 3*(21 + 11*nu)*Power(r2,4))) + 60*dj*(-5 + nu)*r1*r2*Power(d2*r1 - d1*r2,3)*log(r2/r1)))/(5760.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(2,11) = -(alphaS*dr*G*(j1 - j2)*(3*d1*r1 + 2*d2*r1 + 2*d1*r2 + 3*d2*r2))/360. + (E*(dr*(3*d1*Power(d2,2)*(-36*Power(r1,4) + 2*(162 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 + (564 + 25*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) + (-156 - 5*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) + 24*Power(r2,4)) + Power(d2,3)*(-144*Power(r1,4) - 11*(84 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 + (516 + 35*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) - 2*(102 + 5*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) + 36*Power(r2,4)) + 3*Power(d1,2)*d2*(-24*Power(r1,4) + (156 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 + (-564 - 25*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) - 2*(162 + 5*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) + 36*Power(r2,4)) + Power(d1,3)*(-36*Power(r1,4) + 2*(102 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 + (-516 - 35*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) + 11*(84 + 5*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) + 144*Power(r2,4))) - 30*(24 + Power(j1 - j2,2)*(-1 + nu))*r1*r2*Power(d2*r1 - d1*r2,3)*log(r1/r2)))/(8640.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(3,3) = (alphaS*G*(dr*(d1*(-2*(3 + Power(j1 - j2,2))*Power(r1,2) + (21 + Power(j1 - j2,2))*r1*r2 + (-33 + Power(j1 - j2,2))*Power(r2,2)) - d2*((3 + Power(j1 - j2,2))*Power(r1,2) + (-15 + Power(j1 - j2,2))*r1*r2 - 2*(3 + Power(j1 - j2,2))*Power(r2,2))) + 18*Power(r2,2)*(d2*r1 - d1*r2)*log(r1/r2)))/(18.*dj*Power(dr,3));
    KEl(3,4) = -(alphaS*G*Power(j1 - j2,2)*Power(r1 - r2,2)*(d2*(r1 + r2) + d1*(3*r1 + r2)))/(36.*dj*Power(dr,2));
    KEl(3,5) = (alphaS*(3*d1 + d2)*dr*G)/24.;
    KEl(3,6) = (alphaS*G*(dr*(d2*((12 + Power(j1 - j2,2))*Power(r1,2) + (30 + Power(j1 - j2,2))*r1*r2 - 2*(3 + Power(j1 - j2,2))*Power(r2,2)) + d1*(2*(3 + Power(j1 - j2,2))*Power(r1,2) - (30 + Power(j1 - j2,2))*r1*r2 - (12 + Power(j1 - j2,2))*Power(r2,2))) + 36*r1*r2*(d2*r1 - d1*r2)*log(r1/r2)))/(36.*dj*Power(dr,3));
    KEl(3,7) = -(alphaS*G*Power(j1 - j2,2)*Power(r1 - r2,2)*(d1*(r1 + r2) + d2*(r1 + 3*r2)))/(72.*dj*Power(dr,2));
    KEl(3,8) = (alphaS*(d1 + d2)*G*(j1 - j2)*Power(r1 - r2,3))/(24.*dj*Power(dr,2));
    KEl(3,9) = (alphaS*G*(dr*((d2*(-6 + Power(j1 - j2,2)) + d1*(-3 + 2*Power(j1 - j2,2)))*Power(r1,2) - (d1 - d2)*(-15 + Power(j1 - j2,2))*r1*r2 - (d1*(-6 + Power(j1 - j2,2)) + d2*(-3 + 2*Power(j1 - j2,2)))*Power(r2,2)) + 18*r1*r2*(d2*r1 - d1*r2)*log(r2/r1)))/(18.*dj*Power(dr,3));
    KEl(3,10) = -(alphaS*G*Power(j1 - j2,2)*Power(r1 - r2,2)*(d1*(r1 + r2) + d2*(r1 + 3*r2)))/(36.*dj*Power(dr,2));
    KEl(3,11) = (alphaS*(d1 + d2)*G*(j1 - j2)*Power(r1 - r2,3))/(24.*dj*Power(dr,2));
    KEl(4,4) = (alphaS*dj*dr*G*(3*d1*(4*r1 + r2) + d2*(3*r1 + 2*r2)))/180. + (E*(dr*(3*d1*Power(d2,2)*(2*(3 - 3*nu + 8*Power(j1 - j2,2)*(1 + nu))*Power(r1,4) - (-39*(-1 + nu) + 64*Power(j1 - j2,2)*(1 + nu))*Power(r1,3)*r2 + (-141*(-1 + nu) + 16*Power(j1 - j2,2)*(11 + 6*nu))*Power(r1,2)*Power(r2,2) + (8*Power(j1 - j2,2)*(17 - 8*nu) - 81*(-1 + nu))*r1*Power(r2,3) + (9*(-1 + nu) + 8*Power(j1 - j2,2)*(-3 + 2*nu))*Power(r2,4)) + 3*Power(d1,2)*d2*(3*(3 - 3*nu + 8*Power(j1 - j2,2)*(1 + nu))*Power(r1,4) - 3*(-17*(-1 + nu) + 32*Power(j1 - j2,2)*(1 + nu))*Power(r1,3)*r2 + (-129*(-1 + nu) + 8*Power(j1 - j2,2)*(23 + 18*nu))*Power(r1,2)*Power(r2,2) - (-231*(-1 + nu) + 8*Power(j1 - j2,2)*(37 + 12*nu))*r1*Power(r2,3) + 4*(9*(-1 + nu) + 2*Power(j1 - j2,2)*(-7 + 3*nu))*Power(r2,4)) + Power(d2,3)*(3*(3 - 3*nu + 8*Power(j1 - j2,2)*(1 + nu))*Power(r1,4) - 3*(-27*(-1 + nu) + 32*Power(j1 - j2,2)*(1 + nu))*Power(r1,3)*r2 + (141*(-1 + nu) + 8*Power(j1 - j2,2)*(-37 + 18*nu))*Power(r1,2)*Power(r2,2) + (8*Power(j1 - j2,2)*(23 - 12*nu) - 39*(-1 + nu))*r1*Power(r2,3) + 2*(3*(-1 + nu) + 4*Power(j1 - j2,2)*(-7 + 3*nu))*Power(r2,4)) + Power(d1,3)*(12*(3 - 3*nu + 8*Power(j1 - j2,2)*(1 + nu))*Power(r1,4) - 3*(-63*(-1 + nu) + 128*Power(j1 - j2,2)*(1 + nu))*Power(r1,3)*r2 + (-411*(-1 + nu) + 16*Power(j1 - j2,2)*(41 + 36*nu))*Power(r1,2)*Power(r2,2) - (-489*(-1 + nu) + 8*Power(j1 - j2,2)*(83 + 48*nu))*r1*Power(r2,3) + (-411*(-1 + nu) + 8*Power(j1 - j2,2)*(67 + 12*nu))*Power(r2,4))) + 60*(3 + 4*Power(j1 - j2,2) - 3*nu)*Power(r2,2)*Power(-(d2*r1) + d1*r2,3)*log(r1/r2)))/(8640.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(4,5) = (E*(dj*dr*(3*Power(d1,2)*d2*(12*(1 + nu)*Power(r1,4) - (63 + 53*nu)*Power(r1,3)*r2 + (147 + 97*nu)*Power(r1,2)*Power(r2,2) - (243 + 113*nu)*r1*Power(r2,3) - 3*(11 + nu)*Power(r2,4)) + 3*d1*Power(d2,2)*(8*(1 + nu)*Power(r1,4) - (47 + 37*nu)*Power(r1,3)*r2 + (153 + 83*nu)*Power(r1,2)*Power(r2,2) + (73 + 3*nu)*r1*Power(r2,3) + (-7 + 3*nu)*Power(r2,4)) + Power(d2,3)*(12*(1 + nu)*Power(r1,4) - 3*(31 + 21*nu)*Power(r1,3)*r2 + (-123 + 7*nu)*Power(r1,2)*Power(r2,2) + (27 - 23*nu)*r1*Power(r2,3) + (-3 + 7*nu)*Power(r2,4)) + Power(d1,3)*(48*(1 + nu)*Power(r1,4) - 3*(79 + 69*nu)*Power(r1,3)*r2 + (483 + 353*nu)*Power(r1,2)*Power(r2,2) - (537 + 307*nu)*r1*Power(r2,3) + (423 + 173*nu)*Power(r2,4))) - 60*(j1 - j2)*(3 + nu)*Power(r2,2)*Power(-(d2*r1) + d1*r2,3)*log(r1/r2)))/(5760.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(4,6) = (alphaS*dj*G*(d2*(r1 + r2) + d1*(3*r1 + r2)))/72.;
    KEl(4,7) = -(alphaS*dj*G*(r1 - r2)*(3*d1*r1 + 2*d2*r1 + 2*d1*r2 + 3*d2*r2))/360. + (E*(dr*(-(Power(d1,3)*(9*(1 - nu + 2*Power(j1 - j2,2)*(1 + nu))*Power(r1,4) - (-51*(-1 + nu) + 8*Power(j1 - j2,2)*(4 + 9*nu))*Power(r1,3)*r2 + (-129*(-1 + nu) + 4*Power(j1 - j2,2)*(-8 + 27*nu))*Power(r1,2)*Power(r2,2) + (4*Power(j1 - j2,2)*(37 - 18*nu) + 231*(-1 + nu))*r1*Power(r2,3) + 18*(2*(-1 + nu) + Power(j1 - j2,2)*(1 + nu))*Power(r2,4))) + 3*d1*Power(d2,2)*((9*(-1 + nu) + 2*Power(j1 - j2,2)*(1 + nu))*Power(r1,4) - (81*(-1 + nu) + 8*Power(j1 - j2,2)*(6 + nu))*Power(r1,3)*r2 + (-141*(-1 + nu) + 4*Power(j1 - j2,2)*(-22 + 3*nu))*Power(r1,2)*Power(r2,2) + (4*Power(j1 - j2,2)*(3 - 2*nu) + 39*(-1 + nu))*r1*Power(r2,3) + 2*(3 - 3*nu + Power(j1 - j2,2)*(1 + nu))*Power(r2,4)) - 3*Power(d1,2)*d2*(2*(3 - 3*nu + Power(j1 - j2,2)*(1 + nu))*Power(r1,4) + (4*Power(j1 - j2,2)*(3 - 2*nu) + 39*(-1 + nu))*Power(r1,3)*r2 + (-141*(-1 + nu) + 4*Power(j1 - j2,2)*(-22 + 3*nu))*Power(r1,2)*Power(r2,2) - (81*(-1 + nu) + 8*Power(j1 - j2,2)*(6 + nu))*r1*Power(r2,3) + (9*(-1 + nu) + 2*Power(j1 - j2,2)*(1 + nu))*Power(r2,4)) + Power(d2,3)*(18*(2*(-1 + nu) + Power(j1 - j2,2)*(1 + nu))*Power(r1,4) + (4*Power(j1 - j2,2)*(37 - 18*nu) + 231*(-1 + nu))*Power(r1,3)*r2 + (-129*(-1 + nu) + 4*Power(j1 - j2,2)*(-8 + 27*nu))*Power(r1,2)*Power(r2,2) - (-51*(-1 + nu) + 8*Power(j1 - j2,2)*(4 + 9*nu))*r1*Power(r2,3) + 9*(1 - nu + 2*Power(j1 - j2,2)*(1 + nu))*Power(r2,4))) + 60*(2*Power(j1 - j2,2) + 3*(-1 + nu))*r1*r2*Power(d2*r1 - d1*r2,3)*log(r1/r2)))/(8640.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(4,8) = (E*(dj*dr*(Power(d1,3)*(3*(-9 + nu)*Power(r1,4) + (133 - 17*nu)*Power(r1,3)*r2 + (-287 + 43*nu)*Power(r1,2)*Power(r2,2) + (433 - 77*nu)*r1*Power(r2,3) - 12*(-4 + nu)*Power(r2,4)) - 3*Power(d1,2)*d2*((13 + 3*nu)*Power(r1,4) - 7*(11 + nu)*Power(r1,3)*r2 + (253 - 17*nu)*Power(r1,2)*Power(r2,2) + (123 - 47*nu)*r1*Power(r2,3) + 4*(-3 + 2*nu)*Power(r2,4)) - 3*d1*Power(d2,2)*((17 + 7*nu)*Power(r1,4) - 13*(11 + nu)*Power(r1,3)*r2 + (-223 + 107*nu)*Power(r1,2)*Power(r2,2) + (57 - 53*nu)*r1*Power(r2,3) + 4*(-2 + 3*nu)*Power(r2,4)) - Power(d2,3)*((63 + 33*nu)*Power(r1,4) + (373 - 257*nu)*Power(r1,3)*r2 + (-197 + 313*nu)*Power(r1,2)*Power(r2,2) + (73 - 197*nu)*r1*Power(r2,3) + 12*(-1 + 4*nu)*Power(r2,4))) - 60*(j1 - j2)*(-5 + nu)*r1*r2*Power(d2*r1 - d1*r2,3)*log(r1/r2)))/(5760.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(4,9) = (alphaS*dj*G*(d2*(r1 + r2) + d1*(3*r1 + r2)))/36.;
    KEl(4,10) = -(alphaS*dj*G*(r1 - r2)*(3*d1*r1 + 2*d2*r1 + 2*d1*r2 + 3*d2*r2))/180. + (E*(dr*(-(Power(d1,3)*(9*(-1 + nu + 4*Power(j1 - j2,2)*(1 + nu))*Power(r1,4) - (51*(-1 + nu) + 16*Power(j1 - j2,2)*(4 + 9*nu))*Power(r1,3)*r2 + (129*(-1 + nu) + 8*Power(j1 - j2,2)*(-8 + 27*nu))*Power(r1,2)*Power(r2,2) + (8*Power(j1 - j2,2)*(37 - 18*nu) - 231*(-1 + nu))*r1*Power(r2,3) + 36*(1 - nu + Power(j1 - j2,2)*(1 + nu))*Power(r2,4))) + 3*d1*Power(d2,2)*((9 - 9*nu + 4*Power(j1 - j2,2)*(1 + nu))*Power(r1,4) - (-81*(-1 + nu) + 16*Power(j1 - j2,2)*(6 + nu))*Power(r1,3)*r2 + (141*(-1 + nu) + 8*Power(j1 - j2,2)*(-22 + 3*nu))*Power(r1,2)*Power(r2,2) + (8*Power(j1 - j2,2)*(3 - 2*nu) - 39*(-1 + nu))*r1*Power(r2,3) + 2*(3*(-1 + nu) + 2*Power(j1 - j2,2)*(1 + nu))*Power(r2,4)) - 3*Power(d1,2)*d2*((6*(-1 + nu) + 4*Power(j1 - j2,2)*(1 + nu))*Power(r1,4) + (8*Power(j1 - j2,2)*(3 - 2*nu) - 39*(-1 + nu))*Power(r1,3)*r2 + (141*(-1 + nu) + 8*Power(j1 - j2,2)*(-22 + 3*nu))*Power(r1,2)*Power(r2,2) - (-81*(-1 + nu) + 16*Power(j1 - j2,2)*(6 + nu))*r1*Power(r2,3) + (9 - 9*nu + 4*Power(j1 - j2,2)*(1 + nu))*Power(r2,4)) + Power(d2,3)*(36*(1 - nu + Power(j1 - j2,2)*(1 + nu))*Power(r1,4) + (8*Power(j1 - j2,2)*(37 - 18*nu) - 231*(-1 + nu))*Power(r1,3)*r2 + (129*(-1 + nu) + 8*Power(j1 - j2,2)*(-8 + 27*nu))*Power(r1,2)*Power(r2,2) - (51*(-1 + nu) + 16*Power(j1 - j2,2)*(4 + 9*nu))*r1*Power(r2,3) + 9*(-1 + nu + 4*Power(j1 - j2,2)*(1 + nu))*Power(r2,4))) + 60*(3 + 4*Power(j1 - j2,2) - 3*nu)*r1*r2*Power(d2*r1 - d1*r2,3)*log(r1/r2)))/(8640.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(4,11) = (E*(dj*dr*(Power(d1,3)*(3*(-1 + 9*nu)*Power(r1,4) - (3 + 113*nu)*Power(r1,3)*r2 + (57 + 187*nu)*Power(r1,2)*Power(r2,2) - (183 + 173*nu)*r1*Power(r2,3) + 12*(-4 + nu)*Power(r2,4)) + 3*Power(d1,2)*d2*((3 + 13*nu)*Power(r1,4) - 3*(9 + 19*nu)*Power(r1,3)*r2 + (123 + 113*nu)*Power(r1,2)*Power(r2,2) + (93 - 17*nu)*r1*Power(r2,3) + 4*(-3 + 2*nu)*Power(r2,4)) + 3*d1*Power(d2,2)*((7 + 17*nu)*Power(r1,4) - (73 + 83*nu)*Power(r1,3)*r2 + (-153 + 37*nu)*Power(r1,2)*Power(r2,2) + (47 - 43*nu)*r1*Power(r2,3) + 4*(-2 + 3*nu)*Power(r2,4)) + Power(d2,3)*((33 + 63*nu)*Power(r1,4) + (243 - 127*nu)*Power(r1,3)*r2 + (-147 + 263*nu)*Power(r1,2)*Power(r2,2) + (63 - 187*nu)*r1*Power(r2,3) + 12*(-1 + 4*nu)*Power(r2,4))) - 60*(j1 - j2)*(3 + nu)*r1*r2*Power(d2*r1 - d1*r2,3)*log(r1/r2)))/(5760.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(5,5) = (alphaS*dj*dr*G*(3*d1*(4*r1 + r2) + d2*(3*r1 + 2*r2)))/180. + (E*(dr*(Power(d1,3)*(72*Power(r1,4) - 378*Power(r1,3)*r2 + 2*(411 - 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) + (-978 + 35*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) + (822 - 55*Power(j1 - j2,2)*(-1 + nu))*Power(r2,4)) - 3*d1*Power(d2,2)*(-12*Power(r1,4) + 78*Power(r1,3)*r2 + 2*(-141 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) + (-162 + 25*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) + (18 - 5*Power(j1 - j2,2)*(-1 + nu))*Power(r2,4)) + 3*Power(d1,2)*d2*(18*Power(r1,4) - 102*Power(r1,3)*r2 + (258 - 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) + (-462 + 25*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) + 2*(-36 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r2,4)) + Power(d2,3)*(18*Power(r1,4) - 162*Power(r1,3)*r2 + (-282 + 55*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) + (78 - 35*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) + 2*(-6 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r2,4))) + 30*(-12 + Power(j1 - j2,2)*(-1 + nu))*Power(r2,2)*Power(d2*r1 - d1*r2,3)*log(r1/r2)))/(4320.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(5,6) = -(alphaS*(d1 + d2)*dr*G)/24.;
    KEl(5,7) = (E*(j1 - j2)*(dr*(Power(d2,3)*(12*(-4 + nu)*Power(r1,4) + (-433 + 77*nu)*Power(r1,3)*r2 + (287 - 43*nu)*Power(r1,2)*Power(r2,2) + (-133 + 17*nu)*r1*Power(r2,3) - 3*(-9 + nu)*Power(r2,4)) + 3*d1*Power(d2,2)*(4*(-3 + 2*nu)*Power(r1,4) + (123 - 47*nu)*Power(r1,3)*r2 + (253 - 17*nu)*Power(r1,2)*Power(r2,2) - 7*(11 + nu)*r1*Power(r2,3) + (13 + 3*nu)*Power(r2,4)) + 3*Power(d1,2)*d2*(4*(-2 + 3*nu)*Power(r1,4) + (57 - 53*nu)*Power(r1,3)*r2 + (-223 + 107*nu)*Power(r1,2)*Power(r2,2) - 13*(11 + nu)*r1*Power(r2,3) + (17 + 7*nu)*Power(r2,4)) + Power(d1,3)*(12*(-1 + 4*nu)*Power(r1,4) + (73 - 197*nu)*Power(r1,3)*r2 + (-197 + 313*nu)*Power(r1,2)*Power(r2,2) + (373 - 257*nu)*r1*Power(r2,3) + 3*(21 + 11*nu)*Power(r2,4))) + 60*(-5 + nu)*r1*r2*Power(d2*r1 - d1*r2,3)*log(r1/r2)))/(5760.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(5,8) = -(alphaS*dr*G*(j1 - j2)*(3*d1*r1 + 2*d2*r1 + 2*d1*r2 + 3*d2*r2))/360. + (E*(dr*(3*d1*Power(d2,2)*(-36*Power(r1,4) + 2*(162 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 + (564 + 25*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) + (-156 - 5*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) + 24*Power(r2,4)) + Power(d2,3)*(-144*Power(r1,4) - 11*(84 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 + (516 + 35*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) - 2*(102 + 5*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) + 36*Power(r2,4)) + 3*Power(d1,2)*d2*(-24*Power(r1,4) + (156 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 + (-564 - 25*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) - 2*(162 + 5*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) + 36*Power(r2,4)) + Power(d1,3)*(-36*Power(r1,4) + 2*(102 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 + (-516 - 35*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) + 11*(84 + 5*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) + 144*Power(r2,4))) - 30*(24 + Power(j1 - j2,2)*(-1 + nu))*r1*r2*Power(d2*r1 - d1*r2,3)*log(r1/r2)))/(8640.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(5,9) = (alphaS*(d1 + d2)*dr*G)/24.;
    KEl(5,10) = (E*(dr*(j1 - j2)*(Power(d2,3)*(12*(-4 + nu)*Power(r1,4) - (183 + 173*nu)*Power(r1,3)*r2 + (57 + 187*nu)*Power(r1,2)*Power(r2,2) - (3 + 113*nu)*r1*Power(r2,3) + 3*(-1 + 9*nu)*Power(r2,4)) + 3*d1*Power(d2,2)*(4*(-3 + 2*nu)*Power(r1,4) + (93 - 17*nu)*Power(r1,3)*r2 + (123 + 113*nu)*Power(r1,2)*Power(r2,2) - 3*(9 + 19*nu)*r1*Power(r2,3) + (3 + 13*nu)*Power(r2,4)) + 3*Power(d1,2)*d2*(4*(-2 + 3*nu)*Power(r1,4) + (47 - 43*nu)*Power(r1,3)*r2 + (-153 + 37*nu)*Power(r1,2)*Power(r2,2) - (73 + 83*nu)*r1*Power(r2,3) + (7 + 17*nu)*Power(r2,4)) + Power(d1,3)*(12*(-1 + 4*nu)*Power(r1,4) + (63 - 187*nu)*Power(r1,3)*r2 + (-147 + 263*nu)*Power(r1,2)*Power(r2,2) + (243 - 127*nu)*r1*Power(r2,3) + 3*(11 + 21*nu)*Power(r2,4))) + 60*dj*(3 + nu)*r1*r2*Power(d2*r1 - d1*r2,3)*log(r1/r2)))/(5760.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(5,11) = -(alphaS*dr*G*(j1 - j2)*(3*d1*r1 + 2*d2*r1 + 2*d1*r2 + 3*d2*r2))/180. + (E*(dr*(Power(d1,3)*(18*Power(r1,4) + 2*(-51 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 + (258 - 35*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) + 11*(-42 + 5*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) - 72*Power(r2,4)) + 3*Power(d1,2)*d2*(12*Power(r1,4) + (-78 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 + (282 - 25*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) - 2*(-81 + 5*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) - 18*Power(r2,4)) + Power(d2,3)*(72*Power(r1,4) - 11*(-42 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 + (-258 + 35*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) - 2*(-51 + 5*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) - 18*Power(r2,4)) + 3*d1*Power(d2,2)*(18*Power(r1,4) + 2*(-81 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 + (-282 + 25*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) + (78 - 5*Power(j1 - j2,2)*(-1 + nu))*r1*Power(r2,3) - 12*Power(r2,4))) - 30*(-12 + Power(j1 - j2,2)*(-1 + nu))*r1*r2*Power(d2*r1 - d1*r2,3)*log(r1/r2)))/(4320.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(6,6) = (alphaS*G*(dr*(-(d2*((-33 + Power(j1 - j2,2))*Power(r1,2) + (21 + Power(j1 - j2,2))*r1*r2 - 2*(3 + Power(j1 - j2,2))*Power(r2,2))) + d1*(-2*(3 + Power(j1 - j2,2))*Power(r1,2) + (-15 + Power(j1 - j2,2))*r1*r2 + (3 + Power(j1 - j2,2))*Power(r2,2))) + 18*Power(r1,2)*(d2*r1 - d1*r2)*log(r1/r2)))/(18.*dj*Power(dr,3));
    KEl(6,7) = (alphaS*G*Power(j1 - j2,2)*Power(r1 - r2,2)*(d1*(r1 + r2) + d2*(r1 + 3*r2)))/(36.*dj*Power(dr,2));
    KEl(6,8) = -(alphaS*(d1 + 3*d2)*G*(j1 - j2)*Power(r1 - r2,3))/(24.*dj*Power(dr,2));
    KEl(6,9) = (alphaS*G*(dr*(-(d2*((66 + Power(j1 - j2,2))*Power(r1,2) + (-42 + Power(j1 - j2,2))*r1*r2 - 2*(-6 + Power(j1 - j2,2))*Power(r2,2))) + d1*(-2*(-6 + Power(j1 - j2,2))*Power(r1,2) + (30 + Power(j1 - j2,2))*r1*r2 + (-6 + Power(j1 - j2,2))*Power(r2,2))) + 36*Power(r1,2)*(d2*r1 - d1*r2)*log(r2/r1)))/(36.*dj*Power(dr,3));
    KEl(6,10) = (alphaS*G*Power(j1 - j2,2)*Power(r1 - r2,2)*(d1*(r1 + r2) + d2*(r1 + 3*r2)))/(72.*dj*Power(dr,2));
    KEl(6,11) = -(alphaS*(d1 + 3*d2)*G*(j1 - j2)*Power(r1 - r2,3))/(24.*dj*Power(dr,2));
    KEl(7,7) = -(alphaS*dj*G*(r1 - r2)*(d1*(2*r1 + 3*r2) + 3*d2*(r1 + 4*r2)))/180. - (E*(dr*(3*Power(d1,2)*d2*((9*(-1 + nu) + 8*Power(j1 - j2,2)*(-3 + 2*nu))*Power(r1,4) + (8*Power(j1 - j2,2)*(17 - 8*nu) - 81*(-1 + nu))*Power(r1,3)*r2 + (-141*(-1 + nu) + 16*Power(j1 - j2,2)*(11 + 6*nu))*Power(r1,2)*Power(r2,2) - (-39*(-1 + nu) + 64*Power(j1 - j2,2)*(1 + nu))*r1*Power(r2,3) + 2*(3 - 3*nu + 8*Power(j1 - j2,2)*(1 + nu))*Power(r2,4)) + Power(d1,3)*(2*(3*(-1 + nu) + 4*Power(j1 - j2,2)*(-7 + 3*nu))*Power(r1,4) + (8*Power(j1 - j2,2)*(23 - 12*nu) - 39*(-1 + nu))*Power(r1,3)*r2 + (141*(-1 + nu) + 8*Power(j1 - j2,2)*(-37 + 18*nu))*Power(r1,2)*Power(r2,2) - 3*(-27*(-1 + nu) + 32*Power(j1 - j2,2)*(1 + nu))*r1*Power(r2,3) + 3*(3 - 3*nu + 8*Power(j1 - j2,2)*(1 + nu))*Power(r2,4)) + 3*d1*Power(d2,2)*(4*(9*(-1 + nu) + 2*Power(j1 - j2,2)*(-7 + 3*nu))*Power(r1,4) - (-231*(-1 + nu) + 8*Power(j1 - j2,2)*(37 + 12*nu))*Power(r1,3)*r2 + (-129*(-1 + nu) + 8*Power(j1 - j2,2)*(23 + 18*nu))*Power(r1,2)*Power(r2,2) - 3*(-17*(-1 + nu) + 32*Power(j1 - j2,2)*(1 + nu))*r1*Power(r2,3) + 3*(3 - 3*nu + 8*Power(j1 - j2,2)*(1 + nu))*Power(r2,4)) + Power(d2,3)*((-411*(-1 + nu) + 8*Power(j1 - j2,2)*(67 + 12*nu))*Power(r1,4) - (-489*(-1 + nu) + 8*Power(j1 - j2,2)*(83 + 48*nu))*Power(r1,3)*r2 + (-411*(-1 + nu) + 16*Power(j1 - j2,2)*(41 + 36*nu))*Power(r1,2)*Power(r2,2) - 3*(-63*(-1 + nu) + 128*Power(j1 - j2,2)*(1 + nu))*r1*Power(r2,3) + 12*(3 - 3*nu + 8*Power(j1 - j2,2)*(1 + nu))*Power(r2,4))) + 60*(3 + 4*Power(j1 - j2,2) - 3*nu)*Power(r1,2)*Power(d2*r1 - d1*r2,3)*log(r1/r2)))/(8640.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(7,8) = -(E*(dr*(j1 - j2)*(-3*d1*Power(d2,2)*(3*(11 + nu)*Power(r1,4) + (243 + 113*nu)*Power(r1,3)*r2 - (147 + 97*nu)*Power(r1,2)*Power(r2,2) + (63 + 53*nu)*r1*Power(r2,3) - 12*(1 + nu)*Power(r2,4)) + 3*Power(d1,2)*d2*((-7 + 3*nu)*Power(r1,4) + (73 + 3*nu)*Power(r1,3)*r2 + (153 + 83*nu)*Power(r1,2)*Power(r2,2) - (47 + 37*nu)*r1*Power(r2,3) + 8*(1 + nu)*Power(r2,4)) + Power(d1,3)*((-3 + 7*nu)*Power(r1,4) + (27 - 23*nu)*Power(r1,3)*r2 + (-123 + 7*nu)*Power(r1,2)*Power(r2,2) - 3*(31 + 21*nu)*r1*Power(r2,3) + 12*(1 + nu)*Power(r2,4)) + Power(d2,3)*((423 + 173*nu)*Power(r1,4) - (537 + 307*nu)*Power(r1,3)*r2 + (483 + 353*nu)*Power(r1,2)*Power(r2,2) - 3*(79 + 69*nu)*r1*Power(r2,3) + 48*(1 + nu)*Power(r2,4))) + 60*dj*(3 + nu)*Power(r1,2)*Power(d2*r1 - d1*r2,3)*log(r2/r1)))/(5760.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(7,9) = (alphaS*dj*G*(d1*(r1 + r2) + d2*(r1 + 3*r2)))/72.;
    KEl(7,10) = -(alphaS*dj*G*(r1 - r2)*(d1*(2*r1 + 3*r2) + 3*d2*(r1 + 4*r2)))/360. - (E*(dr*(3*Power(d1,2)*d2*((9 - 9*nu + 4*Power(j1 - j2,2)*(-3 + 2*nu))*Power(r1,4) + (4*Power(j1 - j2,2)*(17 - 8*nu) + 81*(-1 + nu))*Power(r1,3)*r2 + (141*(-1 + nu) + 8*Power(j1 - j2,2)*(11 + 6*nu))*Power(r1,2)*Power(r2,2) - (39*(-1 + nu) + 32*Power(j1 - j2,2)*(1 + nu))*r1*Power(r2,3) + 2*(3*(-1 + nu) + 4*Power(j1 - j2,2)*(1 + nu))*Power(r2,4)) + 3*d1*Power(d2,2)*(4*(9 - 9*nu + Power(j1 - j2,2)*(-7 + 3*nu))*Power(r1,4) - (231*(-1 + nu) + 4*Power(j1 - j2,2)*(37 + 12*nu))*Power(r1,3)*r2 + (129*(-1 + nu) + 4*Power(j1 - j2,2)*(23 + 18*nu))*Power(r1,2)*Power(r2,2) - 3*(17*(-1 + nu) + 16*Power(j1 - j2,2)*(1 + nu))*r1*Power(r2,3) + 3*(3*(-1 + nu) + 4*Power(j1 - j2,2)*(1 + nu))*Power(r2,4)) + Power(d1,3)*(2*(3 - 3*nu + 2*Power(j1 - j2,2)*(-7 + 3*nu))*Power(r1,4) + (39*(-1 + nu) - 4*Power(j1 - j2,2)*(-23 + 12*nu))*Power(r1,3)*r2 + (-141*(-1 + nu) + 4*Power(j1 - j2,2)*(-37 + 18*nu))*Power(r1,2)*Power(r2,2) - 3*(27*(-1 + nu) + 16*Power(j1 - j2,2)*(1 + nu))*r1*Power(r2,3) + 3*(3*(-1 + nu) + 4*Power(j1 - j2,2)*(1 + nu))*Power(r2,4)) + Power(d2,3)*((411*(-1 + nu) + 4*Power(j1 - j2,2)*(67 + 12*nu))*Power(r1,4) - (489*(-1 + nu) + 4*Power(j1 - j2,2)*(83 + 48*nu))*Power(r1,3)*r2 + (411*(-1 + nu) + 8*Power(j1 - j2,2)*(41 + 36*nu))*Power(r1,2)*Power(r2,2) - 3*(63*(-1 + nu) + 64*Power(j1 - j2,2)*(1 + nu))*r1*Power(r2,3) + 12*(3*(-1 + nu) + 4*Power(j1 - j2,2)*(1 + nu))*Power(r2,4))) + 60*(2*Power(j1 - j2,2) + 3*(-1 + nu))*Power(r1,2)*Power(d2*r1 - d1*r2,3)*log(r1/r2)))/(8640.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(7,11) = (E*(j1 - j2)*(dr*(3*Power(d1,2)*d2*((-17 + 13*nu)*Power(r1,4) + (143 - 67*nu)*Power(r1,3)*r2 + (223 + 13*nu)*Power(r1,2)*Power(r2,2) - 3*(19 + 9*nu)*r1*Power(r2,3) + 8*(1 + nu)*Power(r2,4)) + Power(d1,3)*((-13 + 17*nu)*Power(r1,4) + (77 - 73*nu)*Power(r1,3)*r2 + (-253 + 137*nu)*Power(r1,2)*Power(r2,2) - 3*(41 + 11*nu)*r1*Power(r2,3) + 12*(1 + nu)*Power(r2,4)) + 3*d1*Power(d2,2)*(9*(-7 + 3*nu)*Power(r1,4) + (-373 + 17*nu)*Power(r1,3)*r2 + (197 + 47*nu)*Power(r1,2)*Power(r2,2) - (73 + 43*nu)*r1*Power(r2,3) + 12*(1 + nu)*Power(r2,4)) + Power(d2,3)*((673 - 77*nu)*Power(r1,4) - (767 + 77*nu)*Power(r1,3)*r2 + (613 + 223*nu)*Power(r1,2)*Power(r2,2) - 3*(89 + 59*nu)*r1*Power(r2,3) + 48*(1 + nu)*Power(r2,4))) + 60*(-5 + nu)*Power(r1,2)*Power(d2*r1 - d1*r2,3)*log(r2/r1)))/(5760.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(8,8) = -(alphaS*dr*G*(j1 - j2)*(d1*(2*r1 + 3*r2) + 3*d2*(r1 + 4*r2)))/180. - (E*(dr*(3*Power(d1,2)*d2*((-18 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,4) + (162 - 25*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 + 2*(141 - 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) - 78*r1*Power(r2,3) + 12*Power(r2,4)) + Power(d1,3)*(2*(-6 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,4) + (78 - 35*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 + (-282 + 55*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) - 162*r1*Power(r2,3) + 18*Power(r2,4)) + 3*d1*Power(d2,2)*(2*(-36 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,4) + (-462 + 25*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 + (258 - 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) - 102*r1*Power(r2,3) + 18*Power(r2,4)) + Power(d2,3)*((822 - 55*Power(j1 - j2,2)*(-1 + nu))*Power(r1,4) + (-978 + 35*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 + 2*(411 - 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) - 378*r1*Power(r2,3) + 72*Power(r2,4))) - 30*(-12 + Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(d2*r1 - d1*r2,3)*log(r1/r2)))/(4320.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(8,9) = (alphaS*(d1 + 3*d2)*dr*G)/24.;
    KEl(8,10) = (E*(-(dr*(j1 - j2)*(3*Power(d1,2)*d2*((-17 + 13*nu)*Power(r1,4) + (143 - 67*nu)*Power(r1,3)*r2 + (223 + 13*nu)*Power(r1,2)*Power(r2,2) - 3*(19 + 9*nu)*r1*Power(r2,3) + 8*(1 + nu)*Power(r2,4)) + Power(d1,3)*((-13 + 17*nu)*Power(r1,4) + (77 - 73*nu)*Power(r1,3)*r2 + (-253 + 137*nu)*Power(r1,2)*Power(r2,2) - 3*(41 + 11*nu)*r1*Power(r2,3) + 12*(1 + nu)*Power(r2,4)) + 3*d1*Power(d2,2)*(9*(-7 + 3*nu)*Power(r1,4) + (-373 + 17*nu)*Power(r1,3)*r2 + (197 + 47*nu)*Power(r1,2)*Power(r2,2) - (73 + 43*nu)*r1*Power(r2,3) + 12*(1 + nu)*Power(r2,4)) + Power(d2,3)*((673 - 77*nu)*Power(r1,4) - (767 + 77*nu)*Power(r1,3)*r2 + (613 + 223*nu)*Power(r1,2)*Power(r2,2) - 3*(89 + 59*nu)*r1*Power(r2,3) + 48*(1 + nu)*Power(r2,4)))) + 60*dj*(-5 + nu)*Power(r1,2)*Power(d2*r1 - d1*r2,3)*log(r2/r1)))/(5760.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(8,11) = -(alphaS*dr*G*(j1 - j2)*(d1*(2*r1 + 3*r2) + 3*d2*(r1 + 4*r2)))/360. - (E*(dr*(Power(d2,3)*((-1644 - 55*Power(j1 - j2,2)*(-1 + nu))*Power(r1,4) + (1956 + 35*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 - 2*(822 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) + 756*r1*Power(r2,3) - 144*Power(r2,4)) + 3*d1*Power(d2,2)*(2*(72 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,4) + (924 + 25*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 + (-516 - 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) + 204*r1*Power(r2,3) - 36*Power(r2,4)) + Power(d1,3)*(2*(12 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,4) + (-156 - 35*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 + (564 + 55*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) + 324*r1*Power(r2,3) - 36*Power(r2,4)) + 3*Power(d1,2)*d2*((36 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,4) + (-324 - 25*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 - 2*(282 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) + 156*r1*Power(r2,3) - 24*Power(r2,4))) - 30*(24 + Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(d2*r1 - d1*r2,3)*log(r1/r2)))/(8640.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(9,9) = (alphaS*G*(dr*(-(d2*((-33 + Power(j1 - j2,2))*Power(r1,2) + (21 + Power(j1 - j2,2))*r1*r2 - 2*(3 + Power(j1 - j2,2))*Power(r2,2))) + d1*(-2*(3 + Power(j1 - j2,2))*Power(r1,2) + (-15 + Power(j1 - j2,2))*r1*r2 + (3 + Power(j1 - j2,2))*Power(r2,2))) + 18*Power(r1,2)*(d2*r1 - d1*r2)*log(r1/r2)))/(18.*dj*Power(dr,3));
    KEl(9,10) = (alphaS*G*Power(j1 - j2,2)*Power(r1 - r2,2)*(d1*(r1 + r2) + d2*(r1 + 3*r2)))/(36.*dj*Power(dr,2));
    KEl(9,11) = (alphaS*(d1 + 3*d2)*G*(j1 - j2)*Power(r1 - r2,3))/(24.*dj*Power(dr,2));
    KEl(10,10) = -(alphaS*dj*G*(r1 - r2)*(d1*(2*r1 + 3*r2) + 3*d2*(r1 + 4*r2)))/180. - (E*(dr*(3*Power(d1,2)*d2*((9*(-1 + nu) + 8*Power(j1 - j2,2)*(-3 + 2*nu))*Power(r1,4) + (8*Power(j1 - j2,2)*(17 - 8*nu) - 81*(-1 + nu))*Power(r1,3)*r2 + (-141*(-1 + nu) + 16*Power(j1 - j2,2)*(11 + 6*nu))*Power(r1,2)*Power(r2,2) - (-39*(-1 + nu) + 64*Power(j1 - j2,2)*(1 + nu))*r1*Power(r2,3) + 2*(3 - 3*nu + 8*Power(j1 - j2,2)*(1 + nu))*Power(r2,4)) + Power(d1,3)*(2*(3*(-1 + nu) + 4*Power(j1 - j2,2)*(-7 + 3*nu))*Power(r1,4) + (8*Power(j1 - j2,2)*(23 - 12*nu) - 39*(-1 + nu))*Power(r1,3)*r2 + (141*(-1 + nu) + 8*Power(j1 - j2,2)*(-37 + 18*nu))*Power(r1,2)*Power(r2,2) - 3*(-27*(-1 + nu) + 32*Power(j1 - j2,2)*(1 + nu))*r1*Power(r2,3) + 3*(3 - 3*nu + 8*Power(j1 - j2,2)*(1 + nu))*Power(r2,4)) + 3*d1*Power(d2,2)*(4*(9*(-1 + nu) + 2*Power(j1 - j2,2)*(-7 + 3*nu))*Power(r1,4) - (-231*(-1 + nu) + 8*Power(j1 - j2,2)*(37 + 12*nu))*Power(r1,3)*r2 + (-129*(-1 + nu) + 8*Power(j1 - j2,2)*(23 + 18*nu))*Power(r1,2)*Power(r2,2) - 3*(-17*(-1 + nu) + 32*Power(j1 - j2,2)*(1 + nu))*r1*Power(r2,3) + 3*(3 - 3*nu + 8*Power(j1 - j2,2)*(1 + nu))*Power(r2,4)) + Power(d2,3)*((-411*(-1 + nu) + 8*Power(j1 - j2,2)*(67 + 12*nu))*Power(r1,4) - (-489*(-1 + nu) + 8*Power(j1 - j2,2)*(83 + 48*nu))*Power(r1,3)*r2 + (-411*(-1 + nu) + 16*Power(j1 - j2,2)*(41 + 36*nu))*Power(r1,2)*Power(r2,2) - 3*(-63*(-1 + nu) + 128*Power(j1 - j2,2)*(1 + nu))*r1*Power(r2,3) + 12*(3 - 3*nu + 8*Power(j1 - j2,2)*(1 + nu))*Power(r2,4))) + 60*(3 + 4*Power(j1 - j2,2) - 3*nu)*Power(r1,2)*Power(d2*r1 - d1*r2,3)*log(r1/r2)))/(8640.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(10,11) = (E*(dr*(j1 - j2)*(-3*d1*Power(d2,2)*(3*(11 + nu)*Power(r1,4) + (243 + 113*nu)*Power(r1,3)*r2 - (147 + 97*nu)*Power(r1,2)*Power(r2,2) + (63 + 53*nu)*r1*Power(r2,3) - 12*(1 + nu)*Power(r2,4)) + 3*Power(d1,2)*d2*((-7 + 3*nu)*Power(r1,4) + (73 + 3*nu)*Power(r1,3)*r2 + (153 + 83*nu)*Power(r1,2)*Power(r2,2) - (47 + 37*nu)*r1*Power(r2,3) + 8*(1 + nu)*Power(r2,4)) + Power(d1,3)*((-3 + 7*nu)*Power(r1,4) + (27 - 23*nu)*Power(r1,3)*r2 + (-123 + 7*nu)*Power(r1,2)*Power(r2,2) - 3*(31 + 21*nu)*r1*Power(r2,3) + 12*(1 + nu)*Power(r2,4)) + Power(d2,3)*((423 + 173*nu)*Power(r1,4) - (537 + 307*nu)*Power(r1,3)*r2 + (483 + 353*nu)*Power(r1,2)*Power(r2,2) - 3*(79 + 69*nu)*r1*Power(r2,3) + 48*(1 + nu)*Power(r2,4))) + 60*dj*(3 + nu)*Power(r1,2)*Power(d2*r1 - d1*r2,3)*log(r2/r1)))/(5760.*dj*Power(dr,5)*(-1 + Power(nu,2)));
    KEl(11,11) = -(alphaS*dr*G*(j1 - j2)*(d1*(2*r1 + 3*r2) + 3*d2*(r1 + 4*r2)))/180. - (E*(dr*(3*Power(d1,2)*d2*((-18 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,4) + (162 - 25*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 + 2*(141 - 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) - 78*r1*Power(r2,3) + 12*Power(r2,4)) + Power(d1,3)*(2*(-6 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,4) + (78 - 35*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 + (-282 + 55*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) - 162*r1*Power(r2,3) + 18*Power(r2,4)) + 3*d1*Power(d2,2)*(2*(-36 + 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,4) + (-462 + 25*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 + (258 - 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) - 102*r1*Power(r2,3) + 18*Power(r2,4)) + Power(d2,3)*((822 - 55*Power(j1 - j2,2)*(-1 + nu))*Power(r1,4) + (-978 + 35*Power(j1 - j2,2)*(-1 + nu))*Power(r1,3)*r2 + 2*(411 - 5*Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(r2,2) - 378*r1*Power(r2,3) + 72*Power(r2,4))) - 30*(-12 + Power(j1 - j2,2)*(-1 + nu))*Power(r1,2)*Power(d2*r1 - d1*r2,3)*log(r1/r2)))/(4320.*dj*Power(dr,5)*(-1 + Power(nu,2)));

    // correction of material law in stiffness matrix / minus sign in azimuthal direction (FIX)
    KEl(1,2) *= -1.;
    KEl(1,3) *= -1.;
    KEl(1,5) *= -1.;
    KEl(1,6) *= -1.;
    KEl(1,8) *= -1.;
    KEl(1,9) *= -1.;
    KEl(1,11) *= -1.;
    KEl(4,5) *= -1.;
    KEl(4,6) *= -1.;
    KEl(4,8) *= -1.;
    KEl(4,9) *= -1.;
    KEl(4,11) *= -1.;
    KEl(7,8) *= -1.;
    KEl(7,9) *= -1.;
    KEl(7,11) *= -1.;
    KEl(10,11) *= -1.;
    KEl(0,1) *= -1.;
    KEl(0,4) *= -1.;
    KEl(0,7) *= -1.;
    KEl(0,10) *= -1.;
    KEl(2,4) *= -1.;
    KEl(2,7) *= -1.;
    KEl(2,10) *= -1.;
    KEl(3,4) *= -1.;
    KEl(3,7) *= -1.;
    KEl(3,10) *= -1.;
    KEl(5,7) *= -1.;
    KEl(5,10) *= -1.;
    KEl(6,7) *= -1.;
    KEl(6,10) *= -1.;
    KEl(8,10) *= -1.;
    KEl(9,10) *= -1.;
  }

  Vec FiniteElement2s13Disk::computeState(const Vec &NodeCoordinates,const Vec &qElement,const Vec &qpElement,const Vec &s,double d1,double d2) {
    Vec X(12,NONINIT);

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

  Mat FiniteElement2s13Disk::JGeneralized(const Vec &NodeCoordinates,const Vec &s) {

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

