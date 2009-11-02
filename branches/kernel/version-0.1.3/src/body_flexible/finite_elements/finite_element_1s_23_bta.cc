/* Copyright (C) 2005-2008  Roland Zander

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
 * Contact:
 *   rzander@users.berlios.de
 *
 */

#include<config.h>
#include "finite_element_1s_23_bta.h"
using namespace fmatvec;

namespace MBSim {

  // functions of Mathematica
  inline double Sec(const double& alpha) {return 1.0/cos(alpha);}
  inline double Power(double base, int exponent) {return pow(base,exponent);}
  // functions of Mathematica

  FiniteElement1s23BTA::FiniteElement1s23BTA(double sl0, double sArho, double sEIyy, double sEIzz, double sItrho, double sGIt, Vec sg, int warnLevel_) :
  warnLevel(warnLevel_), l0(sl0), Arho(sArho), EIyy(sEIyy), EIzz(sEIzz), Itrho(sItrho), GIt(sGIt), g(sg), dTorsional(0.), depsilon(0.), implicit(false),
  M(getSizeOfVelocities(),INIT,0.0), h(getSizeOfVelocities(),INIT,0.), Dhq(8), Dhqp(8), Damp(8,INIT,0.)
  {
	l0h2 = l0*l0;
	l0h3 = l0h2*l0;
  }
 
  void FiniteElement1s23BTA::computeEquationsOfMotion(const Vec& q, const Vec& v)
  {
	const double &a1 = q(0);
	const double &w1 = q(1); const double &b1 = q(2);
	const double &u1 = q(3); const double &g1 = q(4);
	const double &a2 = q(5);
	const double &w2 = q(6); const double &b2 = q(7);
	const double &u2 = q(8); const double &g2 = q(9);

	const double &a1p = v(0);
	const double &w1p = v(1); const double &b1p = v(2);
	const double &u1p = v(3); const double &g1p = v(4);
	const double &a2p = v(5);
	const double &w2p = v(6); const double &b2p = v(7);
	const double &u2p = v(8); const double &g2p = v(9);

	// mass
	M(0,0) = (l0*(420*Itrho + Arho*(5*Power(b1,2)*l0h2 + 2*Power(b2,2)*l0h2 + 5*Power(g1,2)*l0h2 - 5*g1*g2*l0h2 + 2*Power(g2,2)*l0h2 + 65*g1*l0*u1 - 25*g2*l0*u1 + 290*Power(u1,2) + 19*g1*l0*u2 - 17*g2*l0*u2 + 92*u1*u2 + 38*Power(u2,2) - 25*b2*l0*w1 + 290*Power(w1,2) - 17*b2*l0*w2 + 92*w1*w2 + 38*Power(w2,2) + b1*l0*(-5*b2*l0 + 65*w1 + 19*w2))))/1260.;
	M(0,1) = -(Arho*l0*(15*g1*l0 - 7*g2*l0 + 120*u1 + 27*u2))/420.;
	M(0,2) = -(Arho*l0h2*(5*g1*l0 - 3*g2*l0 + 30*u1 + 12*u2))/840.;
	M(0,3) = (Arho*l0*(15*b1*l0 - 7*b2*l0 + 120*w1 + 27*w2))/420.;
	M(0,4) = (Arho*l0h2*(5*b1*l0 - 3*b2*l0 + 30*w1 + 12*w2))/840.;
	M(0,5) = (l0*(420*Itrho + Arho*(5*Power(b1,2)*l0h2 + 5*Power(b2,2)*l0h2 + 5*Power(g1,2)*l0h2 - 8*g1*g2*l0h2 + 5*Power(g2,2)*l0h2 + 50*g1*l0*u1 - 34*g2*l0*u1 + 140*Power(u1,2) + 34*g1*l0*u2 - 50*g2*l0*u2 + 140*u1*u2 + 140*Power(u2,2) + 2*b1*l0*(-4*b2*l0 + 25*w1 + 17*w2) - 2*b2*l0*(17*w1 + 25*w2) + 140*(Power(w1,2) + w1*w2 + Power(w2,2)))))/2520.;
	M(0,6) = -(Arho*l0*(6*g1*l0 - 7*g2*l0 + 27*u1 + 36*u2))/420.;
	M(0,7) = (Arho*l0h2*(3*g1*l0 - 3*g2*l0 + 14*(u1 + u2)))/840.;
	M(0,8) = (Arho*l0*(6*b1*l0 - 7*b2*l0 + 27*w1 + 36*w2))/420.;
	M(0,9) = -(Arho*l0h2*(3*b1*l0 - 3*b2*l0 + 14*(w1 + w2)))/840.;
	// M(1,0) = M(0,1);
	M(1,1) = (13*Arho*l0)/35.;
	M(1,2) = (11*Arho*l0h2)/210.;
	// M(1,3) = 0;
	// M(1,4) = 0;
	M(1,5) = -(Arho*l0*(7*g1*l0 - 6*g2*l0 + 36*u1 + 27*u2))/420.;
	M(1,6) = (9*Arho*l0)/70.;
	M(1,7) = (-13*Arho*l0h2)/420.;
	// M(1,8) = 0;
	// M(1,9) = 0;
	// M(2,0) = M(0,2);
	// M(2,1) = M(1,2);
	M(2,2) = (Arho*l0h3)/105.;
	// M(2,3) = 0;
	// M(2,4) = 0;
	M(2,5) = -(Arho*l0h2*(3*g1*l0 - 3*g2*l0 + 14*(u1 + u2)))/840.;
	M(2,6) = (13*Arho*l0h2)/420.;
	M(2,7) = -(Arho*l0h3)/140.;
	// M(2,8) = 0;
	// M(2,9) = 0;
	// M(3,0) = M(0,3);
	// M(3,1) = M(1,3);
	// M(3,2) = M(2,3);
	M(3,3) = (13*Arho*l0)/35.;
	M(3,4) = (11*Arho*l0h2)/210.;
	M(3,5) = (Arho*l0*(7*b1*l0 - 6*b2*l0 + 36*w1 + 27*w2))/420.;
	M(3,6) = 0;
	M(3,7) = 0;
	M(3,8) = (9*Arho*l0)/70.;
	M(3,9) = (-13*Arho*l0h2)/420.;
	// M(4,0) = M(0,4);
	// M(4,1) = M(1,4);
	// M(4,2) = M(2,4);
	// M(4,3) = M(3,4);
	M(4,4) = (Arho*l0h3)/105.;
	M(4,5) = (Arho*l0h2*(3*b1*l0 - 3*b2*l0 + 14*(w1 + w2)))/840.;
	// M(4,6) = 0;
	// M(4,7) = 0;
	M(4,8) = (13*Arho*l0h2)/420.;
	M(4,9) = -(Arho*l0h3)/140.;
	// M(5,0) = M(0,5);
	// M(5,1) = M(1,5);
	// M(5,2) = M(2,5);
	// M(5,3) = M(3,5);
	// M(5,4) = M(4,5);
	M(5,5) = (l0*(420*Itrho + Arho*(2*Power(b1,2)*l0h2 + 5*Power(b2,2)*l0h2 + 2*Power(g1,2)*l0h2 - 5*g1*g2*l0h2 + 5*Power(g2,2)*l0h2 + 17*g1*l0*u1 - 19*g2*l0*u1 + 38*Power(u1,2) + 25*g1*l0*u2 - 65*g2*l0*u2 + 92*u1*u2 + 290*Power(u2,2) - 19*b2*l0*w1 + 38*Power(w1,2) - 65*b2*l0*w2 + 92*w1*w2 + 290*Power(w2,2) + b1*l0*(-5*b2*l0 + 17*w1 + 25*w2))))/1260.;
	M(5,6) = -(Arho*l0*(7*g1*l0 + 3*(-5*g2*l0 + 9*u1 + 40*u2)))/420.;
	M(5,7) = (Arho*l0h2*(3*g1*l0 - 5*g2*l0 + 12*u1 + 30*u2))/840.;
	M(5,8) = (Arho*l0*(7*b1*l0 + 3*(-5*b2*l0 + 9*w1 + 40*w2)))/420.;
	M(5,9) = -(Arho*l0h2*(3*b1*l0 - 5*b2*l0 + 12*w1 + 30*w2))/840.;
	// M(6,0) = M(0,6);
	// M(6,1) = M(1,6);
	// M(6,2) = M(2,6);
	// M(6,3) = M(3,6);
	// M(6,4) = M(4,6);
	// M(6,5) = M(5,6);
	M(6,6) = (13*Arho*l0)/35.;
	M(6,7) = (-11*Arho*l0h2)/210.;
	// M(6,8) = 0;
	// M(6,9) = 0;
	// M(7,0) = M(0,7);
	// M(7,1) = M(1,7);
	// M(7,2) = M(2,7);
	// M(7,3) = M(3,7);
	// M(7,4) = M(4,7);
	// M(7,5) = M(5,7);
	// M(7,6) = M(6,7);
	M(7,7) = (Arho*l0h3)/105.;
	// M(7,8) = 0;
	// M(7,9) = 0;
	// M(8,0) = M(0,8);
	// M(8,1) = M(1,8);
	// M(8,2) = M(2,8);
	// M(8,3) = M(3,8);
	// M(8,4) = M(4,8);
	// M(8,5) = M(5,8);
	// M(8,6) = M(6,8);
	// M(8,7) = M(7,8);
	M(8,8) = (13*Arho*l0)/35.;
	M(8,9) = (-11*Arho*l0h2)/210.;
	// M(9,0) = M(0,9);
	// M(9,1) = M(1,9);
	// M(9,2) = M(2,9);
	// M(9,3) = M(3,9);
	// M(9,4) = M(4,9);
	// M(9,5) = M(5,9);
	// M(9,6) = M(6,9);
	// M(9,7) = M(7,9);
	// M(9,8) = M(8,9);
	M(9,9) = (Arho*l0h3)/105.;

	// h
	h(0) = (((-a1 + a2)*GIt)/l0 - (Arho*l0*(a2p*(5*b2*b2p*l0h2 + 5*g1*g1p*l0h2 - 4*g1p*g2*l0h2 - 4*g1*g2p*l0h2 + 5*g2*g2p*l0h2 + 25*g1p*l0*u1 - 17*g2p*l0*u1 + 25*g1*l0*u1p - 17*g2*l0*u1p + 140*u1*u1p + 17*g1p*l0*u2 - 25*g2p*l0*u2 + 70*u1p*u2 + 17*g1*l0*u2p - 25*g2*l0*u2p + 70*u1*u2p + 140*u2*u2p - 17*b2p*l0*w1 - 17*b2*l0*w1p + 140*w1*w1p - 25*b2p*l0*w2 + 70*w1p*w2 + b1p*l0*(-4*b2*l0 + 25*w1 + 17*w2) - 25*b2*l0*w2p + 70*w1*w2p + 140*w2*w2p + b1*l0*(5*b1p*l0 - 4*b2p*l0 + 25*w1p + 17*w2p)) + a1p*(4*b2*b2p*l0h2 + 10*g1*g1p*l0h2 - 5*g1p*g2*l0h2 - 5*g1*g2p*l0h2 + 4*g2*g2p*l0h2 + 65*g1p*l0*u1 - 25*g2p*l0*u1 + 65*g1*l0*u1p - 25*g2*l0*u1p + 580*u1*u1p + 19*g1p*l0*u2 - 17*g2p*l0*u2 + 92*u1p*u2 + 19*g1*l0*u2p - 17*g2*l0*u2p + 92*u1*u2p + 76*u2*u2p - 25*b2p*l0*w1 - 25*b2*l0*w1p + 580*w1*w1p - 17*b2p*l0*w2 + 92*w1p*w2 + b1p*l0*(-5*b2*l0 + 65*w1 + 19*w2) - 17*b2*l0*w2p + 92*w1*w2p + 76*w2*w2p + b1*l0*(10*b1p*l0 - 5*b2p*l0 + 65*w1p + 19*w2p))))/1260. );
	// + (Arho*l0*((-18*a1*g1*gy*l0 + Power(a1,3)*g1*gy*l0 + 18*a2*g1*gy*l0 - 3*Power(a1,2)*a2*g1*gy*l0 + 3*a1*Power(a2,2)*g1*gy*l0 - Power(a2,3)*g1*gy*l0 - 12*a1*g2*gy*l0 + 12*a2*g2*gy*l0 - 24*g1*gz*l0 + 6*Power(a1,2)*g1*gz*l0 - 12*a1*a2*g1*gz*l0 + 6*Power(a2,2)*g1*gz*l0 - 24*g2*gz*l0 + 2*Power(a1,2)*g2*gz*l0 - 4*a1*a2*g2*gz*l0 + 2*Power(a2,2)*g2*gz*l0 + 2*b2*((-12 + Power(a1 - a2,2))*gy + 6*(a1 - a2)*gz)*l0 + b1*(6*(-4 + Power(a1 - a2,2))*gy - (-18 + Power(a1 - a2,2))*(a1 - a2)*gz)*l0 - 30*a1*gy*u1 - Power(a1,3)*gy*u1 + 30*a2*gy*u1 + 3*Power(a1,2)*a2*gy*u1 - 3*a1*Power(a2,2)*gy*u1 + Power(a2,3)*gy*u1 - 48*gz*u1 + 6*Power(a1,2)*gz*u1 + Power(a1,4)*gz*u1 - 12*a1*a2*gz*u1 - 4*Power(a1,3)*a2*gz*u1 + 6*Power(a2,2)*gz*u1 + 6*Power(a1,2)*Power(a2,2)*gz*u1 - 4*a1*Power(a2,3)*gz*u1 + Power(a2,4)*gz*u1 + 30*a1*gy*u2 - 30*a2*gy*u2 + 48*gz*u2 - 6*Power(a1,2)*gz*u2 + 12*a1*a2*gz*u2 - 6*Power(a2,2)*gz*u2 - 48*gy*w1 + 6*Power(a1,2)*gy*w1 + Power(a1,4)*gy*w1 - 12*a1*a2*gy*w1 - 4*Power(a1,3)*a2*gy*w1 + 6*Power(a2,2)*gy*w1 + 6*Power(a1,2)*Power(a2,2)*gy*w1 - 4*a1*Power(a2,3)*gy*w1 + Power(a2,4)*gy*w1 + 30*a1*gz*w1 + Power(a1,3)*gz*w1 - 30*a2*gz*w1 - 3*Power(a1,2)*a2*gz*w1 + 3*a1*Power(a2,2)*gz*w1 - Power(a2,3)*gz*w1 - 6*((-8 + Power(a1 - a2,2))*gy + 5*(a1 - a2)*gz)*w2)*cos(a1) + (-6*a1*g1*gy*l0 + 6*a2*g1*gy*l0 - 12*a1*g2*gy*l0 + 12*a2*g2*gy*l0 + 24*g1*gz*l0 + 24*g2*gz*l0 - 2*Power(a1,2)*g2*gz*l0 + 4*a1*a2*g2*gz*l0 - 2*Power(a2,2)*g2*gz*l0 + 6*b1*(4*gy + (a1 - a2)*gz)*l0 - 2*b2*((-12 + Power(a1 - a2,2))*gy + 6*(-a1 + a2)*gz)*l0 - 18*a1*gy*u1 + 18*a2*gy*u1 + 48*gz*u1 + 18*a1*gy*u2 + Power(a1,3)*gy*u2 - 18*a2*gy*u2 - 3*Power(a1,2)*a2*gy*u2 + 3*a1*Power(a2,2)*gy*u2 - Power(a2,3)*gy*u2 - 48*gz*u2 + 48*gy*w1 + 18*a1*gz*w1 - 18*a2*gz*w1 - (48*gy + (18 + Power(a1 - a2,2))*(a1 - a2)*gz)*w2)*cos(a2) + (Power(a2,3)*(-(b1*gy*l0) - g1*gz*l0 + gz*u1 + gy*w1) + Power(a1,4)*(-(gy*u1) + gz*w1) + Power(a2,4)*(-(gy*u1) + gz*w1) + Power(a1,3)*(b1*gy*l0 + g1*gz*l0 + 4*a2*gy*u1 - gz*u1 - (gy + 4*a2*gz)*w1) + Power(a2,2)*(-6*g1*gy*l0 - 2*g2*gy*l0 + 6*gy*(-u1 + u2) + 2*gz*(3*b1*l0 + b2*l0 + 3*w1 - 3*w2)) + 24*(g1*gy*l0 + g2*gy*l0 + 2*gy*(u1 - u2) - gz*(b1*l0 + b2*l0 + 2*w1 - 2*w2)) + 6*a2*(3*b1*gy*l0 + 2*b2*gy*l0 + gz*(3*g1*l0 + 2*g2*l0 + 5*u1 - 5*u2) + 5*gy*(w1 - w2)) + Power(a1,2)*(-6*g1*gy*l0 - 2*g2*gy*l0 + 6*b1*gz*l0 + 2*b2*gz*l0 - 6*gy*u1 + 6*gy*u2 + 6*gz*w1 - 3*a2*(b1*gy*l0 + g1*gz*l0 - gz*u1 - gy*w1) + Power(a2,2)*(-6*gy*u1 + 6*gz*w1) - 6*gz*w2) + a1*(12*a2*g1*gy*l0 + 4*a2*g2*gy*l0 - 18*g1*gz*l0 + 3*Power(a2,2)*g1*gz*l0 - 12*g2*gz*l0 + 3*b1*((-6 + Power(a2,2))*gy - 4*a2*gz)*l0 - 4*b2*(3*gy + a2*gz)*l0 + 12*a2*gy*u1 + 4*Power(a2,3)*gy*u1 - 30*gz*u1 - 3*Power(a2,2)*gz*u1 - 12*a2*gy*u2 + 30*gz*u2 - 30*gy*w1 - 3*Power(a2,2)*gy*w1 - 12*a2*gz*w1 - 4*Power(a2,3)*gz*w1 + 30*gy*w2 + 12*a2*gz*w2))*sin(a1) + (2*Power(a2,2)*(g2*gy - b2*gz)*l0 - a1*(6*b1*gy*l0 + 4*a2*g2*gy*l0 + 6*g1*gz*l0 + 12*g2*gz*l0 + 4*b2*(3*gy - a2*gz)*l0 + 18*gz*u1 - 18*gz*u2 - 3*Power(a2,2)*gz*u2 + 18*gy*w1) + 24*(-(g1*gy*l0) - g2*gy*l0 + 2*gy*(-u1 + u2) + gz*(b1*l0 + b2*l0 + 2*w1 - 2*w2)) + 6*a2*(b1*gy*l0 + 2*b2*gy*l0 + gz*(g1*l0 + 2*g2*l0 + 3*u1 - 3*u2) + 3*gy*(w1 - w2)) + 3*a1*(6 + Power(a2,2))*gy*w2 + Power(a1,3)*(gz*u2 + gy*w2) - Power(a2,3)*(gz*u2 + gy*w2) + Power(a1,2)*(2*g2*gy*l0 - 2*b2*gz*l0 - 3*a2*(gz*u2 + gy*w2)))*sin(a2)))/Power(a1 - a2,5));
	h(1) = ((-15120*b1*EIzz*l0 - 15120*b2*EIzz*l0 + (65*Power(a1p,2) + 50*a1p*a2p + 17*Power(a2p,2))*Arho*b1*Power(l0,5) - (25*Power(a1p,2) + 34*a1p*a2p + 19*Power(a2p,2))*Arho*b2*Power(l0,5) + 4*(-7560*EIzz*(w1 - w2) + Arho*Power(l0,4)*(Power(a1p,2)*(145*w1 + 23*w2) + a2p*(21*g1p*l0 - 18*g2p*l0 + 108*u1p + 81*u2p + 19*a2p*w1 + 23*a2p*w2) + a1p*(45*g1p*l0 - 21*g2p*l0 + 360*u1p + 81*u2p + 35*a2p*(2*w1 + w2)))))/(2520.*l0h3) );
	//+ (Arho*l0*((12*gy - (6 + Power(a1 - a2,2))*(a1 - a2)*gz)*cos(a1) - 6*(2*gy + a1*gz - a2*gz)*cos(a2) + ((6 + Power(a1 - a2,2))*(a1 - a2)*gy + 12*gz)*sin(a1) + 6*(a1*gy - a2*gy - 2*gz)*sin(a2)))/Power(a1 - a2,4));
	h(2) = ((-5040*b2*EIzz*l0 - (5*Power(a1p,2) + 8*a1p*a2p + 5*Power(a2p,2))*Arho*b2*Power(l0,5) + 2*b1*l0*(-5040*EIzz + (5*Power(a1p,2) + 5*a1p*a2p + 2*Power(a2p,2))*Arho*Power(l0,4)) - 15120*EIzz*(w1 - w2) + Arho*Power(l0,4)*(Power(a1p,2)*(65*w1 + 19*w2) + 2*a1p*(15*g1p*l0 - 9*g2p*l0 + 90*u1p + 36*u2p + 25*a2p*w1 + 17*a2p*w2) + a2p*(18*(g1p - g2p)*l0 + 84*u1p + 84*u2p + 17*a2p*w1 + 25*a2p*w2)))/(2520.*l0h2) );
	//- (Arho*l0h2*(((-6 + Power(a1 - a2,2))*gy + 4*(a1 - a2)*gz)*cos(a1) + 2*(3*gy + a1*gz - a2*gz)*cos(a2) + (-4*(a1 - a2)*gy + (-6 + Power(a1 - a2,2))*gz)*sin(a1) + 2*(-(a1*gy) + a2*gy + 3*gz)*sin(a2)))/Power(a1 - a2,4));
	h(3) = ((-15120*EIyy*(g1*l0 + g2*l0 + 2*u1 - 2*u2) + Arho*Power(l0,4)*(Power(a1p,2)*(65*g1*l0 - 25*g2*l0 + 580*u1 + 92*u2) + a2p*(-84*b1p*l0 + 72*b2p*l0 + a2p*(17*g1*l0 - 19*g2*l0 + 76*u1 + 92*u2) - 108*(4*w1p + 3*w2p)) - 2*a1p*(90*b1p*l0 - 42*b2p*l0 - a2p*(25*g1*l0 - 17*g2*l0 + 70*(2*u1 + u2)) + 18*(40*w1p + 9*w2p))))/(2520.*l0h3) );
	//+ (Arho*l0*(((6 + Power(a1 - a2,2))*(a1 - a2)*gy + 12*gz)*cos(a1) + 6*(a1*gy - a2*gy - 2*gz)*cos(a2) + (-12*gy + (6 + Power(a1 - a2,2))*(a1 - a2)*gz)*sin(a1) + 6*(2*gy + a1*gz - a2*gz)*sin(a2)))/Power(a1 - a2,4));
	h(4) = ((-5040*EIyy*(2*g1*l0 + g2*l0 + 3*u1 - 3*u2) + Arho*Power(l0,4)*(Power(a1p,2)*(10*g1*l0 - 5*g2*l0 + 65*u1 + 19*u2) - 2*a1p*(15*b1p*l0 - 9*b2p*l0 - a2p*(5*g1*l0 - 4*g2*l0 + 25*u1 + 17*u2) + 90*w1p + 36*w2p) + a2p*(-18*b1p*l0 + 18*b2p*l0 + a2p*(4*g1*l0 - 5*g2*l0 + 17*u1 + 25*u2) - 84*(w1p + w2p))))/(2520.*l0h2) );
	//+ (Arho*l0h2*(-((-4*(a1 - a2)*gy + (-6 + Power(a1 - a2,2))*gz)*cos(a1)) + 2*(a1*gy - a2*gy - 3*gz)*cos(a2) + ((-6 + Power(a1 - a2,2))*gy + 4*(a1 - a2)*gz)*sin(a1) + 2*(3*gy + a1*gz - a2*gz)*sin(a2)))/Power(a1 - a2,4));
	h(5) = (((a1 - a2)*GIt)/l0 - (Arho*l0*(a1p*(5*b2*b2p*l0h2 + 5*g1*g1p*l0h2 - 4*g1p*g2*l0h2 - 4*g1*g2p*l0h2 + 5*g2*g2p*l0h2 + 25*g1p*l0*u1 - 17*g2p*l0*u1 + 25*g1*l0*u1p - 17*g2*l0*u1p + 140*u1*u1p + 17*g1p*l0*u2 - 25*g2p*l0*u2 + 70*u1p*u2 + 17*g1*l0*u2p - 25*g2*l0*u2p + 70*u1*u2p + 140*u2*u2p - 17*b2p*l0*w1 - 17*b2*l0*w1p + 140*w1*w1p - 25*b2p*l0*w2 + 70*w1p*w2 + b1p*l0*(-4*b2*l0 + 25*w1 + 17*w2) - 25*b2*l0*w2p + 70*w1*w2p + 140*w2*w2p + b1*l0*(5*b1p*l0 - 4*b2p*l0 + 25*w1p + 17*w2p)) + a2p*(10*b2*b2p*l0h2 + 4*g1*g1p*l0h2 - 5*g1p*g2*l0h2 - 5*g1*g2p*l0h2 + 10*g2*g2p*l0h2 + 17*g1p*l0*u1 - 19*g2p*l0*u1 + 17*g1*l0*u1p - 19*g2*l0*u1p + 76*u1*u1p + 25*g1p*l0*u2 - 65*g2p*l0*u2 + 92*u1p*u2 + 25*g1*l0*u2p - 65*g2*l0*u2p + 92*u1*u2p + 580*u2*u2p - 19*b2p*l0*w1 - 19*b2*l0*w1p + 76*w1*w1p - 65*b2p*l0*w2 + 92*w1p*w2 + b1p*l0*(-5*b2*l0 + 17*w1 + 25*w2) - 65*b2*l0*w2p + 92*w1*w2p + 580*w2*w2p + b1*l0*(4*b1p*l0 - 5*b2p*l0 + 17*w1p + 25*w2p))))/1260. );
	//+ (Arho*l0*((12*a1*g1*gy*l0 - 12*a2*g1*gy*l0 + 6*a1*g2*gy*l0 - 6*a2*g2*gy*l0 + 24*g1*gz*l0 - 2*Power(a1,2)*g1*gz*l0 + 4*a1*a2*g1*gz*l0 - 2*Power(a2,2)*g1*gz*l0 + 24*g2*gz*l0 - 2*b1*((-12 + Power(a1 - a2,2))*gy + 6*(a1 - a2)*gz)*l0 + 6*b2*(4*gy - a1*gz + a2*gz)*l0 + 18*a1*gy*u1 + Power(a1,3)*gy*u1 - 18*a2*gy*u1 - 3*Power(a1,2)*a2*gy*u1 + 3*a1*Power(a2,2)*gy*u1 - Power(a2,3)*gy*u1 + 48*gz*u1 - 18*a1*gy*u2 + 18*a2*gy*u2 - 48*gz*u2 + 48*gy*w1 - 18*a1*gz*w1 - Power(a1,3)*gz*w1 + 18*a2*gz*w1 + 3*Power(a1,2)*a2*gz*w1 - 3*a1*Power(a2,2)*gz*w1 + Power(a2,3)*gz*w1 - 6*(8*gy - 3*a1*gz + 3*a2*gz)*w2)*cos(a1) + (12*a1*g1*gy*l0 - 12*a2*g1*gy*l0 + 18*a1*g2*gy*l0 - Power(a1,3)*g2*gy*l0 - 18*a2*g2*gy*l0 + 3*Power(a1,2)*a2*g2*gy*l0 - 3*a1*Power(a2,2)*g2*gy*l0 + Power(a2,3)*g2*gy*l0 - 24*g1*gz*l0 + 2*Power(a1,2)*g1*gz*l0 - 4*a1*a2*g1*gz*l0 + 2*Power(a2,2)*g1*gz*l0 - 24*g2*gz*l0 + 6*Power(a1,2)*g2*gz*l0 - 12*a1*a2*g2*gz*l0 + 6*Power(a2,2)*g2*gz*l0 + b2*(6*(-4 + Power(a1 - a2,2))*gy + (-18 + Power(a1 - a2,2))*(a1 - a2)*gz)*l0 + 2*b1*((-12 + Power(a1 - a2,2))*gy + 6*(-a1 + a2)*gz)*l0 + 30*a1*gy*u1 - 30*a2*gy*u1 - 48*gz*u1 + 6*Power(a1,2)*gz*u1 - 12*a1*a2*gz*u1 + 6*Power(a2,2)*gz*u1 - 30*a1*gy*u2 - Power(a1,3)*gy*u2 + 30*a2*gy*u2 + 3*Power(a1,2)*a2*gy*u2 - 3*a1*Power(a2,2)*gy*u2 + Power(a2,3)*gy*u2 + 48*gz*u2 - 6*Power(a1,2)*gz*u2 - Power(a1,4)*gz*u2 + 12*a1*a2*gz*u2 + 4*Power(a1,3)*a2*gz*u2 - 6*Power(a2,2)*gz*u2 - 6*Power(a1,2)*Power(a2,2)*gz*u2 + 4*a1*Power(a2,3)*gz*u2 - Power(a2,4)*gz*u2 - 48*gy*w1 + 6*Power(a1,2)*gy*w1 - 12*a1*a2*gy*w1 + 6*Power(a2,2)*gy*w1 - 30*a1*gz*w1 + 30*a2*gz*w1 - ((-48 + (6 + Power(a1 - a2,2))*Power(a1 - a2,2))*gy - (30 + Power(a1 - a2,2))*(a1 - a2)*gz)*w2)*cos(a2) + (2*Power(a2,2)*(g1*gy - b1*gz)*l0 + Power(a1,3)*(gz*u1 + gy*w1) - Power(a2,3)*(gz*u1 + gy*w1) + Power(a1,2)*(2*g1*gy*l0 - 2*b1*gz*l0 - 3*a2*(gz*u1 + gy*w1)) + 24*(-(g1*gy*l0) - g2*gy*l0 + 2*gy*(-u1 + u2) + gz*(b1*l0 + b2*l0 + 2*w1 - 2*w2)) - 6*a2*(2*b1*gy*l0 + b2*gy*l0 + gz*(2*g1*l0 + g2*l0 + 3*u1 - 3*u2) + 3*gy*(w1 - w2)) + a1*(6*b2*gy*l0 - 4*a2*g1*gy*l0 + 12*g1*gz*l0 + 6*g2*gz*l0 + 4*b1*(3*gy + a2*gz)*l0 + 18*gz*u1 + 3*Power(a2,2)*gz*u1 - 18*gz*u2 + 18*gy*w1 + 3*Power(a2,2)*gy*w1 - 18*gy*w2))*sin(a1) + (Power(a2,2)*(-2*g1*gy*l0 - 6*g2*gy*l0 + 6*gy*(-u1 + u2) + 2*gz*(b1*l0 + 3*b2*l0 + 3*w1 - 3*w2)) + 24*(g1*gy*l0 + g2*gy*l0 + 2*gy*(u1 - u2) - gz*(b1*l0 + b2*l0 + 2*w1 - 2*w2)) - 6*a2*(2*b1*gy*l0 + 3*b2*gy*l0 + gz*(2*g1*l0 + 3*g2*l0 + 5*u1 - 5*u2) + 5*gy*(w1 - w2)) + Power(a2,3)*(b2*gy*l0 + g2*gz*l0 + gz*u2 + gy*w2) + Power(a1,4)*(gy*u2 - gz*w2) + Power(a2,4)*(gy*u2 - gz*w2) + a1*(4*a2*g1*gy*l0 + 12*a2*g2*gy*l0 + 12*g1*gz*l0 + 18*g2*gz*l0 - 3*Power(a2,2)*g2*gz*l0 + 4*b1*(3*gy - a2*gz)*l0 - 3*b2*((-6 + Power(a2,2))*gy + 4*a2*gz)*l0 + 12*a2*gy*u1 + 30*gz*u1 - 12*a2*gy*u2 - 4*Power(a2,3)*gy*u2 - 30*gz*u2 - 3*Power(a2,2)*gz*u2 + 30*gy*w1 - 12*a2*gz*w1 - 30*gy*w2 - 3*Power(a2,2)*gy*w2 + 12*a2*gz*w2 + 4*Power(a2,3)*gz*w2) - Power(a1,3)*(b2*gy*l0 + gy*(4*a2*u2 + w2) + gz*(g2*l0 + u2 - 4*a2*w2)) + Power(a1,2)*(-2*g1*gy*l0 - 6*g2*gy*l0 + 2*b1*gz*l0 + 6*b2*gz*l0 - 6*gy*u1 + 6*gy*u2 + 6*gz*w1 - 6*gz*w2 + 3*a2*(b2*gy*l0 + g2*gz*l0 + gz*u2 + gy*w2) + 6*Power(a2,2)*(gy*u2 - gz*w2)))*sin(a2)))/Power(a1 - a2,5));
	h(6) = ((15120*b1*EIzz*l0 + 15120*b2*EIzz*l0 + (19*Power(a1p,2) + 34*a1p*a2p + 25*Power(a2p,2))*Arho*b1*Power(l0,5) - (17*Power(a1p,2) + 50*a1p*a2p + 65*Power(a2p,2))*Arho*b2*Power(l0,5) + 4*(7560*EIzz*(w1 - w2) + Arho*Power(l0,4)*(Power(a1p,2)*(23*w1 + 19*w2) + a2p*(21*g1p*l0 - 45*g2p*l0 + 81*u1p + 360*u2p + 23*a2p*w1 + 145*a2p*w2) + a1p*(3*(6*g1p - 7*g2p)*l0 + 81*u1p + 108*u2p + 35*a2p*(w1 + 2*w2)))))/(2520.*l0h3));
	// + (Arho*l0*(-6*(2*gy + (-a1 + a2)*gz)*cos(a1) + (12*gy + (6 + Power(a1 - a2,2))*(a1 - a2)*gz)*cos(a2) - 6*(a1*gy - a2*gy + 2*gz)*sin(a1) + (-((6 + Power(a1 - a2,2))*(a1 - a2)*gy) + 12*gz)*sin(a2)))/Power(a1 - a2,4));
	h(7) = (-(5040*b1*EIzz*l0 + 10080*b2*EIzz*l0 + (5*Power(a1p,2) + 8*a1p*a2p + 5*Power(a2p,2))*Arho*b1*Power(l0,5) - 2*(2*Power(a1p,2) + 5*a1p*a2p + 5*Power(a2p,2))*Arho*b2*Power(l0,5) + 15120*EIzz*(w1 - w2) + Arho*Power(l0,4)*(Power(a1p,2)*(25*w1 + 17*w2) + 2*a1p*(9*g1p*l0 - 9*g2p*l0 + 42*u1p + 42*u2p + 17*a2p*w1 + 25*a2p*w2) + a2p*(18*g1p*l0 - 30*g2p*l0 + 72*u1p + 180*u2p + 19*a2p*w1 + 65*a2p*w2)))/(2520.*l0h2));
	// + (Arho*l0h2*(2*(3*gy + (-a1 + a2)*gz)*cos(a1) + ((-6 + Power(a1 - a2,2))*gy + 4*(-a1 + a2)*gz)*cos(a2) + 2*(a1*gy - a2*gy + 3*gz)*sin(a1) + (4*(a1 - a2)*gy + (-6 + Power(a1 - a2,2))*gz)*sin(a2)))/Power(a1 - a2,4));
	h(8) = ((15120*EIyy*(g1*l0 + g2*l0 + 2*u1 - 2*u2) + Arho*Power(l0,4)*(Power(a1p,2)*(19*g1*l0 - 17*g2*l0 + 92*u1 + 76*u2) + a1p*(-72*b1p*l0 + 84*b2p*l0 + 2*a2p*(17*g1*l0 - 25*g2*l0 + 70*(u1 + 2*u2)) - 108*(3*w1p + 4*w2p)) + a2p*(-84*b1p*l0 + 180*b2p*l0 + a2p*(25*g1*l0 - 65*g2*l0 + 92*u1 + 580*u2) - 36*(9*w1p + 40*w2p))))/(2520.*l0h3) );
	//+ (Arho*l0*(-6*(a1*gy - a2*gy + 2*gz)*cos(a1) + (-((6 + Power(a1 - a2,2))*(a1 - a2)*gy) + 12*gz)*cos(a2) + 6*(2*gy - a1*gz + a2*gz)*sin(a1) + (-12*gy - (6 + Power(a1 - a2,2))*(a1 - a2)*gz)*sin(a2)))/Power(a1 - a2,4));
	h(9) = (-(5040*EIyy*(g1*l0 + 2*g2*l0 + 3*u1 - 3*u2) + Arho*Power(l0,4)*(Power(a1p,2)*(5*g1*l0 - 4*g2*l0 + 25*u1 + 17*u2) - 2*a1p*(9*b1p*l0 - 9*b2p*l0 - a2p*(4*g1*l0 - 5*g2*l0 + 17*u1 + 25*u2) + 42*(w1p + w2p)) + a2p*(-18*b1p*l0 + 30*b2p*l0 + a2p*(5*(g1 - 2*g2)*l0 + 19*u1 + 65*u2) - 36*(2*w1p + 5*w2p))))/(2520.*l0h2) );
	//+ (Arho*l0h2*(2*(a1*gy - a2*gy + 3*gz)*cos(a1) + (4*(a1 - a2)*gy + (-6 + Power(a1 - a2,2))*gz)*cos(a2) - 2*(3*gy + (-a1 + a2)*gz)*sin(a1) - ((-6 + Power(a1 - a2,2))*gy + 4*(-a1 + a2)*gz)*sin(a2)))/Power(a1 - a2,4));


	// Damping
	h(0) += dTorsional * ( a2p - a1p ) ;
	h(5) += dTorsional * ( a1p - a2p ) ;
  }
  
  double FiniteElement1s23BTA::computeKineticEnergy(const Vec& q,const Vec& u) {if(warnLevel>0) cout << "WARNING (FiniteElement1s23BTA:computeKineticEnergy): Not implemented!" << endl; return 0;}
  double FiniteElement1s23BTA::computeGravitationalEnergy(const Vec& q) {if(warnLevel>0) cout << "WARNING (FiniteElement1s23BTA:computeGravitationalEnergy): Not implemented!" << endl; return 0;}     
  double FiniteElement1s23BTA::computeElasticEnergy(const Vec& q) {if(warnLevel>0) cout << "WARNING (FiniteElement1s23BTA:computeElasticEnergy): Not implemented!" << endl; return 0;}
  Mat FiniteElement1s23BTA::computeJacobianOfMinimalRepresentationRegardingPhysics(const Vec& q,const ContourPointData &data) {return JGeneralized(q,data.alpha(0));}

  Vec FiniteElement1s23BTA::Tangent(const Vec& q, const double& s)
  {
	Vec tangent(3,NONINIT);

	const double &a1 = q(0);
	const double &w1 = q(1);  const double &b1 = q(2);
	const double &u1 = q(3);  const double &g1 = q(4);
	const double &a2 = q(5);
	const double &w2 = q(6);  const double &b2 = q(7);
	const double &u2 = q(8);  const double &g2 = q(9);

	tangent(0) = 1.0;
	tangent(1) = -((-(a1/l0) + a2/l0)*(g1*s + u1 - (Power(s,2)*(2*g1*l0 + g2*l0 + 3*u1 - 3*u2))/l0h2 + (Power(s,3)*(g1*l0 + g2*l0 + 2*u1 - 2*u2))/l0h3)*cos((a1*(l0 - s))/l0 + (a2*s)/l0)) + (b1 - (2*s*(2*b1*l0 + b2*l0 + 3*w1 - 3*w2))/l0h2 + (3*Power(s,2)*(b1*l0 + b2*l0 + 2*w1 - 2*w2))/l0h3)*cos((a1*(l0 - s))/l0 + (a2*s)/l0) - (g1 - (2*s*(2*g1*l0 + g2*l0 + 3*u1 - 3*u2))/l0h2 + (3*Power(s,2)*(g1*l0 + g2*l0 + 2*u1 - 2*u2))/l0h3)*sin((a1*(l0 - s))/l0 + (a2*s)/l0) - (-(a1/l0) + a2/l0)*(b1*s + w1 - (Power(s,2)*(2*b1*l0 + b2*l0 + 3*w1 - 3*w2))/l0h2 + (Power(s,3)*(b1*l0 + b2*l0 + 2*w1 - 2*w2))/l0h3)*sin((a1*(l0 - s))/l0 + (a2*s)/l0);
	tangent(2) = (g1 - (2*s*(2*g1*l0 + g2*l0 + 3*u1 - 3*u2))/l0h2 + (3*Power(s,2)*(g1*l0 + g2*l0 + 2*u1 - 2*u2))/l0h3)*cos((a1*(l0 - s))/l0 + (a2*s)/l0) + (-(a1/l0) + a2/l0)*(b1*s + w1 - (Power(s,2)*(2*b1*l0 + b2*l0 + 3*w1 - 3*w2))/l0h2 + (Power(s,3)*(b1*l0 + b2*l0 + 2*w1 - 2*w2))/l0h3)*cos((a1*(l0 - s))/l0 + (a2*s)/l0) - (-(a1/l0) + a2/l0)*(g1*s + u1 - (Power(s,2)*(2*g1*l0 + g2*l0 + 3*u1 - 3*u2))/l0h2 + (Power(s,3)*(g1*l0 + g2*l0 + 2*u1 - 2*u2))/l0h3)*sin((a1*(l0 - s))/l0 + (a2*s)/l0) + (b1 - (2*s*(2*b1*l0 + b2*l0 + 3*w1 - 3*w2))/l0h2 + (3*Power(s,2)*(b1*l0 + b2*l0 + 2*w1 - 2*w2))/l0h3)*sin((a1*(l0 - s))/l0 + (a2*s)/l0);

	tangent /= nrm2(tangent);

	return tangent;
  }

  SqrMat FiniteElement1s23BTA::AWK(const Vec& q, const double& s)
  {
	SqrMat AWK(3,NONINIT);
	Vec bzT(3,INIT,0.);

	double alpha = q(0) * (1-s/l0) + q(5) * (s/l0);

	bzT(1) = -sin(alpha);
	bzT(2) =  cos(alpha);

	AWK.col(0)  = Tangent(q,s);
	AWK.col(1)  = crossProduct(bzT,AWK.col(0));
	AWK.col(1) /= nrm2(AWK.col(1));
	AWK.col(2)  = crossProduct(AWK.col(0),AWK.col(1));

	return AWK;
  }

  Vec FiniteElement1s23BTA::StateAxis(const Vec& q, const Vec& v,const double& s)
  {
	Vec X(12); // x,y,z,alpha and 2x bending angle AND velocities TODO

	const double &a1 = q(0);
	const double &w1 = q(1); const double &b1 = q(2);
	const double &u1 = q(3); const double &g1 = q(4);
	const double &a2 = q(5);
	const double &w2 = q(6); const double &b2 = q(7);
	const double &u2 = q(8); const double &g2 = q(9);

	const double &a1p = v(0);
	const double &w1p = v(1); const double &b1p = v(2);
	const double &u1p = v(3); const double &g1p = v(4);
	const double &a2p = v(5);
	const double &w2p = v(6); const double &b2p = v(7);
	const double &u2p = v(8); const double &g2p = v(9);

	X(0) = s;
	X(1) = (b1*s + w1 - (Power(s,2)*(2*b1*l0 + b2*l0 + 3*w1 - 3*w2))/Power(l0,2) + (Power(s,3)*(b1*l0 + b2*l0 + 2*w1 - 2*w2))/Power(l0,3))*cos((a1*(l0 - s))/l0 + (a2*s)/l0) - (g1*s + u1 - (Power(s,2)*(2*g1*l0 + g2*l0 + 3*u1 - 3*u2))/Power(l0,2) + (Power(s,3)*(g1*l0 + g2*l0 + 2*u1 - 2*u2))/Power(l0,3))*sin((a1*(l0 - s))/l0 + (a2*s)/l0);
	X(2) = (g1*s + u1 - (Power(s,2)*(2*g1*l0 + g2*l0 + 3*u1 - 3*u2))/Power(l0,2) + (Power(s,3)*(g1*l0 + g2*l0 + 2*u1 - 2*u2))/Power(l0,3))*cos((a1*(l0 - s))/l0 + (a2*s)/l0) + (b1*s + w1 - (Power(s,2)*(2*b1*l0 + b2*l0 + 3*w1 - 3*w2))/Power(l0,2) + (Power(s,3)*(b1*l0 + b2*l0 + 2*w1 - 2*w2))/Power(l0,3))*sin((a1*(l0 - s))/l0 + (a2*s)/l0);
	X(3) = (a1*(l0 - s))/l0 + (a2*s)/l0;
	X(4) = (g1 - (2*s*(2*g1*l0 + g2*l0 + 3*u1 - 3*u2))/Power(l0,2) + (3*Power(s,2)*(g1*l0 + g2*l0 + 2*u1 - 2*u2))/Power(l0,3))*cos((a1*(l0 - s))/l0 + (a2*s)/l0) - (-b1 + (2*s*(2*b1*l0 + b2*l0 + 3*w1 - 3*w2))/Power(l0,2) - (3*Power(s,2)*(b1*l0 + b2*l0 + 2*w1 - 2*w2))/Power(l0,3))*sin((a1*(l0 - s))/l0 + (a2*s)/l0);
	X( 5) = (-b1 + (2*s*(2*b1*l0 + b2*l0 + 3*w1 - 3*w2))/Power(l0,2) - (3*Power(s,2)*(b1*l0 + b2*l0 + 2*w1 - 2*w2))/Power(l0,3))*cos((a1*(l0 - s))/l0 + (a2*s)/l0) + (g1 - (2*s*(2*g1*l0 + g2*l0 + 3*u1 - 3*u2))/Power(l0,2) + (3*Power(s,2)*(g1*l0 + g2*l0 + 2*u1 - 2*u2))/Power(l0,3))*sin((a1*(l0 - s))/l0 + (a2*s)/l0);
	X(6) = 0;
	X(7) = -(((a1p*(l0 - s) + a2p*s)*(g1*l0*Power(l0 - s,2)*s + g2*l0*Power(s,2)*(-l0 + s) + Power(l0,3)*u1 - 3*l0*Power(s,2)*u1 + 2*Power(s,3)*u1 + 3*l0*Power(s,2)*u2 - 2*Power(s,3)*u2)*cos((a1*l0 - a1*s + a2*s)/l0))/Power(l0,4)) + (b1p*s + w1p - (Power(s,2)*(2*b1p*l0 + b2p*l0 + 3*w1p - 3*w2p))/Power(l0,2) + (Power(s,3)*(b1p*l0 + b2p*l0 + 2*w1p - 2*w2p))/Power(l0,3))*cos((a1*l0 - a1*s + a2*s)/l0) - (g1p*s + u1p - (Power(s,2)*(2*g1p*l0 + g2p*l0 + 3*u1p - 3*u2p))/Power(l0,2) + (Power(s,3)*(g1p*l0 + g2p*l0 + 2*u1p - 2*u2p))/Power(l0,3))*sin((a1*l0 - a1*s + a2*s)/l0) - ((a1p*(l0 - s) + a2p*s)*(b1*l0*Power(l0 - s,2)*s + b2*l0*Power(s,2)*(-l0 + s) + Power(l0,3)*w1 - 3*l0*Power(s,2)*w1 + 2*Power(s,3)*w1 + 3*l0*Power(s,2)*w2 - 2*Power(s,3)*w2)*sin((a1*l0 - a1*s + a2*s)/l0))/Power(l0,4);
	X(8) = (g1p*s + u1p - (Power(s,2)*(2*g1p*l0 + g2p*l0 + 3*u1p - 3*u2p))/Power(l0,2) + (Power(s,3)*(g1p*l0 + g2p*l0 + 2*u1p - 2*u2p))/Power(l0,3))*cos((a1*l0 - a1*s + a2*s)/l0) + ((a1p*(l0 - s) + a2p*s)*(b1*l0*Power(l0 - s,2)*s + b2*l0*Power(s,2)*(-l0 + s) + Power(l0,3)*w1 - 3*l0*Power(s,2)*w1 + 2*Power(s,3)*w1 + 3*l0*Power(s,2)*w2 - 2*Power(s,3)*w2)*cos((a1*l0 - a1*s + a2*s)/l0))/Power(l0,4) - ((a1p*(l0 - s) + a2p*s)*(g1*l0*Power(l0 - s,2)*s + g2*l0*Power(s,2)*(-l0 + s) + Power(l0,3)*u1 - 3*l0*Power(s,2)*u1 + 2*Power(s,3)*u1 + 3*l0*Power(s,2)*u2 - 2*Power(s,3)*u2)*sin((a1*l0 - a1*s + a2*s)/l0))/Power(l0,4) + (b1p*s + w1p - (Power(s,2)*(2*b1p*l0 + b2p*l0 + 3*w1p - 3*w2p))/Power(l0,2) + (Power(s,3)*(b1p*l0 + b2p*l0 + 2*w1p - 2*w2p))/Power(l0,3))*sin((a1*l0 - a1*s + a2*s)/l0);
	X(9) = (a1p*l0 - a1p*s + a2p*s)/l0;
	X(10) =  -((g1p - (2*s*(2*g1p*l0 + g2p*l0 + 3*u1p - 3*u2p))/Power(l0,2) + (3*Power(s,2)*(g1p*l0 + g2p*l0 + 2*u1p - 2*u2p))/Power(l0,3))*cos((a1*(l0 - s))/l0 + (a2*s)/l0) - (-b1p + (2*s*(2*b1p*l0 + b2p*l0 + 3*w1p - 3*w2p))/Power(l0,2) - (3*Power(s,2)*(b1p*l0 + b2p*l0 + 2*w1p - 2*w2p))/Power(l0,3))*sin((a1*(l0 - s))/l0 + (a2*s)/l0));
	X(11) =  -((-b1p + (2*s*(2*b1p*l0 + b2p*l0 + 3*w1p - 3*w2p))/Power(l0,2) - (3*Power(s,2)*(b1p*l0 + b2p*l0 + 2*w1p - 2*w2p))/Power(l0,3))*cos((a1*(l0 - s))/l0 + (a2*s)/l0) + (g1p - (2*s*(2*g1p*l0 + g2p*l0 + 3*u1p - 3*u2p))/Power(l0,2) + (3*Power(s,2)*(g1p*l0 + g2p*l0 + 2*u1p - 2*u2p))/Power(l0,3))*sin((a1*(l0 - s))/l0 + (a2*s)/l0));

	return X;
  }

  Mat FiniteElement1s23BTA::JGeneralized(const Vec& q,const double& s)
  {
	Mat J(getSizeOfVelocities(),6-1); // no x-direction

	const double &a1 = q(0);
	const double &w1 = q(1); const double &b1 = q(2);
	const double &u1 = q(3); const double &g1 = q(4);
	const double &a2 = q(5);
	const double &w2 = q(6); const double &b2 = q(7);
	const double &u2 = q(8); const double &g2 = q(9);

	J(0,0) = -(((l0 - s)*(((-l0 + s)*(l0*s*(-(g1*l0) + (g1 + g2)*s) + (-l0 + s)*(l0 + 2*s)*u1) + (3*l0 - 2*s)*Power(s,2)*u2)*cos(a1 + ((-a1 + a2)*s)/l0) + ((-l0 + s)*(l0*s*(-(b1*l0) + (b1 + b2)*s) + (-l0 + s)*(l0 + 2*s)*w1) + (3*l0 - 2*s)*Power(s,2)*w2)*sin(a1 + ((-a1 + a2)*s)/l0)))/Power(l0,4));
	J(0,1) = ((l0 - s)*(((-l0 + s)*(l0*s*(-(b1*l0) + (b1 + b2)*s) + (-l0 + s)*(l0 + 2*s)*w1) + (3*l0 - 2*s)*Power(s,2)*w2)*cos(a1 + ((-a1 + a2)*s)/l0) - ((-l0 + s)*(l0*s*(-(g1*l0) + (g1 + g2)*s) + (-l0 + s)*(l0 + 2*s)*u1) + (3*l0 - 2*s)*Power(s,2)*u2)*sin(a1 + ((-a1 + a2)*s)/l0)))/Power(l0,4);
	J(0,2) = 1 - s/l0;
	J(0,3) = -((l0 - s)*((b1*l0*(l0 - 3*s)*(l0 - s) + s*(b2*l0*(-2*l0 + 3*s) - 6*(l0 - s)*(w1 - w2)))*cos(a1 + ((-a1 + a2)*s)/l0) - (g1*l0*(l0 - 3*s)*(l0 - s) + s*(g2*l0*(-2*l0 + 3*s) - 6*(l0 - s)*(u1 - u2)))*sin(a1 + ((-a1 + a2)*s)/l0)))/Power(l0,4);
	J(0,4) = -((l0 - s)*((g1*l0*(l0 - 3*s)*(l0 - s) + s*(g2*l0*(-2*l0 + 3*s) - 6*(l0 - s)*(u1 - u2)))*cos(a1 + ((-a1 + a2)*s)/l0) + (b1*l0*(l0 - 3*s)*(l0 - s) + s*(b2*l0*(-2*l0 + 3*s) - 6*(l0 - s)*(w1 - w2)))*sin(a1 + ((-a1 + a2)*s)/l0)))/Power(l0,4);
	J(1,0) = (Power(l0 - s,2)*(l0 + 2*s)*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
	J(1,1) = (Power(l0 - s,2)*(l0 + 2*s)*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
	J(1,2) = 0;
	J(1,3) = -(6*s*(-l0 + s)*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
	J(1,4) = -(6*(l0 - s)*s*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
	J(2,0) = (Power(l0 - s,2)*s*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);
	J(2,1) = (Power(l0 - s,2)*s*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);
	J(2,2) = 0;
	J(2,3) = -((l0 - 3*s)*(l0 - s)*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);
	J(2,4) = -(-(((l0 - 3*s)*(l0 - s)*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2)));
	J(3,0) = -((Power(l0 - s,2)*(l0 + 2*s)*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3));
	J(3,1) = (Power(l0 - s,2)*(l0 + 2*s)*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
	J(3,2) = 0;
	J(3,3) = -(6*s*(-l0 + s)*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
	J(3,4) = -(6*s*(-l0 + s)*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
	J(4,0) = -((Power(l0 - s,2)*s*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2));
	J(4,1) = (Power(l0 - s,2)*s*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);
	J(4,2) = 0;
	J(4,3) = -((l0 - 3*s)*(l0 - s)*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);
	J(4,4) = -((l0 - 3*s)*(l0 - s)*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);
	J(5,0) = -((s*(((-l0 + s)*(l0*s*(-(g1*l0) + (g1 + g2)*s) + (-l0 + s)*(l0 + 2*s)*u1) + (3*l0 - 2*s)*Power(s,2)*u2)*cos(a1 + ((-a1 + a2)*s)/l0) + ((-l0 + s)*(l0*s*(-(b1*l0) + (b1 + b2)*s) + (-l0 + s)*(l0 + 2*s)*w1) + (3*l0 - 2*s)*Power(s,2)*w2)*sin(a1 + ((-a1 + a2)*s)/l0)))/Power(l0,4));
	J(5,1) = (s*(((-l0 + s)*(l0*s*(-(b1*l0) + (b1 + b2)*s) + (-l0 + s)*(l0 + 2*s)*w1) + (3*l0 - 2*s)*Power(s,2)*w2)*cos(a1 + ((-a1 + a2)*s)/l0) - ((-l0 + s)*(l0*s*(-(g1*l0) + (g1 + g2)*s) + (-l0 + s)*(l0 + 2*s)*u1) + (3*l0 - 2*s)*Power(s,2)*u2)*sin(a1 + ((-a1 + a2)*s)/l0)))/Power(l0,4);
	J(5,2) = s/l0;
	J(5,3) = -(s*((b1*l0*(l0 - 3*s)*(l0 - s) + s*(b2*l0*(-2*l0 + 3*s) - 6*(l0 - s)*(w1 - w2)))*cos(a1 + ((-a1 + a2)*s)/l0) - (g1*l0*(l0 - 3*s)*(l0 - s) + s*(g2*l0*(-2*l0 + 3*s) - 6*(l0 - s)*(u1 - u2)))*sin(a1 + ((-a1 + a2)*s)/l0)))/Power(l0,4);
	J(5,4) = -(s*((g1*l0*(l0 - 3*s)*(l0 - s) + s*(g2*l0*(-2*l0 + 3*s) - 6*(l0 - s)*(u1 - u2)))*cos(a1 + ((-a1 + a2)*s)/l0) + (b1*l0*(l0 - 3*s)*(l0 - s) + s*(b2*l0*(-2*l0 + 3*s) - 6*(l0 - s)*(w1 - w2)))*sin(a1 + ((-a1 + a2)*s)/l0)))/Power(l0,4);
	J(6,0) = ((3*l0 - 2*s)*Power(s,2)*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
	J(6,1) = ((3*l0 - 2*s)*Power(s,2)*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
	J(6,2) = 0;
	J(6,3) = -(6*(l0 - s)*s*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
	J(6,4) = -(6*s*(-l0 + s)*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
	J(7,0) = (Power(s,2)*(-l0 + s)*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);
	J(7,1) = (Power(s,2)*(-l0 + s)*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);
	J(7,2) = 0;
	J(7,3) = -(s*(-2*l0 + 3*s)*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);
	J(7,4) = -((2*l0 - 3*s)*s*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);
	J(8,0) = (Power(s,2)*(-3*l0 + 2*s)*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
	J(8,1) = ((3*l0 - 2*s)*Power(s,2)*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
	J(8,2) = 0;
	J(8,3) = -(6*(l0 - s)*s*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
	J(8,4) = -(6*(l0 - s)*s*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
	J(9,0) = ((l0 - s)*Power(s,2)*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);
	J(9,1) = (Power(s,2)*(-l0 + s)*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);
	J(9,2) = 0;
	J(9,3) = -(s*(-2*l0 + 3*s)*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);
	J(9,4) = -(s*(-2*l0 + 3*s)*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);

	return J;
  }

  Vec FiniteElement1s23BTA::ElementData(const Vec& qElement, const Vec& qpElement)
  {
	Vec Data(8,INIT,0.);
	// 0:  eps
	// 1:  epsp
	// 2:  xS
	// 3:  yS
	// 4:  xSp
	// 5:  ySp
	// 6:  delta_phi      = bL  + bR
	// 7:  delta_phip     = bLp + bRp

	static Vec qLokal(8,INIT,0.0), qpLokal(8,INIT,0.0);
	static SqrMat Jeg(8,INIT,0.0);

	double &xS = qLokal(0); double &yS = qLokal(1);
	double &eps = qLokal(3);
	double &aL = qLokal(4); double &bL = qLokal(5);
	double &aR = qLokal(6); double &bR = qLokal(7);
	
	double &xSp = qpLokal(0); double &ySp = qpLokal(1);
	double &epsp = qpLokal(3);
	double &aLp = qpLokal(4); double &bLp = qpLokal(5);
	double &aRp = qpLokal(6); double &bRp = qpLokal(7);

	qpLokal << Jeg * qpElement;
	Data(0) = (32*(17*Power(aL,2) - 2*aL*aR + 17*Power(aR,2))*(1 + eps) - 12*(8*aL*bL - 3*aR*bL - 3*aL*bR + 8*aR*bR)*(1 + eps)*l0 + 3*(140*eps + 3*(3*Power(bL,2) - 2*bL*bR + 3*Power(bR,2))*(1 + eps))*l0h2)/(420.*l0h2);
	Data(1) = (64*(17*aL*aLp - aLp*aR - aL*aRp + 17*aR*aRp)*(1 + eps) + 32*(17*Power(aL,2) - 2*aL*aR + 17*Power(aR,2))*epsp - 12*((8*aLp*bL - 3*aRp*bL + 8*aL*bLp - 3*aR*bLp - 3*aLp*bR + 8*aRp*bR - 3*aL*bRp + 8*aR*bRp)*(1 + eps) + (8*aL*bL - 3*aR*bL - 3*aL*bR + 8*aR*bR)*epsp)*l0 + 3*(6*(3*bL*bLp - bLp*bR - bL*bRp + 3*bR*bRp)*(1 + eps) + (140 + 9*Power(bL,2) - 6*bL*bR + 9*Power(bR,2))*epsp)*l0h2)/ (420.*l0h2);
	Data(2) = xS;
	Data(3) = yS;
	Data(4) = xSp;
	Data(5) = ySp;
	Data(6) = bL  + bR;
	Data(7) = bLp + bRp;

	return Data;
  }

}

