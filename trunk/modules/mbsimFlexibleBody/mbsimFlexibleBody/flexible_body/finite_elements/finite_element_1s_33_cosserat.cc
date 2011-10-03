/* Copyright (C) 2004-2011 MBSim Development Team
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
#define FMATVEC_NO_INITIALIZATION
#define FMATVEC_NO_BOUNDS_CHECK
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_33_cosserat.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  FiniteElement1s33Cosserat::FiniteElement1s33Cosserat(double l0_, double rho_,double A_, double E_, double G_, double I1_, double I2_, double I0_, const Vec& g_, int currentElement_, bool openStructure_, const Vec& relaxedElement_) : l0(l0_), rho(rho_), A(A_), E(E_), G(G_), I1(I1_), I2(I2_), I0(I0_), g(g_), currentElement(currentElement_), openStructure(openStructure_), relaxedElement(relaxedElement_), M(6,INIT,0.), h(6,INIT,0.), X(12,INIT,0.) {}

  FiniteElement1s33Cosserat::~FiniteElement1s33Cosserat() {}

  void FiniteElement1s33Cosserat::computeM(const Vec& qG) {
    //const double &be = qG(10); // beta from current element -> should produce Cardan lock
    //const double &ga = qG(11); // gamma from current element
    // TODO openstructure and first element (cf. FlexibleBody::BuildElements)

    M(0,0) = 0.5*rho*A*l0;
    M(1,1) = 0.5*rho*A*l0;
    M(2,2) = 0.5*rho*A*l0;
    M(3,3) = rho*l0*I2;//-rho*l0*(pow(cos(be),2.0)*pow(cos(ga),2.0)*(I2-I1)-I2*pow(cos(be),2.0)+I0*(pow(cos(be),2.0)-1.));
    M(3,4) = 0.;//sin(ga)*rho*l0*cos(be)*cos(ga)*(I1-I2);
    M(3,5) = 0.;//0.1E1*rho*l0*I0*sin(be);
    M(4,4) = rho*l0*I1;//-rho*l0*(pow(cos(ga),2.0)*(I1-I2)-I1);
    M(5,5) = rho*l0*I0;
  }

  void FiniteElement1s33Cosserat::computeh(const Vec& qG, const Vec& qGt) {
    /* DOF's 'previous' element. Index 'p'. */
    const double &xp  = qG(0);
    const double &yp  = qG(1);
    const double &zp  = qG(2);
    const double &alp = qG(3);
    const double &bep = qG(4);
    const double &gap = qG(5);

    /* DOF's element. */
    const double &x   = qG(6);
    const double &y   = qG(7);
    const double &z   = qG(8);
    const double &al  = qG(9);
    const double &be  = qG(10);
    const double &ga  = qG(11);

    /* DOF's 'following' element. Index 'f'. */
    const double &xf  = qG(12);
    const double &yf  = qG(13);
    const double &zf  = qG(14);
    const double &alf = qG(15);
    const double &bef = qG(16);
    const double &gaf = qG(17);

    /* time differentiated DOF's. Additional Index 't'. */
    const double &xpt  = qGt(0);
    const double &ypt  = qGt(1);
    const double &zpt  = qGt(2);
    const double &alpt = qGt(3);
    const double &bept = qGt(4);
    const double &gapt = qGt(5);

    const double &xt   = qGt(6);
    const double &yt   = qGt(7);
    const double &zt   = qGt(8);
    const double &alt  = qGt(9);
    const double &bet  = qGt(10);
    const double &gat  = qGt(11);
    
    const double &xft  = qGt(12);
    const double &yft  = qGt(13);
    const double &zft  = qGt(14);
    const double &alft = qGt(15);
    const double &beft = qGt(16);
    const double &gaft = qGt(17);

    /* DOF's at relaxed stage during initialization. Additional Index '_0'. */
    const double &alp_0 = relaxedElement(3);
    const double &bep_0 = relaxedElement(4);
    const double &gap_0 = relaxedElement(5);

    const double &al_0 = relaxedElement(9);
    const double &be_0 = relaxedElement(10);
    const double &ga_0 = relaxedElement(11);

    const double &alf_0 = relaxedElement(15);
    const double &bef_0 = relaxedElement(16);
    const double &gaf_0 = relaxedElement(17);

    /* Differentiation of 'strain and shearing energy' with respect to qG */
    Vec SE1(6,INIT,0.);
    SE1(0) = 0.1E1*(cos(bep)*cos(gap)*(x-xp)/l0+(cos(alp)*sin(gap)+sin(alp)*sin(bep)*cos(gap))*(y-yp)/l0+(sin(alp)*sin(gap)-cos(alp)*sin(bep)*cos(gap))*(z-zp)/l0)*G*A*cos(bep)*cos(gap)-0.1E1*(-cos(bep)*sin(gap)*(x-xp)/l0+(cos(alp)* cos(gap)-sin(alp)*sin(bep)*sin(gap))*(y-yp)/l0+(sin(alp)*cos(gap)+cos(alp)*sin( bep)*sin(gap))*(z-zp)/l0)*G*A*cos(bep)*sin(gap)+0.1E1*(sin(bep)*(x-xp)/l0-sin(alp)*cos(bep)*(y-yp)/l0+cos(alp)*cos(bep)*(z-zp)/l0-1.0)*E*A*sin(bep);
    SE1(1) = 0.1E1*(cos(bep)*cos(gap)*(x-xp)/l0+(cos(alp)*sin(gap)+sin(alp)*sin(bep)*cos(gap))*(y-yp)/l0+(sin(alp)*sin(gap)-cos(alp)*sin(bep)*cos(gap))*(z-zp)/l0)*G*A*(cos(alp)*sin(gap)+sin(alp)*sin(bep)*cos(gap))+0.1E1*(-cos(bep)* sin(gap)*(x-xp)/l0+(cos(alp)*cos(gap)-sin(alp)*sin(bep)*sin(gap))*(y-yp)/l0+(sin(alp)*cos(gap)+cos(alp)*sin(bep)*sin(gap))*(z-zp)/l0)*G*A*(cos(alp)*cos(gap)-sin(alp)*sin(bep)*sin(gap))-0.1E1*(sin(bep)*(x-xp)/l0-sin(alp)*cos(bep)*(y-yp)/l0+cos(alp)*cos(bep)*(z-zp)/l0-1.0)*E*A*sin(alp)*cos(bep);
    SE1(2) = 0.1E1*(cos(bep)*cos(gap)*(x-xp)/l0+(cos(alp)*sin(gap)+sin(alp)*sin(bep)*cos(gap))*(y-yp)/l0+(sin(alp)*sin(gap)-cos(alp)*sin(bep)*cos(gap))*(z-zp)/l0)*G*A*(sin(alp)*sin(gap)-cos(alp)*sin(bep)*cos(gap))+0.1E1*(-cos(bep)*sin(gap)*(x-xp)/l0+(cos(alp)*cos(gap)-sin(alp)*sin(bep)*sin(gap))*(y-yp)/l0+(sin(alp)*cos(gap)+cos(alp)*sin(bep)*sin(gap))*(z-zp)/l0)*G*A*(sin(alp)*cos(gap)+cos(alp)*sin(bep)*sin(gap))+0.1E1*(sin(bep)*(x-xp)/l0-sin(alp)*cos(bep)*(y-yp)/l0+cos(alp)*cos(bep)*(z-zp)/l0-1.0)*E*A*cos(alp)*cos(bep);

    Vec SE2(6,INIT,0.);
    SE2(0) = -0.1E1*(cos(be)*cos(ga)*(xf-x)/l0+(cos(al)*sin(ga)+sin(al)*sin(be)*cos(ga))*(yf-y)/l0+(sin(al)*sin(ga)-cos(al)*sin(be)*cos(ga))*(zf-z)/l0) *G*A*cos(be)*cos(ga)+0.1E1*(-cos(be)*sin(ga)*(xf-x)/l0+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(yf-y)/l0+(sin(al)*cos(ga)+cos(al)*sin(be)*sin(ga))*(zf-z)/ l0)*G*A*cos(be)*sin(ga)-0.1E1*(sin(be)*(xf-x)/l0-sin(al)*cos(be)*(yf-y)/l0+cos (al)*cos(be)*(zf-z)/l0-1.0)*E*A*sin(be);
    SE2(1) = -0.1E1*(cos(be)*cos(ga)*(xf-x)/l0+(cos(al)*sin(ga)+sin(al)*sin(be)*cos(ga))*(yf-y)/l0+(sin(al)*sin(ga)-cos(al)*sin(be)*cos(ga))*(zf-z)/l0)*G*A*(cos(al)*sin(ga)+sin(al)*sin(be)*cos(ga))-0.1E1*(-cos(be)*sin(ga)*(xf-x)/l0+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(yf-y)/l0+(sin(al)*cos(ga)+cos(al)*sin(be)*sin(ga))*(zf-z)/l0)*G*A*(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))+ 0.1E1*(sin(be)*(xf-x)/l0-sin(al)*cos(be)*(yf-y)/l0+cos(al)*cos(be)*(zf-z)/l0-1.0)*E*A*sin(al)*cos(be);
    SE2(2) = -0.1E1*(cos(be)*cos(ga)*(xf-x)/l0+(cos(al)*sin(ga)+sin(al)* sin(be)*cos(ga))*(yf-y)/l0+(sin(al)*sin(ga)-cos(al)*sin(be)*cos(ga))*(zf-z)/l0)*G*A*(sin(al)*sin(ga)-cos(al)*sin(be)*cos(ga))-0.1E1*(-cos(be)*sin(ga)*(xf-x)/l0+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(yf-y)/l0+(sin(al)*cos(ga)+cos(al)*sin(be)*sin(ga))*(zf-z)/l0)*G*A*(sin(al)*cos(ga)+cos(al)*sin(be)*sin(ga))-0.1E1*(sin(be)*(xf-x)/l0-sin(al)*cos(be)*(yf-y)/l0+cos(al)*cos(be)*(zf-z)/l0-1.0)*E*A*cos(al)*cos(be);
    SE2(3) = 0.1E1*(cos(be)*cos(ga)*(xf-x)/l0+(cos(al)*sin(ga)+sin(al)*sin(be)*cos(ga))*(yf-y)/l0+(sin(al)*sin(ga)-cos(al)*sin(be)*cos(ga))*(zf-z)/l0) *l0*G*A*((-sin(al)*sin(ga)+cos(al)*sin(be)*cos(ga))*(yf-y)/l0+(cos(al)*sin(ga)+sin(al)*sin(be)*cos(ga))*(zf-z)/l0) + 0.1E1*(-cos(be)*sin(ga)*(xf-x)/l0+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(yf-y)/l0+(sin(al)*cos(ga)+cos(al)*sin(be)*sin(ga))*(zf-z)/l0)*l0*G*A*((-sin(al)*cos(ga)-cos(al)*sin(be)*sin(ga))*(yf-y)/l0+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(zf-z)/l0)+0.1E1*(sin(be)*(xf-x)/l0-sin(al)*cos(be)*(yf-y)/l0+cos(al)*cos(be)*(zf-z)/l0-1.0)*l0*E*A*(-cos(al)*cos(be)*(yf-y)/l0-sin(al)*cos(be)*(zf-z)/l0);
    SE2(4) = 0.1E1*(cos(be)*cos(ga)*(xf-x)/l0+(cos(al)*sin(ga)+sin(al)*sin(be)*cos(ga))*(yf-y)/l0+(sin(al)*sin(ga)-cos(al)*sin(be)*cos(ga))*(zf-z)/l0)*l0*G*A*(-sin(be)*cos(ga)*(xf-x)/l0+sin(al)*cos(be)*cos(ga)*(yf-y)/l0-cos(al)*cos(be)*cos(ga)*(zf-z)/l0) + 0.1E1*(-cos(be)*sin(ga)*(xf-x)/l0+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(yf-y)/l0+(sin(al)*cos(ga)+cos(al)*sin(be)*sin(ga))*(zf-z)/l0)*l0*G*A*(sin(be)*sin(ga)*(xf-x)/l0-sin(al)*cos(be)*sin(ga)*(yf-y)/l0+cos(al)*cos(be)*sin(ga)*(zf-z)/l0)+0.1E1*(sin(be)*(xf-x)/l0-sin(al)*cos(be)*(yf-y)/l0+cos(al)*cos(be)*(zf-z)/l0-1.0)*l0*E*A*(cos(be)*(xf-x)/l0+sin(al)*sin(be)*(yf-y)/l0-cos(al)*sin(be)*(zf-z)/l0);
    SE2(5) = 0.1E1*(cos(be)*cos(ga)*(xf-x)/l0+(cos(al)*sin(ga)+sin(al)* sin(be)*cos(ga))*(yf-y)/l0+(sin(al)*sin(ga)-cos(al)*sin(be)*cos(ga))*(zf-z)/l0)*l0*G*A*(-cos(be)*sin(ga)*(xf-x)/l0+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(yf-y)/l0+(sin(al)*cos(ga)+cos(al)*sin(be)*sin(ga))*(zf-z)/l0) + 0.1E1*(-cos(be)*sin(ga)*(xf-x)/l0+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(yf-y)/l0+(sin(al)*cos(ga)+cos(al)*sin(be)*sin(ga))*(zf-z)/l0)*l0*G*A*(-cos(be)*cos(ga)*(xf-x)/l0+(-cos(al)*sin(ga)-sin(al)*sin(be)*cos(ga))*(yf-y)/l0+(-sin(al)*sin(ga)+cos(al)*sin(be)*cos(ga))*(zf-z)/l0);

    /* Differentiation of 'bending and torsion energy' with respect to qG */
    Vec BT1(6,INIT,0.);
    BT1(3) = 0.5*((be-bep)/l0*sin(ga/2.0+gap/2.0)+cos(be/2.0+bep/2.0)*cos( ga/2.0+gap/2.0)*(al-alp)/l0)*E*I1*cos(be/2.0+bep/2.0)*cos(ga/2.0+gap/2.0)-0.5*(cos(ga/2.0+gap/2.0)*(be-bep)/l0-sin(ga/2.0+gap/2.0)*(al-alp)/l0*cos(be/2.0+bep/2.0))*E*I2*sin(ga/2.0+gap/2.0)*cos(be/2.0+bep/2.0); 
    BT1(4) = 0.5*((be-bep)/l0*sin(ga/2.0+gap/2.0)+cos(be/2.0+bep/2.0)*cos(ga/2.0+gap/2.0)*(al-alp)/l0)*l0*E*I1*(1/l0*sin(ga/2.0+gap/2.0)-sin(be/2.0+bep/2.0)*cos(ga/2.0+gap/2.0)*(al-alp)/l0/2.0)+0.5*(cos(ga/2.0+gap/2.0)*(be-bep)/l0-sin(ga/2.0+gap/2.0)*(al-alp)/l0*cos(be/2.0+bep/2.0))*l0*E*I2*(cos(ga/2.0+gap/2.0)/l0+sin(ga/2.0+gap/2.0)*(al-alp)/l0*sin(be/2.0+bep/2.0)/2.0);
    BT1(5) = 0.5*((be-bep)/l0*sin(ga/2.0+gap/2.0)+cos(be/2.0+bep/2.0)*cos(ga/2.0+gap/2.0)*(al-alp)/l0)*l0*E*I1*(cos(ga/2.0+gap/2.0)*(be-bep)/l0/2.0-sin( ga/2.0+gap/2.0)*(al-alp)/l0*cos(be/2.0+bep/2.0)/2.0)+0.5*(cos(ga/2.0+gap/2.0)*(be-bep)/l0-sin(ga/2.0+gap/2.0)*(al-alp)/l0*cos(be/2.0+bep/2.0))*l0*E*I2*(-(be-bep)/l0*sin(ga/2.0+gap/2.0)/2.0-cos(be/2.0+bep/2.0)*cos(ga/2.0+gap/2.0)*(al-alp)/l0/2.0);

    Vec BT1_0(6,INIT,0.);
    BT1_0(3) = 0.5*((be_0-bep_0)/l0*sin(ga_0/2.0+gap_0/2.0)+cos(be_0/2.0+bep_0/2.0)*cos( ga_0/2.0+gap_0/2.0)*(al_0-alp_0)/l0)*E*I1*cos(be_0/2.0+bep_0/2.0)*cos(ga_0/2.0+gap_0/2.0)-0.5*(cos(ga_0/2.0+gap_0/2.0)*(be_0-bep_0)/l0-sin(ga_0/2.0+gap_0/2.0)*(al_0-alp_0)/l0*cos(be_0/2.0+bep_0/2.0))*E*I2*sin(ga_0/2.0+gap_0/2.0)*cos(be_0/2.0+bep_0/2.0);
    BT1_0(4) = 0.5*((be_0-bep_0)/l0*sin(ga_0/2.0+gap_0/2.0)+ cos(be_0/2.0+bep_0/2.0)*cos(ga_0/2.0+gap_0/2.0)*(al_0-alp_0)/l0)*l0*E*I1*(1/l0*sin(ga_0/2.0+gap_0/2.0)-sin(be_0/2.0+bep_0/2.0)*cos(ga_0/2.0+gap_0/2.0)*(al_0-alp_0)/l0/2.0)+0.5*(cos(ga_0/2.0+gap_0/2.0)*(be_0-bep_0)/l0-sin(ga_0/2.0+gap_0/2.0)*(al_0-alp_0)/l0*cos(be_0/2.0+bep_0/2.0))*l0*E*I2*(cos(ga_0/2.0+gap_0/2.0)/l0+ sin(ga_0/2.0+gap_0/2.0)*(al_0-alp_0)/l0*sin(be_0/2.0+bep_0/2.0)/2.0);
    BT1_0(5) = 0.5*((be_0-bep_0)/l0*sin(ga_0/2.0+gap_0/2.0)+ cos(be_0/2.0+bep_0/2.0)*cos(ga_0/2.0+gap_0/2.0)*(al_0-alp_0)/l0)*l0*E*I1*(cos(ga_0/2.0+gap_0/2.0)*(be_0-bep_0)/l0/2.0-sin(ga_0/2.0+gap_0/2.0)*(al_0-alp_0)/l0*cos(be_0/2.0+bep_0/2.0)/2.0)+ 0.5*(cos(ga_0/2.0+gap_0/2.0)*(be_0-bep_0)/l0-sin(ga_0/2.0+gap_0/2.0)*(al_0-alp_0)/l0*cos(be_0/2.0+ bep_0/2.0))*l0*E*I2*(-(be_0-bep_0)/l0*sin(ga_0/2.0+gap_0/2.0)/2.0-cos(be_0/2.0+bep_0/2.0)*cos(ga_0/2.0+gap_0/2.0)*(al_0-alp_0)/l0/2.0);

    Vec BT2(6,INIT,0.);
    BT2(3) = -0.5*((bef-be)/l0*sin(gaf/2.0+ga/2.0)+cos(bef/2.0+be/2.0)*cos(gaf/2.0+ga/2.0)*(alf-al)/l0)*E*I1*cos(bef/2.0+be/2.0)*cos(gaf/2.0+ga/2.0)+0.5*(cos(gaf/2.0+ga/2.0)*(bef-be)/l0-sin(gaf/2.0+ga/2.0)*(alf-al)/l0*cos(bef/2.0+be/2.0))*E*I2*sin(gaf/2.0+ga/2.0)*cos(bef/2.0+be/2.0)-0.5*((alf-al)/l0*sin(bef/2.0+be/2.0)+(gaf-ga)/l0)*G*I0*sin(bef/2.0+be/2.0);
    BT2(4) = 0.5*((bef-be)/l0*sin(gaf/2.0+ga/2.0)+cos(bef/2.0+be/2.0)*cos( gaf/2.0+ga/2.0)*(alf-al)/l0)*l0*E*I1*(-1/l0*sin(gaf/2.0+ga/2.0)-sin(bef/2.0+be/2.0)*cos(gaf/2.0+ga/2.0)*(alf-al)/l0/2.0)+0.5*(cos(gaf/2.0+ga/2.0)*(bef-be)/l0-sin(gaf/2.0+ga/2.0)*(alf-al)/l0*cos(bef/2.0+be/2.0))*l0*E*I2*(-cos(gaf/2.0+ga/2.0)/l0+sin(gaf/2.0+ga/2.0)*(alf-al)/l0*sin(bef/2.0+be/2.0)/2.0)+0.25*((alf-al)/l0*sin(bef/2.0+be/2.0)+(gaf-ga)/l0)*G*I0*(alf-al)*cos(bef/2.0+be/2.0);
    BT2(5) = 0.5*((bef-be)/l0*sin(gaf/2.0+ga/2.0)+cos(bef/2.0+be/2.0)*cos( gaf/2.0+ga/2.0)*(alf-al)/l0)*l0*E*I1*(cos(gaf/2.0+ga/2.0)*(bef-be)/l0/2.0-sin( gaf/2.0+ga/2.0)*(alf-al)/l0*cos(bef/2.0+be/2.0)/2.0)+0.5*(cos(gaf/2.0+ga/2.0)*(bef-be)/l0-sin(gaf/2.0+ga/2.0)*(alf-al)/l0*cos(bef/2.0+be/2.0))*l0*E*I2*(-(bef-be)/l0*sin(gaf/2.0+ga/2.0)/2.0-cos(bef/2.0+be/2.0)*cos(gaf/2.0+ga/2.0)*(alf-al)/l0/2.0)-0.5*((alf-al)/l0*sin(bef/2.0+be/2.0)+(gaf-ga)/l0)*G*I0;

    Vec BT2_0(6,INIT,0.);
    BT2_0(3) = -0.5*((bef_0-be_0)/l0*sin(gaf_0/2.0+ga_0/2.0)+cos(bef_0/2.0+be_0/2.0)*cos(gaf_0/2.0+ga_0/2.0)*(alf_0-al_0)/l0)*E*I1*cos(bef_0/2.0+be_0/2.0)*cos(gaf_0/2.0+ga_0/2.0)+0.5*(cos(gaf_0/2.0+ga_0/2.0)*(bef_0-be_0)/l0-sin(gaf_0/2.0+ga_0/2.0)*(alf_0-al_0)/l0*cos(bef_0/2.0+be_0/2.0))*E*I2*sin(gaf_0/2.0+ga_0/2.0)*cos(bef_0/2.0+be_0/2.0)-0.5*((alf_0-al_0)/l0*sin(bef_0/ 2.0+be_0/2.0)+(gaf_0-ga_0)/l0)*G*I0*sin(bef_0/2.0+be_0/2.0);
    BT2_0(4) = 0.5*((bef_0-be_0)/l0*sin(gaf_0/2.0+ga_0/2.0)+cos(bef_0/2.0+be_0/2.0)*cos(gaf_0/2.0+ga_0/2.0)*(alf_0-al_0)/l0)*l0*E*I1*(-1/l0*sin(gaf_0/2.0+ga_0/2.0)-sin(bef_0/2.0+be_0/ 2.0)*cos(gaf_0/2.0+ga_0/2.0)*(alf_0-al_0)/l0/2.0)+0.5*(cos(gaf_0/2.0+ga_0/2.0)*(bef_0-be_0)/l0-sin(gaf_0/2.0+ga_0/2.0)*(alf_0-al_0)/l0*cos(bef_0/2.0+be_0/2.0))*l0*E*I2*(-cos(gaf_0/2.0+ga_0/2.0)/l0+sin(gaf_0/2.0+ga_0/2.0)*(alf_0-al_0)/l0*sin(bef_0/2.0+be_0/2.0)/2.0)+0.25*((alf_0-al_0)/l0*sin(bef_0/2.0+be_0/2.0)+(gaf_0-ga_0)/l0)*G*I0*(alf_0-al_0)*cos(bef_0/2.0+be_0/2.0);
    BT2_0(5) = 0.5*((bef_0-be_0)/l0*sin(gaf_0/2.0+ga_0/2.0)+cos(bef_0/2.0+be_0/2.0)*cos(gaf_0/2.0+ga_0/2.0)*(alf_0-al_0)/l0)*l0*E*I1*(cos(gaf_0/2.0+ga_0/2.0)*(bef_0-be_0)/l0/2.0-sin(gaf_0/2.0+ga_0/2.0)*(alf_0-al_0)/l0*cos(bef_0/2.0+be_0/2.0)/2.0)+0.5*(cos(gaf_0/2.0+ga_0/2.0)*(bef_0-be_0)/l0-sin(gaf_0/2.0+ga_0/2.0)*(alf_0-al_0)/l0*cos(bef_0/2.0+be_0/2.0))*l0*E*I2*(-(bef_0- be_0)/l0*sin(gaf_0/2.0+ga_0/2.0)/2.0-cos(bef_0/2.0+be/2.0)*cos(gaf_0/2.0+ga_0/2.0)*(alf_0-al_0) /l0/2.0)-0.5*((alf_0-al)/l0*sin(bef_0/2.0+be_0/2.0)+(gaf_0-ga_0)/l0)*G*I0;

    Vec V = BT1 + BT2 - BT1_0 - BT2_0;// + SE1 + SE2;

    /* Differentiation of 'dissipative strain and shearing energy' with respect to qGt */
    Vec dissSE1(6,INIT,0.);
    dissSE1(0) = 0.1E1*(-sin(bep)*bept*cos(gap)*(x-xp)/l0-cos(bep)*sin(gap)*gapt*(x-xp)/l0+cos(bep)*cos(gap)*(xt-xpt)/l0+(-sin(alp)*alpt*sin(gap)+cos(alp)*cos(gap)*gapt+cos(alp)*alpt*sin(bep)*cos(gap)+sin(alp)*cos(bep)*bept*cos(gap)-sin(alp)*sin(bep)*sin(gap)*gapt)*(y-yp)/l0+(cos(alp)*sin(gap)+sin(alp)*sin(bep)*cos(gap))*(yt-ypt)/l0+(cos(alp)*alpt*sin(gap)+sin(alp)*cos(gap)*gapt+sin(alp)*alpt*sin(bep)*cos(gap)-cos(alp)*cos(bep)*bept*cos(gap)+cos(alp)*sin(bep)*sin(gap)*gapt)*(z-zp)/l0+(sin(alp)*sin(gap)-cos(alp)*sin(bep)*cos(gap))*(zt-zpt)/l0)*G*A*cos(bep)*cos(gap) - 0.1E1*(sin(bep)*bept*sin(gap)*(x-xp)/l0-cos(bep)*cos(gap)*gapt*(x-xp)/l0-cos(bep)*sin(gap)*(xt-xpt)/l0+(-sin(alp)*alpt*cos(gap)-cos(alp)*sin(gap)*gapt-cos(alp)*alpt*sin(bep)*sin(gap)-sin(alp)*cos(bep)*bept*sin(gap)- sin(alp)*sin(bep)*cos(gap)*gapt)*(y-yp)/l0+(cos(alp)*cos(gap)-sin(alp)*sin(bep)*sin(gap))*(yt-ypt)/l0+(cos(alp)*alpt*cos(gap)-sin(alp)*sin(gap)*gapt-sin(alp)*alpt*sin(bep)*sin(gap)+cos(alp)*cos(bep)*bept*sin(gap)+cos(alp)*sin(bep)*cos(gap)*gapt)*(z-zp)/l0+(sin(alp)*cos(gap)+cos(alp)*sin(bep)*sin(gap))*(zt-zpt)/l0)*G*A*cos(bep)*sin(gap) + 0.1E1*(cos(bept)*bept*(xt-xpt)/l0+sin(bept)*(xt-xpt)/l0-cos(alpt)*alpt*cos(bept)*(yt-ypt)/l0+sin(alpt)*sin(bept)*bept*(yt-ypt)/l0-sin( alpt)*cos(bept)*(yt-ypt)/l0-sin(alpt)*alpt*cos(bept)*(zt-zpt)/l0-cos(alpt)*sin( bept)*bept*(zt-zpt)/l0+cos(alpt)*cos(bept)*(zt-zpt)/l0)*l0*E*A*(cos(bept)*bept/ l0+sin(bept)/l0);
    dissSE1(1) = (-0.1E1*sin(bep)*bept*cos(gap)*(x-xp)/l0-0.1E1*cos(bep)*sin (gap)*gapt*(x-xp)/l0+0.1E1*cos(bep)*cos(gap)*(xt-xpt)/l0+0.1E1*(-sin(alp)*alpt* sin(gap)+cos(alp)*cos(gap)*gapt+cos(alp)*alpt*sin(bep)*cos(gap)+sin(alp)*cos(bep)*bept*cos(gap)-sin(alp)*sin(bep)*sin(gap)*gapt)*(y-yp)/l0+0.1E1*(cos(alp)*sin(gap)+sin(alp)*sin(bep)*cos(gap))*(yt-ypt)/l0+0.1E1*(cos(alp)*alpt*sin(gap)+ sin(alp)*cos(gap)*gapt+sin(alp)*alpt*sin(bep)*cos(gap)-cos(alp)*cos(bep)*bept* cos(gap)+cos(alp)*sin(bep)*sin(gap)*gapt)*(z-zp)/l0+0.1E1*(sin(alp)*sin(gap)-cos(alp)*sin(bep)*cos(gap))*(zt-zpt)/l0)*G*A*(cos(alp)*sin(gap)+sin(alp)*sin(bep)*cos(gap)) + (0.1E1*sin(bep)*bept*sin(gap)*(x-xp)/l0-0.1E1*cos(bep)*cos(gap)*gapt*(x-xp)/l0-0.1E1*cos(bep)*sin(gap)*(xt-xpt)/l0+0.1E1*(-sin(alp)*alpt* cos(gap)-cos(alp)*sin(gap)*gapt-cos(alp)*alpt*sin(bep)*sin(gap)-sin(alp)*cos( bep)*bept*sin(gap)-sin(alp)*sin(bep)*cos(gap)*gapt)*(y-yp)/l0+0.1E1*(cos(alp)* cos(gap)-sin(alp)*sin(bep)*sin(gap))*(yt-ypt)/l0+0.1E1*(cos(alp)*alpt*cos(gap)- sin(alp)*sin(gap)*gapt-sin(alp)*alpt*sin(bep)*sin(gap)+cos(alp)*cos(bep)*bept* sin(gap)+cos(alp)*sin(bep)*cos(gap)*gapt)*(z-zp)/l0+0.1E1*(sin(alp)*cos(gap)+cos(alp)*sin(bep)*sin(gap))*(zt-zpt)/l0)*G*A*(cos(alp)*cos(gap)-sin(alp)*sin(bep)*sin(gap)) + 0.1E1*(cos(bept)*bept*(xt-xpt)/l0+sin(bept)*(xt-xpt)/l0-cos(alpt)*alpt*cos(bept)*(yt-ypt)/l0+sin(alpt)*sin(bept)*bept*(yt-ypt)/l0-sin( alpt)*cos(bept)*(yt-ypt)/l0-sin(alpt)*alpt*cos(bept)*(zt-zpt)/l0-cos(alpt)*sin( bept)*bept*(zt-zpt)/l0+cos(alpt)*cos(bept)*(zt-zpt)/l0)*l0*E*A*(-cos(alpt)*alpt *cos(bept)/l0+sin(alpt)*sin(bept)*bept/l0-sin(alpt)*cos(bept)/l0);
    dissSE1(2) = (-0.1E1*sin(bep)*bept*cos(gap)*(x-xp)/l0-0.1E1*cos(bep)*sin (gap)*gapt*(x-xp)/l0+0.1E1*cos(bep)*cos(gap)*(xt-xpt)/l0+0.1E1*(-sin(alp)*alpt* sin(gap)+cos(alp)*cos(gap)*gapt+cos(alp)*alpt*sin(bep)*cos(gap)+sin(alp)*cos( bep)*bept*cos(gap)-sin(alp)*sin(bep)*sin(gap)*gapt)*(y-yp)/l0+0.1E1*(cos(alp)* sin(gap)+sin(alp)*sin(bep)*cos(gap))*(yt-ypt)/l0+0.1E1*(cos(alp)*alpt*sin(gap)+ sin(alp)*cos(gap)*gapt+sin(alp)*alpt*sin(bep)*cos(gap)-cos(alp)*cos(bep)*bept* cos(gap)+cos(alp)*sin(bep)*sin(gap)*gapt)*(z-zp)/l0+0.1E1*(sin(alp)*sin(gap)-cos(alp)*sin(bep)*cos(gap))*(zt-zpt)/l0)*G*A*(sin(alp)*sin(gap)-cos(alp)*sin(bep)*cos(gap)) + (0.1E1*sin(bep)*bept*sin(gap)*(x-xp)/l0-0.1E1*cos(bep)*cos( gap)*gapt*(x-xp)/l0-0.1E1*cos(bep)*sin(gap)*(xt-xpt)/l0+0.1E1*(-sin(alp)*alpt*cos(gap)-cos(alp)*sin(gap)*gapt-cos(alp)*alpt*sin(bep)*sin(gap)-sin(alp)*cos( bep)*bept*sin(gap)-sin(alp)*sin(bep)*cos(gap)*gapt)*(y-yp)/l0+0.1E1*(cos(alp)* cos(gap)-sin(alp)*sin(bep)*sin(gap))*(yt-ypt)/l0+0.1E1*(cos(alp)*alpt*cos(gap)-sin(alp)*sin(gap)*gapt-sin(alp)*alpt*sin(bep)*sin(gap)+cos(alp)*cos(bep)*bept* sin(gap)+cos(alp)*sin(bep)*cos(gap)*gapt)*(z-zp)/l0+0.1E1*(sin(alp)*cos(gap)+ cos(alp)*sin(bep)*sin(gap))*(zt-zpt)/l0)*G*A*(sin(alp)*cos(gap)+cos(alp)*sin(bep)*sin(gap)) + 0.1E1*(cos(bept)*bept*(xt-xpt)/l0+sin(bept)*(xt-xpt)/l0-cos(alpt)*alpt*cos(bept)*(yt-ypt)/l0+sin(alpt)*sin(bept)*bept*(yt-ypt)/l0-sin( alpt)*cos(bept)*(yt-ypt)/l0-sin(alpt)*alpt*cos(bept)*(zt-zpt)/l0-cos(alpt)*sin( bept)*bept*(zt-zpt)/l0+cos(alpt)*cos(bept)*(zt-zpt)/l0)*l0*E*A*(-sin(alpt)*alpt *cos(bept)/l0-cos(alpt)*sin(bept)*bept/l0+cos(alpt)*cos(bept)/l0);

    Vec dissSE2(6,INIT,0.);
    dissSE2(0) = -0.1E1*(-sin(be)*bet*cos(ga)*(xf-x)/l0-cos(be)*sin(ga)*gat*(xf-x)/l0+cos(be)*cos(ga)*(xft-xt)/l0+(-sin(al)*alt*sin(ga)+cos(al)*cos(ga)* gat+cos(al)*alt*sin(be)*cos(ga)+sin(al)*cos(be)*bet*cos(ga)-sin(al)*sin(be)*sin (ga)*gat)*(yf-y)/l0+(cos(al)*sin(ga)+sin(al)*sin(be)*cos(ga))*(yft-yt)/l0+(cos(al)*alt*sin(ga)+sin(al)*cos(ga)*gat+sin(al)*alt*sin(be)*cos(ga)-cos(al)*cos(be) *bet*cos(ga)+cos(al)*sin(be)*sin(ga)*gat)*(zf-z)/l0+(sin(al)*sin(ga)-cos(al)* sin(be)*cos(ga))*(zft-zt)/l0)*G*A*cos(be)*cos(ga) + 0.1E1*(sin(be)*bet*sin(ga)*(xf-x)/l0-cos(be)*cos(ga)*gat*(xf-x)/l0-cos(be)*sin(ga)*(xft-xt)/l0+(-sin(al)*alt*cos(ga)-cos(al)*sin(ga)*gat- cos(al)*alt*sin(be)*sin(ga)-sin(al)*cos(be)*bet*sin(ga)-sin(al)*sin(be)*cos(ga) *gat)*(yf-y)/l0+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(yft-yt)/l0+(cos(al)*alt*cos(ga)-sin(al)*sin(ga)*gat-sin(al)*alt*sin(be)*sin(ga)+cos(al)*cos(be)*bet*sin(ga)+cos(al)*sin(be)*cos(ga)*gat)*(zf-z)/l0+(sin(al)*cos(ga)+cos(al)*sin(be)*sin(ga))*(zft-zt)/l0)*G*A*cos(be)*sin(ga) - 0.1E1*(cos(be)*bet*(xf-x)/l0+sin(be)*(xft-xt)/l0-cos(al)*alt*cos(be)*(yf-y)/l0+sin(al)*sin(be)*bet*(yft-y)/l0-sin(al)*cos(be)*(yft-y)/l0 -sin(al)*alt*cos(be)*(zft-zt)/l0-cos(al)*sin(be)*bet*(zft-zt)/l0+cos(al)*cos(be)*(zft-zt)/l0)*E*A*sin(be);
    dissSE2(1) = -0.1E1*(-sin(be)*bet*cos(ga)*(xf-x)/l0-cos(be)*sin(ga)*gat *(xf-x)/l0+cos(be)*cos(ga)*(xft-xt)/l0+(-sin(al)*alt*sin(ga)+cos(al)*cos(ga)* gat+cos(al)*alt*sin(be)*cos(ga)+sin(al)*cos(be)*bet*cos(ga)-sin(al)*sin(be)*sin(ga)*gat)*(yf-y)/l0+(cos(al)*sin(ga)+sin(al)*sin(be)*cos(ga))*(yft-yt)/l0+(cos( al)*alt*sin(ga)+sin(al)*cos(ga)*gat+sin(al)*alt*sin(be)*cos(ga)-cos(al)*cos(be)*bet*cos(ga)+cos(al)*sin(be)*sin(ga)*gat)*(zf-z)/l0+(sin(al)*sin(ga)-cos(al)*sin(be)*cos(ga))*(zft-zt)/l0)*G*A*(cos(al)*sin(ga)+sin(al)*sin(be)*cos(ga)) - 0.1E1*(sin(be)*bet*sin(ga)*(xf-x)/l0-cos(be)*cos(ga)*gat*(xf-x)/l0-cos(be)*sin(ga)*(xft-xt)/l0+(-sin(al)*alt*cos(ga)-cos(al)*sin(ga)*gat-cos(al)*alt*sin(be)*sin(ga)-sin(al)*cos(be)*bet*sin(ga)-sin(al)*sin(be)*cos(ga)*gat)*(yf-y)/l0+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(yft-yt)/l0+(cos(al) *alt*cos(ga)-sin(al)*sin(ga)*gat-sin(al)*alt*sin(be)*sin(ga)+cos(al)*cos(be)*bet*sin(ga)+cos(al)*sin(be)*cos(ga)*gat)*(zf-z)/l0+(sin(al)*cos(ga)+cos(al)*sin (be)*sin(ga))*(zft-zt)/l0)*G*A*(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga));
    dissSE2(2) = -0.1E1*(-sin(be)*bet*cos(ga)*(xf-x)/l0-cos(be)*sin(ga)*gat *(xf-x)/l0+cos(be)*cos(ga)*(xft-xt)/l0+(-sin(al)*alt*sin(ga)+cos(al)*cos(ga)* gat+cos(al)*alt*sin(be)*cos(ga)+sin(al)*cos(be)*bet*cos(ga)-sin(al)*sin(be)*sin(ga)*gat)*(yf-y)/l0+(cos(al)*sin(ga)+sin(al)*sin(be)*cos(ga))*(yft-yt)/l0+(cos( al)*alt*sin(ga)+sin(al)*cos(ga)*gat+sin(al)*alt*sin(be)*cos(ga)-cos(al)*cos(be)*bet*cos(ga)+cos(al)*sin(be)*sin(ga)*gat)*(zf-z)/l0+(sin(al)*sin(ga)-cos(al)* sin(be)*cos(ga))*(zft-zt)/l0)*G*A*(sin(al)*sin(ga)-cos(al)*sin(be)*cos(ga)) - 0.1E1*(sin(be)*bet*sin(ga)*(xf-x)/l0-cos(be)*cos(ga)*gat*(xf-x)/l0-cos(be)*sin(ga)*(xft-xt)/l0+(-sin(al)*alt*cos(ga)-cos(al)*sin(ga)*gat-cos(al)*alt*sin(be)*sin(ga)-sin(al)*cos(be)*bet*sin(ga)-sin(al)*sin(be)*cos(ga)*gat)*(yf-y)/l0+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(yft-yt)/l0+(cos(al)*alt*cos(ga)-sin(al)*sin(ga)*gat-sin(al)*alt*sin(be)*sin(ga)+cos(al)*cos(be)*bet*sin(ga)+cos(al)*sin(be)*cos(ga)*gat)*(zf-z)/l0+(sin(al)*cos(ga)+cos(al)*sin(be)*sin(ga))*(zft-zt)/l0)*G*A*(sin(al)*cos(ga)+cos(al)*sin(be)*sin(ga)) + 0.1E1*(cos(be)*bet*(xf-x)/l0+sin(be)*(xft-xt)/l0-cos(al)*alt*cos(be)*(yf-y)/l0+sin(al)*sin(be)*bet*(yft-y)/l0-sin(al)*cos(be)*(yft-y)/l0-sin(al)*alt*cos(be)*(zft-zt)/l0-cos(al)*sin(be)*bet*(zft-zt)/l0+cos(al)*cos(be)*(zft-zt)/l0)*l0*E*A*(sin(al)*alt*cos(be)/l0+cos(al)*sin(be)*bet/l0-cos(al)* cos(be)/l0);
    dissSE2(3) = 0.1E1*l0*(-sin(be)*bet*cos(ga)*(xf-x)/l0-cos(be)*sin(ga)* gat*(xf-x)/l0+cos(be)*cos(ga)*(xft-xt)/l0+(-sin(al)*alt*sin(ga)+cos(al)*cos(ga)*gat+cos(al)*alt*sin(be)*cos(ga)+sin(al)*cos(be)*bet*cos(ga)-sin(al)*sin(be)*sin(ga)*gat)*(yf-y)/l0+(cos(al)*sin(ga)+sin(al)*sin(be)*cos(ga))*(yft-yt)/l0+(cos(al)*alt*sin(ga)+sin(al)*cos(ga)*gat+sin(al)*alt*sin(be)*cos(ga)-cos(al)*cos(be)*bet*cos(ga)+cos(al)*sin(be)*sin(ga)*gat)*(zf-z)/l0+(sin(al)*sin(ga)-cos(al)*sin(be)*cos(ga))*(zft-zt)/l0)*G*A*((-sin(al)*sin(ga)+cos(al)*sin(be)*cos(ga))*(yf-y)/l0 +(cos(al)*sin(ga)+sin(al)*sin(be)*cos(ga))*(zf-z)/l0) + 0.1E1*l0*(sin(be)*bet*sin(ga)*(xf-x)/l0-cos(be)*cos(ga)* gat*(xf-x)/l0-cos(be)*sin(ga)*(xft-xt)/l0+(-sin(al)*alt*cos(ga)-cos(al)*sin(ga) *gat-cos(al)*alt*sin(be)*sin(ga)-sin(al)*cos(be)*bet*sin(ga)-sin(al)*sin(be)*cos(ga)*gat)*(yf-y)/l0+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(yft-yt)/l0+( cos(al)*alt*cos(ga)-sin(al)*sin(ga)*gat-sin(al)*alt*sin(be)*sin(ga)+cos(al)*cos(be)*bet*sin(ga)+cos(al)*sin(be)*cos(ga)*gat)*(zf-z)/l0+(sin(al)*cos(ga)+cos(al)*sin(be)*sin(ga))*(zft-zt)/l0)*G*A*((-sin(al)*cos(ga)-cos(al)*sin(be)*sin(ga))*(yf-y)/l0+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(zf-z)/l0) + 0.1E1*(cos(be)*bet*(xf-x)/l0+sin(be)*(xft-xt)/l0-cos(al)*alt*cos(be)*(yf-y)/l0+sin(al)*sin(be)*bet*(yft-y)/l0-sin(al)*cos(be)*(yft-y)/l0 -sin(al)*alt*cos(be)*(zft-zt)/l0-cos(al)*sin(be)*bet*(zft-zt)/l0+cos(al)*cos(be)*(zft-zt)/l0)*l0*E*A*(-cos(al)*cos(be)*(yf-y)/l0-sin(al)*cos(be)*(zft-zt)/l0);
    dissSE2(4) = 0.1E1*l0*(-sin(be)*bet*cos(ga)*(xf-x)/l0-cos(be)*sin(ga)* gat*(xf-x)/l0+cos(be)*cos(ga)*(xft-xt)/l0+(-sin(al)*alt*sin(ga)+cos(al)*cos(ga) *gat+cos(al)*alt*sin(be)*cos(ga)+sin(al)*cos(be)*bet*cos(ga)-sin(al)*sin(be)* sin(ga)*gat)*(yf-y)/l0+(cos(al)*sin(ga)+sin(al)*sin(be)*cos(ga))*(yft-yt)/l0+( cos(al)*alt*sin(ga)+sin(al)*cos(ga)*gat+sin(al)*alt*sin(be)*cos(ga)-cos(al)*cos(be)*bet*cos(ga)+cos(al)*sin(be)*sin(ga)*gat)*(zf-z)/l0+(sin(al)*sin(ga)-cos(al)*sin(be)*cos(ga))*(zft-zt)/l0)*G*A*(-sin(be)*cos(ga)*(xf-x)/l0+sin(al)*cos(be)*cos(ga)*(yf-y)/l0-cos(al)*cos(be)*cos(ga)*(zf-z)/l0) + 0.1E1*l0*(sin(be)*bet*sin(ga)*(xf-x)/l0-cos(be)*cos(ga)* gat*(xf-x)/l0-cos(be)*sin(ga)*(xft-xt)/l0+(-sin(al)*alt*cos(ga)-cos(al)*sin(ga) *gat-cos(al)*alt*sin(be)*sin(ga)-sin(al)*cos(be)*bet*sin(ga)-sin(al)*sin(be)*cos(ga)*gat)*(yf-y)/l0+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(yft-yt)/l0+(cos(al)*alt*cos(ga)-sin(al)*sin(ga)*gat-sin(al)*alt*sin(be)*sin(ga)+cos(al)*cos(be)*bet*sin(ga)+cos(al)*sin(be)*cos(ga)*gat)*(zf-z)/l0+(sin(al)*cos(ga)+cos(al)*sin(be)*sin(ga))*(zft-zt)/l0)*G*A*(sin(be)*sin(ga)*(xf-x)/l0-sin(al)*cos(be)*sin(ga)*(yf-y)/l0+cos(al)*cos(be)*sin(ga)*(zf-z)/l0) + 0.1E1*(cos(be)*bet*(xf-x)/l0+sin(be)*(xft-xt)/l0-cos(al)* alt*cos(be)*(yf-y)/l0+sin(al)*sin(be)*bet*(yft-y)/l0-sin(al)*cos(be)*(yft-y)/l0-sin(al)*alt*cos(be)*(zft-zt)/l0-cos(al)*sin(be)*bet*(zft-zt)/l0+cos(al)*cos(be)*(zft-zt)/l0)*l0*E*A*(cos(be)*(xf-x)/l0+sin(al)*sin(be)*(yft-y)/l0-cos(al)*sin(be)*(zft-zt)/l0);
    dissSE2(5) = 0.1E1*l0*(-sin(be)*bet*cos(ga)*(xf-x)/l0-cos(be)*sin(ga)* gat*(xf-x)/l0+cos(be)*cos(ga)*(xft-xt)/l0+(-sin(al)*alt*sin(ga)+cos(al)*cos(ga) *gat+cos(al)*alt*sin(be)*cos(ga)+sin(al)*cos(be)*bet*cos(ga)-sin(al)*sin(be)* sin(ga)*gat)*(yf-y)/l0+(cos(al)*sin(ga)+sin(al)*sin(be)*cos(ga))*(yft-yt)/l0+( cos(al)*alt*sin(ga)+sin(al)*cos(ga)*gat+sin(al)*alt*sin(be)*cos(ga)-cos(al)*cos (be)*bet*cos(ga)+cos(al)*sin(be)*sin(ga)*gat)*(zf-z)/l0+(sin(al)*sin(ga)-cos(al)*sin(be)*cos(ga))*(zft-zt)/l0)*G*A*(-cos(be)*sin(ga)*(xf-x)/l0+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(yf-y)/l0+(sin(al)*cos(ga)+cos(al)*sin(be)*sin(ga))*(zf-z)/l0) + 0.1E1*l0*(sin(be)*bet*sin(ga)*(xf-x)/l0-cos(be)*cos(ga)*gat*(xf-x)/l0-cos(be)*sin(ga)*(xft-xt)/l0+(-sin(al)*alt*cos(ga)-cos(al)*sin(ga)*gat-cos(al)*alt*sin(be)*sin(ga)-sin(al)*cos(be)*bet*sin(ga)-sin(al)*sin(be)*cos(ga)*gat)*(yf-y)/l0+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(yft-yt)/l0+(cos(al)*alt*cos(ga)-sin(al)*sin(ga)*gat-sin(al)*alt*sin(be)*sin(ga)+cos(al)*cos(be)*bet*sin(ga)+cos(al)*sin(be)*cos(ga)*gat)*(zf-z)/l0+(sin(al)*cos(ga)+cos(al)*sin(be)*sin(ga))*(zft-zt)/l0)*G*A*(-cos(be)*cos(ga)*(xf-x)/l0+(-cos(al)*sin(ga)-sin(al)*sin(be)*cos(ga))*(yf-y)/l0+(-sin(al)*sin(ga)+cos(al)*sin(be)*cos(ga))*(zf-z)/l0);

    Vec dissBT1(6,INIT,0.);
    dissBT1(3) = 0.5*((bet-bept)*sin(ga/2.0+gap/2.0)/l0+(be-bep)*cos(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)/l0-sin(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)*cos(ga/2.0+gap/2.0)*(al-alp)/l0-cos(be/2.0+bep/2.0)*sin(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(al-alp)/l0+cos(be/2.0+bep/2.0)*cos(ga/2.0+gap/2.0)*(alt-alpt)/l0)*E*I1*cos(be/2.0+bep/2.0)*cos(ga/2.0+gap/2.0) + (-0.5*(-sin(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(be-bep)/l0+cos(ga/2.0+gap/2.0)*(bet-bept)/l0-cos(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(al-alp)*cos(be/2.0+bep/2.0)/l0-sin(ga/2.0+gap/2.0)*(alt-alpt)*cos(be/2.0+bep/2.0)/l0+sin(ga/2.0+gap/2.0)*(al-alp)*sin(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)/l0)*E*I2*sin(ga/2.0+gap/2.0)*cos(be/2.0+bep/2.0)+0.5*((alt-alpt)*sin(be/2.0+bep/2.0)/l0+(al-alp)*cos(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)/l0+(gat-gapt)/l0)*G*I0*sin(be/2.0+bep/2.0));
    dissBT1(4) = 0.5*((bet-bept)*sin(ga/2.0+gap/2.0)/l0+(be-bep)*cos(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)/l0-sin(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)*cos(ga/2.0+gap/2.0)*(al-alp)/l0-cos(be/2.0+bep/2.0)*sin(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(al-alp)/l0+cos(be/2.0+bep/2.0)*cos(ga/2.0+gap/2.0)*(alt-alpt)/l0)*l0*E*I1*(sin(ga/2.0+gap/2.0)/l0-sin(be/2.0+bep/2.0)*cos(ga/2.0+gap/2.0)*(al-alp)/l0/2.0) + 0.5*(-sin(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(be-bep)/l0+cos(ga/2.0+gap/2.0)*(bet-bept)/l0-cos(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(al-alp)*cos(be/2.0+bep/2.0)/l0-sin(ga/2.0+gap/2.0)*(alt-alpt)*cos(be/2.0+bep/2.0)/l0+sin(ga/2.0+gap/2.0)*(al-alp)*sin(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)/l0)*l0*E*I2*(cos(ga/2.0+gap/2.0)/l0+sin(ga/2.0+gap/2.0)*(al-alp)*sin(be/2.0+bep/2.0)/l0/2.0)+0.25*((alt-alpt)*sin(be/2.0+bep/2.0)/l0+(al-alp)*cos(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)/l0+(gat-gapt)/l0)*G*I0*(al-alp)*cos(be/2.0+bep/2.0);
    dissBT1(5) = 0.5*((bet-bept)*sin(ga/2.0+gap/2.0)/l0+(be-bep)*cos(ga/2.0 +gap/2.0)*(gat/2.0+gapt/2.0)/l0-sin(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)*cos(ga/ 2.0+gap/2.0)*(al-alp)/l0-cos(be/2.0+bep/2.0)*sin(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(al-alp)/l0+cos(be/2.0+bep/2.0)*cos(ga/2.0+gap/2.0)*(alt-alpt)/l0)*l0*E*I1 *((be-bep)*cos(ga/2.0+gap/2.0)/l0/2.0-cos(be/2.0+bep/2.0)*sin(ga/2.0+gap/2.0)*( al-alp)/l0/2.0) + 0.5*(-sin(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(be-bep)/l0+ cos(ga/2.0+gap/2.0)*(bet-bept)/l0-cos(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(al- alp)*cos(be/2.0+bep/2.0)/l0-sin(ga/2.0+gap/2.0)*(alt-alpt)*cos(be/2.0+bep/2.0)/ l0+sin(ga/2.0+gap/2.0)*(al-alp)*sin(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)/l0)*l0*E *I2*(-sin(ga/2.0+gap/2.0)*(be-bep)/l0/2.0-cos(ga/2.0+gap/2.0)*(al-alp)*cos(be/2.0+bep/2.0)/l0/2.0)+0.5*((alt-alpt)*sin(be/2.0+bep/2.0)/l0+(al-alp)*cos(be/2.0 +bep/2.0)*(bet/2.0+bept/2.0)/l0+(gat-gapt)/l0)*G*I0;

    Vec dissBT2(6,INIT,0.);
    dissBT2(3) = -cos(bef/2.0+be/2.0)*cos(gaf/2.0+ga/2.0)/l0*(0.25*(bet- bept)*sin(ga/2.0+gap/2.0)/l0+0.25*(be-bep)*cos(ga/2.0+gap/2.0)*(gat/2.0+gapt/ 2.0)/l0-0.25*sin(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)*cos(ga/2.0+gap/2.0)*(al-alp)/l0-0.25*cos(be/2.0+bep/2.0)*sin(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(al-alp)/ l0+0.25*cos(be/2.0+bep/2.0)*cos(ga/2.0+gap/2.0)*(alt-alpt)/l0)*E*I1 + 0.25*((beft-bet)*sin(gaf/2.0+ga/2.0)/l0+(bef-be)*cos(gaf/ 2.0+ga/2.0)*(gaft/2.0+gat/2.0)/l0-sin(bef/2.0+be/2.0)*(beft/2.0+bet/2.0)*cos( gaf/2.0+ga/2.0)*(alf-al)/l0-cos(bef/2.0+be/2.0)*sin(gaf/2.0+ga/2.0)*(gaft/2.0+ gat/2.0)*(alf-al)/l0+cos(bef/2.0+be/2.0)*cos(gaf/2.0+ga/2.0)*(alft-alt)/l0)*cos (be/2.0+bep/2.0)*cos(ga/2.0+gap/2.0)/l0*E*I1+sin(gaf/2.0+ga/2.0)*cos(bef/2.0+be/2.0)/l0*(-0.25*sin(ga/ 2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(be-bep)/l0+0.25*cos(ga/2.0+gap/2.0)*(bet-bept) /l0-0.25*cos(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(al-alp)*cos(be/2.0+bep/2.0)/l0 -0.25*sin(ga/2.0+gap/2.0)*(alt-alpt)*cos(be/2.0+bep/2.0)/l0+0.25*sin(ga/2.0+gap /2.0)*(al-alp)*sin(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)/l0)*E*I2 - 0.25*(-sin(gaf/2.0+ga/2.0)*(gaft/2.0+gat/2.0)*(bef-be)/l0+cos(gaf/2.0+ga/2.0)*(beft-bet)/l0-cos(gaf/2.0+ga/2.0)*(gaft/2.0+ gat/2.0)*(alf-al)*cos(bef/2.0+be/2.0)/l0-sin(gaf/2.0+ga/2.0)*(alft-alt)*cos(bef /2.0+be/2.0)/l0+sin(gaf/2.0+ga/2.0)*(alf-al)*sin(bef/2.0+be/2.0)*(beft/2.0+bet/ 2.0)/l0)*sin(ga/2.0+gap/2.0)*cos(be/2.0+bep/2.0)/l0*E*I2 - sin(bef/2.0+be/2.0)/l0*(0.25*(alt-alpt)*sin( be/2.0+bep/2.0)/l0+0.25*(al-alp)*cos(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)/l0+0.25 *(gat-gapt)/l0)*G*I0+0.25*((alft-alt)*sin(bef/2.0+be/2.0)/l0+(alf-al)*cos(bef/ 2.0+be/2.0)*(beft/2.0+bet/2.0)/l0+(gaft-gat)/l0)*sin(be/2.0+bep/2.0)/l0*G*I0;
    dissBT2(4) = (-sin(gaf/2.0+ga/2.0)/l0-sin(bef/2.0+be/2.0)*cos(gaf/2.0+ ga/2.0)*(alf-al)/l0/2.0)*(0.25*(bet-bept)*sin(ga/2.0+gap/2.0)/l0+0.25*(be-bep)* cos(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)/l0-0.25*sin(be/2.0+bep/2.0)*(bet/2.0+ bept/2.0)*cos(ga/2.0+gap/2.0)*(al-alp)/l0-0.25*cos(be/2.0+bep/2.0)*sin(ga/2.0+ gap/2.0)*(gat/2.0+gapt/2.0)*(al-alp)/l0+0.25*cos(be/2.0+bep/2.0)*cos(ga/2.0+gap /2.0)*(alt-alpt)/l0)*E*I1 + ((beft-bet)*sin(gaf/2.0+ga/2.0)/l0+(bef-be)*cos(gaf/2.0+ga/2.0)*(gaft/2.0+gat/2.0)/l0-sin(bef/2.0+be/2.0)*(beft/2.0+bet/2.0)*cos(gaf/2.0+ ga/2.0)*(alf-al)/l0-cos(bef/2.0+be/2.0)*sin(gaf/2.0+ga/2.0)*(gaft/2.0+gat/2.0)* (alf-al)/l0+cos(bef/2.0+be/2.0)*cos(gaf/2.0+ga/2.0)*(alft-alt)/l0)*(0.25*sin(ga /2.0+gap/2.0)/l0-0.125*sin(be/2.0+bep/2.0)*cos(ga/2.0+gap/2.0)*(al-alp)/l0)*E*I1 + (-cos(gaf/2.0+ga/2.0)/l0+sin(gaf/2.0+ga/2.0)*(alf-al)*sin(bef/2.0+be/2.0)/l0/2.0)*(-0.25*sin(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(be-bep)/l0+0.25*cos(ga/2.0+gap/2.0)*(bet-bept)/l0-0.25*cos(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(al-alp)*cos(be/2.0+bep/2.0)/l0-0.25*sin(ga/2.0+gap/2.0)*(alt-alpt)*cos(be/2.0+bep/2.0)/l0+0.25*sin(ga/2.0+gap/2.0)*(al-alp)*sin(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)/l0)*E*I2 + (-sin(gaf/2.0+ga/2.0)*(gaft/2.0+gat/2.0)*(bef-be)/l0+cos(gaf/2.0+ga/2.0)*(beft-bet)/l0-cos(gaf/2.0+ga/2.0)*(gaft/2.0+gat/2.0)*(alf-al)*cos(bef/2.0+be/2.0)/l0-sin(gaf/2.0+ga/2.0)*(alft-alt)*cos(bef/2.0+be /2.0)/l0+sin(gaf/2.0+ga/2.0)*(alf-al)*sin(bef/2.0+be/2.0)*(beft/2.0+bet/2.0)/l0)*(0.25*cos(ga/2.0+gap/2.0)/l0+0.125*sin(ga/2.0+gap/2.0)*(al-alp)*sin(be/2.0+ bep/2.0)/l0)*E*I2 + (alf-al)*cos(bef/2.0+be/2.0)/l0*(0.25*(alt-alpt)*sin(be/2.0+bep/2.0)/l0+0.25*(al-alp)*cos(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)/l0+0.25*(gat-gapt)/l0)*G*I0/2.0+0.125*((alft-alt)*sin(bef/2.0+be/2.0)/l0+( alf-al)*cos(bef/2.0+be/2.0)*(beft/2.0+bet/2.0)/l0+(gaft-gat)/l0)*(al-alp)*cos( be/2.0+bep/2.0)/l0*G*I0;
    dissBT2(5) = ((bef-be)*cos(gaf/2.0+ga/2.0)/l0/2.0-cos(bef/2.0+be/2.0)* sin(gaf/2.0+ga/2.0)*(alf-al)/l0/2.0)*(0.25*(bet-bept)*sin(ga/2.0+gap/2.0)/l0+ 0.25*(be-bep)*cos(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)/l0-0.25*sin(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)*cos(ga/2.0+gap/2.0)*(al-alp)/l0-0.25*cos(be/2.0+bep/2.0)* sin(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(al-alp)/l0+0.25*cos(be/2.0+bep/2.0)*cos (ga/2.0+gap/2.0)*(alt-alpt)/l0)*E*I1 + ((beft-bet)*sin(gaf/2.0+ga/2.0)/l0+(bef-be)*cos(gaf/2.0+ga /2.0)*(gaft/2.0+gat/2.0)/l0-sin(bef/2.0+be/2.0)*(beft/2.0+bet/2.0)*cos(gaf/2.0+ ga/2.0)*(alf-al)/l0-cos(bef/2.0+be/2.0)*sin(gaf/2.0+ga/2.0)*(gaft/2.0+gat/2.0)* (alf-al)/l0+cos(bef/2.0+be/2.0)*cos(gaf/2.0+ga/2.0)*(alft-alt)/l0)*(0.125*(be-bep)*cos(ga/2.0+gap/2.0)/l0-0.125*cos(be/2.0+bep/2.0)*sin(ga/2.0+gap/2.0)*(al-alp)/l0)*E*I1 + (-sin(gaf/2.0+ga/2.0)*(bef-be)/l0/2.0-cos(gaf/2.0+ga/2.0)* (alf-al)*cos(bef/2.0+be/2.0)/l0/2.0)*(-0.25*sin(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(be-bep)/l0+0.25*cos(ga/2.0+gap/2.0)*(bet-bept)/l0-0.25*cos(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(al-alp)*cos(be/2.0+bep/2.0)/l0-0.25*sin(ga/2.0+gap/2.0)*( alt-alpt)*cos(be/2.0+bep/2.0)/l0+0.25*sin(ga/2.0+gap/2.0)*(al-alp)*sin(be/2.0+ bep/2.0)*(bet/2.0+bept/2.0)/l0)*E*I2 + (-sin(gaf/2.0+ga/2.0)*(gaft/2.0+gat/2.0)*(bef -be)/l0+cos(gaf/2.0+ga/2.0)*(beft-bet)/l0-cos(gaf/2.0+ga/2.0)*(gaft/2.0+gat/2.0)*(alf-al)*cos(bef/2.0+be/2.0)/l0-sin(gaf/2.0+ga/2.0)*(alft-alt)*cos(bef/2.0+be /2.0)/l0+sin(gaf/2.0+ga/2.0)*(alf-al)*sin(bef/2.0+be/2.0)*(beft/2.0+bet/2.0)/l0)*(-0.125*sin(ga/2.0+gap/2.0)*(be-bep)/l0-0.125*cos(ga/2.0+gap/2.0)*(al-alp)* cos(be/2.0+bep/2.0)/l0)*E*I2-1/l0*(0.25*(alt-alpt)*sin(be/2.0+bep/2.0)/l0 +0.25*(al-alp)*cos(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)/l0+0.25*(gat-gapt)/l0)*G* I0+0.25*((alft-alt)*sin(bef/2.0+be/2.0)/l0+(alf-al)*cos(bef/2.0+be/2.0)*(beft/ 2.0+bet/2.0)/l0+(gaft-gat)/l0)/l0*G*I0; 

    Vec D = dissSE1 + dissSE2 + dissBT1 + dissBT1;

    /* Differentation of 'kinetic energy' with respect to qG */
    Vec dTdqG(6,INIT,0.);
    dTdqG(4) = rho*l0*alt*(-1.0*I1*sin(be)*pow(cos(ga),2.0)*cos(be)*alt-1.0*I1*sin(be)*cos(ga)*sin(ga)*bet-1.0*I2*sin(be)*cos(be)*alt+I2*sin(be)*cos(be)* alt*pow(cos(ga),2.0)+I2*sin(be)*sin(ga)*cos(ga)*bet+I0*cos(be)*sin(be)*alt+I0*cos(be)*gat);
    dTdqG(5) = -1.0*rho*l0*(I1*pow(cos(be),2.0)*cos(ga)*alt*alt*sin(ga)-2.0*I1*cos(be)*pow(cos(ga),2.0)*alt*bet+I1*bet*cos(be)*alt-1.0*cos(ga)*bet*bet*I1* sin(ga)-1.0*I2*pow(cos(be),2.0)*cos(ga)*alt*alt*sin(ga)-1.0*I2*bet*cos(be)*alt+2.0*I2*cos(be)*pow(cos(ga),2.0)*alt*bet+cos(ga)*bet*bet*I2*sin(ga));

    /* Differentiation of 'kinetic energy' with respect to qGt, qG */
    SqrMat dTdqGtqG(6,INIT,0.);
    dTdqGtqG(3,4) = rho*l0*(-2.0*sin(be)*pow(cos(ga),2.0)*alt*I1*cos(be)-1.0*I1*sin(be)*cos(ga)*sin(ga)*bet-2.0*sin(be)*alt*I2*cos(be)+2.0*sin(be)*alt*I2* cos(be)*pow(cos(ga),2.0)+I2*sin(be)*sin(ga)*cos(ga)*bet+2.0*cos(be)*alt*I0*sin(be)+I0*cos(be)*gat);
    dTdqGtqG(3,5) = rho*l0*(-2.0*sin(be)*pow(cos(ga),2.0)*alt*I1*cos(be)-1.0*I1*sin(be)*cos(ga)*sin(ga)*bet-2.0*sin(be)*alt*I2*cos(be)+2.0*sin(be)*alt*I2* cos(be)*pow(cos(ga),2.0)+I2*sin(be)*sin(ga)*cos(ga)*bet+2.0*cos(be)*alt*I0*sin(be)+I0*cos(be)*gat);
    dTdqGtqG(4,4) = -1.0*sin(be)*cos(ga)*alt*rho*l0*sin(ga)*(I1-1.0*I2);
    dTdqGtqG(4,5) = -1.0*sin(be)*cos(ga)*alt*rho*l0*sin(ga)*(I1-1.0*I2);
    dTdqGtqG(5,4) = 0.1E1*rho*l0*cos(be)*alt*I0;

    /* Vec of generalized forces */
    h = dTdqG-dTdqGtqG*qGt(6,11)-V;//-D;
  }

  double FiniteElement1s33Cosserat::computeKineticEnergy(const Vec& qG, const Vec& qGt) {
    int k;
    if(openStructure == true && currentElement == 0)
      k = 0;
    else
      k = 3;

    return rho*A*l0*(0.25*pow(nrm2(qGt(6+k,8+k)),2) + 0.5*(pow((I1*(cos(qG(7+k))*cos(qG(8+k))*qGt(6+k)+sin(qG(8+k))*qGt(7+k))),2) + pow((I2*(-cos(qG(7+k))*sin(qG(8+k))*qGt(6+k)+cos(qG(8+k)*qGt(7+k)))),2) + pow((I0*(qGt(8+k)+sin(qG(7+k))*qGt(6+k))),2)));
  }

  double FiniteElement1s33Cosserat::computeGravitationalEnergy(const Vec& qG) {
    int k;
    if(currentElement == 0 && openStructure == true)
      k = 0;
    else
      k = 3;
    return -rho*A*g(1)*qG(5+k);
  }

  const Vec& FiniteElement1s33Cosserat::computeState(const Vec& qG, const Vec& qGt,double s) {
    int k;
    if(currentElement == 0 && openStructure == true)
      k = 0;
    else
      k = 3;

    X(0,2) = qG(3+k,5+k) + s*(qG(9+k,11+k)-qG(3+k,5+k))/l0; // position
    X(6,8) = qGt(3+k,5+k) + s*((qGt(9+k,11+k)-qGt(3+k,5+k))/l0); // velocity

    if(s >= l0/2.) {
      X(3,5) = qG(6+k,8+k) + (s-0.5*l0)*(qG(12+k,14+k)-qG(6+k,8+k))/l0; // angles
      X(9,11) = qGt(6+k,8+k) + (s-0.5*l0)*(qGt(12+k,14+k)-qGt(6+k,8+k))/l0; // time differentiated angels
    }
    else {
      X(3,5) = qG(k,2+k) + (s+0.5*l0)*(qG(6+k,8+k)-qG(k,2+k))/l0; // angles
      X(9,11) = qGt(k,2+k) + (s+0.5*l0)*(qGt(6+k,8+k)-qGt(k,2+k))/l0; // time differentiated angels
    }

    return X;
  }

}
