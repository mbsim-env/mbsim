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
 * Contact: markus.ms.schneider@gmail.com
 */

#include "rigid_contour_functions1s.h"
#include "mbsim/utils/eps.h"
#include "mbsim/utils/nonlinear_algebra.h"
#include <string>

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

double calculateLocalAlpha(const double& alpha) {
  if ((alpha>0) && (alpha<2.*M_PI))
    return alpha;
  else {
    double a=fmod(alpha, 2.*M_PI);
    if(a<0) 
      a+=2.*M_PI;
    return a;
  }
}

Mat ContourXY2angleXY(const Mat &ContourMat_u, double scale, const Vec &rCOG_u , int discretization) { 
  Mat ContourMat;
  Vec rCOG;
  ContourMat = ContourMat_u;
  rCOG = rCOG_u; 
  ContourMat=ContourMat*scale;
  rCOG=rCOG*scale;
  int N = ContourMat.rows();
  Mat angleRxyTMP0(N,4);
  angleRxyTMP0.col(2) = ContourMat.col(0);
  angleRxyTMP0.col(3) = ContourMat.col(1);
  int i;
  for(i=0;i<N;i++) {
    angleRxyTMP0(i,2) = angleRxyTMP0(i,2) - rCOG(0);
    angleRxyTMP0(i,3) = angleRxyTMP0(i,3) - rCOG(1);
  }
  double r,phi;
  for(i=0; i<N; i++) {
    r   = sqrt(angleRxyTMP0(i,2)*angleRxyTMP0(i,2)+ angleRxyTMP0(i,3)*angleRxyTMP0(i,3));
    phi = acos(angleRxyTMP0(i,2)/r);
    if ( angleRxyTMP0(i,3) < 0)
      phi = 2.0*M_PI - phi;
    angleRxyTMP0(i,1) = r;
    angleRxyTMP0(i,0) = phi;
  }
  angleRxyTMP0 = bubbleSort(angleRxyTMP0,0); 
  int identXY =0;
  for (i=1;i < N; i++) 
    if ((abs(angleRxyTMP0(i,2)-angleRxyTMP0(i-1,2))<epsroot())&&(abs(angleRxyTMP0(i,3)-angleRxyTMP0(i-1,3))<epsroot())) 
      identXY++;
  Mat angleRxyTMP(N-identXY,4);
  int j=0;
  angleRxyTMP.row(0) = angleRxyTMP0.row(0);
  for (i=1; i<N;i++) {
    if (!((abs(angleRxyTMP0(i,2)-angleRxyTMP0(i-1,2))<epsroot())&&(abs(angleRxyTMP0(i,3)-angleRxyTMP0(i-1,3))<epsroot())))
      j++;
    angleRxyTMP.row(j) = angleRxyTMP0.row(i); 
  }
  N = N-identXY;
  double rPhi0, r0,rN,phi0,phiN;
  r0 = angleRxyTMP(0,1);    
  rN = angleRxyTMP(N-1,1);  
  phi0 = angleRxyTMP(0,0);  
  phiN = angleRxyTMP(N-1,0);
  rPhi0 =  (r0 - rN) / (phi0 - (phiN-2*M_PI)) * (0 - (phiN-2*M_PI)) + rN;
  double dAngleGrenz= 10.0;
  for (i=1; i<N; i++)
    if (((angleRxyTMP(i,0)-angleRxyTMP(i-1,0)) < dAngleGrenz)&&(abs(angleRxyTMP(i,2)-angleRxyTMP(i-1,2))>epsroot())&&(abs(angleRxyTMP(i,3)-angleRxyTMP(i-1,3))>epsroot())) 
      dAngleGrenz = angleRxyTMP(i,0)-angleRxyTMP(i-1,0); 
  dAngleGrenz *= 0.2; 
  int index0, indexE;
  index0 =0;
  indexE =N-1;
  if (angleRxyTMP(0,0) < dAngleGrenz)
    index0++;
  if (abs(2.*M_PI - angleRxyTMP(N-1,0)) < dAngleGrenz) 
    indexE--;
  int Nneu = 1+ indexE - index0 +2;
  Mat angleRxy(Nneu,4,INIT,0.0);
  angleRxy(0,1) = rPhi0; 
  angleRxy(0,2) = rPhi0; 
  angleRxy(Nneu-1,0) = 2.*M_PI;
  angleRxy(Nneu-1,1) = rPhi0;  
  angleRxy(Nneu-1,2) = rPhi0; 
  angleRxy(1,0,Nneu-2,3) = angleRxyTMP(index0,0,indexE,3);
  int N_diskret = Nneu -1; 
  N_diskret = (N_diskret -1)/discretization +1;
  N_diskret++;
  Mat erg(N_diskret,3);
  j=0;
  for(i=0; i<(Nneu-1); i=i+discretization) {
    erg(j,0) = angleRxy(i,0);
    erg(j,1) = angleRxy(i,2);
    erg(j,2) = angleRxy(i,3);
    j=j+1;
  }
  if (!(j==N_diskret-1))
    throw runtime_error("Error in ContourXY2angleXY");
  erg(j,0) = angleRxy(Nneu-1,0);
  erg(j,1) = angleRxy(Nneu-1,2);
  erg(j,2) = angleRxy(Nneu-1,3);
  return erg;
}

FuncCrPC_PlanePolar::FuncCrPC_PlanePolar() : Cb(Vec("[1; 0; 0]")), pp_r(0), alphaSave(0.), salphaSave(0.), calphaSave(0.), rSave(0.), drdalphaSave(0.), d2rdalpha2Save(0.) {
}

FuncCrPC_PlanePolar::~FuncCrPC_PlanePolar() {
  delete pp_r;
}

void FuncCrPC_PlanePolar::setYZ(const Mat& YZ, int discretization, Vec rYZ) {
  Mat angleYZ=ContourXY2angleXY(YZ, 1, rYZ , 1); 
  Vec r(angleYZ.rows(), NONINIT);
  for (int i=0; i<r.size(); i++)
    r(i)=nrm2(angleYZ(RangeV(i,i), RangeV(1,2)));
  pp_r=new PiecewisePolynomFunction<VecV(double)>;
  pp_r->setParent(this);
  pp_r->setx(angleYZ.col(0));
  pp_r->sety(r);
  pp_r->setInterpolationMethod(PiecewisePolynomFunction<VecV(double)>::cSplinePeriodic);
}   

void FuncCrPC_PlanePolar::init(Element::InitStage stage, const InitConfigSet &config) {
  Function<Vec3(double)>::init(stage, config);
  pp_r->init(stage, config);
}

Vec3 FuncCrPC_PlanePolar::operator()(const double& alpha) {
  updateData(alpha);
  Vec3 f(NONINIT);
  f(2) = 0;
  f(0) = rSave*calphaSave;
  f(1) = rSave*salphaSave;
  return f;
} 

Vec3 FuncCrPC_PlanePolar::parDer(const double& alpha) {
  updateData(alpha);
  Vec3 f(NONINIT);
  f(2) = 0;
  f(0) = drdalphaSave*calphaSave-rSave*salphaSave;
  f(1) = drdalphaSave*salphaSave+rSave*calphaSave;
  return f;
}

Vec3 FuncCrPC_PlanePolar::parDerParDer(const double& alpha) {
  updateData(alpha);
  const double s1=-rSave+d2rdalpha2Save;
  const double s2=2.*drdalphaSave;
  Vec3 f(NONINIT);
  f(2) = 0;
  f(0) = s1*calphaSave-s2*salphaSave;
  f(1) = s1*salphaSave+s2*calphaSave;
  return f;
}

void FuncCrPC_PlanePolar::updateData(const double& alpha) {
  if (fabs(calculateLocalAlpha(alpha)-alphaSave)>epsroot()*epsroot()) {
    alphaSave=calculateLocalAlpha(alpha);
    salphaSave=sin(alphaSave);
    calphaSave=cos(alphaSave);
    rSave=(*pp_r)(alphaSave)(0);
    drdalphaSave=pp_r->parDer(alphaSave)(0);
    d2rdalpha2Save=pp_r->parDerParDer(alphaSave)(0);
  }
}
