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

#include <string>

#include "rigid_contour_functions1s.h"

#include "mbsim/utils/eps.h"
#include "mbsim/utils/function_library.h"
#include "mbsim/utils/nonlinear_algebra.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

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
    throw(123);
  erg(j,0) = angleRxy(Nneu-1,0);
  erg(j,1) = angleRxy(Nneu-1,2);
  erg(j,2) = angleRxy(Nneu-1,3);
  return erg;
}

FuncCrPC::FuncCrPC() : ContourFunction1s(), Cb(3) {
  Cb(0)=1;
  operator_ = &FuncCrPC::operatorPPolynom;
  computeT_ = &FuncCrPC::computeTPPolynom;
  computeB_ = &FuncCrPC::computeBPPolynom;
  computeN_ = &FuncCrPC::computeNPPolynom;
  computeCurvature_ = &FuncCrPC::computeCurvaturePPolynom;
}

void FuncCrPC::setYZ(const Mat& YZ, int discretization, Vec rYZ) {
  Mat angleYZ=ContourXY2angleXY(YZ, 1., rYZ , discretization); 
  pp_y.setXF(angleYZ.col(0), angleYZ.col(1), "csplinePer");
  pp_z.setXF(angleYZ.col(0), angleYZ.col(2), "csplinePer");
}   

//void FuncCrPC::init(const double& alpha) {
//  const Vec CrPC = operator()(alpha);
//  const Vec Ct = diff1(alpha);
//  Cb = crossProduct(CrPC,Ct);
//  Cb = Cb/nrm2(Cb);  
//}

void FuncCrPC::enableTabularFit(double tabularFitLength) {
  vector<double> a;
  a.push_back(0);
  while(a.back()<2.*M_PI) {
    Vec p1=operator()(a.back());
    class PointDistance : public Function1<double, double> {
      public:
        PointDistance(Vec p1_, FuncCrPC * f_, double d_) : p1(p1_), f(f_), d(d_) {}
        double operator()(const double &alpha, const void * = NULL) {
          Vec p2=(*f)(alpha);
          return nrm2(p2-p1)-d;
        }
      private:
        Vec p1;
        FuncCrPC * f;
        double d;
    };
    PointDistance g(p1, this, tabularFitLength);
    RegulaFalsi solver(&g);
    solver.setTolerance(epsroot());
    a.push_back(solver.solve(a.back(), a.back()+M_PI/4.));
  }
  Vec phi(a.size(), INIT, 0);
  Mat O(a.size(), 3, INIT, 0);
  Mat T(a.size(), 3, INIT, 0);
  Mat B(a.size(), 3, INIT, 0);
  Mat N(a.size(), 3, INIT, 0);
  Vec curve(a.size(), INIT, 0);
  for (unsigned int i=0; i<a.size(); i++) {
    double alp=a[i]/a.back()*2.*M_PI;
    phi(i)=alp;
    O.row(i)=trans(operator()(alp));
    T.row(i)=trans(this->computeT(alp));
    B.row(i)=trans(this->computeB(alp));
    N.row(i)=trans(this->computeN(alp));
    curve(i)=this->computeCurvature(alp);
  }
  tab_operator = new TabularFunction1_VS(phi, O);
  tab_T = new TabularFunction1_VS(phi, T);
  tab_B = new TabularFunction1_VS(phi, B);
  tab_N = new TabularFunction1_VS(phi, N);
  tab_curvature = new TabularFunction1_VS(phi, curve);

  operator_ = &FuncCrPC::operatorTabular;
  computeT_ = &FuncCrPC::computeTTabular;
  computeB_ = &FuncCrPC::computeBTabular;
  computeN_ = &FuncCrPC::computeNTabular;
  computeCurvature_ = &FuncCrPC::computeCurvatureTabular;
}

Vec FuncCrPC::diff1(const double& alpha) {
  Vec f(3,NONINIT);
  const double alphaLoc=calculateLocalAlpha(alpha);
  f(0) = 0;
  f(1) = ((pp_y).getDerivative(1))(alphaLoc)(0); 
  f(2) = ((pp_z).getDerivative(1))(alphaLoc)(0); 
  return f;
}

Vec FuncCrPC::diff2(const double& alpha) {
  Vec f(3,NONINIT);
  const double alphaLoc=calculateLocalAlpha(alpha);
  f(0) = 0;
  f(1) = ((pp_y).getDerivative(2))(alphaLoc)(0); 
  f(2) = ((pp_z).getDerivative(2))(alphaLoc)(0); 
  return f;
}

Vec FuncCrPC::operatorPPolynom(const double& alpha) {
  Vec f(3,NONINIT);
  const double alphaLoc=calculateLocalAlpha(alpha);
  f(0) = 0;
  f(1) = (pp_y.getDerivative(0))(alphaLoc)(0); 
  f(2) = (pp_z.getDerivative(0))(alphaLoc)(0); 
  return f;
} 

Vec FuncCrPC::operatorTabular(const double& alpha) {
  return (*tab_operator)(calculateLocalAlpha(alpha));
}

Vec FuncCrPC::computeTPPolynom(const double& alpha) {
  const Vec T = -diff1(alpha);
  return T/nrm2(T); 
}

Vec FuncCrPC::computeTTabular(const double& alpha) {
  return (*tab_T)(calculateLocalAlpha(alpha));
}

Vec FuncCrPC::computeNPPolynom(const double& alpha) { 
  const Vec N = crossProduct(diff1(alpha), Cb);
  // const Vec N = crossProduct(diff1(alpha), computeBPPolynom(alpha));
  return N/nrm2(N);
}

Vec FuncCrPC::computeNTabular(const double& alpha) {
  return (*tab_N)(calculateLocalAlpha(alpha));
}

Vec FuncCrPC::computeBPPolynom(const double& alpha) {
  // const Vec B = crossProduct(operator()(alpha), diff1(alpha));
  // return B/nrm2(B);
  return Cb;
}

Vec FuncCrPC::computeBTabular(const double& alpha) {
  return (*tab_B)(calculateLocalAlpha(alpha));
}

double FuncCrPC::computeCurvaturePPolynom(const double& alpha) {
  const Vec rs = diff1(alpha);
  const double nrm2rs = nrm2(rs);
  return nrm2(crossProduct(rs,diff2(alpha)))/(nrm2rs*nrm2rs*nrm2rs);
}

double FuncCrPC::computeCurvatureTabular(const double& alpha) {
  return (*tab_curvature)(calculateLocalAlpha(alpha))(0);
}

  double FuncCrPC::calculateLocalAlpha(const double& alpha) {
    if ((alpha>0) && (alpha<2.*M_PI))
      return alpha;
    else {
      double a=fmod(alpha, 2.*M_PI);
      if(a<0) 
        a+=2.*M_PI;
      return a;
    }
  }

void FuncCrPC::initializeUsingXML(TiXmlElement * element) {
/* TiXmlElement * e;
  e=element->FirstChildElement(MBSIMVALVETRAINNS"YZ");
  Mat YZ=Element::getMat(e);
  int dis=1;
  e=element->FirstChildElement(MBSIMVALVETRAINNS"discretization");
  if (e)
    dis=atoi(e->GetText());
  Vec rYZ(3);
  e=element->FirstChildElement(MBSIMVALVETRAINNS"rYZ");
  if (e)
    rYZ=Element::getVec(e, 3);
  setYZ(YZ, dis, rYZ);
  e=element->FirstChildElement(MBSIMVALVETRAINNS"enableTabularFit");
  if (e)
    enableTabularFit(Element::getDouble(e->FirstChildElement(MBSIMVALVETRAINNS"fitLength")));*/
}


