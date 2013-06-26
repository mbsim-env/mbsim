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


#include "mbsim/utils/eps.h"
#include "mbsim/utils/function_library.h"
#include "mbsim/utils/nonlinear_algebra.h"
#include "rigid_contour_functions1s.h"

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace MBSim;

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
    throw(123);
  erg(j,0) = angleRxy(Nneu-1,0);
  erg(j,1) = angleRxy(Nneu-1,2);
  erg(j,2) = angleRxy(Nneu-1,3);
  return erg;
}

FuncCrPC::FuncCrPC() : ContourFunction1s() {
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
  MatVx3 O(a.size(), INIT, 0);
  MatVx3 T(a.size(), INIT, 0);
  MatVx3 B(a.size(), INIT, 0);
  MatVx3 N(a.size(), INIT, 0);
  Vec curve(a.size(), INIT, 0);
  for (unsigned int i=0; i<a.size(); i++) {
    double alp=a[i]/a.back()*2.*M_PI;
    phi(i)=alp;
    O.set(i, operator()(alp).T());
    T.set(i, this->computeT(alp).T());
    B.set(i, this->computeB(alp).T());
    N.set(i, this->computeN(alp).T());
    curve(i)=this->computeCurvature(alp);
  }
  tab_operator = new TabularFunction1_VS<Ref,Ref>(phi, Mat(O));
  tab_T = new TabularFunction1_VS<Ref,Ref>(phi, Mat(T));
  tab_B = new TabularFunction1_VS<Ref,Ref>(phi, Mat(B));
  tab_N = new TabularFunction1_VS<Ref,Ref>(phi, Mat(N));
  tab_curvature = new TabularFunction1_VS<Ref,Ref>(phi, curve);

  operator_ = &FuncCrPC::operatorTabular;
  computeT_ = &FuncCrPC::computeTTabular;
  computeB_ = &FuncCrPC::computeBTabular;
  computeN_ = &FuncCrPC::computeNTabular;
  computeCurvature_ = &FuncCrPC::computeCurvatureTabular;
}

Vec3 FuncCrPC::diff1(const double& alpha) {
  Vec3 f(NONINIT);
  const double alphaLoc=calculateLocalAlpha(alpha);
  f(0) = 0;
  f(1) = ((pp_y).getDerivative(1))(alphaLoc)(0); 
  f(2) = ((pp_z).getDerivative(1))(alphaLoc)(0); 
  return f;
}

Vec3 FuncCrPC::diff2(const double& alpha) {
  Vec3 f(NONINIT);
  const double alphaLoc=calculateLocalAlpha(alpha);
  f(0) = 0;
  f(1) = ((pp_y).getDerivative(2))(alphaLoc)(0); 
  f(2) = ((pp_z).getDerivative(2))(alphaLoc)(0); 
  return f;
}

Vec3 FuncCrPC::operatorPPolynom(const double& alpha) {
  Vec f(3,NONINIT);
  const double alphaLoc=calculateLocalAlpha(alpha);
  f(0) = 0;
  f(1) = (pp_y.getDerivative(0))(alphaLoc)(0); 
  f(2) = (pp_z.getDerivative(0))(alphaLoc)(0); 
  return f;
} 

Vec3 FuncCrPC::operatorTabular(const double& alpha) {
  return (*tab_operator)(calculateLocalAlpha(alpha));
}

Vec3 FuncCrPC::computeTPPolynom(const double& alpha) {
  const Vec T = -diff1(alpha);
  return T/nrm2(T); 
}

Vec3 FuncCrPC::computeTTabular(const double& alpha) {
  return (*tab_T)(calculateLocalAlpha(alpha));
}

Vec3 FuncCrPC::computeNPPolynom(const double& alpha) { 
  const Vec3 N = crossProduct(diff1(alpha), Cb);
  // const Vec3 N = crossProduct(diff1(alpha), computeBPPolynom(alpha));
  return N/nrm2(N);
}

Vec3 FuncCrPC::computeNTabular(const double& alpha) {
  return (*tab_N)(calculateLocalAlpha(alpha));
}

Vec3 FuncCrPC::computeBPPolynom(const double& alpha) {
  // const Vec3 B = crossProduct(operator()(alpha), diff1(alpha));
  // return B/nrm2(B);
  return Cb;
}

Vec3 FuncCrPC::computeBTabular(const double& alpha) {
  return (*tab_B)(calculateLocalAlpha(alpha));
}

double FuncCrPC::computeCurvaturePPolynom(const double& alpha) {
  const Vec3 rs = diff1(alpha);
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
  ContourFunction1s::initializeUsingXML(element);
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

FuncCrPC_PlanePolar::FuncCrPC_PlanePolar() : ContourFunction1s(), Cb(Vec("[1; 0; 0]")), pp_r(0), alphaSave(0.), salphaSave(0.), calphaSave(0.), rSave(0.), drdalphaSave(0.), d2rdalpha2Save(0.) {
}

void FuncCrPC_PlanePolar::setYZ(const Mat& YZ, int discretization, Vec rYZ) {
  Mat angleYZ=ContourXY2angleXY(YZ, 1, rYZ , 1); 
  Vec r(angleYZ.rows(), NONINIT);
  for (int i=0; i<r.size(); i++)
    r(i)=nrm2(angleYZ(Index(i,i), Index(1,2)));
  pp_r=new PPolynom<Ref,Ref>();
  pp_r->setXF(angleYZ.col(0), r, "csplinePer");
  updateData(1.);
}   

Vec3 FuncCrPC_PlanePolar::operator()(const double& alpha, const void * ) {
  updateData(alpha);
  Vec3 f(NONINIT);
  f(0) = 0;
  f(1) = rSave*calphaSave;
  f(2) = rSave*salphaSave;
  return f;
} 

Vec3 FuncCrPC_PlanePolar::diff1(const double& alpha) {
  updateData(alpha);
  Vec3 f(NONINIT);
  f(0) = 0;
  f(1) = drdalphaSave*calphaSave-rSave*salphaSave;
  f(2) = drdalphaSave*salphaSave+rSave*calphaSave;
  return f;
}

Vec3 FuncCrPC_PlanePolar::diff2(const double& alpha) {
  updateData(alpha);
  const double s1=-rSave+d2rdalpha2Save;
  const double s2=2.*drdalphaSave;
  Vec3 f(NONINIT);
  f(0) = 0;
  f(1) = s1*calphaSave-s2*salphaSave;
  f(2) = s1*salphaSave+s2*calphaSave;
  return f;
}

Vec3 FuncCrPC_PlanePolar::computeT(const double& alpha) {
  const Vec3 T = -diff1(alpha);
  return T/sqrt(T(1)*T(1)+T(2)*T(2));
}

Vec3 FuncCrPC_PlanePolar::computeN(const double& alpha) { 
  updateData(alpha);
  Vec3 N(NONINIT);
  N(0) = 0;
  N(1) = drdalphaSave*salphaSave+rSave*calphaSave;
  N(2) = -drdalphaSave*calphaSave+rSave*salphaSave;
  return N/sqrt(N(1)*N(1)+N(2)*N(2));
}

Vec3 FuncCrPC_PlanePolar::computeB(const double& alpha) {
  return Cb;
}

double FuncCrPC_PlanePolar::computeCurvature(const double& alpha) {
  updateData(alpha);
  const double r2=rSave*rSave;
  const double drdalphaSave2=drdalphaSave*drdalphaSave;
  const double numer=r2+2.*drdalphaSave2-rSave*d2rdalpha2Save;
  const double denomfactor=r2+drdalphaSave2;
  return abs(numer/sqrt(denomfactor*denomfactor*denomfactor));
}

double FuncCrPC_PlanePolar::computeR(const double& alpha) {
  updateData(alpha);
  return rSave;
}

double FuncCrPC_PlanePolar::computedRdAlpha(const double& alpha) {
  updateData(alpha);
  return drdalphaSave;
}

double FuncCrPC_PlanePolar::computed2RdAlpha2(const double& alpha) {
  updateData(alpha);
  return d2rdalpha2Save;
}

void FuncCrPC_PlanePolar::updateData(const double& alpha) {
  if (fabs(calculateLocalAlpha(alpha)-alphaSave)>epsroot()*epsroot()) {
    alphaSave=calculateLocalAlpha(alpha);
    salphaSave=sin(alphaSave);
    calphaSave=cos(alphaSave);
    rSave=(pp_r->getDerivative(0))(alphaSave)(0);
    drdalphaSave=(pp_r->getDerivative(1))(alphaSave)(0);
    d2rdalpha2Save=(pp_r->getDerivative(2))(alphaSave)(0);
  }
}

void FuncCrPC_PlanePolar::initializeUsingXML(TiXmlElement * element) {
  ContourFunction1s::initializeUsingXML(element);
}



