#include <string>

#include "rigid_contour_functions1s.h"
#include "mbsim/utils/eps.h"

using namespace MBSim;
using namespace fmatvec;
using namespace std;

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

void FuncCrPC::setYZ(const Mat& YZ, int discretization,Vec rYZ) {
  Mat angleYZ=ContourXY2angleXY(YZ, 1., rYZ , discretization); 
  pp_y.setXF(angleYZ.col(0), angleYZ.col(1), "csplinePer");
  pp_z.setXF(angleYZ.col(0), angleYZ.col(2), "csplinePer");
}   

void FuncCrPC::init(double alpha) {
  Vec CrPC = operator()(alpha);
  Vec Ct = diff1(alpha);
  Cb = crossProduct(CrPC,Ct);
  Cb = Cb/nrm2(Cb);  
}

Vec FuncCrPC::operator()(double alpha){
  Vec f(3,INIT);
  alpha=fmod(alpha, 2.*M_PI);
  if(alpha<0) 
    alpha+=2.*M_PI;
  f(1) = (pp_y)(alpha)(0); 
  f(2) = (pp_z)(alpha)(0); 
  return f;
} 

Vec FuncCrPC::diff1(double alpha) {
  Vec f(3,INIT);
  alpha=fmod(alpha, 2.*M_PI);
  if(alpha<0) 
    alpha+=2.*M_PI;
  f(1) = (pp_y).diff1(alpha)(0); 
  f(2) = (pp_z).diff1(alpha)(0); 
  return f;
}

Vec FuncCrPC::diff2(double alpha) {
  Vec f(3,INIT);
  alpha=fmod(alpha, 2.*M_PI);
  if(alpha<0) 
    alpha+=2.*M_PI;
  f(1) = (pp_y).diff2(alpha)(0); 
  f(2) = (pp_z).diff2(alpha)(0); 
  return f;
}


