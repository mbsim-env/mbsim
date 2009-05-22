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
#include "mbsim/contours/nurbs_disk_2s.h"
#include "mbsim/flexible_body/flexible_body_2s_13_disk.h"
#ifdef HAVE_NURBS
using namespace PLib; 
#endif

using namespace std;

namespace MBSim {


  void NurbsDisk2s::updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff) {
    if(nrm2(cp.getLagrangeParameterPosition()) == 0.) { // center of gravity TODO
      cp.getFrameOfReference().setPosition(frameOfReference->getPosition() + frameOfReference->getOrientation().col(2)*q(0)); // z-translation
      cp.getFrameOfReference().setOrientation(frameOfReference->getOrientation()*q(1)); // z-rotation
      cp.getFrameOfReference().setVelocity(frameOfReference->getOrientation().col(2)*u(0)); // z-translation
      cp.getFrameOfReference().setAngularVelocity(frameOfReference->getOrientation().col(2)*u(1)); // z-rotation
    }
    else { // somewhere else
      if(ff==position || ff==position_cosy || ff==all) {
        PLib::Point3Dd Tmppt = Surface->pointAt(cp.getLagrangeParameterPosition()(1),cp.getLagrangeParameterPosition()(0));  //U-direction is azimuthal, V-direction is radial!
        cp.getFrameOfReference().getPosition(0) = Tmppt.x();
        cp.getFrameOfReference().getPosition(1) = Tmppt.y();
        cp.getFrameOfReference().getPosition(2) = Tmppt.z();
      }

      // first tangent: radial-direction, second tangent:  azimuthal-direction
      if(ff==firstTangent ||  ff==secondTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) 
        cp.getFrameOfReference()(0,1,2,2) =  computeDirectionalDerivatives(cp.getLagrangeParameterPosition()(1),cp.getLagrangeParameterPosition()(0),1);

      if(ff==normal || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
        PLib::Point3Dd normal(Surface->normal(cp.getLagrangeParameterPosition()(1),cp.getLagrangeParameterPosition()(0)));
        //TODO NURBS normieren lassen?

        //cout << "NORMALE:            x1=" << normal.x()<< "   x2=" << normal.y() << "   x3=" << normal.z() << endl ; //testing

        //transfers the normal of the Nurbs++-Interface to the fmatvec Vector
        Vec Vec_norm(3);
        Vec_norm(0) = normal.x();
        Vec_norm(1) = normal.y();
        Vec_norm(2) = normal.z();

        Vec_norm *= flagNormal;  // the normal should point into the material

        // norms the vector
        Vec_norm = Vec_norm/nrm2(Vec_norm);

        //cout << "NORMALE(normiert):  x1=" << Vec_norm(0) << "   x2=" << Vec_norm(1) << "    x3=" << Vec_norm(2) << endl;

        cp.getFrameOfReference().col(0) =  Vec_norm;
      }

      if(ff==velocity || ff==velocities || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
        PLib::Point3Dd Tmpv = SurfaceVelocities->pointAt(cp.getLagrangeParameterPosition()(1),cp.getLagrangeParameterPosition()(0));
        cp.getFrameOfReference().getVelocity(0) = Tmppt.x();
        cp.getFrameOfReference().getVelocity(1) = Tmppt.y();
        cp.getFrameOfReference().getVelocity(2) = Tmppt.z();
      }

      if(ff==angularVelocity || ff==velocities || ff==velocities_cosy || ff==all) {
        // TODO
        throw new MBSimError("ERROR(FlexibleBody2s13Disk::updateKinematicsForFrame): Not implemented!");
      }
    }
  }
}

void NurbsDisk2s::init(const int dU, const int dV, const int nBr, const int nBj, const double &innerRadius, const double &outerRadius, const int flagN){
  degU = dU;
  degV = dV;
  nr = nBr;
  nj = nBj;
  Ri = innerRadius;
  Ra = outerRadius;
  flagNormal = flagN;

  computeUVec(nj+degU);
  computeVVec(nr+1);
}

Vec NurbsDisk2s::transformCW(const Vec& WrPoint) {
  return (static_cast<BodyFlexible2s13Disk*>(parent))->transformCW(WrPoint);
}

Mat NurbsDisk2s::computeDirectionalDerivatives(const double &radius, const double &phi, const int &deg) {
  PLib::Matrix<Point3Dd> Derivates(deg+1,deg+1);  // matrix that contains the derivates of the surface

  Surface->deriveAt(phi, radius, deg, Derivates);  // azimuthal-direction is U-direction, radial-direction is V-direction!

  //transform from Matrix of Nurbs to fmatvec Mat(rix)
  Mat ReturnMatrix(3,2);

  //no rule for the sign
  //tangent in u-direction (=azimuthal direction)
  ReturnMatrix(0,1) = Derivates(deg,0).x();
  ReturnMatrix(1,1) = Derivates(deg,0).y();
  ReturnMatrix(2,1) = Derivates(deg,0).z();
  //tangent in v-direction (=radial direction)
  ReturnMatrix(0,0) = Derivates(0,deg).x();
  ReturnMatrix(1,0) = Derivates(0,deg).y();
  ReturnMatrix(2,0) = Derivates(0,deg).z();

  //normalizes the tangent vectors
  ReturnMatrix(0,0,2,0) = 1/nrm2(ReturnMatrix(0,0,2,0)) * ReturnMatrix(0,0,2,0);
  ReturnMatrix(0,1,2,1) = 1/nrm2(ReturnMatrix(0,1,2,1)) * ReturnMatrix(0,1,2,1);

  //testing
  //cout << deg << "-fache Ableitung in U - also radiale    - Richtung:   " << "x1= " << ReturnMatrix(0,0) << "  x2= " << ReturnMatrix(1,0) << "  x3= " << ReturnMatrix(2,0) <<  endl;
  //cout << deg << "-fache Ableitung in V - also azimuthale - Richtung:   " << "x1= " << ReturnMatrix(0,1) << "  x2= " << ReturnMatrix(1,1) << "  x3= " << ReturnMatrix(2,1) <<  endl; 

  return ReturnMatrix;
}


Mat NurbsDisk2s::computeCurvatures(const double &radius, const double &phi) {
  return computeDirectionalDerivatives(radius, phi, 2);
}

void NurbsDisk2s::computeUVec(const int NbPts) {
  uvec = new PLib::Vector<double>(NbPts);
  uVec = new PLib::Vector<double>(NbPts + degU+1);

  const double stepU = 2 * M_PI / nj;

  (*uvec)[0] = 0;
  for(int i=1;i<uvec->size();i++) {
    (*uvec)[i] = (*uvec)[i-1] + stepU;
  }

  (*uVec)[0] = (-degU) * stepU;
  for(int i= 1; i<uVec->size();i++) {
    (*uVec)[i] = (*uVec)[i-1] + stepU;
  }
}

void NurbsDisk2s::computeVVec(const int NbPts) {
  vvec = new PLib::Vector<double>(NbPts);
  vVec = new PLib::Vector<double>(NbPts + degV+1);

  const double stepV = (Ra - Ri) / nr;

  (*vvec)[0] = Ri;
  for(int i=1;i<vvec->size();i++) {
    (*vvec)[i] = (*vvec)[i-1]+ stepV;
  }

  for(int i=degV+1; i<vVec->size()-(degV+1); i++) {
    (*vVec)[i] = (*vvec)[i-(degV-1)];  
  }

  for(int i=0; i < degV+1; i++) { //the first and last (degV+1)-Values have to be equal
    (*vVec)[i] = Ri;
    (*vVec)[vVec->size()-i-1]= Ra;
  }
}

void NurbsDisk2s::computeSurface() {
#ifdef HAVE_NURBS 
  PLib::Matrix<Point3Dd> Nodelist(nj+degU,nr+1); //list of Cartesian node-coordinates for the nurbs interpolation (2*degU+1 is used for the 2 outline of the surface in azimuthal direction)

  // gets Points from body for interpolation
  for(int i=0; i<nr+1; i++) {  
    for(int j=0; j<nj; j++) {  
      ContourPointData cp(i*nj+j);
      static_cast<BodyFlexible2s13Disk*>(parent)->updateKinematicsForFrame(cp,position);
      Nodelist(j,i) = PLib::Point3Dd(cp.getFrameOfReference().getPosition(0),cp.getFrameOfReference().getPosition(1),cp.getFrameOfReference().getPosition(2));
    }
    for(int j=0;j<degU;j++) { //expands the surface addicted to the degree in azimuthal direction
      Nodelist(nj+j,i) = Nodelist(j,i);
    }
  }

  Surface = new PlNurbsSurfaced;
  Surface->globalInterpClosedU_OwnKnotVecs(Nodelist, *uVec, *vVec, *uvec, *vvec, degU, degV);

  /*//TESTBLOCK...

  //testing the "real"values and the surface values at the nodes
  double maxerr = 0;
  double minerr = 1;
  double stepU = 2*M_PI / nj;
  double stepV = ( (*vvec)[vvec->size()-1] - (*vvec)[0]) / nr;
  for(int i = 0; i<= nr; i++)
  for(int j= 0; j < nj; j++)
  {
  cout << "i= " << i << "j= " << j << endl;
  PLib::Point3Dd PointUV = Surface->pointAt(j*stepU,(*vVec)[0]+i*stepV);
  cout << PointUV.x() << " "<< PointUV.y() << " " << PointUV.z() << endl;

  Vec InterpPoint = (static_cast<BodyFlexible2s13Disk*>(parent))->computeWrOC(i*nj+j);
  cout << InterpPoint(0) << " " << InterpPoint(1) << " " << InterpPoint(2) << endl;

  Vec error(3);
  error(0) = PointUV.x()-InterpPoint(0);
  error(1) = PointUV.y()-InterpPoint(1);
  error(2) = PointUV.z()-InterpPoint(2);
  cout << "Fehler=    " << nrm2(error)  << endl;
  if(maxerr < nrm2(error) ) maxerr = nrm2(error);
  if(minerr > nrm2(error) ) minerr = nrm2(error);
  }
  cout << "Maximaler Fehler ist:    " << maxerr << endl;
  cout << "Minimaler Fehler ist:    " << minerr << endl;

  //testing derivations
  const int degDev = 10;
  Vec TestVec[degDev][2][2]; //degDev is for degree of derivation, 4 is for the number of knots, 2 is for the direction of the derviation (U or V)
  int k = 0;
  int l = 0;

  cout << "TEST der Ableitungen" << endl;

  Vec test_pt(3);
  test_pt(0) = ((*vvec)[vvec->size()-1]-(*vvec)[0])/2;
  test_pt(1) = 0;
  cout << "Punkt ist bei phi=0" << endl;
  for(int i=1;i<=degDev;i++)
  {
  TestVec[k][l][0] = computeWd(test_pt(0),test_pt(1),i).col(0);
  TestVec[k++][l][1] = computeWd(test_pt(0),test_pt(1),i).col(1);
  }
  l++; k =0;

  test_pt(1) = 2*M_PI;
  cout << "Punkt ist bei phi=2*PI" << endl;
  for(int i=1;i<=degDev;i++)
  {
  TestVec[k][l][0] = computeWd(test_pt(0),test_pt(1),i).col(0);
  TestVec[k++][l][1] = computeWd(test_pt(0),test_pt(1),i).col(1);
  }


  for(int i=0;i<degDev;i++)
  {
  cout << i+1 << "-te Ableitung ..." << endl;
  for(int j=0;j<2;j++)
  {
  cout << "j=" << j << "(j=0...V-Richtung, j=1...U-Richtung)" << endl;
  cout << "Fehler der Punkte am Rande:    " << computeError(TestVec[i][0][j],TestVec[i][1][j]) << endl;
  }
  }*/


  //return Surface->ok();

#endif
}

void NurbsDisk2s::computeSurfaceVelocities() {
#ifdef HAVE_NURBS 
  PLib::Matrix<Point3Dd> Nodelist(nj+degU,nr+1); // list of Cartesian node-velocities for the nurbs interpolation (2*degU+1 is used for the 2 outline of the surface in azimuthal direction)

  // gets velocities from body for the interpolation
  for(int i=0; i<nr+1; i++) {  
    for(int j=0; j<nj; j++) {  
      ContourPointData cp(i*nj+j);
      static_cast<BodyFlexible2s13Disk*>(parent)->updateKinematicsForFrame(cp,velocity);
      Nodelist(j,i) = PLib::Point3Dd(cp.getFrameOfReference().getVelocity(0),cp.getFrameOfReference().getVelocity(1),cp.getFrameOfReference().getVelocity(2));
    }
    for(int j=0;j<degU;j++) { // expands the surface addicted to the degree in azimuthal direction
      Nodelist(nj+j,i) = Nodelist(j,i);
    }
  }

  SurfaceVelocities = new PlNurbsSurfaced;
  SurfaceVelocities->globalInterpClosedU_OwnKnotVecs(Nodelist, *uVec, *vVec, *uvec, *vvec, degU, degV);
#endif
}

Vec NurbsDisk2s::getCtrlPts(const int u, const int v) {
  Vec TmpVec(3);

  TmpVec(0) = Surface->ctrlPnts(u,v).x();
  TmpVec(1) = Surface->ctrlPnts(u,v).y();
  TmpVec(2) = Surface->ctrlPnts(u,v).z();

  return TmpVec;
}

Vec NurbsDisk2s::getUVec() {
  Vec TmpUVec(Surface->knotU().rows());


  for (int i=0;i<Surface->knotU().rows();i++)
    TmpUVec(i)=Surface->knotU(i);

  return TmpUVec; 
}

Vec NurbsDisk2s::getVVec() {
  Vec TmpVVec(Surface->knotV().rows());

  for (int i=0;i<Surface->knotV().rows();i++)
    TmpVVec(i)=Surface->knotV(i);

  return TmpVVec; 
}

  int NurbsDisk2s::testInsideBounds(const Vec &s) {
    if( (s(0) < Ri) || (s(0)>Ra) )
      return 0;

    if( (s(1) < 0) || (s(1) > 2*M_PI))
      return 0;

    return 1;
  }

double NurbsDisk2s::computeError(const Vec &Vec1, const Vec &Vec2) {
  Vec error(3);
  error(0) = Vec1(0)-Vec2(0);
  error(1) = Vec1(1)-Vec2(1);
  error(2) = Vec1(2)-Vec2(2);

  return nrm2(error);
}

void NurbsDisk2s::dispVec(const Vec& Vec) {
  cout << "x1= " << Vec(0) << "    x2= " << Vec(1) << "    x3= " << Vec(2) << endl;
}

