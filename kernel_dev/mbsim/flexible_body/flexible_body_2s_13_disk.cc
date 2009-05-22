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
#define FMATVEC_DEEP_COPY
#define FMATVEC_NO_INITIALIZATION
#define FMATVEC_NO_BOUNDS_CHECK

#include "mbsim/flexible_body/flexible_body_2s_13_disk.h"
#include "mbsim/flexible_body/finite_elements/finite_element_2s_13_disk.h"
#include "mbsim/contours/nurbs_disk_2s.h"
#include <mbsim/dynamic_system.h>

using namespace std;
using namespace fmatvec;

namespace MBSim {

  Mat condenseMatrixRows(Mat A,Index I) {
    Mat B(A.rows()-(I.end()-I.start()+1),A.cols());
    Index upperPart(0,I.start()-1);
    Index lowerPartA(I.end()+1,A.rows()-1);
    Index lowerPartB(I.start(),B.rows()-1);
    Index AllCols(0,A.cols()-1);

    B(upperPart,AllCols) = A(upperPart,AllCols); // upper
    B(lowerPartB,AllCols) = A(lowerPartA,AllCols); // lower
    return B;
  }

  SymMat condenseMatrix(SymMat A,Index I) {
    // build size of result matrix
    SymMat B(A.size()-(I.end()-I.start()+1));
    Index upperPart(0,I.start()-1);
    Index lowerPartA(I.end()+1,A.size()-1);
    Index lowerPartB(I.start(),B.size()-1);

    // assemble result matrix
    B(upperPart)            << A(upperPart);            // upper left
    B(upperPart,lowerPartB) << A(upperPart,lowerPartA); // upper right
    B(lowerPartB)           << A(lowerPartA);           // lower right
    return B;
  }

  double ArcTan(double x,double y) {
    double phi;
    phi = atan2(y,x);

    if(phi < 0.) phi += 2*M_PI;
    return phi;
  }

  FlexibleBody2s13Disk::FlexibleBody2s13Disk(const string &name) : FlexibleBodyContinuum<Vec>(name),Elements(0),NodeDofs(3),RefDofs(2),E(0.),nu(0.),rho(0.),di(3,INIT,0.),da(3,INIT,0.),Ri(0),Ra(0),dr(0),dj(0),m0(0),J0(0),currentElement(0),nr(0),nj(0),Nodes(0),Dofs(0),LType(innerring) {
    degU=3;degV=3;
    //contour = new NurbsDisk2s("SurfaceContour");  
    //Body::addContour(contour);

    // frame in axis
    Vec s(2,fmatvec::INIT,0.);
    addFrame("COG",s);
  }

  void FlexibleBody2s13Disk::updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff, Frame *frame) {
    if(cp.getContourParameterType() == CONTINUUM) { // frame on continuum
      contour->updateKinematicsForFrame(cp,ff);
    }
    else if(cp.getContourParameterType() == NODE) { // frame on node
      const int &node = cp.getNodeNumber();

      Vec tmp(3,NONINIT);
      if(ff==position || ff==position_cosy || ff==all) {
        tmp(0) = cos(qext(1)) * (NodeCoordinates(node,0) * cos(NodeCoordinates(node,1)) + qext(RefDofs+(node+1)*NodeDofs-1)*(computeThickness(NodeCoordinates(node,0))(0))/2.) - sin(qext(1)) * (NodeCoordinates(node,0) * sin(NodeCoordinates(node,1)) + qext(RefDofs+(node+1)*NodeDofs-2)*(computeThickness(NodeCoordinates(node,0))(0))/2.);
        tmp(1) = sin(qext(1)) * (NodeCoordinates(node,0) * cos(NodeCoordinates(node,1)) + qext(RefDofs+(node+1)*NodeDofs-1)*(computeThickness(NodeCoordinates(node,0))(0))/2.) + cos(qext(1)) * (NodeCoordinates(node,0) * sin(NodeCoordinates(node,1)) + qext(RefDofs+(node+1)*NodeDofs-2)*(computeThickness(NodeCoordinates(node,0))(0))/2.);
        tmp(2) = qext(0) + qext(RefDofs+node*NodeDofs) + (computeThickness(NodeCoordinates(node,0))(0))/2.; 
        cp.getFrameOfReference().setPosition(frameOfReference->getPosition() + frameOfReference->getOrientation() * tmp);
      }

      if(ff==firstTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) throw new MBSimError("ERROR(FlexibleBody2s13Disk::updateKinematicsForFrame): Not implemented!");
      if(ff==normal || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) throw new MBSimError("ERROR(FlexibleBody2s13Disk::updateKinematicsForFrame): Not implemented!");
      if(ff==secondTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) throw new MBSimError("ERROR(FlexibleBody2s13Disk::updateKinematicsForFrame): Not implemented!");

      if(ff==velocity || ff==velocities || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
        tmp(0) = uext(RefDofs+(node+1)*NodeDofs-1)*(computeThickness(NodeCoordinates(node,0))(0))/2.*cos(qext(1)) - uext(RefDofs+(node+1)*NodeDofs-2)*(computeThickness(NodeCoordinates(node,0))(0))/2.*sin(qext(1)) - uext(1)*sin(qext(1))*(qext(RefDofs+(node+1)*NodeDofs-1)*(computeThickness(NodeCoordinates(node,0))(0))/2. + NodeCoordinates(node,0)*cos(NodeCoordinates(node,1))) - uext(1)*cos(qext(1))*(qext(RefDofs+(node+1)*NodeDofs-2)*(computeThickness(NodeCoordinates(node,0))(0))/2. + NodeCoordinates(node,0)*sin(NodeCoordinates(node,1)));
        tmp(1) = uext(RefDofs+(node+1)*NodeDofs-2)*(computeThickness(NodeCoordinates(node,0))(0))/2.*cos(qext(1)) + uext(RefDofs+(node+1)*NodeDofs-1)*(computeThickness(NodeCoordinates(node,0))(0))/2.*sin(qext(1)) + uext(1)*cos(qext(1))*(qext(RefDofs+(node+1)*NodeDofs-1)*(computeThickness(NodeCoordinates(node,0))(0))/2. + NodeCoordinates(node,0)*cos(NodeCoordinates(node,1))) - uext(1)*sin(qext(1))*(qext(RefDofs+(node+1)*NodeDofs-2)*(computeThickness(NodeCoordinates(node,0))(0))/2. + NodeCoordinates(node,0)*sin(NodeCoordinates(node,1)));
        tmp(2) = uext(0) + uext(RefDofs+node*NodeDofs);
        cp.getFrameOfReference().setVelocity(frameOfReference->getOrientation() * tmp);
      }

      if(ff==angularVelocity || ff==velocities || ff==velocities_cosy || ff==all) {
        tmp(0) = uext(RefDofs+(node+1)*NodeDofs-2) * (-cos(NodeCoordinates(node,1)) * sin(qext(1)) - cos(qext(1)) * sin(NodeCoordinates(node,1))) + uext(RefDofs+(node+1)*NodeDofs-1) * (cos(qext(1)) * cos(NodeCoordinates(node,1)) - sin(qext(1)) * sin(NodeCoordinates(node,1)));
        tmp(1) = uext(RefDofs+(node+1)*NodeDofs-1) * (cos(NodeCoordinates(node,1)) * sin(qext(1)) + cos(qext(1)) * sin(NodeCoordinates(node,1))) + uext(RefDofs+(node+1)*NodeDofs-2) * (cos(qext(1)) * cos(NodeCoordinates(node,1)) - sin(qext(1)) * sin(NodeCoordinates(node,1)));
        tmp(2) = uext(1);
        cp.getFrameOfReference().setAngularVelocity(frameOfReference->getOrientation() * tmp);
      }
    }
    else throw new MBSimError("ERROR(FlexibleBody2s13Disk::updateKinematicsForFrame): ContourPointDataType should be 'NODE' or 'CONTINUUM'");

    if(frame!=0) { // frame should be linked to contour point data
      frame->setPosition       (cp.getFrameOfReference().getPosition());
      frame->setOrientation    (cp.getFrameOfReference().getOrientation());
      frame->setVelocity       (cp.getFrameOfReference().getVelocity());
      frame->setAngularVelocity(cp.getFrameOfReference().getAngularVelocity());
    }
  }

  void FlexibleBody2s13Disk::updateJacobiansForFrame(ContourPointData &cp, Frame *frame) {
    Index Wwidth(0,3); // number of columns for Wtmp appears here also as column number
    Mat Wext(Dofs,4);

    if(cp.getContourParameterType() == CONTINUUM) { // force on continuum
      Vec alpha = cp.getLagrangeParameterPosition();

      if(nrm2(alpha) == 0.) { // center of gravity
        Wext(0,0) = 1.;
        Wext(1,1) = 1.;
      } 
      else { // on the disk
        BuildElement(alpha);

        /* Jacobian of element */
        Mat Wtmp = static_cast<FiniteElement2s13Disk*>(discretization[currentElement])->JGeneralized(ElementalNodes[currentElement],alpha);

        /* Jacobian of disk */
        Wext(Index(0,RefDofs-1),Wwidth) = Wtmp(Index(0,RefDofs-1),Wwidth);
        Wext(Index(RefDofs+ElementNodeList(currentElement,0)*NodeDofs,RefDofs+(ElementNodeList(currentElement,0)+1)*NodeDofs-1),Wwidth) = Wtmp(Index(RefDofs,RefDofs+NodeDofs-1),Wwidth);
        Wext(Index(RefDofs+ElementNodeList(currentElement,1)*NodeDofs,RefDofs+(ElementNodeList(currentElement,1)+1)*NodeDofs-1),Wwidth) = Wtmp(Index(RefDofs+NodeDofs,RefDofs+2*NodeDofs-1),Wwidth);
        Wext(Index(RefDofs+ElementNodeList(currentElement,2)*NodeDofs,RefDofs+(ElementNodeList(currentElement,2)+1)*NodeDofs-1),Wwidth) = Wtmp(Index(RefDofs+2*NodeDofs,RefDofs+3*NodeDofs-1),Wwidth);
        Wext(Index(RefDofs+ElementNodeList(currentElement,3)*NodeDofs,RefDofs+(ElementNodeList(currentElement,3)+1)*NodeDofs-1),Wwidth) = Wtmp(Index(RefDofs+3*NodeDofs,RefDofs+4*NodeDofs-1),Wwidth);
      }
    }

    else if(cp.getContourParameterType() == NODE) { // force on node
      int node  = cp.getNodeNumber();

      /* Jacobian of element */
      Mat Wtmp(5,4,INIT,0.); // initialising Ref + 1 Node

      // translation
      Wtmp(0,0) = 1; // ref
      Wtmp(2,0) = 1; // node

      // rotation
      Wtmp(1,3) =  1; // ref
      Wtmp(3,1) = -sin(qext(1) + NodeCoordinates(node,1)); // node
      Wtmp(3,2) =  cos(qext(1) + NodeCoordinates(node,1));
      Wtmp(4,1) =  cos(qext(1) + NodeCoordinates(node,1));
      Wtmp(4,2) =  sin(qext(1) + NodeCoordinates(node,1));

      /* Jacobian of disk */
      // reference 
      Wext(Index(0,RefDofs-1),Wwidth) = Wtmp(Index(0,RefDofs-1),Wwidth);			

      // nodes
      Wext(Index(RefDofs+node*NodeDofs,RefDofs+(node+1)*NodeDofs-1),Wwidth) = Wtmp(Index(RefDofs,RefDofs+NodeDofs-1),Wwidth);		
    }
    else throw new MBSimError("ERROR(FlexibleBody2s13Disk::updateJacobiansForFrame): ContourPointDataType should be 'NODE' or 'CONTINUUM'");

    // condensation
    Mat Jacobian = condenseMatrixRows(Wext,ILocked);
   
    // transformation
    cp.getFrameOfReference().setJacobianOfTranslation(frameOfReference->getOrientation()*trans(Jacobian(0,0,qSize-1,0)));
    cp.getFrameOfReference().setJacobianOfRotation   (frameOfReference->getOrientation()*trans(Jacobian(0,1,qSize-1,3)));

    // cp.getFrameOfReference().setGyroscopicAccelerationOfTranslation(TODO)
    // cp.getFrameOfReference().setGyroscopicAccelerationOfRotation(TODO)
    
    if(frame!=0) { // frame should be linked to contour point data
      frame->setJacobianOfTranslation(cp.getFrameOfReference().getJacobianOfTranslation());
      frame->setJacobianOfRotation   (cp.getFrameOfReference().getJacobianOfRotation());
      frame->setGyroscopicAccelerationOfTranslation(cp.getFrameOfReference().getGyroscopicAccelerationOfTranslation());
      frame->setGyroscopicAccelerationOfRotation   (cp.getFrameOfReference().getGyroscopicAccelerationOfRotation());
    }   
  }

  void FlexibleBody2s13Disk::setNumberElements(int nr_, int nj_) {
    nr = nr_; nj = nj_; 
    Elements = nr*nj;
    Nodes = (nr+1) * nj;

    Dofs  = RefDofs + Nodes*NodeDofs;

    qSize = Dofs - NodeDofs*nj; // missing one node row because of bearing
    uSize[0] = qSize;

    qext = Vec(Dofs); 
    uext = Vec(Dofs); 

    q0.resize(qSize);
    u0.resize(uSize[0]);
  }

  void FlexibleBody2s13Disk::init() {
    FlexibleBodyContinuum<Vec>::init();
    assert(nr>0); // at least on radial row
    assert(nj>1); // at least two azimuthal elements

    for(int i=0;i<Elements;i++) {
      discretization.push_back(new FiniteElement2s13Disk(E,nu,rho));
      qElement.push_back(Vec(discretization[0]->getSizeOfPositions(),INIT,0.));
      uElement.push_back(Vec(discretization[0]->getSizeOfVelocities(),INIT,0.));
      static_cast<FiniteElement2s13Disk*>(discretization[i])->setThickness(di,da);
      ElementalNodes.push_back(Vec(4,INIT,0.));
    }

    // condensation
    switch(LType) {
      case innerring:	// 0: innerring
        ILocked = Index(RefDofs,RefDofs+NodeDofs*nj-1);
        Jext = Mat(Dofs,qSize,INIT,0.);
        Jext(0,0,RefDofs-1,RefDofs-1) << DiagMat(RefDofs,INIT,1.);
        Jext(RefDofs+NodeDofs*nj,RefDofs,Dofs-1,qSize-1) << DiagMat(qSize - RefDofs,INIT,1.);
        break;

      case outerring: // 1: outerring
        ILocked = Index(qSize,Dofs-1);
        Jext = Mat(Dofs,qSize,INIT,0.);
        Jext(0,0,qSize-1,qSize-1) << DiagMat(qSize,INIT,1.);
        break;
    }

    dr = (Ra-Ri)/nr;
    dj =  2*M_PI/nj;

    NodeCoordinates = Mat(Nodes,2);
    ElementNodeList.resize(Elements,4);

    // mapping nodes - node coordinates - elements 
    for(int i=0; i<=nr; i++) {
      for(int j=0; j< nj; j++) {
        // NodeCoordinates(radial,azimuthal)
        // node number increases azimuthally from the inner to the outer ring
        NodeCoordinates(j+i*nj,0) = Ri+dr*i;
        NodeCoordinates(j+i*nj,1) = 0. +dj*j;

        // ElementNodeList(node 1,node 2,node 3,node 4)
        // element number increases azimuthally from the inner to the outer ring

        if(i<nr && j<nj-1) {
          ElementNodeList(j+i*nj,0) =  j    +  i   *nj; // node 1
          ElementNodeList(j+i*nj,1) = (j+1) +  i   *nj; // node 2
          ElementNodeList(j+i*nj,2) =  j    + (i+1)*nj; // node 3
          ElementNodeList(j+i*nj,3) = (j+1) + (i+1)*nj; // node 4
        }
        else if( i<nr && j==nj-1 ) { // ring closure
          ElementNodeList(j+i*nj,0) =  j    +  i   *nj; // node 1
          ElementNodeList(j+i*nj,1) =  0    +  i   *nj; // node 2
          ElementNodeList(j+i*nj,2) =  j    + (i+1)*nj; // node 3
          ElementNodeList(j+i*nj,3) =  0    + (i+1)*nj; // node 4
        }
      }
    }

    // beginning (?)
    Vec alphaS(2); 
    alphaS(0) =  Ri; // radius
    alphaS(1) = 0.; // angle
    // end (?)
    Vec alphaE(2);
    alphaE(0) =     Ra; // radius
    alphaE(1) = 2*M_PI; // angle

    contour->setAlphaStart(alphaS);  contour->setAlphaEnd(alphaE);

    qext = Jext * q0;
    uext = Jext * u0;

    // calculate constant mass- and stiffness matrix
    initMatrices();
    contour->init(degU, degV, nr, nj, Ri, Ra, 1); //flag = 1, because contourR is in positive z-direction of the local disk-coordinate system
    contour->computeSurface();
    contour->computeSurfaceVelocities();

  }

  void FlexibleBody2s13Disk::initPlot() {
    updatePlotFeatures(parent); // REMARK[TS] holt sich alle features, die in Elternklassen definiert sind

    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled) {
        OpenMBV::NurbsDisk *Diskbody = new OpenMBV::NurbsDisk;
        Diskbody->setElementNumberRadial(nr);
        Diskbody->setElementNumberAzimuthal(nj);
        Diskbody->setRadii(Ri,Ra);

        float *openmbvDI = new float[di.size()];
        float *openmbvDA = new float[da.size()];
        for(int i=0;i<di.size();i++) openmbvDI[i]=di(i);
        for(int i=0;i<da.size();i++) openmbvDA[i]=da(i);
        Diskbody->setThickness(openmbvDI,openmbvDA);

        float *openmbvUVec = new float[nj+1+2*degU];
        float *openmbvVVec = new float[nr+2+degV];
        for(int i=0;i<nj+1+2*degU;i++) openmbvUVec[i]=contour->getUVec()(i);
        for(int i=0;i<nr+1+degV+1;i++) openmbvVVec[i]=contour->getVVec()(i);

        Diskbody->setKnotVecAzimuthal(openmbvUVec);
        Diskbody->setKnotVecRadial(openmbvVVec);
        Diskbody->setStaticColor(0.5);

        openMBVBody = Diskbody;
      }
#endif
      FlexibleBodyContinuum<Vec>::initPlot();
    }
  }

  void FlexibleBody2s13Disk::initMatrices() {
    BuildElements();

    // initialising of mass and stiffness matrix
    SymMat Mext(Dofs,INIT,0.);
    SymMat Kext(Dofs,INIT,0.);

    // element loop
    for(int i=0;i< Elements;i++) {
      double r1 = ElementalNodes[i](0);
      double r2 = ElementalNodes[i](2);
      static_cast<FiniteElement2s13Disk*>(discretization[i])->computeConstantSystemMatrices(ElementalNodes[i],computeThickness(r1)(0),computeThickness(r2)(0));

      // definition of variables for element matrices
      SymMat Mplatte = discretization[i]->getMassMatrix();
      SymMat Kplatte = static_cast<FiniteElement2s13Disk*>(discretization[i])->getStiffnessMatrix();

      Index IRef(0,RefDofs-1);

      // reference components
      Mext(IRef) += Mplatte(IRef);
      Kext(IRef) += Kplatte(IRef);

      // nodes sort
      for(int k=0;k<4;k++) {
        // position in global matrix
        Index Ikges    (RefDofs+ElementNodeList(i,k)*NodeDofs, RefDofs+(ElementNodeList(i,k)+1)*NodeDofs-1);
        // position in element matrix
        Index Ikelement(RefDofs+                  k *NodeDofs, RefDofs+                   (k+1)*NodeDofs-1);

        // four nodes
        // ref,0 and 0,Ref
        Mext(Ikges,IRef ) += Mplatte(Ikelement,IRef);
        Kext(Ikges,IRef ) += Kplatte(Ikelement,IRef);

        // int n=k;
        Mext(Ikges) += Mplatte(Ikelement); // diagonal
        Kext(Ikges) += Kplatte(Ikelement); // diagonal
        for(int n=k+1;n<4;n++) {
          Mext(Ikges,Index(RefDofs+ElementNodeList(i,n)*NodeDofs, RefDofs+(ElementNodeList(i,n)+1)*NodeDofs-1)) += Mplatte(Ikelement,Index(RefDofs+n*NodeDofs, RefDofs+(n+1)*NodeDofs-1));
          Kext(Ikges,Index(RefDofs+ElementNodeList(i,n)*NodeDofs, RefDofs+(ElementNodeList(i,n)+1)*NodeDofs-1)) += Kplatte(Ikelement,Index(RefDofs+n*NodeDofs, RefDofs+(n+1)*NodeDofs-1));
        }
      }
    }

    // condensation
    M = condenseMatrix(Mext,ILocked);
    K = condenseMatrix(Kext,ILocked);

    // masse and inertia of shaft
    M(0,0) += m0;
    M(1,1) += J0;


    // LU-decomposition of M
    LLM = facLL(M);
  }

  void FlexibleBody2s13Disk::updateh(double t) {
    // update positions and velocities
    qext = Jext * q;
    uext = Jext * u;

    h = -K*q;
  }

  void FlexibleBody2s13Disk::BuildElement(const Vec &s) 
  {
    //outer-ring s(0) > Ra -> TODO: change s(0) of outer-ring
    assert(Ri <= s(0)); // is the input on the disk?
    // assert(Ra >= s(0)); 

    currentElement = int( (s(0)-Ri)/dr )*nj + int(s(1)/dj); // which element is involved?
    if( Ra<=s(0) ) currentElement-=nj;  //outer-ring-knots lead to existing element 
  }


  void FlexibleBody2s13Disk::BuildElements() {

    for(int i=0;i<Elements;i++) {
      //  3--------4
      //  |        |
      //  1--------2
      // radial and azimuthal coordinates of the FE [ElementalNodes(r1,j1,r2,j2)]
      // r1 and j1 are defined with node 1, r2 and j2 with node 4
      ElementalNodes[i](0,1) << trans(NodeCoordinates.row(ElementNodeList(i,0))); // node 1
      ElementalNodes[i](2,3) << trans(NodeCoordinates.row(ElementNodeList(i,3))); // node 4

      if(ElementalNodes[i](3) <= ElementalNodes[i](1)) { // ring closure
        ElementalNodes[i](3) += 2*M_PI; 
      }


      // mapping node dof position (w, a, b) from global vector to element vector
      // ref, node 1, node 2, node 3, node 4
      qElement[i](                 0,RefDofs           -1) << qext(                                    0,RefDofs                                  -1);
      qElement[i](RefDofs           ,RefDofs+  NodeDofs-1) << qext(RefDofs+ElementNodeList(i,0)*NodeDofs,RefDofs+(ElementNodeList(i,0)+1)*NodeDofs-1);
      qElement[i](RefDofs+  NodeDofs,RefDofs+2*NodeDofs-1) << qext(RefDofs+ElementNodeList(i,1)*NodeDofs,RefDofs+(ElementNodeList(i,1)+1)*NodeDofs-1);
      qElement[i](RefDofs+2*NodeDofs,RefDofs+3*NodeDofs-1) << qext(RefDofs+ElementNodeList(i,2)*NodeDofs,RefDofs+(ElementNodeList(i,2)+1)*NodeDofs-1);
      qElement[i](RefDofs+3*NodeDofs,RefDofs+4*NodeDofs-1) << qext(RefDofs+ElementNodeList(i,3)*NodeDofs,RefDofs+(ElementNodeList(i,3)+1)*NodeDofs-1);

      // mapping node dof velocity from global vector to element vector
      // ref, node 1, node 2, node 3, node 4
      uElement[i](                 0,RefDofs           -1) << uext(                                    0,RefDofs                                  -1);
      uElement[i](RefDofs           ,RefDofs+  NodeDofs-1) << uext(RefDofs+ElementNodeList(i,0)*NodeDofs,RefDofs+(ElementNodeList(i,0)+1)*NodeDofs-1);
      uElement[i](RefDofs+  NodeDofs,RefDofs+2*NodeDofs-1) << uext(RefDofs+ElementNodeList(i,1)*NodeDofs,RefDofs+(ElementNodeList(i,1)+1)*NodeDofs-1);
      uElement[i](RefDofs+2*NodeDofs,RefDofs+3*NodeDofs-1) << uext(RefDofs+ElementNodeList(i,2)*NodeDofs,RefDofs+(ElementNodeList(i,2)+1)*NodeDofs-1);
      uElement[i](RefDofs+3*NodeDofs,RefDofs+4*NodeDofs-1) << uext(RefDofs+ElementNodeList(i,3)*NodeDofs,RefDofs+(ElementNodeList(i,3)+1)*NodeDofs-1); 
    }

  }

  Vec FlexibleBody2s13Disk::computeThickness(const double &r_) {
    Vec d(3);

    // linear
    // double d = (da(2)-da(0))/(Ra-Ri)*r_;
    // double d = (d1*(r-r1)+d2*(r2-r))/(r2-r1);

    // quadratic
    d(1) = di(0) + di(1)*r_ + di(2)*r_*r_; // thickness inner ring
    d(2) = da(0) + da(1)*r_ + da(2)*r_*r_; // thickness outer ring

    d(0) = d(1) + d(2);

    return d;
  }

  void FlexibleBody2s13Disk::plot(double t, double dt) {
    Element::plot(t); 


    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled && openMBVBody) {
        vector<double> data;
        data.push_back(qext(0)); //time

        for(int i=0;i<nr+1; i++) 
        {
          for(int j=0;j<nj+degU;j++)
          {
            data.push_back(contour->getCtrlPts(j,i)(1)); //global x-coordinate
            data.push_back(contour->getCtrlPts(j,i)(2)); //global y-coordinate
            data.push_back(contour->getCtrlPts(j,i)(3)); //global z-coordinate
          }
        }
        ((OpenMBV::NurbsDisk*)openMBVBody)->append(data);
      }
#endif
    }
    FlexibleBodyContinuum<Vec>::plot(t,dt); //changed from double to Vec
  }

  Vec FlexibleBody2s13Disk::transformCW(const Vec& WrPoint) {
    Vec CrPoint(WrPoint.size());

    double &alpha = q(1);

    const double &xt = WrPoint(0);
    const double &yt = WrPoint(1);

    CrPoint(0) = sqrt( xt*xt + yt*yt );
    CrPoint(1) = ArcTan(xt*cos(alpha) + yt*sin(alpha),yt*cos(alpha) - xt*sin(alpha));

    return CrPoint;
  }

  void FlexibleBody2s13Disk::updateStateDependentVariables(double t) {
    FlexibleBodyContinuum<Vec>::updateStateDependentVariables(t);

    contour->computeSurface(); 
    contour->computeSurfaceVelocities();
  }

}

