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

#include "mbsim/flexible_body/flexible_body_2s_13_mfr_mindlin.h"
#include "mbsim/contours/nurbs_disk_2s.h"
#include "mbsim/dynamic_system.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/nurbsdisk.h"
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  Mat condenseMatrixRows_cd(Mat A,Index I) {
    Mat B(A.rows()-(I.end()-I.start()+1),A.cols());
    Index upperPart(0,I.start()-1);
    Index lowerPartA(I.end()+1,A.rows()-1);
    Index lowerPartB(I.start(),B.rows()-1);
    Index AllCols(0,A.cols()-1);

    B(upperPart,AllCols) = A(upperPart,AllCols); // upper
    B(lowerPartB,AllCols) = A(lowerPartA,AllCols); // lower
    return B;
  }

  SymMat condenseMatrix_cd(SymMat A,Index I) {
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

  double ArcTan_cd(double x,double y) {
    double phi;
    phi = atan2(y,x);

    if(phi < 0.) phi += 2*M_PI;
    return phi;
  }

  FlexibleBody2s13MFRMindlin::FlexibleBody2s13MFRMindlin(const string &name) : FlexibleBodyContinuum<Vec>(name),Elements(0),NodeDofs(3),RefDofs(2),E(0.),nu(0.),rho(0.),d(3,INIT,0.),Ri(0),Ra(0),dr(0),dj(0),m0(0),J0(0),degV(3),degU(5),drawDegree(0),currentElement(0),nr(0),nj(0),Nodes(0),Dofs(0),LType(innerring) {
#ifdef HAVE_NURBS
    contour = new NurbsDisk2s("SurfaceContour");  
    Body::addContour(contour);
#else
    cout << "WARNING (FlexibleBody2s13MFRMindlin::FlexibleBody2s13MFRMindlin): No NURBS library installed!" << endl;
#endif

    // frame in axis
    Vec s(2,fmatvec::INIT,0.);
    addFrame("COG",s);
  }

  void FlexibleBody2s13MFRMindlin::updateh(double t) {
    // update positions and velocities
    qext = Jext * q;
    uext = Jext * u;

    h = -K*q;
    hObject = -K*q;
  }

  void FlexibleBody2s13MFRMindlin::updateM(double t) {
    M = MSave.copy(); 
  }

  void FlexibleBody2s13MFRMindlin::updatedhdz(double t) {
    updateh(t);
    for(int i=0;i<dhdq.cols();i++) 
      for(int j=0;j<dhdq.rows();j++) 
        dhdq(i,j) = -K(i,j);
  }

  void FlexibleBody2s13MFRMindlin::updateStateDependentVariables(double t) {
    FlexibleBodyContinuum<Vec>::updateStateDependentVariables(t);

#ifdef HAVE_NURBS
    contour->computeSurface(); 
    contour->computeSurfaceVelocities();
    contour->computeSurfaceJacobians();
#endif
  }

  void FlexibleBody2s13MFRMindlin::BuildElements() {
    for(int i=0;i<Elements;i++) {
      //  4--------3
      //  |        |
      //  1--------2
      // radial and azimuthal coordinates of the FE [ElementalNodes(r1,j1,r2,j2)]
      // r1 and j1 are defined with node 1, r2 and j2 with node 3
      ElementalNodes[i](0,1) << trans(NodeCoordinates.row(ElementNodeList(i,0))); // node 1
      ElementalNodes[i](2,3) << trans(NodeCoordinates.row(ElementNodeList(i,2))); // node 3

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

  void FlexibleBody2s13MFRMindlin::updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff, Frame *frame) {
    if(cp.getContourParameterType() == CONTINUUM) { // frame on continuum
#ifdef HAVE_NURBS
      contour->updateKinematicsForFrame(cp,ff);
#endif
    }
    else if(cp.getContourParameterType() == NODE) { // frame on node
      const int &node = cp.getNodeNumber();

      Vec tmp(3,NONINIT);
      if(ff==position || ff==position_cosy || ff==all) {
        tmp(0) = cos(qext(1)) * (NodeCoordinates(node,0) * cos(NodeCoordinates(node,1)) + qext(RefDofs+(node+1)*NodeDofs-1)*(computeThickness(NodeCoordinates(node,0)))/2.) - sin(qext(1)) * (NodeCoordinates(node,0) * sin(NodeCoordinates(node,1)) + qext(RefDofs+(node+1)*NodeDofs-2)*(computeThickness(NodeCoordinates(node,0)))/2.);
        tmp(1) = sin(qext(1)) * (NodeCoordinates(node,0) * cos(NodeCoordinates(node,1)) + qext(RefDofs+(node+1)*NodeDofs-1)*(computeThickness(NodeCoordinates(node,0)))/2.) + cos(qext(1)) * (NodeCoordinates(node,0) * sin(NodeCoordinates(node,1)) + qext(RefDofs+(node+1)*NodeDofs-2)*(computeThickness(NodeCoordinates(node,0)))/2.);
        tmp(2) = qext(0) + qext(RefDofs+node*NodeDofs) + (computeThickness(NodeCoordinates(node,0)))/2.; 
        cp.getFrameOfReference().setPosition(frameOfReference->getPosition() + frameOfReference->getOrientation() * tmp);
      }

      if(ff==firstTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) throw new MBSimError("ERROR(FlexibleBody2s13MFRMindlin::updateKinematicsForFrame): Not implemented!");
      if(ff==normal || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) throw new MBSimError("ERROR(FlexibleBody2s13MFRMindlin::updateKinematicsForFrame): Not implemented!");
      if(ff==secondTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) throw new MBSimError("ERROR(FlexibleBody2s13MFRMindlin::updateKinematicsForFrame): Not implemented!");

      if(ff==velocity || ff==velocities || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
        tmp(0) = uext(RefDofs+(node+1)*NodeDofs-1)*(computeThickness(NodeCoordinates(node,0)))/2.*cos(qext(1)) - uext(RefDofs+(node+1)*NodeDofs-2)*(computeThickness(NodeCoordinates(node,0)))/2.*sin(qext(1)) - uext(1)*sin(qext(1))*(qext(RefDofs+(node+1)*NodeDofs-1)*(computeThickness(NodeCoordinates(node,0)))/2. + NodeCoordinates(node,0)*cos(NodeCoordinates(node,1))) - uext(1)*cos(qext(1))*(qext(RefDofs+(node+1)*NodeDofs-2)*(computeThickness(NodeCoordinates(node,0)))/2. + NodeCoordinates(node,0)*sin(NodeCoordinates(node,1)));
        tmp(1) = uext(RefDofs+(node+1)*NodeDofs-2)*(computeThickness(NodeCoordinates(node,0)))/2.*cos(qext(1)) + uext(RefDofs+(node+1)*NodeDofs-1)*(computeThickness(NodeCoordinates(node,0)))/2.*sin(qext(1)) + uext(1)*cos(qext(1))*(qext(RefDofs+(node+1)*NodeDofs-1)*(computeThickness(NodeCoordinates(node,0)))/2. + NodeCoordinates(node,0)*cos(NodeCoordinates(node,1))) - uext(1)*sin(qext(1))*(qext(RefDofs+(node+1)*NodeDofs-2)*(computeThickness(NodeCoordinates(node,0)))/2. + NodeCoordinates(node,0)*sin(NodeCoordinates(node,1)));
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
    else throw new MBSimError("ERROR(FlexibleBody2s13MFRMindlin::updateKinematicsForFrame): ContourPointDataType should be 'NODE' or 'CONTINUUM'");

    if(frame!=0) { // frame should be linked to contour point data
      frame->setPosition       (cp.getFrameOfReference().getPosition());
      frame->setOrientation    (cp.getFrameOfReference().getOrientation());
      frame->setVelocity       (cp.getFrameOfReference().getVelocity());
      frame->setAngularVelocity(cp.getFrameOfReference().getAngularVelocity());
    }
  }

  void FlexibleBody2s13MFRMindlin::updateJacobiansForFrame(ContourPointData &cp, Frame *frame) {
    Index Wwidth(0,3); // number of columns for Wtmp appears here also as column number
    Mat Wext(Dofs,4);

    if(cp.getContourParameterType() == CONTINUUM) { // force on continuum
      Vec alpha = cp.getLagrangeParameterPosition();

      if(nrm2(alpha)==0.) { // center of gravity
        Wext(0,0) = 1.;
        Wext(1,1) = 1.;
      } 
      else { // on the disk
        BuildElement(alpha);

        /* Jacobian of element */
        Mat Wtmp = static_cast<FiniteElement2s13MFRMindlin*>(discretization[currentElement])->JGeneralized(ElementalNodes[currentElement],alpha);

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
    else throw new MBSimError("ERROR(FlexibleBody2s13MFRMindlin::updateJacobiansForFrame): ContourPointDataType should be 'NODE' or 'CONTINUUM'");

    // condensation
    Mat Jacobian = condenseMatrixRows_cd(Wext,ILocked);

    // transformation
    cp.getFrameOfReference().setJacobianOfTranslation(frameOfReference->getOrientation().col(2)*trans(Jacobian(0,0,qSize-1,0)));
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

  void FlexibleBody2s13MFRMindlin::init(InitStage stage) {
    if(stage==resize) {
      FlexibleBodyContinuum<Vec>::init(stage);
      assert(nr>0); // at least on radial row
      assert(nj>1); // at least two azimuthal elements

      for(int i=0;i<Elements;i++) {
        discretization.push_back(new FiniteElement2s13MFRMindlin(E,nu,rho,d(0),d(1),d(2)));
        qElement.push_back(Vec(discretization[0]->getqSize(),INIT,0.)); //TODO: discretization[i]?
        uElement.push_back(Vec(discretization[0]->getuSize(),INIT,0.));
        ElementalNodes.push_back(Vec(4,INIT,0.));
      }

      // condensation
      switch(LType) {
        case innerring: // 0: innerring
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
        for(int j=0; j<nj; j++) {
          // NodeCoordinates(radial,azimuthal)
          // node number increases azimuthally from the inner to the outer ring
          NodeCoordinates(j+i*nj,0) = Ri+dr*i;
          NodeCoordinates(j+i*nj,1) = 0. +dj*j;

          // ElementNodeList(node 1,node 2,node 3,node 4)
          // element number increases azimuthally from the inner to the outer ring
          if(i<nr && j<nj-1) {
            //ElementNodeList(No. of Element, local nodenumber) = global nodenumber;
            ElementNodeList(j+i*nj,0) =  j   + i*nj     ; // elementnode 1
            ElementNodeList(j+i*nj,1) =  j   + i*nj + nj; // elementnode 2
            ElementNodeList(j+i*nj,2) =  j+1 + i*nj + nj; // elementnode 3
            ElementNodeList(j+i*nj,3) =  j+1 + i*nj     ; // elementnode 4
          }
          else if(i<nr && j==nj-1) { // ring closure
            ElementNodeList(j+i*nj,0) =  j   + i*nj     ; // elementnode 1
            ElementNodeList(j+i*nj,1) =  j   + i*nj + nj; // elementnode 2
            ElementNodeList(j+i*nj,2) =  j+1 + i*nj     ; // elementnode 3
            ElementNodeList(j+i*nj,3) =  j+1 + i*nj - nj; // elementnode 4
          }
        }
      }

#ifdef HAVE_NURBS
      // borders of contour parametrisation 
      // beginning 
      Vec alphaS(2); 
      alphaS(0) =  Ri; // radius
      alphaS(1) = 0.; // angle

      // end 
      Vec alphaE(2);
      alphaE(0) =     Ra; // radius
      alphaE(1) = 2*M_PI; // angle

      contour->setAlphaStart(alphaS);  contour->setAlphaEnd(alphaE);
#endif

      qext = Jext * q0;
      uext = Jext * u0;

      initMatrices(); // calculate constant stiffness matrix and the constant parts of the mass-matrix

    }
    if(stage==MBSim::plot) {
      updatePlotFeatures(parent); 

      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
#ifdef HAVE_NURBS
        if(getPlotFeature(openMBV)==enabled) {
          OpenMBV::NurbsDisk *Diskbody = new OpenMBV::NurbsDisk;

          drawDegree = 30/nj;
          Diskbody->setStaticColor(0.3);
          Diskbody->setMinimalColorValue(0.);
          Diskbody->setMaximalColorValue(1.);
          Diskbody->setDrawDegree(drawDegree);
          Diskbody->setRadii(Ri,Ra);

          float *openmbvUVec = new float[nj+1+2*degU];
          float *openmbvVVec = new float[nr+2+degV];
          for(int i=0;i<nj+1+2*degU;i++) openmbvUVec[i]=contour->getUVector()(i);
          for(int i=0;i<nr+1+degV+1;i++) openmbvVVec[i]=contour->getVVector()(i);

          Diskbody->setKnotVecAzimuthal(openmbvUVec);
          Diskbody->setKnotVecRadial(openmbvVVec);

          Diskbody->setElementNumberRadial(nr);
          Diskbody->setElementNumberAzimuthal(nj);

          Diskbody->setInterpolationDegreeRadial(degV);  
          Diskbody->setInterpolationDegreeAzimuthal(degU);
          openMBVBody = Diskbody;
        }
#endif
#endif
        FlexibleBodyContinuum<Vec>::init(stage);
      }
    }
    else
      FlexibleBodyContinuum<Vec>::init(stage);

#ifdef HAVE_NURBS
    contour->initContourFromBody(stage); // initialize contour
#endif
  }

  void FlexibleBody2s13MFRMindlin::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
#ifdef HAVE_NURBS
      if(getPlotFeature(openMBV)==enabled && openMBVBody) {
        vector<double> data;
        data.push_back(t); //time

        for(int i=0;i<nr+1; i++) {
          for(int j=0;j<nj+degU;j++) {
            data.push_back(contour->getControlPoints(j,i)(0)); //global x-coordinate
            data.push_back(contour->getControlPoints(j,i)(1)); //global y-coordinate
            data.push_back(contour->getControlPoints(j,i)(2)); //global z-coordinate
          }
        }

        ContourPointData cp;
        cp.getLagrangeParameterPosition() = Vec(2,NONINIT);

        //inner ring
        for(int i=0;i<nj; i++) {
          for(int j=0;j<drawDegree;j++) {
            cp.getLagrangeParameterPosition()(1) = 2*M_PI*(i*drawDegree+j)/(nj*drawDegree);
            cp.getLagrangeParameterPosition()(0) = Ri;
            contour->updateKinematicsForFrame(cp, position);
            Vec pos = cp.getFrameOfReference().getPosition();

            data.push_back(pos(0)); //global x-coordinate
            data.push_back(pos(1)); //global y-coordinate
            data.push_back(pos(2)); //global z-coordinate
          }
        }

        //outer Ring
        for(int i=0;i<nj; i++) {
          for(int j=0;j<drawDegree;j++) {
            cp.getLagrangeParameterPosition()(0) = Ra;
            cp.getLagrangeParameterPosition()(1) = 2*M_PI*(i*drawDegree+j)/(nj*drawDegree);
            contour->updateKinematicsForFrame(cp, position);
            Vec pos = cp.getFrameOfReference().getPosition(); 

            data.push_back(pos(0)); //global x-coordinate
            data.push_back(pos(1)); //global y-coordinate
            data.push_back(pos(2)); //global z-coordinate
          }
        }

        cp.getLagrangeParameterPosition()(0) = 0.;
        cp.getLagrangeParameterPosition()(1) = 0.;
        contour->updateKinematicsForFrame(cp,position_cosy);  // kinematics of the center of gravity of the disk (TODO frame feature)

        data.push_back(cp.getFrameOfReference().getPosition()(0)-cp.getFrameOfReference().getOrientation()(0,2)*d(0)*0.5); //global x-coordinate
        data.push_back(cp.getFrameOfReference().getPosition()(1)-cp.getFrameOfReference().getOrientation()(1,2)*d(0)*0.5); //global y-coordinate
        data.push_back(cp.getFrameOfReference().getPosition()(2)-cp.getFrameOfReference().getOrientation()(2,2)*d(0)*0.5); //global z-coordinate

        for(int i=0;i<3;i++)
          for(int j=0;j<3;j++)
            data.push_back(cp.getFrameOfReference().getOrientation()(i,j));

        ((OpenMBV::NurbsDisk*)openMBVBody)->append(data);
      }
#endif
#endif
    }
    FlexibleBodyContinuum<Vec>::plot(t,dt);
    cout << endl;
  }

  void FlexibleBody2s13MFRMindlin::setNumberElements(int nr_, int nj_) {
    nr = nr_; nj = nj_; 
    degV = min(degV, nr); // radial adaptation of spline degree to have correct knot vector
    degU = min(degU, nj); // azimuthal adaptation of spline degree to have correct knot vector
    Elements = nr*nj;
    Nodes = (nr+1) * nj;

    Dofs  = RefDofs + Nodes*NodeDofs;

    qSize = Dofs - NodeDofs*nj; // missing one node row because of bearing
    uSize[0] = qSize;
    uSize[1] = qSize; // TODO

    qext = Vec(Dofs); 
    uext = Vec(Dofs); 

    q0.resize(qSize);
    u0.resize(uSize[0]);
  }

  Vec FlexibleBody2s13MFRMindlin::transformCW(const Vec& WrPoint) {
    Vec CrPoint(WrPoint.size());

    double &alpha = q(1);

    const double &xt = WrPoint(0);
    const double &yt = WrPoint(1);

    CrPoint(0) = sqrt( xt*xt + yt*yt );
    CrPoint(1) = ArcTan_cd(xt*cos(alpha) + yt*sin(alpha),yt*cos(alpha) - xt*sin(alpha));
    CrPoint(2) = WrPoint(2);

    return CrPoint;
  }

  void FlexibleBody2s13MFRMindlin::BuildElement(const Vec &s) {
    assert(Ri <= s(0)); // is the input on the disk?
    assert(Ra >= s(0)); 

    currentElement = int( (s(0)-Ri)/dr )*nj + int( s(1)/dj ); // which element is involved?
  }

  void FlexibleBody2s13MFRMindlin::initMatrices() {
    BuildElements();

    // initialising of mass and stiffness matrix
    SymMat Mext(Dofs,INIT,0.);
    SymMat Kext(Dofs,INIT,0.);
    

    //for Testing ...
    SymMat K_old = Kext.copy();

    SymMat KEl;

    // element loop
    for(int i=0;i<Elements;i++) {
      double r1 = ElementalNodes[i](0);
      double r2 = ElementalNodes[i](2);
      static_cast<FiniteElement2s13MFRMindlin*>(discretization[i])->computeConstantSystemMatrices(ElementalNodes[i],computeThickness(r1),computeThickness(r2));

      // definition of variables for element matrices
      SymMat Mplatte = discretization[i]->getM();
      SymMat Kplatte = static_cast<FiniteElement2s13MFRMindlin*>(discretization[i])->getK();


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
      
      /*Test to proof, wehter the Element-Node-List is correct*/
      cout << "Element-Nummer: " << i << endl;
      cout.precision(8);
      cout.setf(ios::scientific);
      cout << "   ";
      for(int j=2; j<Kext.size(); j++){
        if(j<10) cout << "       " << j << "        ";
        else     cout << "       " << j << "       ";
      }
      for(int k=2; k<Kext.size(); k++)
      {
        //Zeilennummer
        if (k<10) cout << endl <<  k << "  ";
        else      cout << endl <<  k << " ";
        
        for(int j=2; j<Kext.size(); j++){
          if     (Kext(k,j)-K_old(k,j)>0)    cout << "  " << Kext(k,j)-K_old(k,j);
          else if(Kext(k,j)-K_old(k,j)==0)   cout << "        0       "; 
          else if(Kext(k,j)-K_old(k,j)<0)    cout << " "  <<Kext(k,j)-K_old(k,j);
        }
      }

      cout << endl;
      
      K_old = Kext.copy();

      /*END testing*/
    }

    // condensation
    MSave = condenseMatrix_cd(Mext,ILocked);
    K = condenseMatrix_cd(Kext,ILocked);

    /*Test Stiffness-Matrix*/
    //cout << "KEL: " << KEl << endl;

    Vec F_test(K.size()-2,INIT, 0.);

    F_test(nj*(nr-1)*3) = -10000;
    
    SymMat Ktmp(K.size()-2,INIT,0.);
    for(int i=0;i<Ktmp.size();i++)
      for(int j=i;j<Ktmp.size();j++)
        Ktmp(i,j) = K(i+2,j+2);

    //cout << "Steifigkeitsmatrix: " << Ktmp << endl;
    //cout << "Kraftlastvektor: " << F_test << endl;

    Vec q_test = slvLL(Ktmp, F_test);

    /*Ausgabe als Matrix für maxima*/
    //cout << "qf=matrix([" ;
    //for(int i=0; i<q_test.size(); i++)
    //  cout << q_test(i) << ",";

    //cout << "]);" << endl;

    /*Ausgabe der Knotendurchsenkungen in x-Richtung (y=0)*/
    for(int i=0; i<nr; i++)
      cout << q_test(i*nj*3) << endl;

    /************************/

    // masse and inertia of shaft
    MSave(0,0) += m0;
    MSave(1,1) += J0;

    // LU-decomposition of M
    LLM = facLL(MSave);
  }

  double FlexibleBody2s13MFRMindlin::computeThickness(const double &r_) {
    return d(0) + d(1)*r_ + d(2)*r_*r_; // quadratic paramatrisation
  }

}

