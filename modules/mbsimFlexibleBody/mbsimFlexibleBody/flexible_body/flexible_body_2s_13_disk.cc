/* Copyright (C) 2004-2015 MBSim Development Team
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
 * Contact: thorsten.schindler@mytum.de
 */

#include<config.h>
#include "mbsimFlexibleBody/flexible_body/flexible_body_2s_13_disk.h"
#include "mbsimFlexibleBody/frames/frame_2s.h"
#include "mbsimFlexibleBody/frames/node_frame.h"
//#include "mbsimFlexibleBody/contours/nurbs_disk_2s.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/utils/eps.h"
#include "mbsim/utils/utils.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
//#include "openmbvcppinterface/nurbsdisk.h"
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  FlexibleBody2s13Disk::FlexibleBody2s13Disk(const string &name) : FlexibleBody2s13(name) {
    RefDofs = 2;
  }

  void FlexibleBody2s13Disk::BuildElements(double t) {
    for (int i = 0; i < Elements; i++) {
      //  ^ phi
      //  |
      //  |   2--------4
      //  |   |        |
      //  |   1--------3
      //  |
      //  | --------------> r
      // radial and azimuthal coordinates of the FE [ElementalNodes(r1,j1,r2,j2)]
      // r1 and j1 are defined with node 1, r2 and j2 with node 4
      ElementalNodes[i](0, 1) << NodeCoordinates.row(ElementNodeList(i, 0)).T(); // node 1
      ElementalNodes[i](2, 3) << NodeCoordinates.row(ElementNodeList(i, 3)).T(); // node 4

      if (ElementalNodes[i](3) <= ElementalNodes[i](1)) { // ring closure
        ElementalNodes[i](3) += 2 * M_PI;
      }

      // mapping node dof position (w, a, b) from global vector to element vector
      // ref, node 1, node 2, node 3, node 4
      qElement[i](0, RefDofs - 1) << getqExt(t)(0, RefDofs - 1);
      qElement[i](RefDofs, RefDofs + NodeDofs - 1) << getqExt(t)(RefDofs + ElementNodeList(i, 0) * NodeDofs, RefDofs + (ElementNodeList(i, 0) + 1) * NodeDofs - 1);
      qElement[i](RefDofs + NodeDofs, RefDofs + 2 * NodeDofs - 1) << getqExt(t)(RefDofs + ElementNodeList(i, 1) * NodeDofs, RefDofs + (ElementNodeList(i, 1) + 1) * NodeDofs - 1);
      qElement[i](RefDofs + 2 * NodeDofs, RefDofs + 3 * NodeDofs - 1) << getqExt(t)(RefDofs + ElementNodeList(i, 2) * NodeDofs, RefDofs + (ElementNodeList(i, 2) + 1) * NodeDofs - 1);
      qElement[i](RefDofs + 3 * NodeDofs, RefDofs + 4 * NodeDofs - 1) << getqExt(t)(RefDofs + ElementNodeList(i, 3) * NodeDofs, RefDofs + (ElementNodeList(i, 3) + 1) * NodeDofs - 1);

      // mapping node dof velocity from global vector to element vector
      // ref, node 1, node 2, node 3, node 4
      uElement[i](0, RefDofs - 1) << getuExt(t)(0, RefDofs - 1);
      uElement[i](RefDofs, RefDofs + NodeDofs - 1) << getuExt(t)(RefDofs + ElementNodeList(i, 0) * NodeDofs, RefDofs + (ElementNodeList(i, 0) + 1) * NodeDofs - 1);
      uElement[i](RefDofs + NodeDofs, RefDofs + 2 * NodeDofs - 1) << getuExt(t)(RefDofs + ElementNodeList(i, 1) * NodeDofs, RefDofs + (ElementNodeList(i, 1) + 1) * NodeDofs - 1);
      uElement[i](RefDofs + 2 * NodeDofs, RefDofs + 3 * NodeDofs - 1) << getuExt(t)(RefDofs + ElementNodeList(i, 2) * NodeDofs, RefDofs + (ElementNodeList(i, 2) + 1) * NodeDofs - 1);
      uElement[i](RefDofs + 3 * NodeDofs, RefDofs + 4 * NodeDofs - 1) << getuExt(t)(RefDofs + ElementNodeList(i, 3) * NodeDofs, RefDofs + (ElementNodeList(i, 3) + 1) * NodeDofs - 1);
    }
    updEle = false;
  }

  void FlexibleBody2s13Disk::updatePositions(double t, Frame2s *frame) {
    if(nrm2(frame->getParameters()) < epsroot()) { // center of gravity
      frame->setOrientation(getA());
      switch(RefDofs) {
        case 2:
        frame->setPosition(Vec3("[0;0;1]") * getq()(0));
        break;
        case 6:
        frame->setPosition(getq()(0,2));
        break;
        default:
        THROW_MBSIMERROR("(NurbsDisk2s::updateKinematicsForFrame): Unknown number of reference dofs!");
      }
    }
    else
        THROW_MBSIMERROR("(NurbsDisk2s::updateKinematicsForFrame): Parameters must be zero!");
  }

  void FlexibleBody2s13Disk::updateVelocities(double t, Frame2s *frame) {
    if(nrm2(frame->getParameters()) < epsroot()) { // center of gravity
      switch(RefDofs) {
        case 2:
        frame->setVelocity(Vec3("[0;0;1]") * getu()(0));
        frame->setAngularVelocity(Vec3("[0;0;1]") * getu()(1));
        break;
        case 6:
        frame->setPosition(getq()(0,2));
        frame->setVelocity(getu()(0,2));
        frame->setAngularVelocity(getA() * getG() * getu()(3,5));
        break;
        default:
        THROW_MBSIMERROR("(NurbsDisk2s::updateVelocities): Unknown number of reference dofs!");
      }
    }
    else
      THROW_MBSIMERROR("(NurbsDisk2s::updateKinematicsForFrame): Parameters must be zero!");
  }

  void FlexibleBody2s13Disk::updateAccelerations(double t, Frame2s *frame) {
    THROW_MBSIMERROR("(FlexibleBody2s13Disk::updateAccelerations): Not implemented.");
  }

  void FlexibleBody2s13Disk::updateJacobians(double t, Frame2s *frame, int j) {
    Index Wwidth(0, 3); // number of columns for Wtmp appears here also as column number
    Mat Wext(Dofs, 4);

    Vec2 alpha = frame->getParameters();

    if (nrm2(alpha) < epsroot()) { // center of gravity
      Wext(0, 0) = 1.;
      Wext(1, 1) = 1.;
    }
    else { // on the disk
      BuildElement(alpha);

      /* Jacobian of element */
      Mat Wtmp = static_cast<FiniteElement2s13Disk*>(discretization[currentElement])->JGeneralized(ElementalNodes[currentElement], alpha);

      /* Jacobian of disk */
      Wext(Index(0, RefDofs - 1), Wwidth) = Wtmp(Index(0, RefDofs - 1), Wwidth);
      Wext(Index(RefDofs + ElementNodeList(currentElement, 0) * NodeDofs, RefDofs + (ElementNodeList(currentElement, 0) + 1) * NodeDofs - 1), Wwidth) = Wtmp(Index(RefDofs, RefDofs + NodeDofs - 1), Wwidth);
      Wext(Index(RefDofs + ElementNodeList(currentElement, 1) * NodeDofs, RefDofs + (ElementNodeList(currentElement, 1) + 1) * NodeDofs - 1), Wwidth) = Wtmp(Index(RefDofs + NodeDofs, RefDofs + 2 * NodeDofs - 1), Wwidth);
      Wext(Index(RefDofs + ElementNodeList(currentElement, 2) * NodeDofs, RefDofs + (ElementNodeList(currentElement, 2) + 1) * NodeDofs - 1), Wwidth) = Wtmp(Index(RefDofs + 2 * NodeDofs, RefDofs + 3 * NodeDofs - 1), Wwidth);
      Wext(Index(RefDofs + ElementNodeList(currentElement, 3) * NodeDofs, RefDofs + (ElementNodeList(currentElement, 3) + 1) * NodeDofs - 1), Wwidth) = Wtmp(Index(RefDofs + 3 * NodeDofs, RefDofs + 4 * NodeDofs - 1), Wwidth);
    }

    Mat Jacobian = condenseMatrixRows(Wext, ILocked);

    // transformation
    frame->setJacobianOfTranslation(R->getOrientation(t).col(2) * Jacobian(0, 0, qSize - 1, 0).T(),j);
    frame->setJacobianOfRotation(R->getOrientation() * Jacobian(0, 1, qSize - 1, 3).T(),j);
  }

  void FlexibleBody2s13Disk::updateGyroscopicAccelerations(double t, Frame2s *frame) {
    THROW_MBSIMERROR("(FlexibleBody2s13Disk::updateGyroscopicAccelerations): Not implemented.");
  }

  void FlexibleBody2s13Disk::updatePositions(double t, NodeFrame *frame) {
    Vec3 tmp(NONINIT);
    int node = frame->getNodeNumber();

    tmp(0) = NodeCoordinates(node, 0) + getqExt(t)(RefDofs + (node + 1) * NodeDofs - 2) * (computeThickness(NodeCoordinates(node, 0))) / 2.; // in deformation direction
    tmp(1) = -getqExt(t)(RefDofs + (node + 1) * NodeDofs - 1) * (computeThickness(NodeCoordinates(node, 0))) / 2.;

    Vec2 tmp_add(NONINIT);
    tmp_add(0) = cos(NodeCoordinates(node, 1)) * tmp(0) - sin(NodeCoordinates(node, 1)) * tmp(1); // in sheave local frame
    tmp_add(1) = sin(NodeCoordinates(node, 1)) * tmp(0) + cos(NodeCoordinates(node, 1)) * tmp(1);

    tmp(0) = cos(getqExt(t)(1)) * tmp_add(0) - sin(getqExt(t)(1)) * tmp_add(1); // in sheave frame of reference
    tmp(1) = sin(getqExt(t)(1)) * tmp_add(0) + cos(getqExt(t)(1)) * tmp_add(1);

    tmp(2) = getqExt(t)(0) + getqExt(t)(RefDofs + node * NodeDofs) + (computeThickness(NodeCoordinates(node, 0))) / 2.;
    frame->setPosition(R->getPosition(t) + R->getOrientation(t) * tmp);

//    cout << "(FlexibleBody2s13Disk::updateOrientation): Not implemented!" << endl;
    //frame->getOrientation(false).set(0, R->getOrientation() * angle->computet(Phi));
    //frame->getOrientation(false).set(1, R->getOrientation() * angle->computen(Phi));
    //frame->getOrientation(false).set(2, crossProduct(frame->getOrientation().col(0), frame->getOrientation().col(1)));
  }

  void FlexibleBody2s13Disk::updateVelocities(double t, NodeFrame *frame) {
    Vec3 tmp(NONINIT);
    int node = frame->getNodeNumber();

    tmp(0) = NodeCoordinates(node, 0) + getqExt(t)(RefDofs + (node + 1) * NodeDofs - 2) * (computeThickness(NodeCoordinates(node, 0))) / 2.; // in deformation direction
    tmp(1) = -getqExt(t)(RefDofs + (node + 1) * NodeDofs - 1) * (computeThickness(NodeCoordinates(node, 0))) / 2.;

    Vec tmp_add_1(2, NONINIT);
    tmp_add_1(0) = cos(NodeCoordinates(node, 1)) * tmp(0) - sin(NodeCoordinates(node, 1)) * tmp(1); // in sheave local frame
    tmp_add_1(1) = sin(NodeCoordinates(node, 1)) * tmp(0) + cos(NodeCoordinates(node, 1)) * tmp(1);

    tmp(0) = -sin(getqExt(t)(1)) * tmp_add_1(0) - cos(getqExt(t)(1)) * tmp_add_1(1); // in sheave frame of reference
    tmp(1) = cos(getqExt(t)(1)) * tmp_add_1(0) - sin(getqExt(t)(1)) * tmp_add_1(1);

    tmp(0) *= getuExt(t)(1);
    tmp(1) *= getuExt(t)(1);

    Vec tmp_add_2(2, NONINIT);
    tmp_add_2(0) = getuExt(t)(RefDofs + (node + 1) * NodeDofs - 2) * (computeThickness(NodeCoordinates(node, 0))) / 2.; // in deformation direction
    tmp_add_2(1) = -getuExt(t)(RefDofs + (node + 1) * NodeDofs - 1) * (computeThickness(NodeCoordinates(node, 0))) / 2.;

    Vec tmp_add_3(2, NONINIT);
    tmp_add_3(0) = cos(NodeCoordinates(node, 1)) * tmp_add_2(0) - sin(NodeCoordinates(node, 1)) * tmp_add_2(1); // in sheave local frame
    tmp_add_3(1) = sin(NodeCoordinates(node, 1)) * tmp_add_2(0) + cos(NodeCoordinates(node, 1)) * tmp_add_2(1);

    tmp_add_2(0) = cos(getqExt(t)(1)) * tmp_add_3(0) - sin(getqExt(t)(1)) * tmp_add_3(1); // in sheave frame of reference
    tmp_add_2(1) = sin(getqExt(t)(1)) * tmp_add_3(0) + cos(getqExt(t)(1)) * tmp_add_3(1);

    tmp(0) += tmp_add_2(0);
    tmp(1) += tmp_add_2(1);

    tmp(2) = getuExt(t)(0) + getuExt(t)(RefDofs + node * NodeDofs);

    frame->setVelocity(R->getOrientation(t) * tmp);

    tmp(0) = getuExt(t)(RefDofs + (node + 1) * NodeDofs - 2) * (-cos(NodeCoordinates(node, 1)) * sin(getqExt(t)(1)) - cos(getqExt(t)(1)) * sin(NodeCoordinates(node, 1))) + getuExt(t)(RefDofs + (node + 1) * NodeDofs - 1) * (cos(getqExt(t)(1)) * cos(NodeCoordinates(node, 1)) - sin(getqExt(t)(1)) * sin(NodeCoordinates(node, 1)));
    tmp(1) = getuExt(t)(RefDofs + (node + 1) * NodeDofs - 1) * (cos(NodeCoordinates(node, 1)) * sin(getqExt(t)(1)) + cos(getqExt(t)(1)) * sin(NodeCoordinates(node, 1))) + getuExt(t)(RefDofs + (node + 1) * NodeDofs - 2) * (cos(getqExt(t)(1)) * cos(NodeCoordinates(node, 1)) - sin(getqExt(t)(1)) * sin(NodeCoordinates(node, 1)));
    tmp(2) = getuExt(t)(1);

    frame->setAngularVelocity(R->getOrientation() * tmp);
  }

  void FlexibleBody2s13Disk::updateAccelerations(double t, NodeFrame *frame) {
    THROW_MBSIMERROR("(FlexibleBody2s13Disk::updateAccelerations): Not implemented.");
  }

  void FlexibleBody2s13Disk::updateJacobians(double t, NodeFrame *frame, int j) {
    Index Wwidth(0, 3); // number of columns for Wtmp appears here also as column number
    Mat Wext(Dofs, 4);

    int node = frame->getNodeNumber();

    /* Jacobian of element */
    Mat Wtmp(5, 4, INIT, 0.); // initialising Ref + 1 Node

    // translation
    Wtmp(0, 0) = 1; // ref
    Wtmp(2, 0) = 1; // node

    // rotation
    Wtmp(1, 3) = 1; // ref
    Wtmp(3, 1) = -sin(getqExt(t)(1) + NodeCoordinates(node, 1)); // node
    Wtmp(3, 2) = cos(getqExt(t)(1) + NodeCoordinates(node, 1));
    Wtmp(4, 1) = cos(getqExt(t)(1) + NodeCoordinates(node, 1));
    Wtmp(4, 2) = sin(getqExt(t)(1) + NodeCoordinates(node, 1));

    /* Jacobian of disk */
    // reference
    Wext(Index(0, RefDofs - 1), Wwidth) = Wtmp(Index(0, RefDofs - 1), Wwidth);

    // nodes
    Wext(Index(RefDofs + node * NodeDofs, RefDofs + (node + 1) * NodeDofs - 1), Wwidth) = Wtmp(Index(RefDofs, RefDofs + NodeDofs - 1), Wwidth);

    // condensation
    Mat Jacobian = condenseMatrixRows(Wext, ILocked);

    // transformation
    frame->setJacobianOfTranslation(R->getOrientation(t).col(2) * Jacobian(0, 0, qSize - 1, 0).T());
    frame->setJacobianOfRotation(R->getOrientation() * Jacobian(0, 1, qSize - 1, 3).T());
  }

  void FlexibleBody2s13Disk::updateGyroscopicAccelerations(double t, NodeFrame *frame) {
    THROW_MBSIMERROR("(FlexibleBody2s13Disk::updateGyroscopicAccelerations): Not implemented.");
  }

  void FlexibleBody2s13Disk::init(InitStage stage) {
    if (stage == preInit) {
      assert(nr > 0); // at least on radial row
      assert(nj > 1); // at least two azimuthal elements

      // condensation
      switch (LType) {
        case innerring: // 0: innerring
          ILocked = Index(RefDofs, RefDofs + NodeDofs * nj - 1);
          Jext = Mat(Dofs, qSize, INIT, 0.);
          Jext(0, 0, RefDofs - 1, RefDofs - 1) << DiagMat(RefDofs, INIT, 1.);
          Jext(RefDofs + NodeDofs * nj, RefDofs, Dofs - 1, qSize - 1) << DiagMat(qSize - RefDofs, INIT, 1.);
        break;

        case outerring: // 1: outerring
          ILocked = Index(qSize, Dofs - 1);
          Jext = Mat(Dofs, qSize, INIT, 0.);
          Jext(0, 0, qSize - 1, qSize - 1) << DiagMat(qSize, INIT, 1.);
        break;
      }

      dr = (Ra - Ri) / nr;
      dj = 2 * M_PI / nj;

      NodeCoordinates = Mat(Nodes, 2);
      ElementNodeList.resize(Elements, 4);

      // mapping nodes - node coordinates - elements 
      for (int i = 0; i <= nr; i++) {
        for (int j = 0; j < nj; j++) {
          // node number increases azimuthally from the inner to the outer ring
          NodeCoordinates(j + i * nj, 0) = Ri + dr * i;
          NodeCoordinates(j + i * nj, 1) = 0. + dj * j;

          // element number increases azimuthally from the inner to the outer ring
          if (i < nr && j < nj - 1) {
            ElementNodeList(j + i * nj, 0) = j + i * nj; // node 1
            ElementNodeList(j + i * nj, 1) = (j + 1) + i * nj; // node 2
            ElementNodeList(j + i * nj, 2) = j + (i + 1) * nj; // node 3
            ElementNodeList(j + i * nj, 3) = (j + 1) + (i + 1) * nj; // node 4
          }
          else if (i < nr && j == nj - 1) { // ring closure
            ElementNodeList(j + i * nj, 0) = j + i * nj; // node 1
            ElementNodeList(j + i * nj, 1) = 0 + i * nj; // node 2
            ElementNodeList(j + i * nj, 2) = j + (i + 1) * nj; // node 3
            ElementNodeList(j + i * nj, 3) = 0 + (i + 1) * nj; // node 4
          }
        }
      }

      for (int i = 0; i < Elements; i++) {
        discretization.push_back(new FiniteElement2s13Disk(E, nu, rho));
        qElement.push_back(Vec(discretization[0]->getqSize(), INIT, 0.));
        uElement.push_back(Vec(discretization[0]->getuSize(), INIT, 0.));
        ElementalNodes.push_back(Vec(4, INIT, 0.));
      }
      FlexibleBody2s13::init(stage);
    }
    else if (stage == unknownStage) {
      initMatrices(); // calculate constant mass- and stiffness matrix

      FlexibleBody2s13::init(stage);
    }
    else if (stage == plotting) {
      updatePlotFeatures();

      if (getPlotFeature(plotRecursive) == enabled) {
//#ifdef HAVE_OPENMBVCPPINTERFACE
//#ifdef HAVE_NURBS
//        if (getPlotFeature(openMBV) == enabled) {
//          boost::shared_ptr<OpenMBV::NurbsDisk> Diskbody = OpenMBV::ObjectFactory::create<OpenMBV::NurbsDisk>();
//
//          drawDegree = 30 / nj;
//          Diskbody->setDiffuseColor(0.46667, 1, 1);
//          Diskbody->setMinimalColorValue(0.);
//          Diskbody->setMaximalColorValue(1.);
//          Diskbody->setDrawDegree(drawDegree);
//          Diskbody->setRadii(Ri, Ra);
//
//          Diskbody->setKnotVecAzimuthal(contour->getUVector());
//          Diskbody->setKnotVecRadial(contour->getVVector());
//
//          Diskbody->setElementNumberRadial(nr);
//          Diskbody->setElementNumberAzimuthal(nj);
//
//          Diskbody->setInterpolationDegreeRadial(degV);
//          Diskbody->setInterpolationDegreeAzimuthal(degU);
//          openMBVBody=Diskbody;
//        }
//#endif
//#endif
        FlexibleBody2s13::init(stage);
      }
    }
    else
      FlexibleBody2s13::init(stage);

//#ifdef HAVE_NURBS
//    contour->initContourFromBody(stage); // initialize contour
//#endif
  }

  Vec FlexibleBody2s13Disk::transformCW(const Vec& WrPoint) {
    Vec CrPoint(WrPoint.size());

    double &alpha = q(1);

    const double &xt = WrPoint(0);
    const double &yt = WrPoint(1);

    CrPoint(0) = sqrt(xt * xt + yt * yt);
    CrPoint(1) = ArcTan(xt * cos(alpha) + yt * sin(alpha), yt * cos(alpha) - xt * sin(alpha));
    CrPoint(2) = WrPoint(2) - q(0);

    return CrPoint;
  }

  void FlexibleBody2s13Disk::initMatrices() {
    BuildElements(0);

    // initialising of mass and stiffness matrix
    SymMat Mext(Dofs, INIT, 0.);
    SymMat Kext(Dofs, INIT, 0.);

    // element loop
    for (int i = 0; i < Elements; i++) {
      double r1 = ElementalNodes[i](0);
      double r2 = ElementalNodes[i](2);
      static_cast<FiniteElement2s13Disk*>(discretization[i])->computeConstantSystemMatrices(ElementalNodes[i], computeThickness(r1), computeThickness(r2));

      // definition of variables for element matrices
      SymMat Mplatte = discretization[i]->getM();
      SymMat Kplatte = static_cast<FiniteElement2s13Disk*>(discretization[i])->getK();

      Index IRef(0, RefDofs - 1);

      // reference components
      Mext(IRef) += Mplatte(IRef);
      Kext(IRef) += Kplatte(IRef);

      // nodes sort
      for (int k = 0; k < 4; k++) {
        // position in global matrix
        Index Ikges(RefDofs + ElementNodeList(i, k) * NodeDofs, RefDofs + (ElementNodeList(i, k) + 1) * NodeDofs - 1);
        // position in element matrix
        Index Ikelement(RefDofs + k * NodeDofs, RefDofs + (k + 1) * NodeDofs - 1);

        // four nodes
        // ref,0 and 0,Ref
        Mext(Ikges, IRef) += Mplatte(Ikelement, IRef);
        Kext(Ikges, IRef) += Kplatte(Ikelement, IRef);

        // int n=k;
        Mext(Ikges) += Mplatte(Ikelement); // diagonal
        Kext(Ikges) += Kplatte(Ikelement); // diagonal
        for (int n = k + 1; n < 4; n++) {
          Mext(Ikges, Index(RefDofs + ElementNodeList(i, n) * NodeDofs, RefDofs + (ElementNodeList(i, n) + 1) * NodeDofs - 1)) += Mplatte(Ikelement, Index(RefDofs + n * NodeDofs, RefDofs + (n + 1) * NodeDofs - 1));
          Kext(Ikges, Index(RefDofs + ElementNodeList(i, n) * NodeDofs, RefDofs + (ElementNodeList(i, n) + 1) * NodeDofs - 1)) += Kplatte(Ikelement, Index(RefDofs + n * NodeDofs, RefDofs + (n + 1) * NodeDofs - 1));
        }
      }
    }

    // condensation
    MConst = condenseMatrix(Mext, ILocked);
    K = condenseMatrix(Kext, ILocked);

    /* STATIC TEST */
    //Index Iall(RefDofs,K.size()-1);
    //
    //// load
    //Vec F_test(K.size()-RefDofs,INIT,0.);
    //F_test((nr-1)*nj*3) = 1e10;
    //
    //// displacements in MBSim
    //Vec q_test = slvLL(K(Iall),F_test);
    //Vec u_mbsim(12,NONINIT);
    //// first: positive x-axis
    //u_mbsim(0) = q_test(0);
    //u_mbsim(1) = q_test(nr/2*nj*3);
    //u_mbsim(2) = q_test((nr-1)*nj*3);
    //// second: positive y-axis
    //u_mbsim(3) = q_test(nj/4*3);
    //u_mbsim(4) = q_test(nr/2*nj*3+nj/4*3);
    //u_mbsim(5) = q_test((nr-1)*nj*3+nj/4*3);
    //// third: negative x-axis
    //u_mbsim(6) = q_test(nj/2*3);
    //u_mbsim(7) = q_test(nr/2*nj*3+nj/2*3);
    //u_mbsim(8) = q_test((nr-1)*nj*3+nj/2*3);
    //// fourth: negative y-axis
    //u_mbsim(9) = q_test(3*nj/4*3);
    //u_mbsim(10) = q_test(nr/2*nj*3+3*nj/4*3);
    //u_mbsim(11) = q_test((nr-1)*nj*3+3*nj/4*3);
    //
    //// displacements in ANSYS
    //Vec u_ansys("[0.10837E-15; 16.590; 50.111; -0.18542E-04; -0.85147; -2.4926; 0.0000; -0.17509; -0.31493; -0.18542E-04; -0.85147; -2.4926 ]");
    //
    //// error
    //double maxerr = nrmInf(u_ansys-u_mbsim);
    //
    //// output
    //ofstream file_static;
    //stringstream filename_static;
    //filename_static <<  "Static" <<  nr << "x" << nj << ".txt";
    //file_static.open(filename_static.str().c_str());
    //file_static<<"error="<<maxerr<<endl;
    //file_static<<"static_mbsim=matrix([";
    //for(int i = 0; i<u_mbsim.size(); i++)
    //  file_static<<u_mbsim(i)<<",";
    //file_static<<"]);"<<endl;
    //file_static.close();
    ///*END-Static-Test*/
    // masse and inertia of shaft
    MConst(0, 0) += m0;
    MConst(1, 1) += J0(2, 2);

    /*Eigenfrequencies*/
    //stringstream filename;
    //filename << "Invertation" << nr << "x" << nj;
    //MapleOutput(static_cast<Mat>(MConst), "M", filename.str());
    //MapleOutput(static_cast<Mat>(K), "K", filename.str());
    //throw new MBSimError("Natural Harmonics of flexible_body_2s13_mfr_mindlin were computed -> exit now ...");
    /*END-Eigenfrequencies*/

    // LU-decomposition of M
    M[0] = MConst;
    LLM[0] = facLL(MConst);
  }

  void FlexibleBody2s13Disk::updateAG() {
    //A(0,0) = cos(q(1)); // not used at the moment
    //A(0,1) = -sin(q(1));

    //A(1,0) = sin(q(1));
    //A(1,1) = cos(q(1));
  }

  void FlexibleBody2s13Disk::GlobalVectorContribution(int CurrentElement, const Vec& locVec, Vec& gloVec) {
    THROW_MBSIMERROR("(FlexibleBody2s13Disk::GlobalVectorContribution): Not implemented!");
  }

  void FlexibleBody2s13Disk::GlobalMatrixContribution(int CurrentElement, const Mat& locMat, Mat& gloMat) {
    THROW_MBSIMERROR("(FlexibleBody2s13Disk::GlobalMatrixContribution): Not implemented!");
  }

  void FlexibleBody2s13Disk::GlobalMatrixContribution(int CurrentElement, const SymMat& locMat, SymMat& gloMat) {
    THROW_MBSIMERROR("(FlexibleBody2s13Disk::GlobalMatrixContribution): Not implemented!");
  }

}
