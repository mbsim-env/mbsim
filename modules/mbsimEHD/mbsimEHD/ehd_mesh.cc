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
 * Contact: martin.o.foerg@googlemail.com
 */

#include "ehd_mesh.h"
#include "ehd_pressure_element.h"
#include <mbsim/utils/utils.h>
#include <mbsim/mbsim_event.h>

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimEHD {

  EHDMesh::EHDMesh(const EHDPressureElement & ele, const fmatvec::MatVx2 & xb, const fmatvec::VecInt & neled) :
      ele(ele), xb(xb), neled(neled) {
  }

  EHDMesh::EHDMesh(const EHDPressureElement & ele, const fmatvec::MatVx2 & xb, const fmatvec::VecInt & neled, const std::vector<fmatvec::RowVecV> & hd) :
      ele(ele), hd(hd) {
    /*
     * TODO...

     int nele_tmp;
     int ndim = ele.shape.ndim;
     double s;
     fmatvec::VecV Hd = xb.col(1) - xb.col(0);
     this->neled = neled;

     // Compute number of elements in spatial directions
     this->neled = VecInt(2, INIT, 0);

     for (int i = 0; i < ndim; i++) {
     s = 0;
     for (int j = 0; j < hd.rows(); j++) {
     s = s + hd(j, i);
     }
     if (nrm1(s - Hd(i)) > 1e-12) {   // TODO: write function sum in fmatvec
     throw MBSim::MBSimError("EHD mesh boundary is not compatible to defined mesh!");
     }
     }

     // Count number of elements in spatial directions
     for (int i = 0; i < ndim; i++) {
     int j = 0;
     while (hd(i, j) > 0) {
     j++;
     }
     this->neled(i) = j;
     }

     // Compute total number of elements
     nele_tmp = 1;
     for (int i = 0; i < ndim; i++) {
     nele_tmp = nele_tmp * this->neled(i);
     }
     this->nele = nele_tmp;

     // Compute node positions and build location matrices
     if (ele.shape.name == "line2" || ele.shape.name == "line3") {
     this->pos = Pos1D(xb);
     this->locX = LocLine(ndim);
     this->locD = LocLine(ele.ndofpernod);
     }
     else if (ele.shape.name == "quad4" || ele.shape.name == "quad8" || ele.shape.name == "quad9") {
     this->pos = Pos2D(xb);
     this->locX = LocQuad(ndim);
     this->locD = LocQuad(ele.ndofpernod);
     }
     else if (ele.shape.name == "quad8on") {
     //                       msh.pos = Pos2D(msh, xb);
     //                       locX1 = LocQuad(msh, ndim);
     //                       msh.locX = locX1(:, [1, 2, 9, 10, 3, 4, 11, 12, ...
     //                           5, 6, 13, 14, 7, 8, 15, 16]);
     //                       locD1 = LocQuad(msh, ele.ndofpernod);
     //                       if ele.ndofpernod == 1
     //                           msh.locD = locD1(:, [1, 5, 2, 6, 3, 7, 4, 8]);
     //                       elseif ele.ndofpernod == 2
     //                           msh.locD = locD1(:, [1, 2, 9, 10, 3, 4, 11, 12, ...
     //                           5, 6, 13, 14, 7, 8, 15, 16]);
     throw MBSim::MBSimError("type quad8on of element shape not yet implemented!");
     }
     else {
     throw MBSim::MBSimError("Wrong type of element shape!");
     }

     // Compute total number of DOFs
     this->ndof = this->nnod * ele.ndofpernod;
     */
  }

  EHDMesh::~EHDMesh() {

  }

  void EHDMesh::init(MBSim::Element::InitStage stage) {
    GeneralizedForceLaw::init(stage);
    if (stage == MBSim::Element::modelBuildup) {
      initializeMesh();
      evaluateBoundary();
      finishMesh();
    }
    else if (stage == MBSim::Element::resize) {
      D = fmatvec::VecV(ndof);
    }
    else if (stage == MBSim::Element::calculateLocalInitialValues) {
      ForceMatrixAssembly();
    }
  }

  void EHDMesh::Boundary(EHDBoundaryConditionType type, EHDBoundaryConditionPosition boundary) {
    boundaries.insert(std::pair<EHDBoundaryConditionPosition, EHDBoundaryConditionType>(boundary, type));
  }

  void EHDMesh::evaluateBoundary() {
    for (map<EHDBoundaryConditionPosition, EHDBoundaryConditionType>::iterator i = boundaries.begin(); i != boundaries.end(); i++) {
      EHDBoundaryConditionPosition boundary = i->first;
      EHDBoundaryConditionType type = i->second;
      // Get DOFs at boundary
      fmatvec::VecInt b;
      if (ele.shape.ndim == 1) {
        b = Boundary1D(boundary);
      }
      else if (ele.shape.ndim == 2) {
        b = Boundary2D(boundary);
      }
      else {
        throw MBSimError("Wrong dimension of element shape!");
      }

      // Decide between type of boundary
      // Note: The indexing 1:1:end is used to get a row vector,
      // because union(x, b) with an empty set x = [] would create
      // a coloumn vector

      // ToDo: Check if union is needed
      if (type == dbc) {
//      dbc = union(dbc, b);
        if (dbcV.size()==0) {
          dbcV = b;
        }
        else{
          int size_dbcV = dbcV.size();
          VecInt b_old = dbcV;
          int j=0;
          dbcV.resize(dbcV.size()+b.size());
          for (int i=0; i<size_dbcV; i++){
            dbcV(i)=b_old(j);
            j++;
          }
          j=0;
          for (int i=size_dbcV; i<dbcV.size(); i++){
            dbcV(i)=b(j);
            j++;
          }
        }

      }
      else if (type == nbc) {
//      nbc = union(nbc, b);
        nbcV = b;
      }
      else if (type == per1) {
//      per1 = union(per1, b);
        per1V = b;
      }
      else if (type == per2) {
//      per1 = union(per2, b);
        per2V = b;
      }
      else {
        throw MBSimError("Wrong type of boundary!");
      }

      // ToDo: Check if needed
      // msh.dbc = setdiff(msh.dbc, msh.per2);
      // msh.nbc = setdiff(msh.nbc, msh.per2);

    }
  }

  void EHDMesh::initializeMesh() {
    // Get spatial dimension according to used element shape
    int ndim = ele.shape.ndim;      // friend class!

    // Create vector with domain size in spatial directions
    fmatvec::VecV Hd = xb.col(1) - xb.col(0);

    // Create vector with element sizes in spatial directions
    for (int i = 0; i < ndim; i++) {
      hd.push_back(RowVecV(neled(i), INIT, 0.));
      for (int j = 0; j < neled(i); j++)
        hd[i](j) = Hd(i) / neled(i);
    }

    // Compute total number of elements
    int nele_tmp = 1;
    for (int i = 0; i < ndim; i++) {
      nele_tmp = nele_tmp * this->neled(i);
    }
    this->nele = nele_tmp;

    // Compute node positions and build location matrices
    if (ele.shape.name == "line2" || ele.shape.name == "line3") {
      ck->pos = Pos1D(xb);
      this->locX = LocLine(ndim);
      this->locD = LocLine(ele.ndofpernod);
    }
    else if (ele.shape.name == "quad4" || ele.shape.name == "quad8" || ele.shape.name == "quad9") {
      ck->pos = Pos2D(xb);
      this->locX = LocQuad(ndim);
      this->locD = LocQuad(ele.ndofpernod);
    }
    else if (ele.shape.name == "quad8on") {
      //                       msh.pos = Pos2D(msh, xb);
      //                       locX1 = LocQuad(msh, ndim);
      //                       msh.locX = locX1(:, [1, 2, 9, 10, 3, 4, 11, 12, ...
      //                           5, 6, 13, 14, 7, 8, 15, 16]);
      //                       locD1 = LocQuad(msh, ele.ndofpernod);
      //                       if ele.ndofpernod == 1
      //                           msh.locD = locD1(:, [1, 5, 2, 6, 3, 7, 4, 8]);
      //                       elseif ele.ndofpernod == 2
      //                           msh.locD = locD1(:, [1, 2, 9, 10, 3, 4, 11, 12, ...
      //                           5, 6, 13, 14, 7, 8, 15, 16]);
      throw MBSim::MBSimError("type quad8on of element shape not yet implemented!");
    }
    else {
      throw MBSim::MBSimError("Wrong type of element shape!");
    }

    // Compute total number of DOFs
    ndof = nnod * ele.ndofpernod;

    ck->setNumberOfPotentialContactPoints(nnod);        //TODO: really bad hack --> the mesh takes some kinematics tasks which should be done by the contact kinematics
  }

  void EHDMesh::finishMesh() {
    fmatvec::VecInt ndof = VecInt(this->ndof, INIT, 0);
    fmatvec::VecInt ndof_dbc_per2;
    int k = this->ndof;
    int n = 0;

    // Set free DOFs (without Dirichlet and periodic boundary 2)
    for (int i = 0; i < this->ndof; i++) {
      ndof(i) = i + 1;
    }
    ndof_dbc_per2 = ndof;

    for (int j = 0; j < (this->per2V.size()); j++) {
      ndof_dbc_per2(this->per2V(j) - 1) = 0;
    }
    cout << dbcV << endl;
    for (int j = 0; j < this->dbcV.size(); j++) {
      ndof_dbc_per2(this->dbcV(j) - 1) = 0;
    }
    for (int j = 0; j < this->ndof; j++) {
      if (ndof_dbc_per2(j) == 0) {
        k--;
      }
    }
    this->freedofs = VecInt(k, INIT, 0);
    for (int j = 0; j < this->ndof; j++) {
      if (ndof_dbc_per2(j) > 0) {
        this->freedofs(n) = ndof_dbc_per2(j);
        n++;
      }
    }
    this->nfree = k;

    // Eliminate DOFs of per2 with per1 in location matrix
    for (int i = 0; i < this->per2V.size(); i++) {
      for (int j = 0; j < this->locD.rows(); j++) {
        for (int k = 0; k < this->locD.cols(); k++) {
          if (this->locD(j, k) == this->per2V(i)) {
            this->locD(j, k) = this->per1V(i);
            break; //jump out of element
          }
        }
      }
    }

    // ToDo: Look for matlab find-function in c++: msh.locD(msh.locD == msh.per2(i)) = msh.per1(i);
  }

  fmatvec::VecV EHDMesh::Pos1D(const fmatvec::MatVx2 & xb) {
    // Define abbreviations
    int ndim = 1;

    int nnodde[2] = {this->ele.shape.nnodd[0], this->ele.shape.nnodd[1]};

    // Compute number of nodes in spatial directions
    this->nnodd(0) = this->neled(0) * (nnodde[0] - 1) + 1;

    // Get total number of nodes
    this->nnod = this->nnodd(0);

    // Get element sizes
    RowVecV hd1 = this->hd[0];

    // Compute node positions
    fmatvec::VecV pos = VecV(this->nnod * ndim, INIT, 1);

    double xd1 = 0;
    int l = 0;
    for (int e = 0; e < this->neled(0); e++) {
      for (int i = 0; i < nnodde[0] - 1; i++) {
        pos(l) = xb(0, 0) + i * hd1(e) / (nnodde[0] - 1) + xd1;
        l++;
      }
      xd1 = xd1 + hd1(e);
    }
    // last node
    pos(l) = xb(0, 1);

    return pos;

  }

  fmatvec::VecV EHDMesh::Pos2D(const fmatvec::MatVx2 & xb) {
    // Define abbreviations
    int ndim = 2;
    int nnodde[2] = {this->ele.shape.nnodd[0], this->ele.shape.nnodd[1]};
    int ser = this->ele.shape.ser;
    int x1, x2;

    // Compute number of nodes in spatial directions
    this->nnodd(0) = this->neled(0) * (nnodde[0] - 1) + 1;
    this->nnodd(1) = this->neled(1) * (nnodde[1] - 1) + 1;

    // Compute number of nodes considering also serendipity elements
    this->nnod = this->nnodd(0) * this->nnodd(1) - pow(ser, 2) * this->nele;

    // Get element sizes in spatial directions
    RowVecV hd1 = this->hd[0];
    RowVecV hd2 = this->hd[1];

    // Compute node positions
    fmatvec::VecV pos = VecV(this->nnod * ndim, INIT, 0.);

    int l = 0;
    double xd[2] = {0, 0};
    for (int e2 = 0; e2 < this->neled(1); e2++) {
      for (int j = 0; j < nnodde[1] - 1; j++) {
        xd[0] = 0;
        for (int e1 = 0; e1 < this->neled(0); e1++) {
          for (int i = 0; i < nnodde[0] - 1; i++) {
            if ((i % (ser + 1)) == 0 || (j % (ser + 1)) == 0) {   // ????
              pos(l) = xb(0, 0) + i * hd1(e1) / (nnodde[0] - 1) + xd[0];
              l++;
              pos(l) = xb(1, 0) + j * hd2(e2) / (nnodde[1] - 1) + xd[1];
              l++;
            }
          }
          xd[0] = xd[0] + hd1(e1);
        }
        // last node in column
        pos(l) = xb(0, 1);
        l++;
        pos(l) = xb(1, 0) + j * hd2(e2) / (nnodde[1] - 1) + xd[1];
        l++;
      }
      xd[1] = xd[1] + hd2(e2);
    }
    // last node row
    xd[0] = 0;
    int j = nnodde[1] - 1;
    for (int e1 = 0; e1 < this->neled(0); e1++) {
      for (int i = 0; i < nnodde[0] - 1; i++) {
        if ((i % (ser + 1)) == 0 || (j % (ser + 1)) == 0) {
          pos(l) = xb(0, 0) + i * hd1(e1) / (nnodde[0] - 1) + xd[0];
          l++;
          pos(l) = xb(1, 1);
          l++;
        }
      }
      xd[0] = xd[0] + hd1(e1);
    }
    // last node
    pos(l) = xb(0, 1);
    l++;
    pos(l) = xb(1, 1);
    l++;

    return pos;
  }

  fmatvec::MatVI EHDMesh::LocLine(const int & nnodval) {
    fmatvec::MatVI loc = MatVI(this->nele, this->ele.shape.nnod * nnodval, INIT, 0);
    int d1;

    // Define abbreviations
    int nnodde[2] = {this->ele.shape.nnodd[0], this->ele.shape.nnodd[1]};
    std::string shapeName = this->ele.shape.name;

    // Build location matrix
    for (int e = 0; e < this->neled(0); e++) {
      for (int i = 0; i < nnodval; i++) {
        d1 = (nnodde[0] - 1) * nnodval;
        loc(e, i) = i + 1 + d1 * (e);
        loc(e, i + nnodval) = i + 1 + d1 * (e + 1);
        if (shapeName == "line3") {
          loc(e, i + 2 * nnodval) = loc(e, i) + nnodval;
        }
      }
    }

    return loc;

  }

  fmatvec::MatVI EHDMesh::LocQuad(const int & nnodval) {
    fmatvec::MatVI loc = MatVI(this->nele, this->ele.shape.nnod * nnodval, INIT, 0);
    int e = 0;
    int d1, d2;

    // Define abbreviations
    int nnodde[2] = {this->ele.shape.nnodd[0], this->ele.shape.nnodd[1]};
    int ser = this->ele.shape.ser;
    std::string shapeName = this->ele.shape.name;

    // Build location matrix
    for (int e2 = 0; e2 < this->neled(1); e2++) {
      for (int e1 = 0; e1 < this->neled(0); e1++) {
        e = e1 + e2 * this->neled(0);
        for (int i = 0; i < nnodval; i++) {
          d1 = (nnodde[0] - 1) * nnodval;
          d2 = (this->nnodd(0) * (nnodde[1] - 1) - this->neled(0) * pow(ser, 2)) * nnodval;
          loc(e, i) = (i + 1) + d1 * e1 + d2 * e2;
          loc(e, i + nnodval) = (i + 1) + d1 * (e1 + 1) + d2 * e2;
          loc(e, i + 2 * nnodval) = (i + 1) + d1 * (e1 + 1) + d2 * (e2 + 1);
          loc(e, i + 3 * nnodval) = (i + 1) + d1 * e1 + d2 * (e2 + 1);
          if ((shapeName == "quad8") || (shapeName == "quad9") || (shapeName == "quad8on")) {
            loc(e, i + 4 * nnodval) = loc(e, i) + nnodval;
            loc(e, i + 5 * nnodval) = loc(e, i + nnodval) + (this->nnodd(0) - ser * (e1 + 1)) * nnodval;
            loc(e, i + 6 * nnodval) = loc(e, i + 2 * nnodval) - nnodval;
            loc(e, i + 7 * nnodval) = loc(e, i + nnodval) + (this->nnodd(0) - ser * (e1 + 1) - 2 + ser) * nnodval;
          }
          if (shapeName == "quad9") {
            loc(e, i + 8 * nnodval) = loc(e, i + nnodval) + (this->nnodd(0) - ser * (e1 + 1) - 1) * nnodval;
          }
        }
      }
    }

    return loc;

//     TODO: Check location matrix for higher order elements such as
//           quad12 (serendipity) or quad16 elements

  }

  fmatvec::VecInt EHDMesh::Boundary1D(const EHDBoundaryConditionPosition & boundary) {
    // Define abbreviations
    int ndofpernod = this->ele.ndofpernod;

    fmatvec::VecInt b = VecInt(ndofpernod, INIT, 0);

    // Get dofs at boundary
    if (boundary == x1p) {
      for (int i = 0; i < ndofpernod; i++) {
        b(i) = (this->nnod - 1) * ndofpernod + i + 1;
      }
    }
    else if (boundary == x1m) {
      for (int i = 0; i < ndofpernod; i++) {
        b(i) = i + 1;
      }
    }
    else {
      throw MBSim::MBSimError("Invalid boundary position");
    }

    return b;

  }

  fmatvec::VecInt EHDMesh::Boundary2D(const EHDBoundaryConditionPosition & boundary) {
    int l = 0;
    int nnodser = 0;

    fmatvec::VecInt b;
    // Define abbreviations
    int ndofpernod = this->ele.ndofpernod;
    int ser = this->ele.shape.ser;

    // Get dofs at boundary
    if (boundary == x1p) {
      b = VecInt(this->nnodd(1) * ndofpernod, INIT, 0);
      for (int n2 = 0; n2 < this->nnodd(1); n2++) {
        if ((n2 % (ser + 1)) != 0) {
          nnodser = nnodser + this->neled(0) * ser * ndofpernod;
        }
        for (int i = 0; i < ndofpernod; i++) {
          b(l) = i + 1 + ((n2 + 1) * this->nnodd(0) - 1) * ndofpernod - nnodser;
          l++;
        }
      }
    }
    else if (boundary == x1m) {
      b = VecInt(this->nnodd(1) * ndofpernod, INIT, 0);
      for (int n2 = 0; n2 < this->nnodd(1); n2++) {
        for (int i = 0; i < ndofpernod; i++) {
          b(l) = i + 1 + ((n2 + 1) - 1) * this->nnodd(0) * ndofpernod - nnodser;
          l++;
        }
        if ((n2 % (ser + 1)) != 0) {
          nnodser = nnodser + this->neled(0) * ser * ndofpernod;
        }
      }
    }
    else if (boundary == x2p) {
      b = VecInt(this->nnodd(0) * ndofpernod, INIT, 0);
      for (int n1 = 0; n1 < this->nnodd(0); n1++) {
        for (int i = 0; i < ndofpernod; i++) {
          b(l) = i + 1 + ndofpernod * ((n1 + 1) - 1 + this->nnod - this->nnodd(0));
          l++;
        }
      }
    }
    else if (boundary == x2m) {
      b = VecInt(this->nnodd(0) * ndofpernod, INIT, 0);
      for (int n1 = 0; n1 < this->nnodd(0); n1++) {
        for (int i = 0; i < ndofpernod; i++) {
          b(l) = i + 1 + ndofpernod * ((n1 + 1) - 1);
          l++;
        }
      }
    }
    else {
      throw MBSim::MBSimError("Invalid boundary position");
    }

    return b;

  }

  void EHDMesh::PressureAssembly() {
    // Assemble element residuum vector and element tangent matrix
    // Michael Hofer, 18.01.2015
    //
    // Input:
    //   D:      Global vector with nodal values for pressure
    //   sys:    Object of system
    //   msh:    Object of mesh
    //   lub:    Object of lubricant
    //
    // Output:
    //   R:      Global residuum vector
    //   KT:     Global tangential matrix

    // Define abbreviations
    int ndofe = ele.getndof();

    // Initialize row and value vector for residuum vector
//    VecV rR(ndofe * nele);
//    VecV vR(ndofe * nele);

    // Initialize row, coloumn and value vector for tangential matrix
//    VecV rKT(ndofe * ndofe * nele);
//    VecV cKT(ndofe * ndofe * nele);
//    VecV vKT(ndofe * ndofe * nele);

    //TODO: should be possible to make it once and then never again (only set to zero...)
    R = VecV(ndof);
    KT = SqrMatV(ndof);

    // Loop through all elements to assemble element residuum vector and
    // element tangent matrix
    int i = 0;
    int j = 0;

    for (int e = 0; e < nele; e++) {
      // Extract element location vector for geometry and pressure
      RowVecVI locXe = getlocX().row(e);
      RowVecVI locDe = getlocD().row(e);

      // Extract positions of element nodes and vector with nodal values
      VecV pose = subVec(ck->pos, locXe.T(), -1);

      VecV de = subVec(D, locDe.T(), -1);

      // Evaluate element
      VecV re;
      SqrMatV kTe;
      ele.EvaluateElement(e, pose, de, re, kTe);    //TODO: change evaluate element to match with MBSim! (withou the sys)

      // Assembly
      for (int row = 0; row < ndofe; row++) {
        R(locDe(row) - 1) += re(row);
        //      i = i + 1;
        //      rR(i) = locDe(row);
        //      vR(i) = re(row);
        for (int col = 0; col < ndofe; col++) {
          KT(locDe(row) - 1, locDe(col) - 1) += kTe(row, col);
          //        j = j + 1;
          //        rKT(j) = locDe(row);
          //        cKT(j) = locDe(col);
          //        vKT(j) = kTe(row, col);
        }
      }
    }

    // Finally set up sparse residuum vector and sparse tangential matrix
    //TODO: sparse=?
    //  R = sparse(rR, 1, vR, ndof, 1);
    //  R = rR;
    //  KT = sparse(rKT, cKT, vKT, ndof, ndof);
    //  KT = rKT;
  }

  void EHDMesh::ForceMatrixAssembly(void) {
    // Assemble force matrix
    // Michael Hofer, 18.01.2015
    //
    // Output:
    //   Cff:    Global force matrix

    // Define abbreviations
    int ndof = getndof();
    int nele = getnele();
    int ndofe = ele.getndof();

    // Initialize row, coloumn and value vector for force matrix
    VecV rCff(ndofe * ndofe * nele);
    VecV cCff(ndofe * ndofe * nele);
    VecV vCff(ndofe * ndofe * nele);

    Cff = SqrMatV(ndof);

    // Loop through all elements to assemble element force matrix
    int i = 0;
    int j = 0;

    for (int e = 0; e < nele; e++) {
      // Extract element location vector for geometry and pressure
      RowVecVI locXe = getlocX().row(e);
      RowVecVI locDe = getlocD().row(e);

      // Extract positions of element nodes and vector with nodal values
      VecV pose = subVec(ck->pos, locXe.T(), -1);

      // Evaluate element
      SqrMatV cffe(ndofe);
      ele.CalculateForceMatrixElement(e, pose, cffe);    //TODO: change evaluate element to match with MBSim! (withou the sys)

      // Assembly
      for (int rowe = 0; rowe < ndofe; rowe++) {
        for (int cole = 0; cole < ndofe; cole++) {
          Cff(locDe(rowe) - 1, locDe(cole) - 1) += cffe(rowe, cole);
        }
      }
    }

  }
  void EHDMesh::solvePressure(const double & tolD, const int & iterMax) {
    // Newton-Josephy method for solving a nonlinear complementarity
    // problem (NCP) by linearization in each iteration step resulting
    // in a linear complementarity problems (LCP)
    // Michael Hofer, 17.01.2015
    //
    // Input:
    //   D0:         Start point for iteration
    //   tolD:       Tolerance for increment norm
    //   iterMax:    Maximal number of iterations
    //   output:     Flag for command window output
    //
    // Output:
    //   D:          Global vector with nodal solution
    //
    // Definition:
    //   NCP:        0 <= D perp R(D) >= 0
    //   LCPs:       0 <= Z perp W = -M * Z + Q >= 0 (in each iteration step)

    // Penalty regularization has to be disabled
    if (ele.pp > 0) {
      ele.pp = 0;
      throw MBSim::MBSimError("Penalty regularization has to be disabled!");
    }

//        // Set flag for drawing solution in each iteration step if not given
//        if nargin < 7 || isempty(iterDraw)
//            iterDraw = false;
//        end

    // Define abbreviation
    VecInt f = getfreedofs();
    VecInt per1 = getper1();
    VecInt per2 = getper2();
    int nfree = getnfree();
    Vec Dfree = Vec(nfree);
    Vec Z = Vec(nfree);
    Vec Zsol = Vec(2*nfree);
    Vec deltaDeff = Vec(nfree);
    Vec Q = Vec(nfree);
    SqrMat M = SqrMat(nfree);


    // Initialize iteration counter
    int iter = 0;
//        // Print header for iteration list
//        if output
//            fprintf('//-16s//-20s//s\n', 'Iteration', 'Increment norm', 'Goenka iterations');
//        end

//        // Set arbitrary start set for region1 in Goenka algorithm
//        region1 = 1:1:(length(f) - 2);

    // Retrieve global residuum vector and global tangential matrix at D
    D = VecV(ndof);    //using old solution is not successful

    // Newton-Josephy iteration loop
    int info;
    while (iter < iterMax) {
      // Compute matrix M and vector Q of LCP
      PressureAssembly();
      Dfree = subVec(D, f, -1);
      M = subMat2(KT, f, -1);  // TODO: subMat for SqrMat
      VecV Rfree = subVec(R, f, -1); //TODO: assemble -R directly?
      Q = Rfree - M*Dfree;

      //TODO: here is the solution
      LinearComplementarityProblem lcp(M,Q);
      lcp.setStrategy(LinearComplementarityProblem::LCPSolvingStrategy(2));
      lcp.setSystem(M,Q);

      // Z = lcp.solve(Dfree);
      Zsol = lcp.solve(Dfree);

//      SqrMatV JLU = facLU(M, ipiv);
//      Z = slvLUFac(JLU, Rfree, ipiv);

      // UPDATE of full D-vector with new solution
      // Insert new solution at free DOFs
      for (int i = 0; i < nfree; i++) {
        Z(i) = Zsol(nfree+i);
        D(f(i) - 1) += Zsol(nfree + i);
      }

      // Compute effective solution increment for convergence check
      deltaDeff = Z;


      // Overwrite solution at eliminated periodic boundary
      for (int i = 0; i < per2.size(); i++) {
        D(per2(i) - 1) = D(per1(i) - 1);
      }

      // cout << D << endl;

      // Update iteration counter
      iter = iter + 1;

//      cout << nrm1(deltaDeff) << endl;

//            if output
//                fprintf('//-16d//-20e//d\n', iter, normdeltaDeff, iterGoenka);
//            end
      break;

      if (fmatvec::nrm1(deltaDeff) < tolD) {
        break;
      }
    }
//    for (int i = 0; i < nfree; i++) {
//      if (D(f(i) - 1) < 0) {
//        D(f(i) - 1) = 0;
//      }
//    }
    if (iter == iterMax) {
      throw MBSim::MBSimError("Newton-Josephy iteration unconverged after " + numtostr(iter) + " iterations.");
    }

  }

  void EHDMesh::computeSmoothForces(std::vector<std::vector<SingleContact> > & contacts) {
    double TolD = 1e-2; //TODO: TolD and MaxIter outside of this function
    int maxIter = 30;

    solvePressure(TolD, maxIter);

    VecV laN = Cff * D;

    // cout << D << endl;

    for (int i = 0; i < contacts.size(); i++) {
      for (int j = 0; j < contacts[i].size(); j++) {
        contacts[i][j].getlaN()(0) = laN(j + i * contacts.size()); //TODO: is the index correct?
      }
    }
  }

}

