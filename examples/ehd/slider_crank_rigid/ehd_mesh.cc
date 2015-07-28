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
#include <fmatvec/fmatvec.h>

using namespace fmatvec;

namespace MBSimEHD {

  EHDMesh::EHDMesh(const PressureElement & ele, const fmatvec::MatVx2 & xb, const fmatvec::VecInt & neled):ele(ele) {
    int nele_tmp;

    // Get spatial dimension according to used element shape
    int ndim = ele.shape.ndim; // friend class!

    // Create vector with domain size in spatial directions
    fmatvec::VecV Hd = xb.col(1)-xb.col(0); // ??

    // Save number of elements in spatial directions
    this->neled = neled;

    // Create vector with element sizes in spatial directions
    this->hd = MatVx2I(ndim,INIT,0);
    for (int i = 0; i < ndim; i++) {
      this->hd.col(i)=VecInt(neled(i),init,1) * Hd.row(i) / neled(i);
    }
//                   % Create vector with element sizes in spatial directions
//                   hd = cell(ndim, 1);
//                   for i = 1:1:ndim
//                       hd{i} = ones(1, msh.neled(i)) * Hd(i) / msh.neled(i);
//                   end
//                   msh.hd = hd;
//
    // Compute total number of elements
    nele_tmp = 1;
    for (int i = 0; i < ndim; i++) {
      nele_tmp = nele_tmp * this->neled(i);
    }
    this->nele = nele_tmp;
//               % Compute total number of elements
//               msh.nele = 1;
//               for i = 1:1:ndim
//                   msh.nele = msh.nele * msh.neled(i);
//               end

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
          throw MBSimError("type quad8on of element shape not yet implemented!");
        }

    else {
          throw MBSimError("Wrong type of element shape!");
    }

    // Compute total number of DOFs
    this->ndof = this->nnod * ele.ndofpernod;

  }

  EHDMesh::EHDMesh(const EHDElement & ele, const fmatvec::MatVx2 & xb, const fmatvec::VecInt & neled, const fmatvec::MatVx2I & hd) {
      int nele_tmp;
      this->ele = ele;
      int ndim = ele.shape.ndim; // ??
      fmatvec::MatVx2 Hd = xb.col(2)-xb.col(1); // ??
      this->neled = neled;

      //  Save element sizes in spatial directions
      this->hd = hd;  // ??

      // Compute number of elements in spatial directions
      this->neled = VecInt(2,init,0);
      for (int i = 0; i < ndim; i++) {
        if (abs(sum(hd.col(i)) - Hd(i)) > 1e-12) {   // sum ??
          throw MBSimError("EHD mesh boundary is not compatible to defined mesh!");
        }
        this->neled(i) = length(hd.col(i));
      }

      // Compute total number of elements
      nele_tmp = 1;
      for (int i = 0; i < ndim; i++) {
        nele_tmp = nele_tmp * this->neled(i);
      }
      this->nele = nele_tmp;
  //               % Compute total number of elements
  //               msh.nele = 1;
  //               for i = 1:1:ndim
  //                   msh.nele = msh.nele * msh.neled(i);
  //               end

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
            throw MBSimError("type quad8on of element shape not yet implemented!");
          }

      else {
            throw MBSimError("Wrong type of element shape!");
      }

      // Compute total number of DOFs
      this->ndof = this->nnod * ele.ndofpernod;

    }
//  EHDMesh::~EHDMesh() {
//
//  }

  EHDMesh::Boundary(EHDBoundaryConditionType type, EHDBoundaryConditionPosition boundary) {
    // Get DOFs at boundary
    fmatvec::VecInt b;
    if (ele.shape.ndim == 1) {
      b = Boundary1D(boundary);
    }
    else if (el.shape.ndim == 2) {
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
      dbc = b;
    }
    else if(type == nbc) {
//      nbc = union(nbc, b);
      nbc = b;
    }
    else if(type == per1) {
//      per1 = union(per1, b);
      per1 = b;
    }
    else if(type == per2) {
//      per1 = union(per2, b);
      per2 = b;
    }
    else {
      throw MBSimError("Wrong type of boundary!");
    }

  // ToDo: Check if needed
  // msh.dbc = setdiff(msh.dbc, msh.per2);
  // msh.nbc = setdiff(msh.nbc, msh.per2);

  }

  EHDMesh::FinishMesh(){
    fmatvec::VecInt ndof = VecInt(this->ndof,init,0);  // ?
    fmatvec::VecInt ndof_dbc_per2;
    int k=this->ndof;
    int n=0;

    // Set free DOFs (without Dirichlet and periodic boundary 2)
    for(int i=0; i<this->ndof; i++) {
      ndof(i)=i+1;
    }
    ndof_dbc_per2 = ndof;

    for(int j=0; j<this->length(this->per2); j++) {
          ndof_dbc_per2(this->per2(j)-1)=0;
    }
    for(int j=0; j<this->length(this->dbc); j++) {
          ndof_dbc_per2(this->dbc(j)-1)=0;
    }
    for(int j=0; j<length(ndof_dbc_per2); j++) {
      if (ndof_dbc_per2(j)==0){
        k--;
      }
    }
    this->freedofs = VecInt(k,init,0);
    for(int j=0; j<this->ndof; j++) {
      if (ndof_dbc_per2(j)>0){
        this->freedofs(n)=ndof_dbc_per2(j);
            n++;
      }
    }
    this->nfree = k;

    // Eliminate DOFs of per2 with per1 in location matrix
    for (int i=0; i<length(this->per2); i++) { // length ??
      for (int j=0; j<length(this->locD); j++){
        if (this->locD(j)==this->per2(i)){
          this->locD(j)=this->per1(i);
          break;
        }
      }
    }
    // ToDo: Look for matlab find-function in c++: msh.locD(msh.locD == msh.per2(i)) = msh.per1(i);
  }

  fmatvec::VecV Pos1D(const fmatvec::MatVx2 & xb) {
    // Define abbreviations
    int ndim = 1;

    int nnodde = this->ele.shape.nnodd;          //ToDo: mesh/ele übergeben???
    int l=0;
    int x;
    // Compute number of nodes in spatial directions
    this->nnodd(1) = this->neled(1) * (nnodde(1) - 1) + 1;  //nnodd[1],nnodd(1) ???

    // Get total number of nodes
    this->nnod = this->nnodd(1);

    // Get element sizes
    double hd1 = this->hd.col(1);

    // Compute node positions
    fmatvec::VecV pos = VecV(this->nnod * ndim, 1);

    for(int e=1; e<this->neled(1)+1; e++){
      if (e==(this->neled(1))){
        x=0;
      }
      else{
        x=1;
      }

      for (int i=1; i<nnodde(1) - x + 1; i++){
        pos(l) = xb(1,1) + (i-1)*hd1(e-1) / (nnodde(1) - 1) + sum(hd1,e-1-1); // ToDo accumulate/sum until e-1 ??
        l++;
      }
    }
    return pos;

    //ToDo: Check loop indices!
  }

  fmatvec::VecV Pos2D(const fmatvec::MatVx2 & xb) {
    // Define abbreviations
    int ndim = 2;
    int nnodde = this->ele.shape.nnodd;          //ToDo: mesh/ele übergeben???
    int ser = this->ele.shape.ser;
    int l=0;
    int x1, x2;

    // Compute number of nodes in spatial directions
    this->nnodd(1) = this->neled(1) * (nnodde(1) - 1) + 1;  //nnodd[1],nnodd(1) ???
    this->nnodd(2) = this->neled(2) * (nnodde(2) - 1) + 1;  //nnodd[1],nnodd(1) ???

    // Compute number of nodes considering also serendipity elements
    this->nnod = this->nnodd(1) * this->nnodd(2) - pow(ser,2) * this->nele;

    // Get element sizes in spatial directions
    hd1 = this->hd.col(1);
    hd2 = this->hd.col(2);

    // Compute node positions
    fmatvec::VecV pos = VecV(this->nnod * ndim, 1);

    for(int e2=1; e2<this->neled(2)+1; e2++){
      if (e2==this->neled(2)){
        x2=0;
      }
      else{
        x2=1;
      }
      for (int j=1; j<nnodde(2)- x2 + 1; j++){
        for(int e1=1; e1<this->neled(1) + 1; e1++){
            if (e1==(this->neled(1))){
              x1=0;
            }
            else{
              x1=1;
            }
            for (int i=1; i<nnodde(1)- x1 + 1; i++){
              if (mod(i-1,ser+1)==0 || mod(j-1,ser+1)==0){
                pos(l) = xb(1,1) + (i-1)*hd1(e1-1) / (nnodde(1) - 1) + sum(hd1,e1-1-1); // ToDo accumulate/sum until e-1 ??
                l++;
                pos(l) = xb(2,1) + (j-1)*hd2(e2-1) / (nnodde(2) - 1) + sum(hd1,e2-1-1); // ToDo accumulate/sum until e-1 ??
                l++;
              }
            }
          }
      }
    }

    return pos;
    //ToDo: Check loop indices!
  }
