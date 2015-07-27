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

namespace MBSimEHD {

//  EHDMesh::EHDMesh() :
//      nele(0), nnod(0) {
//
//  }
//
//  EHDMesh::~EHDMesh() {
//
//  }
//
//  EHDMesh::Boundary(EHDBoundaryConditionType type, EHDBoundaryConditionPosition) {
//    // Get DOFs at boundary
//    int b;
//    if (ele.shape.ndim == 1) {
//      b = Boundary1D(msh, boundary);
//    }
//    else if (el.shape.ndim == 2) {
//      b = Boundary2D(msh, boundary);
//    }
//    else {
//      throw MBSimError("Wrong dimension of element shape!");
//    }
//
//    // Decide between type of boundary
//    // Note: The indexing 1:1:end is used to get a row vector,
//    // because union(x, b) with an empty set x = [] would create
//    // a coloumn vector
//    if (type == dbc) {
//dbc    = union(dbc, b);
//  }
//  else if(type == nbc) {
//    nbc = union(nbc, b);
//  }
//  else if(type == per1) {
//    per1 = union(per1, b);
//  }
//  else if(type == per2) {
//    per1 = union(per2, b);
//  }
//  else {
//    throw MBSimError("Wrong type of boundary!");
//  }
//
//  // ToDo: Check if needed
//  // msh.dbc = setdiff(msh.dbc, msh.per2);
//  // msh.nbc = setdiff(msh.nbc, msh.per2);
//
//  }

}
