#include "ehd_mesh.h"

namespace MBSimEHD {

  EHDMesh::EHDMesh() :
      nele(0), nnod(0) {

  }

  EHDMesh::~EHDMesh() {

  }

  EHDMesh::Boundary(EHDBoundaryConditionType type, EHDBoundaryConditionPosition) {
    // Get DOFs at boundary
    int b;
    if (ele.shape.ndim == 1) {
      b = Boundary1D(msh, boundary);
    }
    else if (el.shape.ndim == 2) {
      b = Boundary2D(msh, boundary);
    }
    else {
      throw MBSimError("Wrong dimension of element shape!");
    }

    // Decide between type of boundary
    // Note: The indexing 1:1:end is used to get a row vector,
    // because union(x, b) with an empty set x = [] would create
    // a coloumn vector
    if (type == dbc) {
dbc    = union(dbc, b);
  }
  else if(type == nbc) {
    nbc = union(nbc, b);
  }
  else if(type == per1) {
    per1 = union(per1, b);
  }
  else if(type == per2) {
    per1 = union(per2, b);
  }
  else {
    throw MBSimError("Wrong type of boundary!");
  }

  // ToDo: Check if needed
  // msh.dbc = setdiff(msh.dbc, msh.per2);
  // msh.nbc = setdiff(msh.nbc, msh.per2);

  }

}
