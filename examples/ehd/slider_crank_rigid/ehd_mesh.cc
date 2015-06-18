#include "ehd_mesh.h"


namespace MBSimEHD {

EHDMesh::EHDMesh() : nele(0), nnod(0) {

}

EHDMesh::~EHDMesh() {

}

EHDMesh::Boundary(EHDBoundaryConditionType type, EHDBoundaryConditionPosition) {
  // Get DOFs at boundary
  int b;
  if(ele.shape.ndim == 1) {
    b = Boundary1D(msh, boundary);
  }
  else if(el.shape.ndim == 2) {
    b = Boundary2D(msh, boundary);
  }
  else {
    throw MBSimError("Wrong dimension of element shape!");
  }

  // Decide between type of boundary
  // Note: The indexing 1:1:end is used to get a row vector,
  // because union(x, b) with an empty set x = [] would create
  // a coloumn vector
  if(type == dbc) {

  }
  else if(type == nbc) {

  }

  switch type
      case 'dbc'
          msh.dbc = union(msh.dbc(1:1:end), b);
      case 'nbc'
          msh.nbc = union(msh.nbc(1:1:end), b);
      case 'per1'
          msh.per1 = union(msh.per1(1:1:end), b);
      case 'per2'
          msh.per2 = union(msh.per2(1:1:end), b);

          % ToDo: Check if needed
          % msh.dbc = setdiff(msh.dbc, msh.per2);
          % msh.nbc = setdiff(msh.nbc, msh.per2);
  end
}


}
