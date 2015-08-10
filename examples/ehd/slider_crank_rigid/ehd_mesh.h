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

#ifndef _EHD_MESH_H_
#define _EHD_MESH_H_

#include "ehd_pressure_element.h"

#include <fmatvec/fmatvec.h>
#include <mbsim/mbsim_event.h>

#include <string>

namespace MBSimEHD {

  class EHDMesh {
    public:
      /** \brief type of boundary condition */
      enum EHDBoundaryConditionType {
        dbc, nbc, per1, per2
      };
      /** \brief type of boundary condition */
      enum EHDBoundaryConditionPosition {
        x1m, x1p, x2m, x2p
      };

      //          Constructor for regular structured mesh

      //          Input:
      //            ele:    Object of element for discretization
      //            xb:     Lower and upper boundary of domain
      //                     [x1l, x1u; x2l, x2u]
      //            neled:  Number of elements in spatial directions
      //                     [nelex1; nelex2]
      //
      //          Output:
      //            msh:    Object of created mesh
      EHDMesh(const EHDPressureElement & ele, const fmatvec::MatVx2 & xb, const fmatvec::VecInt & neled);

      //          Constructor for user structured Mesh

      //          Input:
      //            ele:    Object of element for discretization
      //            xb:     Lower and upper boundary of domain
      //                     [x1l, x1u; x2l, x2u]
      //            neled:  Number of elements in spatial directions
      //                     [nelex1; nelex2]
      //            hd:     Element sizes in spatial directions
      //                    {[h1x1, h2x1, ...], [h1x2, h2x2, ...]}
      //
      //          Output:
      //            msh:    Object of created mesh
      EHDMesh(const EHDPressureElement & ele, const fmatvec::MatVx2 & xb, const fmatvec::VecInt & neled, const std::vector<fmatvec::RowVecV> & hd);

      /*!
       * \brief destructor
       */
      ~EHDMesh();

      //      Set boundary of domain
      //
      //      Input:
      //        msh:        Object of msh
      //        type:       Type of boundary
      //                    ('dbc', 'nbc', 'per1', 'per2')
      //        boundary:   Lower or upper boundary of domain
      //                    ('x1m', 'x1p', 'x2m', 'x2p')
      void Boundary(EHDBoundaryConditionType type, EHDBoundaryConditionPosition boundary);

      // Finish mesh generation
      void FinishMesh(void);

      //  Node positions for one-dimensional case
      //   Input:
      //     xb:     Lower and upper boundary of domain
      //
      //   Output:
      //     pos:    Node positions x_i^k (i: direction, k: node)
      //             [x_1^1; x_1^2; x_1^3; ...]
      fmatvec::VecV Pos1D(const fmatvec::MatVx2 & xb);

      //  Node positions for two-dimensional case
      //   Input:
      //     xb:     Lower and upper boundary of domain
      //
      //   Output:
      //     pos:    Node positions x_i^k (i: direction, k: node)
      //             [x_1^1; x_1^2; x_1^3; ...]
      fmatvec::VecV Pos2D(const fmatvec::MatVx2 & xb);

      //    Location matrix for positions and solution for line elements
      //
      //    Input:
      //      nnodval:    Number of nodal values, e.g. for one-dimensional case:
      //                  1 for pressure and position (locD = locX)
      //
      //    Output:
      //      loc:        Location matrix with global DOFs associated to
      //                  element DOFs in each row (locD), same for number of
      //                  node positions (locX)
      fmatvec::MatVI LocLine(const int & nnodval);

      //    Location matrix for positions and solution for quad elements
      //
      //    Input:
      //      nnodval:    Number of nodal values, e.g. for two-dimensional case:
      //                  1 for pressure (locD) and 2 for positions (locX) or
      //                  2 for displacements and positions (locD = locX)
      //
      //    Output:
      //      loc:        Location matrix with global DOFs associated to
      //                  element DOFs in each row (locD), same for number of
      //                  node positions (locX)
      fmatvec::MatVI LocQuad(const int & nnodval);

      //    DOFs at boundary of one-dimensional domain
      //
      //    Input:
      //      boundary:   Lower or upper boundary of domain
      //
      //    Output:
      //      b:          DOFs at boundary
      fmatvec::VecInt Boundary1D(const EHDBoundaryConditionPosition & boundary);

      //    DOFs at boundary of two-dimensional domain
      //
      //    Input:
      //      boundary:   Lower or upper boundary of domain
      //
      //    Output:
      //      b:          DOFs at boundary
      fmatvec::VecInt Boundary2D(const EHDBoundaryConditionPosition & boundary);

      fmatvec::VecV getpos() {
        return pos;
      }

      fmatvec::MatVI getlocX() {
        return locX;
      }

      fmatvec::MatVI getlocD() {
        return locD;
      }
      fmatvec::VecInt getper1() {
        return per1V;
      }
      fmatvec::VecInt getper2() {
        return per2V;
      }
      fmatvec::VecInt getdbc() {
        return dbcV;
      }
      fmatvec::VecInt getnbc() {
        return nbcV;
      }

    private:
      EHDPressureElement ele;        // Object of class pressure element

      int nele;       // Number of elements
      int nnod;       // Number of nodes
      int ndof;       // Number of DOFs
      int nfree;      // Number of free DOFs (without dbc and per2 DOFs)

      fmatvec::VecInt neled;      // Number of elements in spatial directions
      fmatvec::Vec2I nnodd;      // Number of nodes in spatial directions
      std::vector<fmatvec::RowVecV> hd;         // Element size in spatial directions (rectilinear mesh)

      fmatvec::VecV pos;        // Node positions
      fmatvec::MatVI locX;            // Location matrix for geometry
      fmatvec::MatVI locD;       // Location matrix for solution

      fmatvec::VecInt dbcV;        // DOFs at Dirichlet boundary
      fmatvec::VecInt nbcV;        // DOFs at Neumman boundary
      fmatvec::VecInt per1V;       // DOFs at periodic boundary 1
      fmatvec::VecInt per2V;       // DOFs at periodic boundary 2 (to be eliminated)
      fmatvec::VecInt freedofs;   // Free DOFs (without dbc and per2 DOFs)

  };

}

#endif
