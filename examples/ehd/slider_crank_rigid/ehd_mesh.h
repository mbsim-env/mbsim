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

#include "ehd_pressure_element.h"

#include <fmatvec/fmatvec.h>


namespace MBSimEHD {

  //TODO: copy class from matlab

//  class EHDMesh {
//    public:
//      /** \brief type of boundary condition */
//      enum EHDBoundaryConditionType {
//        dbc, nbc, per1, per2
//      };
//      /** \brief type of boundary condition */
//      enum EHDBoundaryConditionPosition {
//        x1m, x1p, x2m, x2p
//      };
//
//      /*!
//       * \brief Constructor
//       */
//      EHDMesh();
//
//      /*!
//       * \brief destructor
//       */
//      ~EHDMesh();
//
//      /*
//       *    Set boundary of domain
//
//            Input:
//              msh:        Object of msh
//              type:       Type of boundary
//                          ('dbc', 'nbc', 'per1', 'per2')
//              boundary:   Lower or upper boundary of domain
//                          ('x1-', 'x1+', 'x2-', 'x2+')
//       */
//      void Boundary(EHDBoundaryConditionType type, EHDBoundaryConditionPosition boundary);
//
//    private:
//      EHDElement ele;        // Object of element
//
//      int nele;       // Number of elements
//      int nnod;       // Number of nodes
//      int ndof;       // Number of DOFs
//      int nfree;      // Number of free DOFs (without dbc and per2 DOFs)
//
//      int neled[2];      // Number of elements in spatial directions
//      int nnodd[2];      // Number of nodes in spatial directions
//      fmatvec::MatVx2I hd;         // Element size in spatial directions (rectilinear mesh)
//
//      std::vector<double> pos;        // Node positions
//      fmatvec::MatVI locX;
//      //std::vector<std::vector<int> > locX;       // Location matrix for geometry
//      fmatvec::MatVI locD;       // Location matrix for solution
//
//dbc      ;        // DOFs at Dirichlet boundary
//      nbc;// DOFs at Neumman boundary
//      per1;// DOFs at periodic boundary 1
//      per2;// DOFs at periodic boundary 2 (to be eliminated)
//      freedofs;// Free DOFs (without dbc and per2 DOFs)
//
//      /*!
//       * \brief comment
//       */
//      fmatvec::VecV Pos1(const double & boundary[2]);
//
//    };

  }
