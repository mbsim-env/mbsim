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

#include <mbsim/constitutive_laws.h>
#include <mbsim/mbsim_event.h>

#include <string>

namespace MBSimEHD {

  class EHDMesh : public MBSim::GeneralizedForceLaw {
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
      //            neled:  Number of elements in spatial directions
      //                     [nelex1; nelex2]
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
      EHDMesh(const EHDPressureElement & ele, const fmatvec::MatVx2 & xb, const fmatvec::VecInt & neled, const std::vector<fmatvec::RowVecV> & hd);

      /*!
       * \brief destructor
       */
      ~EHDMesh();

      void init(MBSim::Element::InitStage stage);

      //      Set boundary of domain
      //
      //      Input:
      //        msh:        Object of msh
      //        type:       Type of boundary
      //                    ('dbc', 'nbc', 'per1', 'per2')
      //        boundary:   Lower or upper boundary of domain
      //                    ('x1m', 'x1p', 'x2m', 'x2p')
      void Boundary(EHDBoundaryConditionType type, EHDBoundaryConditionPosition boundary);



      //  Node positions for one-dimensional case
      //   Input:
      //     xb:     Lower and upper boundary of domain
      //
      //   Output:
      //     pos
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
      //      loc:        Location matrix saving the indices of the global DOFs associated to
      //                  element DOFs in each row (locD), same for number of
      //                  node positions (locX)
      // Example 1D-Line Element:  loc = [1 2; 2 3; 3 4; ...]
      // Example 2D-quad8 Element: loc = [1 2 3 8 9 12 13 14; 3 4 5 9 10 14 15 16; ...]
      // (3 elements in first direction, 1 DoF per node -> nnodval = 1)
      // Example 2D-quad8 Element: loc = [1 2 3 4 5 6 16 17 18 19 24 25 26 27 28 29; 6 7 8 9 10 11 18 19 20 21 28 29 30 31 32 33; ...]
      // (3 elements in first direction, 2 DoF per node -> nnodval = 2)
      fmatvec::MatVI LocLine(const int & nnodval);

      //    Location matrix for positions and solution for quad elements
      //
      //    Input:
      //      nnodval:    Number of nodal values, e.g. for two-dimensional case:
      //                  1 for pressure (locD) and 2 for positions (locX) or
      //                  2 for displacements and positions (locD = locX)
      //
      //    Output:
      //      loc:        Location matrix saving the indices of the global DOFs associated to
      //                  element DOFs in each row (locD), same for number of
      //                  node positions (locX)
      // Example 1D-Line Element:  loc = [1 2; 2 3; 3 4; ...]
      // Example 2D-quad8 Element: loc = [1 2 3 8 9 12 13 14; 3 4 5 9 10 14 15 16; ...]
      // (3 elements in first direction, 1 DoF per node -> nnodval = 1)
      // Example 2D-quad8 Element: loc = [1 2 3 4 5 6 16 17 18 19 24 25 26 27 28 29; 6 7 8 9 10 11 18 19 20 21 28 29 30 31 32 33; ...]
      // (3 elements in first direction, 2 DoF per node -> nnodval = 2)
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

      /*!
       * \brief solve for the pressure at the nodes
       */
      void solvePressure(const double & tolD = 1e-1, const int & iterMax = 50);


      virtual void setContactKinematics(ContactKinematicsEHDInterface * cK) {
        ck = cK;
        ele.setContactKinematics(cK);
      }

      int getndof() const {
        return ndof;
      }

      int getnfree() const {
        return nfree;
      }

      fmatvec::VecInt getfreedofs() const {
        return freedofs;
      }

      int getnele() const {
        return nele;
      }

      const EHDPressureElement & getElement() const {
        return ele;
      }

      fmatvec::MatVI getlocX() const {
        return locX;
      }

      fmatvec::MatVI getlocD() const {
        return locD;
      }

      fmatvec::VecInt getper1() const {
        return per1V;
      }

      fmatvec::VecInt getper2() const {
        return per2V;
      }

      fmatvec::VecInt getdbc() const {
        return dbcV;
      }

      fmatvec::VecInt getnbc() const {
        return nbcV;
      }

      fmatvec::VecV getR() const {
        return R;
      }

      fmatvec::SqrMatV getKT() const {
        return KT;
      }

      /* INHERITED FUNCTIONS*/
      virtual bool isSetValued() const {
        return false;
      }
      virtual void computeSmoothForces(std::vector<std::vector<MBSim::SingleContact> > & contacts);
      /**********************/

    private:
      EHDPressureElement ele;        // Object of class pressure element

      int nele;       // Number of elements
      int nnod;       // Number of nodes
      int ndof;       // Number of DOFs
      int nfree;      // Number of free DOFs (without dbc and per2 DOFs)

      fmatvec::VecInt neled;      // Number of elements in spatial directions
      fmatvec::Vec2I nnodd;      // Number of nodes in spatial directions

      /*!
       * \brief Lower and upper boundary of domain. xb = [x1l, x1u; x2l, x2u]
       *
       * For 1D examples the second row is not important
       */
      fmatvec::MatVx2 xb;

      std::vector<fmatvec::RowVecV> hd;         // Element size in spatial directions (rectilinear mesh)

      /*!
       * \brief Location matrix for geometry
       */
      fmatvec::MatVI locX;

      /*!
       * \brief Location matrix for solution
       */
      fmatvec::MatVI locD;

      fmatvec::VecInt dbcV;     // DOFs at Dirichlet boundary
      fmatvec::VecInt nbcV;        // DOFs at Neumman boundary
      fmatvec::VecInt per1V;       // DOFs at periodic boundary 1
      fmatvec::VecInt per2V;       // DOFs at periodic boundary 2 (to be eliminated)
      fmatvec::VecInt freedofs;   // Free DOFs (without dbc and per2 DOFs)

      /*!
       * \brief residual vector after pressure assembly
       */
      fmatvec::VecV R;

      /*!
       * \brief stiffness matrix after pressure assembly
       */
      fmatvec::SqrMatV KT;

      /*!
       * \brief force calculation matrix
       *
       * Integrates the pressures to the normal forces at the nodes
       *     F_N = integral over (Np^T*p*n dA) = Cff*D
       */
      fmatvec::SqrMatV Cff;

      /*!
       * \brief pressure at nodes
       */
      fmatvec::VecV D;

      /*!
       * \brief contact kinematics for calls inside Evaluate Element
       *
       * \todo: shouldn't be like this --> probably the mesh has to die to implement a clean structure (the contact knows the contact kinematics anyway!)
       */
      ContactKinematicsEHDInterface * ck;

      /*!
       * \todo: description
       */
      void PressureAssembly();

      /*!
       * \brief initilaize the mesh with all given values
       *
       * \todo: it implements the first constructor possibility coming from matlab, the second one is missing!
       */
      void initializeMesh();

      /*
       * \brief Finish mesh generation
       */
      void finishMesh();

      /*!
       * \brief assembles the matrix Cff for the integration of the pressures to calculate the forces in normal direction
       */
      void ForceMatrixAssembly();

  };

}

#endif
