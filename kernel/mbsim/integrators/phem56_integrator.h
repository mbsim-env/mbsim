/* Copyright (C) 2004-2018  Martin Förg
 
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
 * Contact:
 *   martin.o.foerg@googlemail.com
 *
 */

#ifndef _PHEM56_INTEGRATOR_H_
#define _PHEM56_INTEGRATOR_H_

#include "root_finding_integrator.h"

namespace MBSimIntegrator {

  /** \brief DAE-Integrator PHEM56
  */

  class PHEM56Integrator : public RootFindingIntegrator {

    public:

      enum LinearAlgebra {
        DEC=0,
        DGETRF,
        unknown
      };

    private:

      static void fprob(int* ifcn, int* nq, int* nv, int* nu, int* nl, int* nzg, int* nzf, int* lrda, int* nblk, int* nmrc, int* npgp, int* npfl, int* indgr, int* indgc, int* indflr, int* indflc,  double* t, double* p, double* v, double* u, double* xl, double* g, double* gp, double* f, double* gpp, double* gt, double* fl, double* qdot, double* udot, double* am);
      static void solout(int* nr, int* nq, int* nv, int* nu, int* nl, int* lrdo, double* q, double* v, double* u, double* a, double* rlam, double* dowk, int* irtrn);

      double tPlot{0};
      double dtOut{0};
      double s0; 
      double time{0};

      /** linear algebra */
      LinearAlgebra linearAlgebra{DGETRF};
      /** general V */
      bool generalVMatrix{true};
      /** initial projection */
      bool initialProjection{false};
      /** number of steps between projections */
      int numberOfStepsBetweenProjections{0};
      /** project onto index 1 constraint manifold */
      bool projectOntoIndex1ConstraintManifold{false};
      /** Absolute Toleranz */
      fmatvec::Vec aTol;
      /** Relative Toleranz */
      fmatvec::Vec rTol;
      /** step size for the first step */
      double dt0{1e-3};
      /** maximum number of steps */
      int maxSteps{2000000000};
      /** maximale step size */
      double dtMax{0};

    public:

      ~PHEM56Integrator() override = default;

      void setAbsoluteTolerance(const fmatvec::Vec &aTol_) { aTol = aTol_; }
      void setAbsoluteTolerance(double aTol_) { aTol = fmatvec::Vec(1,fmatvec::INIT,aTol_); }
      void setRelativeTolerance(const fmatvec::Vec &rTol_) { rTol = rTol_; }
      void setRelativeTolerance(double rTol_) { rTol = fmatvec::Vec(1,fmatvec::INIT,rTol_); }
      void setInitialStepSize(double dt0_) { dt0 = dt0_; }
      void setMaximumStepSize(double dtMax_) { dtMax = dtMax_; }
      void setStepLimit(int maxSteps_) { maxSteps = maxSteps_; }
      void setLinearAlgebra(LinearAlgebra linearAlgebra_) { linearAlgebra = linearAlgebra_; }
      void setGeneralVMatrix(bool generalVMatrix_) { generalVMatrix = generalVMatrix_; }
      void setInitialProjection(bool initialProjection_) { initialProjection = initialProjection_; }
      void setNumberOfStepsBetweenProjections(int numberOfStepsBetweenProjections_) { numberOfStepsBetweenProjections = numberOfStepsBetweenProjections_; }
      void setProjectOntoIndex1ConstraintManifold(bool projectOntoIndex1ConstraintManifold_) { projectOntoIndex1ConstraintManifold = projectOntoIndex1ConstraintManifold_; }

      const fmatvec::Vec& getAbsoluteTolerance() const { return aTol; }
      const fmatvec::Vec& getRelativeTolerance() const { return rTol; }

      using Integrator::integrate;
      void integrate() override;

      void initializeUsingXML(xercesc::DOMElement *element) override;
  };

}

#endif
