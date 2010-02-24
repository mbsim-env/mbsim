/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: mfoerg@users.berlios.de
 */

#ifndef _RKSUITE_INTEGRATOR_H_
#define _RKSUITE_INTEGRATOR_H_

#include "integrator.h"

namespace MBSim {

  /** \brief ODE-Integrator RKSuite.
    Integrator for ODEs.
    This integrator uses rksuite from http://www.netlib.org . */
  class RKSuiteIntegrator : public Integrator {
    public:
      /**
       * \brief constructor
       */
      RKSuiteIntegrator();
      
      /**
       * \brief destructor
       */
      ~RKSuiteIntegrator() {}

      void preIntegrate(DynamicSystemSolver& system);
      void subIntegrate(DynamicSystemSolver& system, double tStop);
      void postIntegrate(DynamicSystemSolver& system);

      /* GETTER / SETTER */
      void setMethod(int method_) {method = method_;}
      void setrTol(double rTol_) {rTol = rTol_;}
      void setThreshold(const fmatvec::Vec &thres_) {thres.resize() = thres_;}
      void setThreshold(double thres_) {thres.resize() = fmatvec::Vec(1,fmatvec::INIT,thres_);}
      void setInitialStepSize(double dt0_) {dt0 = dt0_;}
      /***************************************************/


      /* INHERITED INTERFACE OF INTEGRATOR */
      virtual void integrate(DynamicSystemSolver& system);
      virtual void initializeUsingXML(TiXmlElement *element);
      /***************************************************/

    private:

      static void fzdot(double* t, double* z_, double* zd_);

      static int zSize;

      int method;
      /** Absolute Toleranz */
      fmatvec::Vec thres;
      /** Relative Toleranz */
      double rTol;
      /** step size for the first step */
      double dt0;


      int ndworkarray, messages, integrationSteps;
      double t, tPlot, s0, time;
      double * dworkarray;
      fmatvec::Vec z, zdGot, zMax;

      std::ofstream integPlot;

  };

}

#endif
