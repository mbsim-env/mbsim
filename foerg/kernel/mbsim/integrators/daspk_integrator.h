/* Copyright (C) 2004-2006  Martin Förg
 
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

//  #ifndef _DASPK_INTEGRATOR_H_
//  #define _DASPK_INTEGRATOR_H_
//  
//  #include "integrator.h"
//  
//  namespace MBSim {
//  
//    /** \brief ODE-Integrator DASPK
//      Integrator with root finding for ODEs.
//      This integrator uses DASPK from http://www.netlib.org . */
//    class DASPKIntegrator : public Integrator {
//  
//      private:
//  
//        static void res(double *t, double *z, double *zd, double *cj, double *delta, int *ires, double *rpar, int *ipar);
//        static void fsv(int* zSize, double* t, double* z_, int* nsv, double* sv_);
//  
//        /** maximal step size */
//        double dtMax;
//        /** minimal step size */
//        double dtMin;
//        /** Absolute Toleranz */
//        fmatvec::Vec aTol;
//        /** Relative Toleranz */
//        double rTol;
//        /** step size for the first step */
//        double dt0;
//        /**  maximum number of steps allowed during one call to the solver. (default 10000) */
//        int maxSteps;
//        /** use stiff (BDF) method */
//        bool stiff;
//        /** number of differential equations */
//        static int zSize;
//  
//      public:
//  
//        DASPKIntegrator();
//        ~DASPKIntegrator() {}
//  
//        void setMaximalStepSize(double dtMax_) {dtMax = dtMax_;}
//        void setMinimalStepSize(double dtMin_) {dtMin = dtMin_;}
//        void setRelativeTolerance(double rTol_) {rTol = rTol_;}
//        void setAbsoluteTolerance(const fmatvec::Vec &aTol_) {aTol.resize() = aTol_;}
//        void setAbsoluteTolerance(double aTol_) {aTol.resize() = fmatvec::Vec(1,fmatvec::INIT,aTol_);}
//        void setInitialStepSize(double dt0_) {dt0 = dt0_;}
//        void setmaxSteps(int maxSteps_) {maxSteps = maxSteps_;}
//        void setStiff(bool flag) {stiff = flag;}
//  
//        void integrate(DynamicSystemSolver& system);
//        
//        virtual void initializeUsingXML(TiXmlElement *element);
//    };
//  
//  }
//  
//  #endif
