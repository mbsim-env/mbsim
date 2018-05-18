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

#include <config.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/utils/eps.h>
#include "fortran/fortran_wrapper.h"
#include "phem56_integrator.h"
#include <ctime>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimIntegrator {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMINT, PHEM56Integrator)

  void PHEM56Integrator::fprob(int* ifcn, int* nq, int* nv, int* nu, int* nl, int* nzg, int* nzf, int* lrda, int* nblk, int* nmrc,  int* npgp, int* npfl, int* indgr, int* indgc, int* indflr, int* indflc,  double* t, double* q, double* v, double* u, double* xl, double* g_, double* gp, double* f, double* gpp_, double* gt, double* fl, double* qdot, double* udot, double* am) {
    auto self=*reinterpret_cast<PHEM56Integrator**>(&nq[1]);
    self->getSystem()->setTime(*t);
    self->getSystem()->setq(Vec(*nq,q));
    self->getSystem()->setu(Vec(*nv,v));
    self->getSystem()->setx(Vec(*nu,u));
    self->getSystem()->resetUpToDate();
    if(*ifcn==0) {
      Vec xd(*nu, udot);
      self->getSystem()->setStepSize(1);
      xd = self->getSystem()->evaldx();;
      self->getSystem()->setStepSize(0);
    }
    else if(*ifcn==1) {
      Vec h(*nv, f);
      h = self->getSystem()->evalh();
      Mat M(*lrda,*nv, am);
      M(RangeV(0,*nv-1),RangeV(0,*nv-1)) = self->getSystem()->evalM();
      Vec qd(nq[0], qdot);
      self->getSystem()->setStepSize(1);
      qd = self->getSystem()->evaldq();;
      self->getSystem()->setStepSize(0);
      if(self->generalVMatrix) {
        Mat FL(*nv,*nl, fl);
        FL = -self->getSystem()->evalV();
      }
      else {
        Mat GQ(*nl,*nv, gp);
        GQ = -self->getSystem()->evalW().T();
      }
    }
    else if(*ifcn==2) {
      Vec qd(nq[0], qdot);
      self->getSystem()->setStepSize(1);
      qd = self->getSystem()->evaldq();;
      self->getSystem()->setStepSize(0);
    }
    else if(*ifcn==3) {
      Vec ih(*nl, gt);
      ih = self->getSystem()->evalW().T()*self->getSystem()->getu()-self->getSystem()->evalgd();
    }
    else if(*ifcn==4) {
      Vec g(*nl, g_);
      g = -self->getSystem()->evalg();
    }
    else if(*ifcn==5) {
      Vec h(*nv, f);
      h = self->getSystem()->evalh();
      Vec gpp(*nl, gpp_);
      gpp = -self->getSystem()->evalwb();
    }
    else if(*ifcn==6) {
      Mat GQ(*nl,*nv, gp);
      GQ = -self->getSystem()->evalW().T();
      Vec ih(*nl, gt);
      ih = self->getSystem()->evalW().T()*self->getSystem()->getu()-self->getSystem()->evalgd();
    }
    else if(*ifcn==7) {
      Vec h(*nv, f);
      h = self->getSystem()->evalh();
      Mat M(*lrda,*nv, am);
      M(RangeV(0,*nv-1),RangeV(0,*nv-1)) = self->getSystem()->evalM();
      Vec gpp(*nl, gpp_);
      gpp = -self->getSystem()->evalwb();
    }
    else if(*ifcn==8) {
      Vec h(*nv, f);
      h = self->getSystem()->evalh();
      Mat M(*lrda,*nv, am);
      M(RangeV(0,*nv-1),RangeV(0,*nv-1)) = self->getSystem()->evalM();
      if(self->generalVMatrix) {
        Mat FL(*nv,*nl, fl);
        FL = -self->getSystem()->evalV();
      }
    }
    else if(*ifcn==9) {
      Mat M(*lrda,*nv, am);
      M(RangeV(0,*nv-1),RangeV(0,*nv-1)) = self->getSystem()->evalM();
    }
    else if(*ifcn==10) {
      Vec ih(*nl, gt);
      ih = self->getSystem()->evalW().T()*self->getSystem()->getu()-self->getSystem()->evalgd();
      Mat GQ(*nl,*nv, gp);
      GQ = -self->getSystem()->evalW().T();
      Mat M(*lrda,*nv, am);
      M(RangeV(0,*nv-1),RangeV(0,*nv-1)) = self->getSystem()->evalM();
      Vec qd(nq[0], qdot);
      self->getSystem()->setStepSize(1);
      qd = self->getSystem()->evaldq();;
      self->getSystem()->setStepSize(0);
      if(self->generalVMatrix) {
        Mat FL(*nv,*nl, fl);
        FL = -self->getSystem()->evalV();
      }
    }
    else if(*ifcn==11) {
      Vec ih(*nl, gt);
      ih = self->getSystem()->evalW().T()*self->getSystem()->getu()-self->getSystem()->evalgd();
      Mat M(*lrda,*nv, am);
      M(RangeV(0,*nv-1),RangeV(0,*nv-1)) = self->getSystem()->evalM();
      Mat GQ(*nl,*nv, gp);
      GQ = -self->getSystem()->evalW().T();
      if(self->generalVMatrix) {
        Mat FL(*nv,*nl, fl);
        FL = -self->getSystem()->evalV();
      }
    }
  }

  void PHEM56Integrator::solout(int* nr, int* nq, int* nv, int* nu, int* nl, int* lrdo, double* q, double* v, double* u, double* a, double* rlam, double* dowk, int* irtrn) {
    if(*nr>1) {
      auto self=*reinterpret_cast<PHEM56Integrator**>(&nq[1]);

      double told = dowk[0];
      double t = told + dowk[1];

      double curTimeAndState = -1;
      double tRoot = t;
      // root-finding
      if(self->getSystem()->getsvSize()) {
        self->getSystem()->setTime(t);
        curTimeAndState = t;
        self->getSystem()->setq(Vec(nq[0], q));
        self->getSystem()->setu(Vec(*nv, v));
        self->getSystem()->setx(Vec(*nu, u));
        self->getSystem()->resetUpToDate();
        self->shift = self->signChangedWRTsvLast(self->getSystem()->evalsv());
        // if a root exists in the current step ...
        if(self->shift) {
          // ... search the first root and set step.second to this time
          double dt = t-told;
          while(dt>self->dtRoot) {
            dt/=2;
            double tCheck = tRoot-dt;
            self->getSystem()->setTime(tCheck);
            curTimeAndState = tCheck;
            int first = true;
            for(int i=1; i<=self->system->getzSize(); i++)
              self->getSystem()->getState()(i-1) = POL4(&i,&first,nq,nv,nu,lrdo,&tCheck,dowk);
            self->getSystem()->resetUpToDate();
            if(self->signChangedWRTsvLast(self->getSystem()->evalsv()))
              tRoot = tCheck;
          }
          if(curTimeAndState != tRoot) {
            curTimeAndState = tRoot;
            self->getSystem()->setTime(tRoot);
            int first = true;
            for(int i=1; i<=self->system->getzSize(); i++)
              self->getSystem()->getState()(i-1) = POL4(&i,&first,nq,nv,nu,lrdo,&tRoot,dowk);
          }
          self->getSystem()->resetUpToDate();
          auto &sv = self->getSystem()->evalsv();
          auto &jsv = self->getSystem()->getjsv();
          for(int i=0; i<sv.size(); ++i)
            jsv(i)=self->svLast(i)*sv(i)<0;
        }
      }

      while(tRoot >= self->tPlot) {
        if(curTimeAndState != self->tPlot) {
          curTimeAndState = self->tPlot;
          self->getSystem()->setTime(self->tPlot);
          int first = true;
          for(int i=1; i<=self->system->getzSize(); i++)
            self->getSystem()->getState()(i-1) = POL4(&i,&first,nq,nv,nu,lrdo,&self->tPlot,dowk);
        }
        self->getSystem()->resetUpToDate();
        self->getSystem()->plot();
        if(self->msgAct(Status))
          self->msg(Status) << "   t = " <<  self->tPlot << ",\tdt = "<< t-told << flush;

        double s1 = clock();
        self->time += (s1-self->s0)/CLOCKS_PER_SEC;
        self->s0 = s1; 

        self->tPlot += self->dtOut;
      }

      if(self->shift) {
        // shift the system
        if(curTimeAndState != tRoot) {
          self->getSystem()->setTime(tRoot);
          int first = true;
          for(int i=1; i<=self->system->getzSize(); i++)
            self->getSystem()->getState()(i-1) = POL4(&i,&first,nq,nv,nu,lrdo,&tRoot,dowk);
        }
        if(self->plotOnRoot) {
          self->getSystem()->resetUpToDate();
          self->getSystem()->plot();
        }
        self->getSystem()->resetUpToDate();
        self->getSystem()->shift();
        self->system->calcgdSize(3); // IH
        self->system->updategdRef(self->system->getgdParent()(0,self->system->getgdSize()-1));
        if(self->plotOnRoot) {
          self->getSystem()->resetUpToDate();
          self->getSystem()->plot();
        }
        self->getSystem()->resetUpToDate();
        self->svLast=self->getSystem()->evalsv();
        *irtrn = -1;
      }
      else {
        // check drift
        bool projVel = true;
        if(self->getToleranceForPositionConstraints()>=0) {
          self->getSystem()->setTime(t);
          self->getSystem()->setq(Vec(nq[0], q));
          self->getSystem()->setu(Vec(*nv, v));
          self->getSystem()->setx(Vec(*nu, u));
          self->getSystem()->resetUpToDate();
          if(self->getSystem()->positionDriftCompensationNeeded(self->getToleranceForPositionConstraints())) { // project both, first positions and then velocities
            self->getSystem()->projectGeneralizedPositions(3);
            self->getSystem()->projectGeneralizedVelocities(3);
            projVel = false;
            *irtrn=-1;
          }
        }
        if(self->getToleranceForVelocityConstraints()>=0 and projVel) {
          self->getSystem()->setTime(t);
          self->getSystem()->setq(Vec(nq[0], q));
          self->getSystem()->setu(Vec(*nv, v));
          self->getSystem()->setx(Vec(*nu, u));
          self->getSystem()->resetUpToDate();
          if(self->getSystem()->velocityDriftCompensationNeeded(self->getToleranceForVelocityConstraints())) { // project velicities
            self->getSystem()->projectGeneralizedVelocities(3);
            *irtrn=-1;
          }
        }
      }
    }
  }

  void PHEM56Integrator::integrate() {
    if(linearAlgebra==unknown)
      throwError("(PHEM56Integrator::integrate): linear algebra unknown");

    debugInit();

    int nq[1+sizeof(void*)/sizeof(int)+1];
    nq[0] = system->getqSize();
    int nv = system->getuSize();
    int nu = system->getxSize();
    int nl = system->getlaSize();

    double t = tStart;

    Vec ud(nv);
    Vec la(nl);
    Vec z(system->getzSize());
    Vec q = z(0,nq[0]-1);
    Vec u = z(nq[0],nq[0]+nv-1);
    Vec x = z(nq[0]+nv,nq[0]+nv+nu-1);
    if(z0.size()) {
      if(z0.size() != system->getzSize())
        throwError("(PHEM56Integrator::integrate): size of z0 does not match, must be " + to_string(system->getzSize()));
      z = z0;
    }
    else
      z = system->evalz0();

    system->setTime(t);
    system->setq(q);
    system->setu(u);
    system->setx(x);
    system->resetUpToDate();
    system->computeInitialCondition();
    system->calcgdSize(3); // IH
    system->updategdRef(system->getgdParent()(0,system->getgdSize()-1));
    if(initialProjection or numberOfStepsBetweenProjections) {
      system->calcgSize(2); // IB
      system->updategRef(system->getgParent()(0,system->getgSize()-1));
    }
    system->plot();
    svLast = system->evalsv();

    nl = system->getlaSize();

    if(aTol.size() == 0)
      aTol.resize(1,INIT,1e-6);
    if(rTol.size() == 0)
      rTol.resize(1,INIT,1e-6);

    int iTol;
    if(aTol.size() == 1)
      iTol = 0;
    else {
      iTol = 1;
      if(aTol.size() != system->getzSize())
        throwError("(PHEM56Integrator::integrate): size of aTol does not match, must be " + to_string(system->getzSize()));
    }
    if(rTol.size() != aTol.size())
      throwError("(PHEM56Integrator::integrate): size of rTol does not match aTol, must be " + to_string(aTol.size()));

    int iout = 2;

    int liwk = 95+2*(nv+nl)+2*1000;
    VecInt iwk(liwk);
    int lrwk = 19+27*nq[0]+28*nv+27*nu+5*(nv+nl)+2*1000+8*nl*nv+4*(nl+nv)*(nl+nv);
    Vec rwk(lrwk);

    rwk(4) = dtMax; // maximum step size
    iwk(10) = maxSteps; // step limit
    iwk(11) = initialProjection; // initial projection
    iwk(12) = numberOfStepsBetweenProjections; // number of steps between projection
    iwk(13) = 2*linearAlgebra + generalVMatrix; // mode
    iwk(14) = projectOntoIndex1ConstraintManifold; // project onto index 1 constraint manifold

    PHEM56Integrator *self=this;
    memcpy(&nq[1], &self, sizeof(void*));

    double h = dt0;

    int idid;

    tPlot = t + dtPlot;
    dtOut = dtPlot;

    s0 = clock();

    while(t<tEnd-epsroot) {

      PHEM56(nq,&nv,&nu,&nl,fprob,&t,q(),u(),x(),ud(),la(),&tEnd,&h,rTol(),aTol(),&iTol,solout,&iout,rwk(),&lrwk,iwk(),&liwk,&idid);

      if(shift) {
        h = dt0;
        nl = system->getlaSize();
      }

      t = system->getTime();
      z = system->getState();
    }
  }

  void PHEM56Integrator::initializeUsingXML(DOMElement *element) {
    RootFindingIntegrator::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"absoluteTolerance");
    if(e) setAbsoluteTolerance(E(e)->getText<Vec>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"absoluteToleranceScalar");
    if(e) setAbsoluteTolerance(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"relativeTolerance");
    if(e) setRelativeTolerance(E(e)->getText<Vec>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"relativeToleranceScalar");
    if(e) setRelativeTolerance(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"initialStepSize");
    if(e) setInitialStepSize(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"maximumStepSize");
    if(e) setMaximumStepSize(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"stepLimit");
    if(e) setStepLimit(E(e)->getText<int>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"linearAlgebra");
    if(e) {
      string linearAlgebraStr=string(X()%E(e)->getFirstTextChild()->getData()).substr(1,string(X()%E(e)->getFirstTextChild()->getData()).length()-2);
      if(linearAlgebraStr=="DEC") linearAlgebra=DEC;
      else if(linearAlgebraStr=="DGETRF") linearAlgebra=DGETRF;
      else linearAlgebra=unknown;
    }
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"generalVMatrix");
    if(e) setGeneralVMatrix(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"initialProjection");
    if(e) setInitialProjection(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"numberOfStepsBetweenProjections");
    if(e) setNumberOfStepsBetweenProjections(E(e)->getText<int>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"projectOntoIndex1ConstraintManifold");
    if(e) setProjectOntoIndex1ConstraintManifold(E(e)->getText<bool>());
  }

}
