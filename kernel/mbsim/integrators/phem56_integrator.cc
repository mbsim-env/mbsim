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

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, PHEM56Integrator)

  void PHEM56Integrator::fprob(int* ifcn, int* nq, int* nu, int* nx, int* nla, int* nzg, int* nzf, int* lrda, int* nblk, int* nmrc,  int* npgp, int* npfl, int* indgr, int* indgc, int* indflr, int* indflc,  double* t, double* q, double* u, double* x, double* xl, double* g_, double* WT_, double* f, double* wb_, double* deltagd_, double* V_, double* qd_, double* xd_, double* M_) {
    auto self=*reinterpret_cast<PHEM56Integrator**>(&nq[1]);
    if(self->exception) // if a exception was already thrown in a call before -> do nothing and return
      return;
    try { // catch exception -> C code must catch all exceptions
      self->system->setTime(*t);
      self->system->setq(Vec(*nq,q));
      self->system->setu(Vec(*nu,u));
      self->system->setx(Vec(*nx,x));
      self->system->resetUpToDate();
      if(*ifcn==0) {
        Vec xd(*nx, xd_);
        self->system->updatexd();
        xd = self->system->getxd(false);
      }
      else if(*ifcn==1) {
        Vec h(*nu, f);
        h = self->system->evalh();
        Mat M(*lrda,*nu, M_);
        M.set(self->Rv,self->Rv, self->system->evalM());
        Vec qd(nq[0], qd_);
        self->system->updateqd();
        qd = self->system->getqd(false);;
        if(self->generalVMatrix) {
          Mat V(*nu,*nla, V_);
          V = -self->system->evalV();
        }
        else {
          Mat WT(*nla,*nu, WT_);
          WT = -self->system->evalW().T();
        }
      }
      else if(*ifcn==2) {
        Vec qd(nq[0], qd_);
        self->system->updateqd();
        qd = self->system->getqd(false);
      }
      else if(*ifcn==3) {
        Vec deltagd(*nla, deltagd_);
        deltagd = self->system->evalW().T()*self->system->getu()-self->system->evalgd();
      }
      else if(*ifcn==4) {
        Vec g(*nla, g_);
        g = -self->system->evalg();
      }
      else if(*ifcn==5) {
        Vec h(*nu, f);
        h = self->system->evalh();
        Vec wb(*nla, wb_);
        wb = -self->system->evalwb();
      }
      else if(*ifcn==6) {
        Mat WT(*nla,*nu, WT_);
        WT = -self->system->evalW().T();
        Vec deltagd(*nla, deltagd_);
        deltagd = self->system->evalW().T()*self->system->getu()-self->system->evalgd();
      }
      else if(*ifcn==7) {
        Vec h(*nu, f);
        h = self->system->evalh();
        Mat M(*lrda,*nu, M_);
        M.set(self->Rv,self->Rv, self->system->evalM());
        Vec wb(*nla, wb_);
        wb = -self->system->evalwb();
      }
      else if(*ifcn==8) {
        Vec h(*nu, f);
        h = self->system->evalh();
        Mat M(*lrda,*nu, M_);
        M.set(self->Rv,self->Rv, self->system->evalM());
        if(self->generalVMatrix) {
          Mat V(*nu,*nla, V_);
          V = -self->system->evalV();
        }
      }
      else if(*ifcn==9) {
        Mat M(*lrda,*nu, M_);
        M.set(self->Rv,self->Rv, self->system->evalM());
      }
      else if(*ifcn==10) {
        Vec deltagd(*nla, deltagd_);
        deltagd = self->system->evalW().T()*self->system->getu()-self->system->evalgd();
        Mat WT(*nla,*nu, WT_);
        WT = -self->system->evalW().T();
        Mat M(*lrda,*nu, M_);
        M.set(self->Rv,self->Rv, self->system->evalM());
        Vec qd(nq[0], qd_);
        self->system->updateqd();
        qd = self->system->getqd(false);
        if(self->generalVMatrix) {
          Mat V(*nu,*nla, V_);
          V = -self->system->evalV();
        }
      }
      else if(*ifcn==11) {
        Vec deltagd(*nla, deltagd_);
        deltagd = self->system->evalW().T()*self->system->getu()-self->system->evalgd();
        Mat M(*lrda,*nu, M_);
        M.set(self->Rv,self->Rv, self->system->evalM());
        Mat WT(*nla,*nu, WT_);
        WT = -self->system->evalW().T();
        if(self->generalVMatrix) {
          Mat V(*nu,*nla, V_);
          V = -self->system->evalV();
        }
      }
    }
    catch(...) { // if a exception is thrown catch and store it in self
      self->exception = current_exception();
    }
  }

  void PHEM56Integrator::solout(int* nr, int* nq, int* nu, int* nx, int* nla, int* lrdo, double* q, double* u, double* x, double* a, double* rlam, double* dowk, int* irtrn) {
    if(*nr>1) {
      auto self=*reinterpret_cast<PHEM56Integrator**>(&nq[1]);
      if(self->exception) { // if a exception was already thrown in a call before -> do nothing but set interrupt flag of DOPRI5 and return
        *irtrn=-1;
        return;
      }
      try { // catch exception -> C code must catch all exceptions
        double told = dowk[0];
        double t = told + dowk[1];

        double curTimeAndState = numeric_limits<double>::min(); // just a value which will never be reached
        double tRoot = t;

        // root-finding
        if(self->system->getsvSize()) {
          self->system->setTime(t);
          curTimeAndState = t;
          self->system->setq(Vec(nq[0], q));
          self->system->setu(Vec(*nu, u));
          self->system->setx(Vec(*nx, x));
          self->system->resetUpToDate();
          self->shift = self->signChangedWRTsvLast(self->system->evalsv());
          // if a root exists in the current step ...
          if(self->shift) {
            // ... search the first root and set step.second to this time
            double dt = t-told;
            while(dt>self->dtRoot) {
              dt/=2;
              double tCheck = tRoot-dt;
              self->system->setTime(tCheck);
              curTimeAndState = tCheck;
              int first = true;
              for(int i=1; i<=self->system->getzSize(); i++)
                self->system->getState()(i-1) = POL4(&i,&first,nq,nu,nx,lrdo,&tCheck,dowk);
              self->system->resetUpToDate();
              if(self->signChangedWRTsvLast(self->system->evalsv()))
                tRoot = tCheck;
            }
            if(curTimeAndState != tRoot) {
              curTimeAndState = tRoot;
              self->system->setTime(tRoot);
              int first = true;
              for(int i=1; i<=self->system->getzSize(); i++)
                self->system->getState()(i-1) = POL4(&i,&first,nq,nu,nx,lrdo,&tRoot,dowk);
            }
            self->system->resetUpToDate();
            auto &sv = self->system->evalsv();
            auto &jsv = self->system->getjsv();
            for(int i=0; i<sv.size(); ++i)
              jsv(i)=self->svLast(i)*sv(i)<0;
          }
        }

        while(tRoot >= self->tPlot) {
          if(curTimeAndState != self->tPlot) {
            curTimeAndState = self->tPlot;
            self->system->setTime(self->tPlot);
            int first = true;
            for(int i=1; i<=self->system->getzSize(); i++)
              self->system->getState()(i-1) = POL4(&i,&first,nq,nu,nx,lrdo,&self->tPlot,dowk);
          }
          self->system->resetUpToDate();
          self->system->plot();
          if(self->msgAct(Status))
            self->msg(Status) << "   t = " <<  self->tPlot << ",\tdt = "<< t-told << flush;

          self->system->updateInternalState();

          double s1 = clock();
          self->time += (s1-self->s0)/CLOCKS_PER_SEC;
          self->s0 = s1;

          self->tPlot += self->dtOut;
        }

        if(self->shift) {
          // shift the system
          if(curTimeAndState != tRoot) {
            self->system->setTime(tRoot);
            int first = true;
            for(int i=1; i<=self->system->getzSize(); i++)
              self->system->getState()(i-1) = POL4(&i,&first,nq,nu,nx,lrdo,&tRoot,dowk);
          }
          if(self->plotOnRoot) {
            self->system->resetUpToDate();
            self->system->plot();
          }
          self->system->resetUpToDate();
          self->system->shift();
          self->system->calcgdSize(3); // IH
          self->system->updategdRef(self->system->getgdParent());
          if(self->plotOnRoot) {
            self->system->resetUpToDate();
            self->system->plot();
          }
          self->system->resetUpToDate();
          self->svLast=self->system->evalsv();
          *irtrn = -1;
        }
        else {
          // check drift
          bool projVel = true;
          if(self->getToleranceForPositionConstraints()>=0) {
            self->system->setTime(t);
            self->system->setq(Vec(nq[0], q));
            self->system->setu(Vec(*nu, u));
            self->system->setx(Vec(*nx, x));
            self->system->resetUpToDate();
            if(self->system->positionDriftCompensationNeeded(self->getToleranceForPositionConstraints())) { // project both, first positions and then velocities
              self->system->projectGeneralizedPositions(3);
              self->system->projectGeneralizedVelocities(3);
              projVel = false;
              self->drift = true;
              *irtrn=-1;
            }
          }
          if(self->getToleranceForVelocityConstraints()>=0 and projVel) {
            self->system->setTime(t);
            self->system->setq(Vec(nq[0], q));
            self->system->setu(Vec(*nu, u));
            self->system->setx(Vec(*nx, x));
            self->system->resetUpToDate();
            if(self->system->velocityDriftCompensationNeeded(self->getToleranceForVelocityConstraints())) { // project velicities
              self->system->projectGeneralizedVelocities(3);
              self->drift = true;
              *irtrn=-1;
            }
          }
          self->system->updateStopVectorParameters();
        }

        self->system->updateInternalState();
      }
      catch(...) { // if a exception is thrown catch and store it in self and set the interrupt flag of DOPRI5
        self->exception = current_exception();
        *irtrn=-1;
      }
    }
  }

  void PHEM56Integrator::integrate() {
    if(linearAlgebra==unknown)
      throwError("(PHEM56Integrator::integrate): linear algebra unknown");

    debugInit();

    int nq[1+sizeof(void*)/sizeof(int)+1];
    nq[0] = system->getqSize();
    int nu = system->getuSize();
    int nx = system->getxSize();
    int nla = system->getlaSize();

    Rv = RangeV(0,nu-1);

    double t = tStart;

    Vec ud(nu);
    Vec la(nla);
    Vec z(system->getzSize());
    Vec q, u, x;
    q.ref(z,RangeV(0,system->getqSize()-1));
    u.ref(z,RangeV(system->getqSize(),system->getqSize()+system->getuSize()-1));
    x.ref(z,RangeV(system->getqSize()+system->getuSize(),system->getzSize()-1));
    if(z0.size()) {
      if(z0.size() != system->getzSize()+system->getisSize())
        throwError("(PHEM56Integrator::integrate): size of z0 does not match, must be " + to_string(system->getzSize()+system->getisSize()));
      z = z0(RangeV(0,system->getzSize()-1));
      system->setInternalState(z0(RangeV(system->getzSize(),z0.size()-1)));
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
    system->updategdRef(system->getgdParent());
    if(initialProjection or numberOfStepsBetweenProjections) {
      system->calcgSize(2); // IB
      system->updategRef(system->getgParent());
    }
    system->plot();
    svLast <<= system->evalsv();
    z = system->getState(); // needed, as computeInitialCondition may change the state

    nla = system->getlaSize();

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

    int liwk = 95+2*(nu+nla)+2*1000;
    VecInt iwk(liwk);
    int lrwk = 19+27*nq[0]+28*nu+27*nx+5*(nu+nla)+2*1000+8*nla*nu+4*(nla+nu)*(nla+nu);
    Vec rwk(lrwk);

    rwk(5) = dtMax; // maximum step size
    iwk(10) = maxSteps; // step limit
    iwk(11) = initialProjection; // initial projection
    iwk(12) = numberOfStepsBetweenProjections; // number of steps between projection
    iwk(13) = 2*linearAlgebra + generalVMatrix; // mode
    iwk(14) = projectOntoIndex1ConstraintManifold; // project onto index 1 constraint manifold

    exception=nullptr;

    PHEM56Integrator *self=this;
    memcpy(&nq[1], &self, sizeof(void*));

    double h = dt0;

    int idid = 0;

    tPlot = t + dtPlot;
    dtOut = dtPlot;

    s0 = clock();

    while(t<tEnd-epsroot) {
      drift = false;

      PHEM56(nq,&nu,&nx,&nla,fprob,&t,q(),u(),x(),ud(),la(),&tEnd,&h,rTol(),aTol(),&iTol,solout,&iout,rwk(),&lrwk,iwk(),&liwk,&idid);
      if(exception)
        rethrow_exception(exception);
      if(idid < 0)
        throw runtime_error("PHEM56 failed with idid = "+to_string(idid));

      if(shift) {
        h = dt0;
        nla = system->getlaSize();
      }

      if(shift || drift) {
        // set new state
        t = system->getTime();
        z = system->getState();
        h = dt0;
      }
    }

    msg(Info)<<"nrFCN: "<<iwk(33)<<endl;
    msg(Info)<<"nrGCN: "<<iwk(34)<<endl;
    msg(Info)<<"nrSteps: "<<iwk(30)<<endl;
    msg(Info)<<"nrStepsAccepted: "<<iwk(31)<<endl;
    msg(Info)<<"nrStepsRejected (excluding first step): "<<iwk(32)<<endl;
    msg(Info)<<"nrLUdecom: "<<iwk(36)<<endl;
    msg(Info)<<"nrForwardBackwardSubs: "<<iwk(37)<<endl;
  }

  void PHEM56Integrator::initializeUsingXML(DOMElement *element) {
    RootFindingIntegrator::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"absoluteTolerance");
    if(e) setAbsoluteTolerance(E(e)->getText<Vec>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"absoluteToleranceScalar");
    if(e) setAbsoluteTolerance(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"relativeTolerance");
    if(e) setRelativeTolerance(E(e)->getText<Vec>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"relativeToleranceScalar");
    if(e) setRelativeTolerance(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"initialStepSize");
    if(e) setInitialStepSize(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"maximumStepSize");
    if(e) setMaximumStepSize(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"stepLimit");
    if(e) setStepLimit(E(e)->getText<int>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"linearAlgebra");
    if(e) {
      string linearAlgebraStr=string(X()%E(e)->getFirstTextChild()->getData()).substr(1,string(X()%E(e)->getFirstTextChild()->getData()).length()-2);
      if(linearAlgebraStr=="DEC") linearAlgebra=DEC;
      else if(linearAlgebraStr=="DGETRF") linearAlgebra=DGETRF;
      else linearAlgebra=unknown;
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"generalVMatrix");
    if(e) setGeneralVMatrix(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"initialProjection");
    if(e) setInitialProjection(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"numberOfStepsBetweenProjections");
    if(e) setNumberOfStepsBetweenProjections(E(e)->getText<int>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"projectOntoIndex1ConstraintManifold");
    if(e) setProjectOntoIndex1ConstraintManifold(E(e)->getText<bool>());
  }

}
