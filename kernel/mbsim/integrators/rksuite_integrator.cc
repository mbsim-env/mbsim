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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include <mbsim/dynamic_system_solver.h>
#include "rksuite_integrator.h"
#include <mbsim/utils/eps.h>
#include "fortran/fortran_wrapper.h"
#include <time.h>

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimIntegrator {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMINT, RKSuiteIntegrator)

  void RKSuiteIntegrator::preIntegrate() {
    if(method==unknownMethod)
      throwError("(RKSuiteIntegrator::integrate): method unknown");
    if(task==unknownTask)
      throwError("(RKSuiteIntegrator::integrate): task unknown");

    debugInit();

    if(selfStatic)
      throwError("RKSuiteIntegrator can only integrate one system.");
    selfStatic = this;
    zSize=system->getzSize();

    if(not zSize)
      throwError("(RKSuiteIntegrator::integrate): dimension of the system must be at least 1");

    z.resize(zSize);
    if(z0.size()) {
      if(z0.size() != zSize)
        throwError("(RKSuiteIntegrator::integrate): size of z0 does not match, must be " + toStr(zSize));
      z = z0;
    }
    else
      z = system->evalz0();

    if(thres.size() == 0)
      thres.resize(zSize,INIT,1e-10);
    else if(thres.size() == 1) {
      double thres_ = thres(0);
      thres.resize(zSize,INIT,thres_);
    } 
    if(thres.size() != zSize)
      throwError("(RKSuiteIntegrator::integrate): size of thres does not match, must be " + toStr(zSize));

    lenwrk = 2*32*zSize;
    work = new double[lenwrk];
    lenint = 2*6*zSize;
    workint = new double[lenint];
    messages = 1;

    zd.resize(zSize);
    zMax.resize(zSize);
    zWant.resize(zSize);
    zdWant.resize(zSize);

    t = tStart;
    system->setTime(t);
    system->setState(z);
    system->resetUpToDate();
    system->computeInitialCondition();
    system->plot();
    svLast = system->evalsv();

    tPlot = t + dtPlot;
    if(plotIntegrationData) integPlot.open((name + ".plt").c_str());

    integrationSteps = 0;

    s0 = clock();
    time = 0;
  }

  bool RKSuiteIntegrator::signChangedWRTsvLast(const fmatvec::Vec &svStepEnd) const {
    for(int i=0; i<svStepEnd.size(); i++)
      if(svLast(i)*svStepEnd(i)<0)
        return true;
    return false;
  }

  void RKSuiteIntegrator::subIntegrate(double tStop) {

    int result=0, errass=0;
    char taskStr = task==usual?'U':'C';
    char request = 'S';
    int method_ = method;

    SETUP(&zSize, &t, z(), &tStop, &rTol, thres(), &method_, &taskStr,
        &errass, &dt0, work, &lenwrk, &messages);

    if(task==complex) {
      while(t<tStop-epsroot) {

        integrationSteps++;

        double dtLast = 0;
        CT(fzdot, &t, z(), zd(), work, &result, &dtLast);

        int n = system->getzSize();
        bool restart = false;
        if(result==1 or result==2) {
          double curTimeAndState = -1;
          double tRoot = t;
          // root-finding
          if(getSystem()->getsvSize()) {
            getSystem()->setTime(t);
            curTimeAndState = t;
            getSystem()->setState(z);
            getSystem()->resetUpToDate();
            shift = signChangedWRTsvLast(getSystem()->evalsv());
            // if a root exists in the current step ...
            double dt = dtLast;
            if(shift) {
              // ... search the first root and set step.second to this time
              while(dt>1e-10) {
                dt/=2;
                double tCheck = tRoot-dt;
                curTimeAndState = tCheck;
                INTRP(&tCheck,&request,&n,zWant(),zdWant(),fzdot,work,workint,&lenint);
                getSystem()->setTime(tCheck);
                getSystem()->setState(zWant);
                getSystem()->resetUpToDate();
                if(signChangedWRTsvLast(getSystem()->evalsv()))
                  tRoot = tCheck;
              }
              if(curTimeAndState != tRoot) {
                curTimeAndState = tRoot;
                INTRP(&tRoot,&request,&n,zWant(),zdWant(),fzdot,work,workint,&lenint);
                getSystem()->setTime(tRoot);
                getSystem()->setState(zWant);
              }
              getSystem()->resetUpToDate();
              auto &sv = getSystem()->evalsv();
              auto &jsv = getSystem()->getjsv();
              for(int i=0; i<sv.size(); ++i)
                jsv(i)=svLast(i)*sv(i)<0;
            }
          }

          while(tRoot >= tPlot) {
            if(curTimeAndState != tPlot) {
              curTimeAndState = tPlot;
              INTRP(&tPlot,&request,&n,zWant(),zdWant(),fzdot,work,workint,&lenint);
              getSystem()->setTime(tPlot);
              getSystem()->setState(zWant);
            }
            getSystem()->resetUpToDate();
            getSystem()->plot();
            if(msgAct(Status))
              msg(Status) << "   t = " <<  tPlot << ",\tdt = "<< dtLast << flush;

            double s1 = clock();
            time += (s1-s0)/CLOCKS_PER_SEC;
            s0 = s1;

            if(plotIntegrationData) integPlot<< tPlot << " " << dtLast << " " << time << endl;
            tPlot += dtPlot;
          }

          if(shift) {
            // shift the system
            if(curTimeAndState != tRoot) {
              INTRP(&tRoot,&request,&n,zWant(),zdWant(),fzdot,work,workint,&lenint);
              getSystem()->setTime(tRoot);
              getSystem()->setState(zWant);
            }
            if(plotOnRoot) {
              getSystem()->resetUpToDate();
              getSystem()->plot();
            }
            getSystem()->resetUpToDate();
            getSystem()->shift();
            if(plotOnRoot) {
              getSystem()->resetUpToDate();
              getSystem()->plot();
            }
            getSystem()->resetUpToDate();
            svLast=getSystem()->evalsv();
            restart = true;
          }
          else {
            // check drift
            bool projVel = true;
            if(gMax>=0) {
              getSystem()->setTime(t);
              getSystem()->setState(z);
              getSystem()->resetUpToDate();
              if(getSystem()->positionDriftCompensationNeeded(gMax)) { // project both, first positions and then velocities
                getSystem()->projectGeneralizedPositions(3);
                getSystem()->projectGeneralizedVelocities(3);
                projVel = false;
                restart = true;
              }
            }
            if(gdMax>=0 and projVel) {
              getSystem()->setTime(t);
              getSystem()->setState(z);
              getSystem()->resetUpToDate();
              if(getSystem()->velocityDriftCompensationNeeded(gdMax)) { // project velicities
                getSystem()->projectGeneralizedVelocities(3);
                restart = true;
              }
            }
          }
          if(restart) {
//            cout << "t = " << t << endl;
            cout << "tSys = " << system->getTime() << endl;
//            cout << "tStop = " << tStop << endl;
            t = system->getTime();
            z = system->getState();
            SETUP(&zSize, &t, z(), &tStop, &rTol, thres(), &method_, &taskStr,
                &errass, &dt0, work, &lenwrk, &messages);
          }
        }
        else if(result==3 or result==4)
          continue;
        else if(result>=5)
          throwError("Integrator RKSUITE failed with result = "+toString(result));
      }
    }
    else {
      while(t<tStop-epsroot) {
        integrationSteps++;

        double dtLast = 0;

        UT(fzdot, &tPlot, &t, z(), zd(), zMax(), work, &result, &dtLast);

        if(result==1 or result==2) {
          system->setTime(t);
          system->setState(z);
          system->resetUpToDate();
          system->plot();

          if(msgAct(Status)) msg(Status) << "   t = " <<  t << ",\tdt = "<< dtLast << flush;

          const double s1 = clock();
          time += (s1-s0)/CLOCKS_PER_SEC;
          s0 = s1; 
          if(plotIntegrationData) integPlot<< t << " " << dtLast << " " << time << endl;

          tPlot = min(tStop, tPlot+dtPlot);

          // check drift
          bool restart = false;
          if(gMax>=0 and system->positionDriftCompensationNeeded(gMax)) { // project both, first positions and then velocities
            system->projectGeneralizedPositions(3);
            system->projectGeneralizedVelocities(3);
            restart = true;
          }
          else if(gdMax>=0 and system->velocityDriftCompensationNeeded(gdMax)) { // project velicities
            system->projectGeneralizedVelocities(3);
            restart = true;
          }
          if(restart) {
            z = system->getState();
            SETUP(&zSize, &t, z(), &tStop, &rTol, thres(), &method_, &taskStr,
                &errass, &dt0, work, &lenwrk, &messages);
          }
        }
        else if(result==3 or result==4)
          continue;
        else if(result>=5)
          throwError("Integrator RKSUITE failed with result = "+toString(result));
      }
    }
  }

  void RKSuiteIntegrator::postIntegrate() {
    if(plotIntegrationData) integPlot.close();

    if(writeIntegrationSummary) {
      ofstream integSum((name + ".sum").c_str());
      integSum << "Integration time: " << time << endl;
      integSum << "Integration steps: " << integrationSteps << endl;
      integSum.close();
    }

    selfStatic = nullptr;
  }

  void RKSuiteIntegrator::integrate() {
    preIntegrate();
    subIntegrate(tEnd);
    postIntegrate();
  }

  void RKSuiteIntegrator::initializeUsingXML(DOMElement *element) {
    Integrator::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"method");
    if(e) {
      string methodStr=string(X()%E(e)->getFirstTextChild()->getData()).substr(1,string(X()%E(e)->getFirstTextChild()->getData()).length()-2);
      if(methodStr=="RK23") method=RK23;
      else if(methodStr=="RK45") method=RK45;
      else if(methodStr=="RK78") method=RK78;
      else method=unknownMethod;
    }
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"task");
    if(e) {
      string taskStr=string(X()%E(e)->getFirstTextChild()->getData()).substr(1,string(X()%E(e)->getFirstTextChild()->getData()).length()-2);
      if(taskStr=="usual") task=usual;
      else if(taskStr=="complex") task=complex;
      else task=unknownTask;
    }
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"relativeToleranceScalar");
    if(e) setRelativeTolerance(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"threshold");
    if(e) setThreshold(E(e)->getText<Vec>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"thresholdScalar");
    if(e) setThreshold(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"initialStepsize");
    if(e) setInitialStepSize(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"plotOnRoot");
    if(e) setPlotOnRoot(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"toleranceForPositionConstraints");
    if(e) setToleranceForPositionConstraints(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"toleranceForVelocityConstraints");
    if(e) setToleranceForVelocityConstraints(E(e)->getText<double>());
  }

  void RKSuiteIntegrator::fzdot(double* t, double* z_, double* zd_) {
    Vec zd(selfStatic->zSize, zd_);
    selfStatic->getSystem()->setTime(*t);
    selfStatic->getSystem()->setState(Vec(selfStatic->zSize, z_));
    selfStatic->getSystem()->resetUpToDate();
    zd = selfStatic->getSystem()->evalzd();
  }

  RKSuiteIntegrator *RKSuiteIntegrator::selfStatic = nullptr;

}
