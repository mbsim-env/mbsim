#ifndef _PENDULUM_H
#define _PENDULUM_H

#include "mbsim/dynamic_system_solver.h"
#include <string>

class System : public MBSim::DynamicSystemSolver {
  public:
    System(const std::string &projectName); 
//    void getsv(const fmatvec::Vec& zParent, fmatvec::Vec& svExt, double t) {
//      if(sv()!=svExt()) {
//	updatesvRef(svExt);
//      }
//
//      if(q()!=zParent())
//	updatezRef(zParent);
//
//      if(qd()!=zdParent()) 
//	updatezdRef(zdParent);
//      sv.init(1);
//
//    }
//    fmatvec::Vec zdot(const fmatvec::Vec &zParent, double t) {
//    if(q()!=zParent()) {
//      updatezRef(zParent);
//    }
//    updateStateDependentVariables(t);
//    updateg(t);
//    updategd(t);
//    updateT(t); 
//    updateJacobians(t);
//    updateh(t); 
//    updateM(t); 
//    facLLM(); 
//    checkActiveg(); // Prüfen welche Kontakte geschlossen
//    checkActivegd(); // Prüfen welche Kontakte geschlossen bleiben und haften
//    checkActiveLinks(); // Nur für Performance: Nicht alle Links durchlaufen
//    //calcgdSizeActive();
//    calclaSize(); // Prüft nach gdActive
//    calcrFactorSize();
//    updateWRef(WParent(fmatvec::Index(0,getuSize()-1),fmatvec::Index(0,getlaSize()-1)));
//    updateVRef(VParent(fmatvec::Index(0,getuSize()-1),fmatvec::Index(0,getlaSize()-1)));
//    updatelaRef(laParent(0,laSize-1));
//    updatewbRef(wbParent(0,laSize-1));
//    updaterFactorRef(rFactorParent(0,rFactorSize-1));
//    std::cout << t << std::endl;
//    std::cout << "laSize = " << laSize << std::endl;
//    if(laSize) {
//      updateW(t); 
//      updateV(t); 
//      updateG(t); 
//      updatewb(t); 
//  b.resize() = W.T()*slvLLFac(LLM,h) + wb;
//     int iter;
//        iter = solveConstraints();
//    std::cout << "iter = " << iter << std::endl;
//    std::cout << la << std::endl;
////     computeConstraintForces(t); 
//    }
//    updater(t); 
//    updatezd(t);
//    std::cout <<  q << std::endl;
//    std::cout <<  u << std::endl;
//    std::cout <<  qd << std::endl;
//    std::cout <<  ud << std::endl;
//    std::cout << q(1) - 0.25 << std::endl;
//    std::cout <<  std::endl;
//
//    return zdParent;
//   }
//    virtual void plot(const fmatvec::Vec& zParent, double t, double dt=1) {
//   if(q()!=zParent()) {
//      updatezRef(zParent);
//    }
//
//    if(qd()!=zdParent()) 
//      updatezdRef(zdParent);
//    updateStateDependentVariables(t);
//    updateg(t);
//    updategd(t);
//    updateT(t); 
//    updateJacobians(t);
//    updateh(t); 
//    updateM(t); 
//    facLLM(); 
//    checkActiveg();
//    checkActivegd();
//    checkActiveLinks();
//    //calcgdSizeActive();
//    calclaSize();
//    calcrFactorSize();
//    updateWRef(WParent(fmatvec::Index(0,getuSize()-1),fmatvec::Index(0,getlaSize()-1)));
//    updateVRef(VParent(fmatvec::Index(0,getuSize()-1),fmatvec::Index(0,getlaSize()-1)));
//    updatelaRef(laParent(0,laSize-1));
//    updatewbRef(wbParent(0,laSize-1));
//    updaterFactorRef(rFactorParent(0,rFactorSize-1));
//    //std::cout << t << std::endl;
//    //std::cout << "laSize = " << laSize << std::endl;
//    if(laSize) {
//      updateW(t); 
//      updateV(t); 
//      updateG(t); 
//      updatewb(t); 
//  b.resize() = W.T()*slvLLFac(LLM,h) + wb;
//     int iter;
//        iter = solveConstraints();
//    //std::cout << "iter = " << iter << std::endl;
//    //std::cout << la << std::endl;
//    //std::cout <<  std::endl;
////     computeConstraintForces(t); 
//    }
//    DynamicSystemSolver::plot(t,dt);
//
//    if(++flushCount > flushEvery) {
//      flushCount = 0;
//      H5::FileSerie::flushAllFiles();
//    }
//
//    }
//
};

#endif

