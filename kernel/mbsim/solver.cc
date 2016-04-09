/* Copyright (C) 2004-2014 MBSim Development Team
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
#include "solver.h"
#include "mbsim/dynamic_system_solver.h"

using namespace fmatvec;

namespace MBSim {

  DynamicSystemSolver * Solver::system = 0;

  void Solver::zdot(const Vec &zd, const Vec &z, double t) {
    if (system->getq()() != z()) system->updatezRef(z);
    if (system->getqd()() != zd()) system->updatezdRef(zd);
    system->setTime(t);
    system->resetUpToDate();
    if (system->getlaSize()) system->computeConstraintForces();
    system->updatezd();
  }

  Vec Solver::zdot(const Vec &z, double t) {
    if (system->getq()() != z()) system->updatezRef(z);
    system->setTime(t);
    system->resetUpToDate();
    if (system->getlaSize()) system->computeConstraintForces();
    return system->evalzd();
  }

  void Solver::plot(const Vec &z, double t) {
    if (system->getq()() != z()) system->updatezRef(z);
    system->updatezdRef();
    system->setTime(t);
    system->resetUpToDate();
    if (system->getlaSize()) {
      if(system->getUseConstraintSolverForPlot()) {
        system->getb(false) = system->evalW().T() * slvLLFac(system->evalLLM(), system->evalh()) + system->evalwb();
        system->solveConstraints();
      }
      else
        system->computeConstraintForces();
    }
    system->updatezd();
    system->computeInverseKinetics();
    system->plot();
  }

  void Solver::stopVector(const Vec& z, Vec& sv, double t) {
    if (system->getq()() != z()) system->updatezRef(z);
    if (system->getsv(false)() != sv()) system->updatesvRef(sv);
    system->setTime(t);
    system->resetUpToDate();
    if (system->getlaSize()) {
      system->getb(false) << system->evalW().T() * slvLLFac(system->evalLLM(), system->evalh()) + system->evalwb();
      system->solveConstraints();
    }
    system->updateStopVector();
    sv(sv.size() - 1) = 1;
  }

}
