/* Copyright (C) 2004-2008  Martin Förg
 
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
 *   mfoerg@users.berlios.de
 *
 */

#ifndef _RIGID_CONNECTION_H_
#define _RIGID_CONNECTION_H_

#include "connection.h"
#include "constitutive_laws.h"

namespace MBSim {

  /*! \brief Class for rigid connections
   *
   * */
  class RigidConnection : public Connection {

    protected:

      Mat fF[2], fM[2];

      ConstraintLaw *ffl, *fml;
      NormalImpactLaw *fifl, *fiml;

    public:

      RigidConnection(const string &name);

      void init();

      void setForceLaw(ConstraintLaw * rc) {ffl = rc;}
      void setMomentLaw(ConstraintLaw * rc) {fml = rc;}
      void setImpactForceLaw(NormalImpactLaw * rc) {fifl = rc;}
      void setImpactMomentLaw(NormalImpactLaw * rc) {fiml = rc;}

      void updateKinetics(double t);
      void updateW(double t);
      void projectJ(double dt);
      void projectGS(double dt);
      void solveGS(double dt);
      void checkForTermination(double dt);
      void updateb(double t);
      std::string getTerminationInfo(double dt);

      void residualProj(double dt);
      void residualProjJac(double dt);

      void updaterFactors();
  };

  typedef RigidConnection ConnectionRigid;
}

#endif
