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
 *   mfoerg@users.berlios.de
 *
 */

#ifndef _RIGID_CONTACT_H_
#define _RIGID_CONTACT_H_

#include "contact.h"
#include "constitutive_laws.h"

namespace MBSim {
 
  /*! \brief Class for rigid contacts
   *
   * */
  class RigidContact : public Contact {

    protected:

      double argN;
      Vec argT;

      void checkActive();

      ConstraintLaw *fcl;
      DryFriction *fdf;
      NormalImpactLaw *fnil;
      TangentialImpactLaw *ftil;

      Vec gdd;

    public: 

      RigidContact(const string &name);

      void init();

      void calcsvSize();

      void load(const string& path, ifstream &inputfile);
      void save(const string &path, ofstream &outputfile);

      void solveImpactFixpointSingle();
      void solveContactFixpointSingle();
      void solveImpactGaussSeidel();
      void solveContactGaussSeidel();
      void solveImpactRootFinding();
      void solveContactRootFinding();
      void jacobianContact();
      void jacobianImpact();

      void updaterFactors();

      void updateCondition();

      
      void checkContactForTermination();
      void checkImpactForTermination();

      std::string getTerminationInfo(double dt);

      void updateW(double t);
      void updateV(double t);
      void updatewb(double t);

      void updateStopVector(double t);

      void setNormalImpactLaw(NormalImpactLaw *fnil_) {fnil = fnil_;}
      void setTangentialImpactLaw(TangentialImpactLaw *ftil_) {ftil = ftil_;}
      void setContactLaw(ConstraintLaw *fcl_) {fcl = fcl_;}
      void setFrictionLaw(DryFriction *fdf_) {fdf = fdf_;}

      int getFrictionDirections() {return fdf ? fdf->getFrictionDirections() : 0;}

      string getType() const {return "RigidContact";}

      bool isActive() const {return gActive;}

      void checkActiveg();
      void checkActivegd();
      void checkActivegdn();
      void checkActivegdd(); 
      void checkAllgd();
  };

}

#endif   

