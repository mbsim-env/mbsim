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

#ifndef _GROUP_H_
#define _GROUP_H_

#include "subsystem.h"

namespace MBSim {

  class Group : public Subsystem {
    protected:

   public:
      /*! Constructor */
      Group(const string &name);
      /*! Destructor */
      ~Group();
      void facLLM(); 
      void updatezd(double t);
      void updatedu(double t, double dt);
      void updateKinematics(double t);
      void updateJacobians(double t);
      void load(const string &path, ifstream &inputfile);
      void save(const string &path, ofstream &outputfile);

      void addSubsystem(Subsystem *subsystem, const Vec &RrRK, const SqrMat &ARK, const CoordinateSystem* refCoordinateSystem=0);
      void addObject(Object *object);

      // Compatibility functions
      void addObject(TreeRigid *tree);
      void addObject(BodyRigid *body);


      virtual string getType() const {return "Group";}
  };
}

#endif

