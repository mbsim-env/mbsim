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

#ifndef _GROUP_H_
#define _GROUP_H_

#include <mbsim/subsystem.h>

namespace MBSim {

  /**
   * \brief group ingredients do not depend on each other
   * \author Martin Foerg
   * \date 2009-03-26 some comments (Thorsten Schindler)
   */
  class Group : public Subsystem {
    public:
      /**
       * \brief constructor
       * \param name of group
       */
      Group(const string &name);

      /**
       * \brief destructor
       */
      virtual ~Group();

      /* INHERITED INTERFACE OF SUBSYSTEM */
      virtual void updateJacobians(double t);
      virtual void facLLM();
      using Subsystem::addObject;
      /***************************************************/

      /* INHERITED INTERFACE OF OBJECTINTERFACE */
      virtual void updateKinematics(double t);
      virtual void updatedu(double t, double dt);
      virtual void updatezd(double t);
      virtual void updateSecondJacobians(double t);
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual void load(const string &path, ifstream &inputfile);
      virtual void save(const string &path, ofstream &outputfile);
      virtual string getType() const { return "Group"; }
      /***************************************************/

      /**
       * \param subsystem to add
       * \param relative position of subsystem
       * \param relative orientation of subsystem
       * \param relation frame
       */
      void addSubsystem(Subsystem *subsystem, const Vec &RrRK, const SqrMat &ARK, const Frame* refFrame=0);

      // TODO delete compatibility functions
      void addObject(TreeRigid *tree);
      void addObject(BodyRigid *body);
  };
}

#endif

