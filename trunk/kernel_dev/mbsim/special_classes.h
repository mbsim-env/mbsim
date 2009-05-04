/* Copyright (C) 2004-2009 MBSim Development Team
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

#ifndef _SPECIAL_CLASSES_H
#define _SPECIAL_CLASSES_H

#include "mbsim/dynamic_system_solver.h"
#include "mbsim/rigid_body.h"
#include "mbsim/tree.h"
#include <string>

namespace MBSim {

  /*! 
   * \brief class for automatic tree build-up
   * \author Martin Foerg
   * \date 2009-04-03 some comments (Thorsten Schindler)
   */
  class SpecialGroup : public Group {
    protected:
      /**
       * \brief TODO
       */
      void addToTree(Tree* tree, Node* node, fmatvec::SqrMat &A, int i, std::vector<Object*> &objList);

    public:
      /**
       * \brief constructor
       * \param name of group
       */
      SpecialGroup(const std::string &name);

      /**
       * \brief TODO
       */
      void preinit();
  };

}

#endif

