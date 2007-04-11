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

#ifndef _BODY_H_
#define _BODY_H_

#include "object.h"

namespace MBSim {

  /*! 
   *  \brief base class for all mechanical bodies with mass and generalised coordinates
   * 
   * */
  class Body : public Object {

    protected:
      double m;

    public:
      Body(const string &name);
      void init();

      // ATTIC CODE - implemented in object
      /*     /\* !  */
      /*       compute kinetic energy, which is the quadratic form \f$\frac{1}{2}\boldsymbol{u}^T\boldsymbol{M}\boldsymbol{u}\f$ for all bodies */
      /*      *\/ */
      /*     double computeKineticEnergy() { return 0.5*trans(u)*M*u; } */

      /*     /\*! compute potential energy, holding every potential!!! */
      /*      *\/ */
      /*     virtual double computePotentialEnergy() {return 0;} */
      // ATTIC CODE - implemented in object

  };

}

#endif
