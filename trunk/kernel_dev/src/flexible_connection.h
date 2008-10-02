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

#ifndef _FLEXIBLE_CONNECTION_H_
#define _FLEXIBLE_CONNECTION_H_

#include "connection.h"
#include "constitutive_laws.h"

namespace MBSim {

  /*! \brief Class for flexible connections
   *
   * */
  class FlexibleConnection : public Connection {

    protected:
      Vec WF[2], WM[2];

      RegularizedConnection *ffl, *fml;

    public:

      FlexibleConnection(const string &name);

      void setForceLaw(RegularizedConnection * rc) {ffl = rc;}
      void setMomentLaw(RegularizedConnection * rc) {fml = rc;}

      void updateKinetics(double t);
      void updateh(double t);

      double computePotentialEnergy();
  };

  typedef FlexibleConnection ConnectionFlexible;
}

#endif
