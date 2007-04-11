/* Copyright (C) 2006  Roland Zander
 
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
 *   rzander@users.berlios.de
 *
 */

#ifndef _CONTOUR_PDATA_H_
#define _CONTOUR_PDATA_H_

#include "fmatvec.h"
#include <vector>

using namespace fmatvec;
using namespace std;

namespace MBSim {

  class Point;

  enum ContourParameterType {NODE, CONTINUUM, EXTINTERPOL};

  /*! \brief struct for data-management for single point on a Contour
  */
  struct ContourPointData {
    /* --------- used for all bodies, sufficient inforomation for rigid bodies ------------------ */

    /** kartesian coordinates of contact point in world system*/
    Vec WrOC;

    /* --------- used for elastic bodies ------------------ */

    /** Type of data representation: node, continuum, interpolation (extinterpol) */
    ContourParameterType type;
    /** ID of node or other discret interface within body -> FiniteElements */
    int ID;
    /** contour parameter(s) */
    Vec alpha;
    /** interpolation weights */
    Vec iWeights;
    /*! list of nodes used in interpolation 
      The (Body specific ID) can be accessed using ->iPoint[NNumber]->getID(); */
    vector<Point*> iPoints;

    Vec Wn;
    Mat Wt;
  };

}

#endif
