/* Copyright (C) 2004-2006  Robert Huber
 
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
#ifndef AMVISINTERFACE_UTILS_H_
#define  AMVISINTERFACE_UTILS_H_

#include "polygonpoint.h"
#include "contour.h"
#include "fmatvec.h"
#include <vector>

using namespace AMVis;

namespace MBSim {

  /*! \brief calculate <vector>-Container of PolygonPoints to define plane Contour for AMVisBody Extrusion vis Contour1sAnalytical
    @param cont1sanaly 	The UserFunction of this Contour1sAnalytical between getAlphaStart and getAlphaEnd defines contour 
    @param AUserFunc2xy 	Transformationmatrix to project the Userfunction Output into the xy Plane; 
    if AUserFunc2xy is omitted [0 1 0; 0 0 1] is used (up to now, the camprofile is defined in yzPlane!)
    @param nPoints 	Number of PolygonPoints
    @param rendersmooth  Edges are rendered smooth
   * */

  vector<PolygonPoint*>* makeAMVisPolygonPointVector(Contour1sAnalytical *cont1sanaly, Mat AUserFunc2xy, int nPoints, bool renderSmooth);
  vector<PolygonPoint*>* makeAMVisPolygonPointVector(Contour1sAnalytical *cont1sanaly, int nPoints, bool renderSmooth);

}

#endif

