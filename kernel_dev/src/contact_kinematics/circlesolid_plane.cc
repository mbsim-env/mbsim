/* Copyright (C) 2007  Martin Förg, Roland Zander
 
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
 *   rzander@users.berlios.de
 *
 */

#include <config.h> 
#include "circlesolid_plane.h"
#include "contour.h"

namespace MBSim {

  void ContactKinematicsCircleSolidPlane::assignContours(const vector<Contour*> &contour)
  {
    if(dynamic_cast<CircleSolid*>(contour[0])) {
      icircle = 0;
      iplane = 1;
      circlesolid = static_cast<CircleSolid*>(contour[0]);
      plane = static_cast<Plane*>(contour[1]);
    } 
    else {
      icircle = 1;
      iplane = 0;
      circlesolid = static_cast<CircleSolid*>(contour[1]);
      plane = static_cast<Plane*>(contour[0]);
    }
  }

  void ContactKinematicsCircleSolidPlane::updateg(Vec &g, ContourPointData *cpData) {
    cpData[iplane].cosy.setOrientation(plane->getFrame()->getOrientation());
    cpData[icircle].cosy.getOrientation().col(0) = -plane->getFrame()->getOrientation().col(0);
    cpData[icircle].cosy.getOrientation().col(1) = -plane->getFrame()->getOrientation().col(1);
    cpData[icircle].cosy.getOrientation().col(2) = plane->getFrame()->getOrientation().col(2);

    Vec Wd;
    Vec Wn = cpData[iplane].cosy.getOrientation().col(0);
    Vec Wb = circlesolid->getFrame()->getOrientation().col(2);
    double t_EC = trans(Wn)*Wb;
    if(t_EC>0) {
      Wb *= -1.;
      t_EC *= -1;	
    }
    //cout << Wn << endl;
    //cout << Wb << endl;
    Vec z_EC = Wn - t_EC*Wb;
    double z_EC_nrm2 = nrm2(z_EC);
    if(z_EC_nrm2 <= 1e-8) { // infinite possible contact points
      Wd = circlesolid->getFrame()->getPosition() - plane->getFrame()->getPosition();
    } 
    else { // exactly one possible contact point
      Wd =  (circlesolid->getFrame()->getPosition() - (circlesolid->getRadius()/z_EC_nrm2)*z_EC) - plane->getFrame()->getPosition();
    }
    //cout << z_EC<<endl;
    //cout << Wd << endl;
    //cout << Wn << endl;
    g(0) = trans(Wn)*Wd;
    //cout << g(0) << endl;
    cpData[icircle].cosy.setPosition(circlesolid->getFrame()->getPosition() - (circlesolid->getRadius()/z_EC_nrm2)*z_EC);
    cpData[iplane].cosy.setPosition(cpData[icircle].cosy.getPosition() - Wn*g(0));

  }

  void ContactKinematicsCircleSolidPlane::updategd(const Vec& g, Vec &gd, ContourPointData *cpData) {}


  //  void ContactKinematicsCircleSolidPlane::stage1(Vec &g, vector<ContourPointData> &cpData) 
  //  {
  //	Vec Wd;
  //	Vec Wbcircle = circlesolid->computeWb();
  //    cpData[iplane].Wn = plane->computeWn();
  //    cpData[icircle].Wn = -cpData[iplane].Wn;
  //   
  //	double t_EC = trans(cpData[iplane].Wn)*Wbcircle;
  //	if(t_EC>0) {
  //		Wbcircle *= -1.;
  //		t_EC *= -1;	
  //	}
  //	Vec z_EC = cpData[iplane].Wn - t_EC*Wbcircle;
  //	double z_EC_nrm2 = nrm2(z_EC);
  //	
  //    if(z_EC_nrm2 <= 1e-8) { // infinite possible contact points
  //      Wd = plane->getWrOP() - circlesolid->getWrOP();
  //      genBuf = Vec(3); // 0-Vector
  //    } 
  //    else { // exactly one possible contact point
  //      genBuf = (circlesolid->getRadius()/z_EC_nrm2)*z_EC;
  //      Wd = plane->getWrOP() - (circlesolid->getWrOP() + genBuf);
  //    }
  //    g(0) = trans(cpData[iplane].Wn)*Wd;
  //  }
  //
  //  void ContactKinematicsCircleSolidPlane::stage2(const Vec &g, Vec &gd, vector<ContourPointData> &cpData) 
  //  {
  //	Vec WrPC[2], WvC[2];
  //    WrPC[icircle] = genBuf;
  //    cpData[icircle].WrOC = circlesolid->getWrOP() + WrPC[icircle];
  //    cpData[iplane].WrOC  = cpData[icircle].WrOC + cpData[iplane].Wn*g(0);
  //    WrPC[iplane]  = cpData[iplane].WrOC - plane->getWrOP();
  //    WvC[icircle] = circlesolid->getWvP() + crossProduct(circlesolid->getWomegaC(),WrPC[icircle]);
  //    WvC[iplane] = plane->getWvP() + crossProduct(plane->getWomegaC(),WrPC[iplane]);
  //    Vec WvD = WvC[iplane] - WvC[icircle];
  //
  //    gd(0) = trans(cpData[iplane].Wn)*WvD;
  //
  //    if(cpData[iplane].Wt.cols()) {
  //      if(cpData[iplane].Wt.cols() == 1) {
  //    	  	cout << "ERROR: Two tangential contact directions necessary for spatial contact!" << endl;
  //			throw(1);
  //   	  }
  //      cpData[iplane].Wt.col(0) = computeTangential(cpData[iplane].Wn);            
  //      cpData[iplane].Wt.col(1) = crossProduct(cpData[iplane].Wn,cpData[iplane].Wt.col(0));
  //      cpData[icircle].Wt = -cpData[iplane].Wt;
  //      static Index iT(1,cpData[iplane].Wt.cols());
  //      gd(iT) = trans(cpData[iplane].Wt)*WvD;
  //    }
  //  }

}
