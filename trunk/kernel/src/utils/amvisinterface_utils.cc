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
#include <config.h>
#include "amvisinterface_utils.h"

namespace MBSim {

  //----------------------------------------------------------------------

  vector<PolygonPoint*>* makeAMVisPolygonPointVector(Contour1sAnalytical *cont1sanaly, int nPoints, bool renderSmooth) {

    // Standard fuer jetzige Contour1sanalytical (Nockenprofil; Ventiltrieb), da NockenProfil z.Zeit in y-z-Ebene definiert 
    Mat AUserFunc2xy = Mat("[0, 1, 0; 0, 0, 1]");
    return makeAMVisPolygonPointVector(cont1sanaly, AUserFunc2xy, nPoints, renderSmooth) ;
  }
  //----------------------------------------------------------------------


  vector<PolygonPoint*>* makeAMVisPolygonPointVector(Contour1sAnalytical *cont1sanaly, Mat AUserFunc2xy, int nPoints, bool renderSmooth) {

    // Zur Erzeugung des C-Containers vector<PolygonPoint*> zur Konturdefinition des AMVis Koerpers Extrusion
    // (Structur PolyhonPoint besteht aus x,y (Koordinaten) und b (Rendering-Modus der Ecken); definiert in AMVisC++Interface)
    // Berechnet wird die Userfunction funcCrPC (aus contour1sanalytical) fuer den Parameter s;
    // Dieser Vektor wird durch die Matrix AUserFunc2xy in die x-y Ebene projeziert.
    // Der Konturparameter s wird dabei aequidistant zwischen getAlphaStart und getAlphaEnd variert.
    // Insgesamt werden nPoints PolygonPoints im Container zurueckgegeben


    vector<PolygonPoint*>  *VektorPolygon = new vector<PolygonPoint*>();

    double s, ds;    							// Parameter s und Schrittweite ds
    Vec tmpVec;
    UserFunction *f = cont1sanaly->getUserFunction(); 
    PolygonPoint *tmpPPoint  = new PolygonPoint();

    s  = cont1sanaly->getAlphaStart();

    assert(AUserFunc2xy.rows()==2);
    //  cout << "Error in makeAMVisPolygonPointVector: Matrix AUserFunc2xy must have two rows!" <<endl; 
    assert(AUserFunc2xy.cols()==((*f)(s)).size());
    //  cout<< "Error in makeAMVisPolygonPointVector: Matrix AUserFunc2xy wrong number of columns!" <<endl;


    int b =1;
    if (renderSmooth)
      b =0;

    tmpVec =  AUserFunc2xy * (*f)(s);
    tmpPPoint->x = tmpVec(0);
    tmpPPoint->y = tmpVec(1);
    tmpPPoint->b = b;

    VektorPolygon->push_back(tmpPPoint);

    if (nPoints > 1) {
      ds = (cont1sanaly->getAlphaEnd() - s)/(nPoints-1.0); 
      for (int i=1; i<(nPoints-1); i++) {
	tmpVec = AUserFunc2xy * (*f)(s+i*ds);
	tmpPPoint  = new PolygonPoint(); 
	tmpPPoint->x = tmpVec(0);
	tmpPPoint->y = tmpVec(1);
	tmpPPoint->b = b;
	VektorPolygon->push_back(tmpPPoint);
      }
      s  = cont1sanaly->getAlphaEnd();
      tmpVec =  AUserFunc2xy * (*f)(s);
      tmpPPoint  = new PolygonPoint(); 
      tmpPPoint->x = tmpVec(0);
      tmpPPoint->y = tmpVec(1);
      tmpPPoint->b = b;
      VektorPolygon->push_back(tmpPPoint);
    }


    return VektorPolygon;
  };

}

