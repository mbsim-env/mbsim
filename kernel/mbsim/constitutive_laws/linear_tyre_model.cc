/* Copyright (C) 2004-2024 MBSim Development Team
 * 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 * 
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. 
 *
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "linear_tyre_model.h"
#include "mbsim/links/tyre_contact.h"
#include "mbsim/contours/tyre.h"
#include "mbsim/frames/contour_frame.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, LinearTyreModel)

  void LinearTyreModel::initPlot(vector<string> &plotColumns) {
    plotColumns.emplace_back("camber angle");
    plotColumns.emplace_back("rolling velocity");
    plotColumns.emplace_back("spin component of longitudinal velocity");
    plotColumns.emplace_back("longitudinal slip");
    plotColumns.emplace_back("slip angle");
  }

  void LinearTyreModel::plot(vector<double> &plotVector) {
    static_cast<TyreContact*>(parent)->evalGeneralizedForce(); // Enforce variables to be up to date
    plotVector.push_back(ga);
    plotVector.push_back(vcx);
    plotVector.push_back(vsx-vcx);
    plotVector.push_back(ka);
    plotVector.push_back(al);
  }

  void LinearTyreModel::initializeUsingXML(DOMElement *element) {
    TyreModel::initializeUsingXML(element);
    DOMElement* e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"cz");
    setcz(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"dz");
    setdz(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"cka");
    setcka(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"cal");
    setcal(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"cga");
    setcga(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"cMzga");
    setcMzga(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"t");
    sett(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"scaleFactorForLongitudinalForce");
    if(e) setScaleFactorForLongitudinalForce(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"scaleFactorForLateralForce");
    if(e) setScaleFactorForLateralForce(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"scaleFactorForAligningMoment");
    if(e) setScaleFactorForAligningMoment(E(e)->getText<double>());
  }

  VecV LinearTyreModel::getContourParameters() const {
    TyreContact *contact = static_cast<TyreContact*>(parent);
    Tyre *tyre = static_cast<Tyre*>(contact->getContour(1));
    return tyre->getContourParameters();
  }

  double LinearTyreModel::evalFreeRadius() {
    TyreContact *contact = static_cast<TyreContact*>(parent);
    Tyre *tyre = static_cast<Tyre*>(contact->getContour(1));
    return tyre->getRadius();
  }

  void LinearTyreModel::updateGeneralizedForces() {
    TyreContact *contact = static_cast<TyreContact*>(parent);
    Tyre *tyre = static_cast<Tyre*>(contact->getContour(1));
    double Fx, Fy, Fz, Mz;
    if(contact->evalGeneralizedRelativePosition()(0)>0)
      Fz = 0;
    else
      Fz = -cz*contact->getGeneralizedRelativePosition()(0)-dz*contact->evalGeneralizedRelativeVelocity()(2);
    if(Fz>0) {
      vcx = max(1.,abs(contact->evalForwardVelocity()(0)));
      vcy = contact->getForwardVelocity()(1);
      vsx = contact->getGeneralizedRelativeVelocity()(0);
      ka = -vsx/vcx;
      al = atan(vcy/vcx);
      ga = asin(tyre->getFrame()->getOrientation().col(1).T()*contact->getContourFrame(0)->getOrientation().col(2));

      Fx = cka*ka*sfFx;
      Fy = -(cal*al + cga*ga)*sfFy;
      Mz = -(cMzga*ga + t*Fy)*sfMz;
    }
    else {
      Fz = 0;
      Fx = 0;
      Fy = 0;
      Mz = 0;
      ga = 0;
      ka = 0;
      al = 0;
    }

    contact->getGeneralizedForce(false)(0) = Fx;
    contact->getGeneralizedForce(false)(1) = Fy;
    contact->getGeneralizedForce(false)(2) = Fz;
    contact->getGeneralizedForce(false)(3) = Mz;
  }

  VecV LinearTyreModel::getData() const {
    static_cast<TyreContact*>(parent)->evalGeneralizedForce(); // Enforce variables to be up to date
    VecV data(8,NONINIT);
    data(0) = ga;
    data(1) = vcx;
    data(2) = vsx-vcx;
    data(3) = ka;
    data(4) = al;
    return data;
  }

}
