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
    plotColumns.emplace_back("longitudinal slip");
    plotColumns.emplace_back("slip angle");
    plotColumns.emplace_back("camber angle");
    plotColumns.emplace_back("deflection");
    plotColumns.emplace_back("effective rolling radius");
    plotColumns.emplace_back("scrub radius");
    plotColumns.emplace_back("rolling velocity");
    plotColumns.emplace_back("spin component of longitudinal velocity");
    plotColumns.emplace_back("cornering stiffness");
  }

  void LinearTyreModel::plot(vector<double> &plotVector) {
    static_cast<TyreContact*>(parent)->evalGeneralizedForce(); // Enforce variables to be up to date
    plotVector.push_back(ka);
    plotVector.push_back(al);
    plotVector.push_back(ga);
    plotVector.push_back(rhoz);
    plotVector.push_back(Re);
    plotVector.push_back(Rs);
    plotVector.push_back(vcx);
    plotVector.push_back(vsx-vcx);
    plotVector.push_back(cal);
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
    return static_cast<Tyre*>(getTyreContact()->getContour(1))->getContourParameters();
  }

  double LinearTyreModel::evalFreeRadius() {
    return static_cast<Tyre*>(getTyreContact()->getContour(1))->getRadius();
  }

  void LinearTyreModel::updateGeneralizedForces() {
    auto *contact = getTyreContact();
    auto *tyre = contact->getRigidContour(1);
    double Fx, Fy, Fz, Mz;
    if(contact->evalGeneralizedRelativePosition()(0)>0)
      Fz = 0;
    else
      Fz = -cz*contact->getGeneralizedRelativePosition()(0)-dz*contact->evalGeneralizedRelativeVelocity()(2);
    if(Fz>0) {
      vcx = max(1.,abs(contact->evalForwardVelocity()(0)));
      vcy = contact->getForwardVelocity()(1);
      vsx = contact->evalGeneralizedRelativeVelocity()(0);
      ka = -vsx/vcx;
      al = atan(vcy/vcx);
      ga = asin(tyre->getFrame()->getOrientation().col(1).T()*contact->getContourFrame(0)->getOrientation().col(2));

      Fx = cka*ka*sfFx;
      Fy = -(cal*al + cga*ga)*sfFy;
      Mz = -(cMzga*ga + t*Fy)*sfMz;

      rhoz = -contact->getGeneralizedRelativePosition(false)(0);
      Vec3 WrWC = contact->getContourFrame(1)->getPosition()-tyre->getFrame()->getPosition();
      Vec3 BrBC = contact->getContourFrame(1)->getOrientation(false).T()*WrWC;
      Re = fabs(-BrBC(1)*sin(ga) + BrBC(2)*cos(ga));
      Rs = fabs(BrBC(1)*cos(ga) + BrBC(2)*sin(ga));
    }
    else {
      Fz = 0;
      Fx = 0;
      Fy = 0;
      Mz = 0;
      ga = 0;
      ka = 0;
      al = 0;
      Re = 0;
      Rs = 0;
    }

    contact->getGeneralizedForce(false)(0) = Fx;
    contact->getGeneralizedForce(false)(1) = Fy;
    contact->getGeneralizedForce(false)(2) = Fz;
    contact->getGeneralizedForce(false)(3) = Mz;
  }

  VecV LinearTyreModel::getData() const {
    static_cast<TyreContact*>(parent)->evalGeneralizedForce(); // Enforce variables to be up to date
    VecV data(getDataSize(),NONINIT);
    data(0) = ka;
    data(1) = al;
    data(2) = ga;
    data(3) = rhoz;
    data(4) = Re;
    data(5) = Rs;
    data(6) = 0;
    data(7) = 0;
    data(8) = vcx;
    data(9) = vsx-vcx;
    data(10) = cal;
    data(11) = static_cast<TyreContact*>(parent)->getGeneralizedForce(false)(0);
    data(12) = static_cast<TyreContact*>(parent)->getGeneralizedForce(false)(1);
    data(13) = static_cast<TyreContact*>(parent)->getGeneralizedForce(false)(2);
    data(14) = 0;
    data(15) = 0;
    data(16) = static_cast<TyreContact*>(parent)->getGeneralizedForce(false)(3);
    return data;
  }

}
