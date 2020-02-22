/* Copyright (C) 2004-2019 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "mbsim/contours/bevel_gear.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/utils/nonlinear_algebra.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, BevelGear)

  double BevelGear::Residuum::operator()(const double &eta) {
    double phi = -r0/r1*eta;
    double xi = (s*cos(phi-be)+r1*sin(phi)*pow(sin(al),2)*sin(be))/(-sin(phi-be)*pow(sin(al),2)*sin(be)+cos(phi-be)*cos(be));
    double l = (sin(phi)/cos(phi-be)*r1+xi*tan(phi-be))*sin(al);
    double x = -l*sin(al)*cos(phi-be)+xi*sin(phi-be)+r1*sin(phi);
    double b = signi*l*cos(al);
    double c = l*sin(al)*sin(phi-be)+xi*cos(phi-be)+r1*cos(phi);
    double y = b*cos(ga)-c*sin(ga);
    double z = b*sin(ga)+c*cos(ga);
    double hm = z*tan(ga)+h/cos(ga);
    return x*x+y*y-hm*hm;
  }

  Vec3 BevelGear::evalKrPS(const Vec2 &zeta) {
    static Vec3 KrPS(NONINIT);
    double eta = zeta(0);
    double xi = zeta(1);
    double phi = -r0/r1*eta;
    double l = (sin(phi)/cos(phi-be)*r1+xi*tan(phi-be))*sin(al);
    double a = -l*sin(al)*cos(phi-be)+xi*sin(phi-be)+r1*sin(phi);
    double b = signi*l*cos(al);
    double c = l*sin(al)*sin(phi-be)+xi*cos(phi-be)+r1*cos(phi);
    KrPS(0) = a*cos(eta)-b*sin(eta)*cos(ga)+c*sin(eta)*sin(ga);
    KrPS(1) = a*sin(eta)+b*cos(eta)*cos(ga)-c*cos(eta)*sin(ga);
    KrPS(2) = b*sin(ga)+c*cos(ga)-d;
    return BasicRotAIKz(k*2*M_PI/N-signi*delh)*KrPS;
  }

  Vec3 BevelGear::evalKs(const Vec2 &zeta) {
    static Vec3 Ks(NONINIT);
    double eta = zeta(0);
    double xi = zeta(1);
    double phi = -r0/r1*eta;
    double l = (sin(phi)/cos(phi-be)*r1+xi*tan(phi-be))*sin(al);
    double a = -l*sin(al)*cos(phi-be)+xi*sin(phi-be)+r1*sin(phi);
    double b = signi*l*cos(al);
    double c = l*sin(al)*sin(phi-be)+xi*cos(phi-be)+r1*cos(phi);
    double phis = -r0/r1;
    double ls = ((cos(phi)/cos(phi-be)+sin(phi)/pow(cos(phi-be),2)*sin(phi-be))*r1+xi/pow(cos(phi-be),2))*phis*sin(al);
    double as = -ls*sin(al)*cos(phi-be)+(l*sin(al)*sin(phi-be)+xi*cos(phi-be)+r1*cos(phi))*phis;
    double bs = signi*ls*cos(al);
    double cs = ls*sin(al)*sin(phi-be)+(l*sin(al)*cos(phi-be)-xi*sin(phi-be)-r1*sin(phi))*phis;
    Ks(0) = as*cos(eta)-a*sin(eta)-(bs*sin(eta)+b*cos(eta))*cos(ga)+(cs*sin(eta)+c*cos(eta))*sin(ga);
    Ks(1) = as*sin(eta)+a*cos(eta)+(bs*cos(eta)-b*sin(eta))*cos(ga)-(cs*cos(eta)-c*sin(eta))*sin(ga);
    Ks(2) = bs*sin(ga)+cs*cos(ga);
    return BasicRotAIKz(k*2*M_PI/N-signi*delh)*Ks;
  }

  Vec3 BevelGear::evalKt(const Vec2 &zeta) {
    static Vec3 Kt(NONINIT);
    double eta = zeta(0);
    double phi = -r0/r1*eta;
    double lz = tan(phi-be)*sin(al);
    double az = -lz*sin(al)*cos(phi-be)+sin(phi-be);
    double bz = signi*lz*cos(al);
    double cz = lz*sin(al)*sin(phi-be)+cos(phi-be);
    Kt(0) = az*cos(eta)-bz*sin(eta)*cos(ga)+cz*sin(eta)*sin(ga);
    Kt(1) = az*sin(eta)+bz*cos(eta)*cos(ga)-cz*cos(eta)*sin(ga);
    Kt(2) = bz*sin(ga)+cz*cos(ga);
    return BasicRotAIKz(k*2*M_PI/N-signi*delh)*Kt;
  }

  Vec3 BevelGear::evalParDer1Ks(const Vec2 &zeta) {
    static Vec3 parDer1Ks(NONINIT);
    double eta = zeta(0);
    double xi = zeta(1);
    double phi = -r0/r1*eta;
    double l = (sin(phi)/cos(phi-be)*r1+xi*tan(phi-be))*sin(al);
    double a = -l*sin(al)*cos(phi-be)+xi*sin(phi-be)+r1*sin(phi);
    double b = signi*l*cos(al);
    double c = l*sin(al)*sin(phi-be)+xi*cos(phi-be)+r1*cos(phi);
    double phis = -r0/r1;
    double ls = ((cos(phi)/cos(phi-be)+sin(phi)/pow(cos(phi-be),2)*sin(phi-be))*r1+xi/pow(cos(phi-be),2))*phis*sin(al);
    double as = -ls*sin(al)*cos(phi-be)+(l*sin(al)*sin(phi-be)+xi*cos(phi-be)+r1*cos(phi))*phis;
    double bs = signi*ls*cos(al);
    double cs = ls*sin(al)*sin(phi-be)+(l*sin(al)*cos(phi-be)-xi*sin(phi-be)-r1*sin(phi))*phis;
    double lss = ((-sin(phi)/cos(phi-be)+cos(phi)/pow(cos(phi-be),2)*sin(phi-be)+cos(phi)/pow(cos(phi-be),2)*sin(phi-be)+2*sin(phi)/pow(cos(phi-be),3)*pow(sin(phi-be),2)+sin(phi)/pow(cos(phi-be),2)*cos(phi-be))*r1+2*xi/pow(cos(phi-be),3)*sin(phi-be))*phis*phis*sin(al);
    double ass = -lss*sin(al)*cos(phi-be)+(2*ls*sin(al)*sin(phi-be)+(l*sin(al)*cos(phi-be)-xi*sin(phi-be)-r1*sin(phi))*phis)*phis;
    double bss = signi*lss*cos(al);
    double css = lss*sin(al)*sin(phi-be)+(2*ls*sin(al)*cos(phi-be)-(l*sin(al)*sin(phi-be)+xi*cos(phi-be)+r1*cos(phi))*phis)*phis;
    parDer1Ks(0) = ass*cos(eta)-2*as*sin(eta)-a*cos(eta)-(bss*sin(eta)+2*bs*cos(eta)-b*sin(eta))*cos(ga)+(css*sin(eta)+2*cs*cos(eta)-c*sin(eta))*sin(ga);
    parDer1Ks(1) = ass*sin(eta)+2*as*cos(eta)-a*sin(eta)+(bss*cos(eta)-2*bs*sin(eta)-b*cos(eta))*cos(ga)-(css*cos(eta)-2*cs*sin(eta)-c*cos(eta))*sin(ga);
    parDer1Ks(2) = bss*sin(ga)+css*cos(ga);
    return BasicRotAIKz(k*2*M_PI/N-signi*delh)*parDer1Ks;
  }

  Vec3 BevelGear::evalParDer2Ks(const Vec2 &zeta) {
    static Vec3 parDer2Ks;
    return parDer2Ks;
  }

  Vec3 BevelGear::evalParDer1Kt(const Vec2 &zeta) {
    static Vec3 parDer1Kt;
    return parDer1Kt;
  }

  Vec3 BevelGear::evalParDer2Kt(const Vec2 &zeta) {
    static Vec3 parDer2Kt;
    return parDer2Kt;
  }

  double BevelGear::getPhiMax(double h, double s, int signi) {
    Residuum f(h,s,r0,r1,al,be,ga,signi);
    NewtonMethod newton(&f);
    double phi = newton.solve(0);
    if(newton.getInfo()!=0) throw 1;
    return phi;
  }

  void BevelGear::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      delh = (M_PI/2-b/m*cos(be))/N;
      r0 = m*N/cos(be)/2;
      r1 = r0/sin(ga);
      d = sqrt(pow(r1,2)-pow(r0,2));
      double h[2];
      h[0] = m;
      h[1] = -m/2;
      for(int i=0; i<2; i++) {
        int signi=i?-1:1;
      double mpsp = getPhiMax(h[i],w/2,signi);
      double mpsm = getPhiMax(h[i],-w/2,signi);
      double mmsp = getPhiMax(h[not i],w/2,signi);
      double mmsm = getPhiMax(h[not i],-w/2,signi);
      if(mpsp>=mpsm) {
        phiMaxHigh[i] = mpsp;
        phiMaxLow[i] = mpsm;
        sPhiMaxHigh[i] = w/2;
      }
      else {
        phiMaxHigh[i] = mpsm;
        phiMaxLow[i] = mpsp;
        sPhiMaxHigh[i] = -w/2;
      }
      if(mmsp<=mmsm) {
        phiMinHigh[i] = mmsp;
        phiMinLow[i] = mmsm;
        sPhiMinHigh[i] = w/2;
      }
      else {
        phiMinHigh[i] = mmsm;
        phiMinLow[i] = mmsp;
        sPhiMinHigh[i] = -w/2;
      }
      }
    }
    else if(stage==plotting) {
      if(plotFeature[openMBV] && openMBVRigidBody) {
        static_pointer_cast<OpenMBV::BevelGear>(openMBVRigidBody)->setNumberOfTeeth(N);
        static_pointer_cast<OpenMBV::BevelGear>(openMBVRigidBody)->setWidth(w);
        static_pointer_cast<OpenMBV::BevelGear>(openMBVRigidBody)->setHelixAngle(be);
        static_pointer_cast<OpenMBV::BevelGear>(openMBVRigidBody)->setPitchAngle(ga);
        static_pointer_cast<OpenMBV::BevelGear>(openMBVRigidBody)->setModule(m);
        static_pointer_cast<OpenMBV::BevelGear>(openMBVRigidBody)->setPressureAngle(al);
        static_pointer_cast<OpenMBV::BevelGear>(openMBVRigidBody)->setBacklash(b);
      }
    }
    RigidContour::init(stage, config);
  }

  void BevelGear::initializeUsingXML(DOMElement *element) {
    RigidContour::initializeUsingXML(element);
    DOMElement* e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"numberOfTeeth");
    setNumberOfTeeth(E(e)->getText<int>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"width");
    setWidth(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"helixAngle");
    if(e) setHelixAngle(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"pitchAngle");
    if(e) setPitchAngle(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"module");
    if(e) setModule(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"pressureAngle");
    if(e) setPressureAngle(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"backlash");
    if(e) setBacklash(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      OpenMBVColoredBody ombv;
      ombv.initializeUsingXML(e);
      openMBVRigidBody=ombv.createOpenMBV<OpenMBV::BevelGear>(); 
    }
  }

}
