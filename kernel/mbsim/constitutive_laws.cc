/* Copyright (C) 2004-2009 MBSim Development Team
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
#include "mbsim/constitutive_laws.h"
#include "mbsim/contact.h"
#include "mbsim/element.h"
#include "mbsim/objectfactory.h"
#include "mbsim/utils/nonsmooth_algebra.h"
#include "mbsim/utils/utils.h"

#include <map>

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;

namespace MBSim {

  //Vec UnilateralContact::project(const Vec& la, const Vec& gdn, const Vec& gda, const Vec& r) {
  //  if(fabs(gda(0)) > gd_limit)
  //    return Vec(1,INIT,proxCN(la(0)-r(0)*(gdn(0)+epsilon*gda(0))));
  //  else
  //    return Vec(1,INIT,proxCN(la(0)-r(0)*gdn(0)));
  //}
  //Vec UnilateralContact::diff(const Vec& la, const Vec& gdn, const Vec& gda, const Vec& r) {
  //  Vec d(2,NONINIT);
  //  if(la(0)-r(0)*gdn(0) < 0) {
  //    d.init(0);
  //  } else {
  //    d(0) = 1;
  //    d(1) = -r(0);
  //  }
  //  return d;
  //}
  //Vec UnilateralContact::solve(const SqrMat& G, const Vec& gdn, const Vec& gda) {
  //  //if(fabs(gda(0)) > gd_limit)
  //    //gdn(0) += epsilon*gda(0);
  //  if(gdn(0) >= 0)
  //    return Vec(1,INIT,0);
  //  else {
  //  if(fabs(gda(0)) > gd_limit)
  //    return Vec(1,INIT,-(gdn(0)+epsilon*gda(0))/G(0,0));
  //  else
  //    return Vec(1,INIT,-(gdn(0))/G(0,0));
  //  }
  //}
  //    bool UnilateralContact::isFulfilled(const Vec& la, const Vec& gdn, const Vec& gda, double laTol, double gdTol) {
  //    double gdn_ = gdn(0);
  //    if(fabs(gda(0)) > gd_limit)
  //      gdn_ += epsilon*gda(0);
  //    if(gdn_ >= -gdTol && fabs(la(0)) <= laTol)
  //      return true;
  //    else if(la(0) >= -laTol && fabs(gdn_) <= gdTol)
  //      return true;
  //    else 
  //      return false;
  //    
  //  }

  TiXmlElement* GeneralizedForceLaw::writeXMLFile(TiXmlNode *parent) { 
    TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType());
    if(forceFunc) {
      TiXmlElement *ele1 = new TiXmlElement( MBSIMNS"forceFunction" );
      forceFunc->writeXMLFile(ele1);
      ele0->LinkEndChild(ele1);
    }
    parent->LinkEndChild(ele0);
    return ele0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(GeneralizedForceLaw, UnilateralConstraint, MBSIMNS"UnilateralConstraint")

  double UnilateralConstraint::project(double la, double gdn, double r, double laMin) {
    return proxCN(la - r * gdn, laMin);
  }

  Vec UnilateralConstraint::diff(double la, double gdn, double r, double laMin) {
    Vec d(2, NONINIT);
    if (la - r * gdn < laMin)
      d.init(0);
    else {
      d(0) = 1;
      d(1) = -r;
    }
    return d;
  }

  double UnilateralConstraint::solve(double G, double gdn) {
    if (gdn >= 0)
      return 0;
    else
      return -gdn / G;
  }

  bool UnilateralConstraint::isFulfilled(double la, double gdn, double laTol, double gdTol, double laMin) {
    if (gdn >= -gdTol && fabs(la - laMin) <= laTol)
      return true;
    else if (la - laMin >= -laTol && fabs(gdn) <= gdTol)
      return true;
    else
      return false;
  }

  //Vec BilateralContact::project(const Vec& la, const Vec& gdn, const Vec& gda, const Vec& r) {
  //  return Vec(1,INIT,la(0)-r(0)*gdn(0));
  //}
  //Vec BilateralContact::diff(const Vec& la,  const Vec& gdn, const Vec& gda, const Vec& r) {
  //  Vec d(2,NONINIT);
  //  d(0) = 1;
  //  d(1) = -r(0);
  //  return d;
  //}
  //Vec BilateralContact::solve(const SqrMat& G, const Vec& gdn, const Vec& gda) {
  //  return Vec(1,INIT,-gdn(0)/G(0,0));
  //}
  //bool BilateralContact::isFulfilled(const Vec& la, const Vec& gdn, const Vec& gda, double laTol, double gdTol) {
  //  return (fabs(gdn(0)) <= gdTol);
  //}

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(GeneralizedForceLaw, BilateralConstraint, MBSIMNS"BilateralConstraint")

  double BilateralConstraint::project(double la, double gdn, double r, double laMin) {
    return la - r * gdn;
  }

  Vec BilateralConstraint::diff(double la, double gdn, double r, double laMin) {
    Vec d(2, NONINIT);
    d(0) = 1;
    d(1) = -r;
    return d;
  }

  double BilateralConstraint::solve(double G, double gdn) {
    return -gdn / G;
  }

  bool BilateralConstraint::isFulfilled(double la, double gdn, double laTol, double gdTol, double laMin) {
    return fabs(gdn) <= gdTol;
  }

  TiXmlElement* GeneralizedImpactLaw::writeXMLFile(TiXmlNode *parent) { 
    TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType());
    parent->LinkEndChild(ele0);
    return ele0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(GeneralizedImpactLaw, UnilateralNewtonImpact, MBSIMNS"UnilateralNewtonImpact")

  double UnilateralNewtonImpact::project(double la, double gdn, double gda, double r, double laMin) {
    if (gda <= -gd_limit) {       // 2 Aenderungen :
      gdn += epsilon * gda;       // elastischer Anteil nur bei negativer Annäherungsgeschw. ueber gd_limit
    }                           // zwischen gd_limit und gd_limit/10 wird eps stetig auf 0 zurueckgefuehrt
    else {
      if (gda < -0.1 * gd_limit) {
        double epsi = epsilon * 0.5 * (cos(M_PI / (0.9 * gd_limit) * (fabs(gda) - gd_limit)) + 1);
        gdn += epsi * gda;
      }
    }
    return proxCN(la - r * gdn, laMin);
  }

  Vec UnilateralNewtonImpact::diff(double la, double gdn, double gda, double r, double laMin) {
    Vec d(2, NONINIT);
    if (la - laMin - r * gdn < 0)
      d.init(0);
    else {
      d(0) = 1;
      d(1) = -r;
    }
    return d;
  }

  double UnilateralNewtonImpact::solve(double G, double gdn, double gda) {
    if (gda <= -gd_limit) {
      gdn += epsilon * gda;
    }
    else {
      if (gda < -0.1 * gd_limit) {
        double epsi = epsilon * 0.5 * (cos(M_PI / (0.9 * gd_limit) * (fabs(gda) - gd_limit)) + 1);
        gdn += epsi * gda;
      }
    }

    if (gdn >= 0)
      return 0;
    else
      return -gdn / G;
  }

  bool UnilateralNewtonImpact::isFulfilled(double la, double gdn, double gda, double laTol, double gdTol, double laMin) {
    if (gda <= -gd_limit) {
      gdn += epsilon * gda;
    }
    else {
      if (gda < -0.1 * gd_limit) {
        double epsi = epsilon * 0.5 * (cos(M_PI / (0.9 * gd_limit) * (fabs(gda) - gd_limit)) + 1);
        gdn += epsi * gda;
      }
    }
    if (gdn >= -gdTol && fabs(la - laMin) <= laTol)
      return true;
    else if (la - laMin >= -laTol && fabs(gdn) <= gdTol)
      return true;
    else
      return false;
  }

  void UnilateralNewtonImpact::initializeUsingXML(TiXmlElement *element) {
    GeneralizedImpactLaw::initializeUsingXML(element);
    TiXmlElement *e;
    e = element->FirstChildElement(MBSIMNS"restitutionCoefficient");
    epsilon = Element::getDouble(e);
  }

  TiXmlElement* UnilateralNewtonImpact::writeXMLFile(TiXmlNode *parent) { 
    TiXmlElement *ele0 = GeneralizedImpactLaw::writeXMLFile(parent);
    addElementText(ele0,MBSIMNS"restitutionCoefficient",epsilon);
    return ele0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(GeneralizedImpactLaw, BilateralImpact, MBSIMNS"BilateralImpact")

  double BilateralImpact::project(double la, double gdn, double gda, double r, double laMin) {
    return la - r * gdn;
  }

  Vec BilateralImpact::diff(double la, double gdn, double gda, double r, double laMin) {
    Vec d(2, NONINIT);
    d(0) = 1;
    d(1) = -r;
    return d;
  }

  double BilateralImpact::solve(double G, double gdn, double gda) {
    return -gdn / G;
  }

  bool BilateralImpact::isFulfilled(double la, double gdn, double gda, double laTol, double gdTol, double laMin) {
    return fabs(gdn) <= gdTol;
  }

  TiXmlElement* FrictionForceLaw::writeXMLFile(TiXmlNode *parent) { 
    TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType());
    if(frictionForceFunc) {
      TiXmlElement *ele1 = new TiXmlElement( MBSIMNS"frictionForceFunction" );
      frictionForceFunc->writeXMLFile(ele1);
      ele0->LinkEndChild(ele1);
    }
    parent->LinkEndChild(ele0);
    return ele0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FrictionForceLaw, PlanarCoulombFriction, MBSIMNS"PlanarCoulombFriction")

  Vec PlanarCoulombFriction::project(const Vec& la, const Vec& gdn, double laN, double r) {
    return Vec(1, INIT, proxCT2D(la(0) - r * gdn(0), mu * fabs(laN)));
  }

  Mat PlanarCoulombFriction::diff(const Vec& la, const Vec& gdn, double laN, double r) {
    double argT = la(0) - r * gdn(0);
    Mat d(1, 3, NONINIT);
    if (abs(argT) < mu * fabs(laN)) {
      //d_dargT = Mat(2,2,EYE);
      d(0, 0) = 1;
      d(0, 1) = -r;
      d(0, 2) = 0;
    }
    else {
      d(0, 0) = 0;
      d(0, 1) = 0;
      d(0, 2) = sign(argT) * sign(laN) * mu;
    }
    return d;
  }

  Vec PlanarCoulombFriction::solve(const SqrMat& G, const Vec& gdn, double laN) {
    double laNmu = fabs(laN) * mu;
    double sdG = -gdn(0) / G(0, 0);
    if (fabs(sdG) <= laNmu)
      return Vec(1, INIT, sdG);
    else
      return Vec(1, INIT, (laNmu <= sdG) ? laNmu : -laNmu);
  }

  bool PlanarCoulombFriction::isFulfilled(const Vec& la, const Vec& gdn, double laN, double laTol, double gdTol) {
    if (fabs(la(0) + gdn(0) / fabs(gdn(0)) * mu * fabs(laN)) <= laTol)
      return true;
    else if (fabs(la(0)) <= mu * fabs(laN) + laTol && fabs(gdn(0)) <= gdTol)
      return true;
    else
      return false;
  }

  Vec PlanarCoulombFriction::dlaTdlaN(const Vec& gd, double laN) {
    return Vec(1, INIT, -mu * sign(gd(0)));
  }

  void PlanarCoulombFriction::initializeUsingXML(TiXmlElement *element) {
    FrictionForceLaw::initializeUsingXML(element);
    TiXmlElement *e;
    e = element->FirstChildElement(MBSIMNS"frictionCoefficient");
    setFrictionCoefficient(Element::getDouble(e));
  }

  TiXmlElement* PlanarCoulombFriction::writeXMLFile(TiXmlNode *parent) { 
    TiXmlElement *ele0 = FrictionForceLaw::writeXMLFile(parent);
    addElementText(ele0,MBSIMNS"frictionCoefficient",mu);
    return ele0;
  }
  
  //Vec CoulombFriction::project(const Vec& la, const Vec& gdn, const Vec& r) {
  //  int nFric = gdn.size()-1;
  ////Vec lan(la.size()-1);
  //  if(nFric==1) 
  //    return Vec(1,INIT,proxCT2D(la(1)-r(1)*gdn(1),mu*fabs(la(0))));
  //  else //if(nFric == 2) 
  //    return proxCT3D(la(1,nFric)-r(1)*gdn(1,nFric),mu*fabs(la(0)));
  //}
  //Vec CoulombFriction::solve(const SqrMat& G, const Vec& laN, const Vec& gdn) {
  //  double laNmu = fabs(laN(0))*mu;
  //  double sdG = -gdn(1)/G(1,1);
  //  if(fabs(sdG)<=laNmu) 
  //    return Vec(1,INIT,sdG);
  //  else 
  //    return Vec(1,INIT, (laNmu<=sdG) ? laNmu : -laNmu);
  //}
  //Vec CoulombFriction::solve(const SqrMat& G, const Vec& laN, const Vec& gdn, double laN) {
  //  double laNmu = fabs(laN)*mu;
  //  double sdG = -gdn(0)/G(0,0);
  //  if(fabs(sdG)<=laNmu) 
  //    return sdG;
  //  else 
  //    return (laNmu<=sdG) ? laNmu : -laNmu;
  //}
  //    Mat CoulombFriction::diff(const Vec& la, const Vec& gdn, const Vec& r) {
  //    int nFric = gdn.size()-1;
  //  //Vec lan(la.size()-1);
  //    if(nFric==1) {
  //      double argT = la(1)-r(1)*gdn(1);
  //      Mat d(1,4,NONINIT);
  //      if(abs(argT) < mu*fabs(la(0))) {
  //	//d_dargT = Mat(2,2,EYE);
  //	d(0,0) = 0;
  //	d(0,1) = 1;
  //	d(0,2) = 0;
  //	d(0,3) = -r(1);
  //      } else {
  //	d(0,0) = sign(argT)*sign(la(0))*mu;
  //	d(0,1) = 0;
  //	d(0,2) = 0;
  //	d(0,3) = 0;
  //      }
  //      return d;
  //    } else { //if(nFric == 2) 
  //      Vec argT = la(1,nFric)-r(1)*gdn(1,nFric);
  //      Mat E(2,2,EYE);
  //      Mat d(2,6,NONINIT);
  //      if(nrm2(argT) < mu*fabs(la(0))) {
  //	//d_dargT = Mat(2,2,EYE);
  //	d(Index(0,1),Index(0,0)).init(0);
  //	d(Index(0,1),Index(1,2)) = E;
  //	d(Index(0,1),Index(3,3)).init(0);
  //	d(Index(0,1),Index(4,5)) = -r*E;
  //      } else {
  //	Mat d_dargT = (E - (argT*trans(argT))/(trans(argT)*argT))*mu*la(0)/nrm2(argT);
  //	d(Index(0,1),Index(0,0)) = argT/nrm2(argT)*mu;
  //	d(Index(0,1),Index(1,2)) = d_dargT;
  //	d(Index(0,1),Index(3,3)).init(0);
  //	d(Index(0,1),Index(4,5)) = -r(1)*d_dargT;
  //      }
  //      return d;
  //    }
  //  }
  // bool CoulombFriction::isFulfilled(const Vec& la, const Vec& gdn, double laTol, double gdTol) {
  //    int nFric = gdn.size()-1;
  //    if(nFric == 1) {
  //      if(fabs(la(1) + gdn(1)/fabs(gdn(1))*mu*fabs(la(0))) <= laTol)
  //	return true;
  //      else if(fabs(la(1)) <= mu*fabs(la(0))+laTol && fabs(gdn(1)) <= gdTol)
  //	return true;
  //      else 
  //	return false;
  //    } else if(nFric==2) {
  //      if(nrm2(la(1,2) + gdn(1,2)/nrm2(gdn(1,2))*mu*fabs(la(0))) <= laTol)
  //	return true;
  //      else if(nrm2(la(1,2)) <= mu*fabs(la(0))+laTol && nrm2(gdn(1,2)) <= gdTol)
  //	return true;
  //      else 
  //	return false;
  //    }
  //    return false;
  //  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FrictionForceLaw, SpatialCoulombFriction, MBSIMNS"SpatialCoulombFriction")

  Vec SpatialCoulombFriction::project(const Vec& la, const Vec& gdn, double laN, double r) {
    return proxCT3D(la - r * gdn, mu * fabs(laN));
  }

  Mat SpatialCoulombFriction::diff(const Vec& la, const Vec& gdn, double laN, double r) {
    Vec argT = la - r * gdn;
    Mat E(2, 2, EYE);
    Mat d(2, 5, NONINIT);
    if (nrm2(argT) < mu * fabs(laN)) {
      //d_dargT = Mat(2,2,EYE);
      d(Index(0, 1), Index(0, 1)) = E;
      d(Index(0, 1), Index(2, 3)) = -r * E;
      d(Index(0, 1), Index(4, 4)).init(0);
    }
    else {
      Mat d_dargT = (E - (argT * argT.T()) / (argT.T() * argT)) * mu * la(0) / nrm2(argT);
      d(Index(0, 1), Index(0, 1)) = d_dargT;
      d(Index(0, 1), Index(2, 3)) = -r * d_dargT;
      d(Index(0, 1), Index(4, 4)) = argT / nrm2(argT) * mu;
    }
    return d;
  }

  Vec SpatialCoulombFriction::solve(const SqrMat& G, const Vec& gdn, double laN) {
    throw MBSimError("ERROR (SpatialCoulombFriction::solve): Not implemented!");
  }

  bool SpatialCoulombFriction::isFulfilled(const Vec& la, const Vec& gdn, double laN, double laTol, double gdTol) {
    if (nrm2(la + gdn / nrm2(gdn) * mu * fabs(laN)) <= laTol)
      return true;
    else if (nrm2(la) <= mu * fabs(laN) + laTol && nrm2(gdn) <= gdTol)
      return true;
    else
      return false;
  }

  Vec SpatialCoulombFriction::dlaTdlaN(const Vec& gd, double laN) {
    return -mu * gd / nrm2(gd);
  }

  void SpatialCoulombFriction::initializeUsingXML(TiXmlElement *element) {
    FrictionForceLaw::initializeUsingXML(element);
    TiXmlElement *e;
    e = element->FirstChildElement(MBSIMNS"frictionCoefficient");
    setFrictionCoefficient(Element::getDouble(e));
  }

  TiXmlElement* SpatialCoulombFriction::writeXMLFile(TiXmlNode *parent) { 
    TiXmlElement *ele0 = FrictionForceLaw::writeXMLFile(parent);
    addElementText(ele0,MBSIMNS"frictionCoefficient",mu);
    return ele0;
  }

  Vec PlanarStribeckFriction::project(const Vec& la, const Vec& gdn, double laN, double r) {
    return Vec(1, INIT, proxCT2D(la(0) - r * gdn(0), (*fmu)(0) * fabs(laN)));
  }

  Mat PlanarStribeckFriction::diff(const Vec& la, const Vec& gdn, double laN, double r) {
    double argT = la(0) - r * gdn(0);
    Mat d(1, 3, NONINIT);
    if (abs(argT) < (*fmu)(0) * fabs(laN)) {
      d(0, 0) = 1;
      d(0, 1) = -r;
      d(0, 2) = 0;
    }
    else {
      d(0, 0) = 0;
      d(0, 1) = 0;
      d(0, 2) = sign(argT) * sign(laN) * (*fmu)(0);
    }
    return d;
  }

  Vec PlanarStribeckFriction::solve(const SqrMat& G, const Vec& gdn, double laN) {
    double laNmu = fabs(laN) * (*fmu)(0);
    double sdG = -gdn(0) / G(0, 0);
    if (fabs(sdG) <= laNmu)
      return Vec(1, INIT, sdG);
    else
      return Vec(1, INIT, (laNmu <= sdG) ? laNmu : -laNmu);
  }

  bool PlanarStribeckFriction::isFulfilled(const Vec& la, const Vec& gdn, double laN, double laTol, double gdTol) {
    if (fabs(la(0) + gdn(0) / fabs(gdn(0)) * (*fmu)(0) * fabs(laN)) <= laTol)
      return true;
    else if (fabs(la(0)) <= (*fmu)(0) * fabs(laN) + laTol && fabs(gdn(0)) <= gdTol)
      return true;
    else
      return false;
  }

  Vec PlanarStribeckFriction::dlaTdlaN(const Vec& gd, double laN) {
    return Vec(1, INIT, -(*fmu)(fabs(gd(0))) * sign(gd(0)));
  }

  Vec SpatialStribeckFriction::project(const Vec& la, const Vec& gdn, double laN, double r) {
    return proxCT3D(la - r * gdn, (*fmu)(nrm2(gdn)) * fabs(laN));
  }

  Mat SpatialStribeckFriction::diff(const Vec& la, const Vec& gdn, double laN, double r) {
    Vec argT = la - r * gdn;
    Mat E(2, 2, EYE);
    Mat d(2, 5, NONINIT);
    if (nrm2(argT) < (*fmu)(nrm2(gdn)) * fabs(laN)) {
      d(Index(0, 1), Index(0, 1)) = E;
      d(Index(0, 1), Index(2, 3)) = -r * E;
      d(Index(0, 1), Index(4, 4)).init(0);
    }
    else {
      Mat d_dargT = (E - (argT * argT.T()) / (argT.T() * argT)) * (*fmu)(nrm2(gdn)) * la(0) / nrm2(argT);
      d(Index(0, 1), Index(0, 1)) = d_dargT;
      d(Index(0, 1), Index(2, 3)) = -r * d_dargT;
      d(Index(0, 1), Index(4, 4)) = argT / nrm2(argT) * (*fmu)(nrm2(gdn));
    }
    return d;
  }

  Vec SpatialStribeckFriction::solve(const SqrMat& G, const Vec& gdn, double laN) {
    throw MBSimError("ERROR (SpatialStribeckFriction::solve): Not implemented!");
  }

  bool SpatialStribeckFriction::isFulfilled(const Vec& la, const Vec& gdn, double laN, double laTol, double gdTol) {
    if (nrm2(la + gdn / nrm2(gdn) * (*fmu)(nrm2(gdn)) * fabs(laN)) <= laTol)
      return true;
    else if (nrm2(la) <= (*fmu)(nrm2(gdn)) * fabs(laN) + laTol && nrm2(gdn) <= gdTol)
      return true;
    else
      return false;
  }

  Vec SpatialStribeckFriction::dlaTdlaN(const Vec& gd, double laN) {
    return -(*fmu)(nrm2(gd)) * gd / nrm2(gd);
  }

  TiXmlElement* FrictionImpactLaw::writeXMLFile(TiXmlNode *parent) { 
    TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType());
    parent->LinkEndChild(ele0);
    return ele0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FrictionImpactLaw, PlanarCoulombImpact, MBSIMNS"PlanarCoulombImpact")

  Vec PlanarCoulombImpact::project(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r) {
    return Vec(1, INIT, proxCT2D(la(0) - r * gdn(0), mu * fabs(laN)));
  }

  Mat PlanarCoulombImpact::diff(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r) {
    double argT = la(0) - r * gdn(0);
    Mat d(1, 3, NONINIT);
    if (abs(argT) < mu * fabs(laN)) {
      //d_dargT = Mat(2,2,EYE);
      d(0, 0) = 1;
      d(0, 1) = -r;
      d(0, 2) = 0;
    }
    else {
      d(0, 0) = 0;
      d(0, 1) = 0;
      d(0, 2) = sign(argT) * sign(laN) * mu;
    }
    return d;
  }

  Vec PlanarCoulombImpact::solve(const SqrMat& G, const Vec& gdn, const Vec& gda, double laN) {
    double laNmu = fabs(laN) * mu;
    double sdG = -gdn(0) / G(0, 0);
    if (fabs(sdG) <= laNmu)
      return Vec(1, INIT, sdG);
    else
      return Vec(1, INIT, (laNmu <= sdG) ? laNmu : -laNmu);
  }

  bool PlanarCoulombImpact::isFulfilled(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double laTol, double gdTol) {
    if (fabs(la(0) + gdn(0) / fabs(gdn(0)) * mu * fabs(laN)) <= laTol)
      return true;
    else if (fabs(la(0)) <= mu * fabs(laN) + laTol && fabs(gdn(0)) <= gdTol)
      return true;
    else
      return false;
  }

  int PlanarCoulombImpact::isSticking(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double laTol, double gdTol) {
    if (fabs(la(0)) <= mu * fabs(laN) + laTol && fabs(gdn(0)) <= gdTol)
      return 1;
    else
      return 0;
  }

  void PlanarCoulombImpact::initializeUsingXML(TiXmlElement *element) {
    FrictionImpactLaw::initializeUsingXML(element);
    TiXmlElement *e;
    e = element->FirstChildElement(MBSIMNS"frictionCoefficient");
    setFrictionCoefficient(Element::getDouble(e));
  }

  TiXmlElement* PlanarCoulombImpact::writeXMLFile(TiXmlNode *parent) { 
    TiXmlElement *ele0 = FrictionImpactLaw::writeXMLFile(parent);
    addElementText(ele0,MBSIMNS"frictionCoefficient",mu);
    return ele0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FrictionImpactLaw, SpatialCoulombImpact, MBSIMNS"SpatialCoulombImpact")

  Vec SpatialCoulombImpact::project(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r) {
    return proxCT3D(la - r * gdn, mu * fabs(laN));
  }

  Mat SpatialCoulombImpact::diff(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r) {
    Vec argT = la - r * gdn;
    Mat E(2, 2, EYE);
    Mat d(2, 5, NONINIT);
    if (nrm2(argT) < mu * fabs(laN)) {
      //d_dargT = Mat(2,2,EYE);
      d(Index(0, 1), Index(0, 1)) = E;
      d(Index(0, 1), Index(2, 3)) = -r * E;
      d(Index(0, 1), Index(4, 4)).init(0);
    }
    else {
      Mat d_dargT = (E - (argT * argT.T()) / (argT.T() * argT)) * mu * la(0) / nrm2(argT);
      d(Index(0, 1), Index(0, 1)) = d_dargT;
      d(Index(0, 1), Index(2, 3)) = -r * d_dargT;
      d(Index(0, 1), Index(4, 4)) = argT / nrm2(argT) * mu;
    }
    return d;
  }

  Vec SpatialCoulombImpact::solve(const SqrMat& G, const Vec& gdn, const Vec& gda, double laN) {
    throw MBSimError("ERROR (SpatialCoulombImpact::solve): Not implemented!");
  }

  bool SpatialCoulombImpact::isFulfilled(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double laTol, double gdTol) {
    if (nrm2(la + gdn / nrm2(gdn) * mu * fabs(laN)) <= laTol)
      return true;
    else if (nrm2(la) <= mu * fabs(laN) + laTol && nrm2(gdn) <= gdTol)
      return true;
    else
      return false;
  }

  int SpatialCoulombImpact::isSticking(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double laTol, double gdTol) {
    if (nrm2(la) <= mu * fabs(laN) + laTol && nrm2(gdn) <= gdTol)
      return 1;
    else
      return 0;
  }

  void SpatialCoulombImpact::initializeUsingXML(TiXmlElement *element) {
    FrictionImpactLaw::initializeUsingXML(element);
    TiXmlElement *e;
    e = element->FirstChildElement(MBSIMNS"frictionCoefficient");
    setFrictionCoefficient(Element::getDouble(e));
  }

  TiXmlElement* SpatialCoulombImpact::writeXMLFile(TiXmlNode *parent) { 
    TiXmlElement *ele0 = FrictionImpactLaw::writeXMLFile(parent);
    addElementText(ele0,MBSIMNS"frictionCoefficient",mu);
    return ele0;
  }

  Vec PlanarStribeckImpact::project(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r) {
    return Vec(1, INIT, proxCT2D(la(0) - r * gdn(0), (*fmu)(fabs(gdn(0))) * fabs(laN)));
  }

  Mat PlanarStribeckImpact::diff(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r) {
    double argT = la(0) - r * gdn(0);
    Mat d(1, 3, NONINIT);
    if (abs(argT) < (*fmu)(fabs(gdn(0))) * fabs(laN)) {
      d(0, 0) = 1;
      d(0, 1) = -r;
      d(0, 2) = 0;
    }
    else {
      d(0, 0) = 0;
      d(0, 1) = 0;
      d(0, 2) = sign(argT) * sign(laN) * (*fmu)(fabs(gdn(0)));
    }
    return d;
  }

  Vec PlanarStribeckImpact::solve(const SqrMat& G, const Vec& gdn, const Vec& gda, double laN) {
    double laNmu = fabs(laN) * (*fmu)(fabs(gdn(0)));
    double sdG = -gdn(0) / G(0, 0);
    if (fabs(sdG) <= laNmu)
      return Vec(1, INIT, sdG);
    else
      return Vec(1, INIT, (laNmu <= sdG) ? laNmu : -laNmu);
  }

  bool PlanarStribeckImpact::isFulfilled(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double laTol, double gdTol) {
    if (fabs(la(0) + gdn(0) / fabs(gdn(0)) * (*fmu)(fabs(gdn(0))) * fabs(laN)) <= laTol)
      return true;
    else if (fabs(la(0)) <= (*fmu)(fabs(gdn(0))) * fabs(laN) + laTol && fabs(gdn(0)) <= gdTol)
      return true;
    else
      return false;
  }

  int PlanarStribeckImpact::isSticking(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double laTol, double gdTol) {
    if (fabs(la(0)) <= (*fmu)(fabs(gdn(0))) * fabs(laN) + laTol && fabs(gdn(0)) <= gdTol)
      return 1;
    else
      return 0;
  }

  Vec SpatialStribeckImpact::project(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r) {
    return proxCT3D(la - r * gdn, (*fmu)(nrm2(gdn)) * fabs(laN));
  }

  Mat SpatialStribeckImpact::diff(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r) {
    Vec argT = la - r * gdn;
    Mat E(2, 2, EYE);
    Mat d(2, 5, NONINIT);
    if (nrm2(argT) < (*fmu)(nrm2(gdn)) * fabs(laN)) {
      d(Index(0, 1), Index(0, 1)) = E;
      d(Index(0, 1), Index(2, 3)) = -r * E;
      d(Index(0, 1), Index(4, 4)).init(0);
    }
    else {
      Mat d_dargT = (E - (argT * argT.T()) / (argT.T() * argT)) * (*fmu)(nrm2(gdn)) * la(0) / nrm2(argT);
      d(Index(0, 1), Index(0, 1)) = d_dargT;
      d(Index(0, 1), Index(2, 3)) = -r * d_dargT;
      d(Index(0, 1), Index(4, 4)) = argT / nrm2(argT) * (*fmu)(nrm2(gdn));
    }
    return d;
  }

  Vec SpatialStribeckImpact::solve(const SqrMat& G, const Vec& gdn, const Vec& gda, double laN) {
    throw MBSimError("ERROR (SpatialStribeckImpact::solve): Not implemented!");
  }

  bool SpatialStribeckImpact::isFulfilled(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double laTol, double gdTol) {
    if (nrm2(la + gdn / nrm2(gdn) * (*fmu)(nrm2(gdn)) * fabs(laN)) <= laTol)
      return true;
    else if (nrm2(la) <= (*fmu)(nrm2(gdn)) * fabs(laN) + laTol && nrm2(gdn) <= gdTol)
      return true;
    else
      return false;
  }

  int SpatialStribeckImpact::isSticking(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double laTol, double gdTol) {
    if (nrm2(la) <= (*fmu)(nrm2(gdn)) * fabs(laN) + laTol && nrm2(gdn) <= gdTol)
      return 1;
    else
      return 0;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(GeneralizedForceLaw, RegularizedUnilateralConstraint, MBSIMNS"RegularizedUnilateralConstraint")

  void RegularizedUnilateralConstraint::initializeUsingXML(TiXmlElement *element) {
    GeneralizedForceLaw::initializeUsingXML(element);
    TiXmlElement *e;
    e = element->FirstChildElement(MBSIMNS"forceFunction");
    Function2<double, double, double> *f = ObjectFactory<Function>::create<Function2<double,double,double> >(e->FirstChildElement());
    setForceFunction(f);
    f->initializeUsingXML(e->FirstChildElement());
  }

  void RegularizedUnilateralConstraint::computeSmoothForces(std::vector<std::vector<SingleContact> > & contacts) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        (*jter).getlaN()(0) = (*forceFunc)((*jter).getg()(0), (*jter).getgdN()(0));
      }
    }
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(GeneralizedForceLaw, MaxwellUnilateralConstraint, MBSIMNS"MaxwellUnilateralConstraint")

  MaxwellUnilateralConstraint::MaxwellUnilateralConstraint(const double & damping, const double & gapLimit) :
      lcpSolvingStrategy(Standard), dampingCoefficient(damping), gLim(gapLimit), matConst(0), matConstSetted(false), DEBUGLEVEL(0) {

  }

  void MaxwellUnilateralConstraint::initializeContourCouplings(Contact* parent) {
    for(size_t i = 0; i < referenceXML.size(); i++) {
      Contour* contour1 = parent->getByPath<Contour>(referenceXML[i].name1);
      Contour* contour2 = parent->getByPath<Contour>(referenceXML[i].name2);
      addContourCoupling(contour1, contour2, referenceXML[i].function);
    }
  }

  void MaxwellUnilateralConstraint::addContourCoupling(Contour *contour1, Contour *contour2, InfluenceFunction *fct) {
    pair<Contour*, Contour*> Pair(contour1, contour2);
    if (contour2 < contour1)
      Pair = pair<Contour*, Contour*>(contour2, contour1);
    if (!influenceFunctions.count(Pair)) {
      influenceFunctions[Pair] = fct;
    }
    else {
      cout << "WARNING: Function existed for contour-pair: \"" << contour1->getName() << "\" + \"" << contour2->getName() << "\"." << endl;
      cout << "         No Function has been added." << endl;
    }

  }

  void MaxwellUnilateralConstraint::computeSmoothForces(std::vector<std::vector<SingleContact> > & contacts) {
    updatePossibleContactPoints(contacts);

    //Apply damping force
    //TODO: use damping function for that (to be more flexible...)
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        if ((*jter).getg()(0) < gLim and (*jter).getgdN()(0) < 0)
          (*jter).getlaN()(0) = -dampingCoefficient * (*jter).getgdN()(0);
        else
          (*jter).getlaN()(0) = 0;
      }
    }

    if (possibleContactPoints.size()) {
      updateInfluenceMatrix(contacts);
      updateRigidBodyGap(contacts);

      LinearComplementarityProblem LCP(C, rigidBodyGap, lcpSolvingStrategy);

      map<Index, double> tolerances;
      tolerances.insert(pair<Index, double>(Index(0, possibleContactPoints.size() - 1), 1e-8)); //tolerances for distances
      tolerances.insert(pair<Index, double>(Index(possibleContactPoints.size(), 2 * possibleContactPoints.size() - 1), 1e-3)); //tolerances for forces
      LocalResidualCriteriaFunction* critfunc = new LocalResidualCriteriaFunction(tolerances);
      LCP.setNewtonCriteriaFunction(critfunc);
      LCP.setDebugLevel(0);

      solution0.resize() = LCP.solve(solution0);

      delete critfunc;

      Vec lambda = solution0(rigidBodyGap.size(), 2 * rigidBodyGap.size() - 1);

      if (DEBUGLEVEL >= 3) {
        cout << "lambda = " << lambda << endl;
      }

      for (size_t i = 0; i < possibleContactPoints.size(); ++i) {
        contacts[possibleContactPoints[i].first][possibleContactPoints[i].second].getlaN()(0) += lambda(i);
      }
    }
  }

  void MaxwellUnilateralConstraint::initializeUsingXML(TiXmlElement* element) {
    TiXmlElement *e;
    e = element->FirstChildElement(MBSIMNS"InfluenceFunction");
    while(e) {
      xmlInfo info;
      info.function = ObjectFactory<Function>::create<InfluenceFunction>(e->FirstChildElement());
      info.name1 = e->FirstChildElement()->Attribute("contourName1");
      info.name2 = e->FirstChildElement()->Attribute("contourName2");

      info.function->initializeUsingXML(e->FirstChildElement());

      referenceXML.push_back(info);

      e = e->NextSiblingElement();
    }

  }

void MaxwellUnilateralConstraint::updatePossibleContactPoints(const std::vector<std::vector<SingleContact> > & contacts) {
    possibleContactPoints.clear();
    for (size_t i = 0; i < contacts.size(); ++i) {
      for (size_t j = 0; j < contacts[i].size(); ++j) {
        if (contacts[i][j].getg()(0) < 0) { //TODO: use gActive, but only at timestep No 2...
          possibleContactPoints.push_back(pair<int,int>(i,j));
        }
      }
    }
  }

  void MaxwellUnilateralConstraint::updateInfluenceMatrix(std::vector<std::vector<SingleContact> > & contacts) {
    C.resize(possibleContactPoints.size());

    for (size_t i = 0; i < possibleContactPoints.size(); i++) {
      //get index of contours of current possible contactPoint
      const std::pair<int, int> & currentContactIndex = possibleContactPoints[i];

      C(i, i) = computeInfluenceCoefficient(contacts, currentContactIndex);

      for (size_t j = i + 1; j < possibleContactPoints.size(); j++) {
        //get index of coupled contour
        const std::pair<int, int> & coupledContactIndex = possibleContactPoints[j];

        C(i, j) = computeInfluenceCoefficient(contacts, currentContactIndex, coupledContactIndex);
      }
    }

    if (DEBUGLEVEL >= 5) {
      cout << "The InfluenceMatrix is: " << C << endl;
      cout << "With eigenvalues: " << eigval(C) << endl;
    }
  }

  void MaxwellUnilateralConstraint::updateRigidBodyGap(const std::vector<std::vector<SingleContact> > & contacts) {
    /*save rigidBodyGaps in vector*/
    rigidBodyGap.resize(possibleContactPoints.size());
    for (size_t i = 0; i < possibleContactPoints.size(); i++) {
      rigidBodyGap(i) = contacts[possibleContactPoints[i].first][possibleContactPoints[i].second].getg()(0);
    }

    if (DEBUGLEVEL >= 5)
      cout << "rigidBodyGap: " << rigidBodyGap << endl;
  }

  double MaxwellUnilateralConstraint::computeInfluenceCoefficient(std::vector<std::vector<SingleContact> > & contacts, const std::pair<int, int> & contactIndex) {
    double FactorC = 0.;

    for (int i = 0; i < 2; i++) {

      //get involved contours
      Contour * contour = contacts[contactIndex.first][contactIndex.second].getContour()[i];
      pair<Contour*, Contour*> contourPair = pair<Contour*, Contour*>(contour, contour);

      if (influenceFunctions.count(contourPair)) { //If there is a function, there is a coupling between these contours
        InfluenceFunction *fct = influenceFunctions[contourPair];
        Vec lagrangeParameter;
        lagrangeParameter.resize() = contour->computeLagrangeParameter(contacts[contactIndex.first][contactIndex.second].getcpData()[i].getFrameOfReference().getPosition());

        if (DEBUGLEVEL >= 3) {
          cout << "LagrangeParameter of contour \"" << contour->getShortName() << "\" is:" << lagrangeParameter << endl;
        }

        FactorC += (*fct)(lagrangeParameter, lagrangeParameter);
      }
    }

    if (fabs(FactorC) <= macheps()) {
      throw MBSimError("No elasticity is given for one of the following contours:\n  -" + contacts[contactIndex.first][contactIndex.second].getContour()[0]->getShortName() + "\n  -" + contacts[contactIndex.first][contactIndex.second].getContour()[0]->getShortName() + "\nThat is not an option!");
    }

    return FactorC;
  }

  double MaxwellUnilateralConstraint::computeInfluenceCoefficient(std::vector<std::vector<SingleContact> > & contacts, const std::pair<int, int> & contactIndex, const std::pair<int, int> & coupledContactIndex) {
    double FactorC = 0;

    for (int affectedContourIterator = 0; affectedContourIterator < 2; affectedContourIterator++) {
      for (int coupledContourIterator = 0; coupledContourIterator < 2; coupledContourIterator++) {
        //get involved contours
        Contour *contour1 = contacts[contactIndex.first][contactIndex.second].getContour()[affectedContourIterator];
        Contour *contour2 = contacts[coupledContactIndex.first][coupledContactIndex.second].getContour()[coupledContourIterator];

        pair<Contour*, Contour*> Pair;

        if (contour1 < contour2)
          Pair = pair<Contour*, Contour*>(contour1, contour2);
        else
          Pair = pair<Contour*, Contour*>(contour2, contour1);

        if (influenceFunctions.count(Pair)) { //If there is a function, there is a coupling between these contours
          InfluenceFunction *fct = influenceFunctions[Pair];
          Vec firstLagrangeParameter = Vec(2, NONINIT);
          Vec secondLagrangeParameter = Vec(2, NONINIT);
          firstLagrangeParameter = contour1->computeLagrangeParameter(contacts[contactIndex.first][contactIndex.second].getcpData()[affectedContourIterator].getFrameOfReference().getPosition());
          secondLagrangeParameter = contour2->computeLagrangeParameter(contacts[coupledContactIndex.first][coupledContactIndex.second].getcpData()[coupledContourIterator].getFrameOfReference().getPosition());

          if (DEBUGLEVEL >= 3) {
            cout << "First LagrangeParameter of contour \"" << contour1->getShortName() << "\" is:" << firstLagrangeParameter << endl;
            cout << "Second LagrangeParameter contour \"" << contour2->getShortName() << "\" is:" << secondLagrangeParameter << endl;
          }

          FactorC += (*fct)(firstLagrangeParameter, secondLagrangeParameter);
        }
      }
    }
    return FactorC;
  }

  void MaxwellUnilateralConstraint::computeMaterialConstant() {
    if (!matConstSetted and possibleContactPoints.size()) {
      /*update Material constant*/
      Vec Eigvals = eigval(C);
      double eigvalSum = 0;
      for (int i = 0; i < Eigvals.size(); i++) {
        eigvalSum += Eigvals(i);
      }
      matConst = eigvalSum / Eigvals.size();

      matConstSetted = true;
    }
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(GeneralizedForceLaw, RegularizedBilateralConstraint, MBSIMNS"RegularizedBilateralConstraint")

  void RegularizedBilateralConstraint::initializeUsingXML(TiXmlElement *element) {
    GeneralizedForceLaw::initializeUsingXML(element);
    TiXmlElement *e;
    e = element->FirstChildElement(MBSIMNS"forceFunction");
    Function2<double, double, double> *f = ObjectFactory<Function>::create<Function2<double,double,double> >(e->FirstChildElement());
    setForceFunction(f);
    f->initializeUsingXML(e->FirstChildElement());
  }

  void RegularizedBilateralConstraint::computeSmoothForces(std::vector<std::vector<SingleContact> > & contacts) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        (*jter).getlaN()(0) = (*forceFunc)((*jter).getg()(0), (*jter).getgdN()(0));
      }
    }
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FrictionForceLaw, RegularizedPlanarFriction, MBSIMNS"RegularizedPlanarFriction")

  void RegularizedPlanarFriction::initializeUsingXML(TiXmlElement *element) {
    FrictionForceLaw::initializeUsingXML(element);
    TiXmlElement *e;
    e = element->FirstChildElement(MBSIMNS"frictionForceFunction");
    Function2<Vec, Vec, double> *f = ObjectFactory<Function>::create<Function2<Vec,Vec,double> >(e->FirstChildElement());
    setFrictionForceFunction(f);
    f->initializeUsingXML(e->FirstChildElement());
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FrictionForceLaw, RegularizedSpatialFriction, MBSIMNS"RegularizedSpatialFriction")

  void RegularizedSpatialFriction::initializeUsingXML(TiXmlElement *element) {
    FrictionForceLaw::initializeUsingXML(element);
    TiXmlElement *e;
    e = element->FirstChildElement(MBSIMNS"frictionForceFunction");
    Function2<Vec, Vec, double> *f = ObjectFactory<Function>::create<Function2<Vec,Vec,double> >(e->FirstChildElement());
    setFrictionForceFunction(f);
    f->initializeUsingXML(e->FirstChildElement());
  }

}

