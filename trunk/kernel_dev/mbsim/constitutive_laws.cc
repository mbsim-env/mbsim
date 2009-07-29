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
 * Contact: mfoerg@users.berlios.de
 */

#include <config.h>
#include "mbsim/constitutive_laws.h"
#include "mbsim/element.h"
#include "mbsim/utils/nonsmooth_algebra.h"
#include "mbsim/utils/utils.h"

using namespace std;
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

  double UnilateralConstraint::project(double la, double gdn, double r) {
    return proxCN(la-r*gdn);
  }

  Vec UnilateralConstraint::diff(double la, double gdn, double r) {
    Vec d(2,NONINIT);
    if(la-r*gdn < 0) {
      d.init(0);
    } else {
      d(0) = 1;
      d(1) = -r;
    }
    return d;
  }

  double UnilateralConstraint::solve(double G, double gdn) {
    if(gdn >= 0)
      return 0;
    else 
      return -gdn/G;
  }

  bool UnilateralConstraint::isFulfilled(double la, double gdn, double laTol, double gdTol) {
    if(gdn >= -gdTol && fabs(la) <= laTol)
      return true;
    else if(la >= -laTol && fabs(gdn) <= gdTol)
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

  double BilateralConstraint::project(double la, double gdn, double r) {
    return la-r*gdn;
  }

  Vec BilateralConstraint::diff(double la, double gdn, double r) {
    Vec d(2,NONINIT);
    d(0) = 1;
    d(1) = -r;
    return d;
  }

  double BilateralConstraint::solve(double G, double gdn) {
    return -gdn/G;
  }

  bool BilateralConstraint::isFulfilled(double la, double gdn, double laTol, double gdTol) {
    return fabs(gdn) <= gdTol;
  }

  double UnilateralNewtonImpact::project(double la, double gdn, double gda, double r) {
    if(fabs(gda) > gd_limit)
      gdn += epsilon*gda;
    return proxCN(la-r*gdn);
  }

  Vec UnilateralNewtonImpact::diff(double la, double gdn, double gda, double r) {
    Vec d(2,NONINIT);
    if(la-r*gdn < 0) {
      d.init(0);
    } else {
      d(0) = 1;
      d(1) = -r;
    }
    return d;
  }

  double UnilateralNewtonImpact::solve(double G, double gdn, double gda) {
    if(fabs(gda) > gd_limit)
      gdn += epsilon*gda;

    if(gdn >= 0)
      return 0;
    else 
      return -gdn/G;
  }

  bool UnilateralNewtonImpact::isFulfilled(double la, double gdn, double gda, double laTol, double gdTol) {
    if(fabs(gda) > gd_limit)
      gdn += epsilon*gda;

    if(gdn >= -gdTol && fabs(la) <= laTol)
      return true;
    else if(la >= -laTol && fabs(gdn) <= gdTol)
      return true;
    else 
      return false;
  }

  void UnilateralNewtonImpact::initializeUsingXML(TiXmlElement *element) {
    GeneralizedImpactLaw::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMNS"restitutionCoefficient");
    epsilon=atof(e->GetText());
  }

  double BilateralImpact::project(double la, double gdn, double gda, double r) {
    return la-r*gdn;
  }

  Vec BilateralImpact::diff(double la, double gdn, double gda, double r) {
    Vec d(2,NONINIT);
    d(0) = 1;
    d(1) = -r;
    return d;
  }

  double BilateralImpact::solve(double G, double gdn, double gda) {
    return -gdn/G;
  }

  bool BilateralImpact::isFulfilled(double la, double gdn, double gda, double laTol, double gdTol) {
    return fabs(gdn) <= gdTol;
  }

  Vec PlanarCoulombFriction::project(const Vec& la, const Vec& gdn, double laN, double r) {
    return Vec(1,INIT,proxCT2D(la(0)-r*gdn(0),mu*fabs(laN)));
  }

  Mat PlanarCoulombFriction::diff(const Vec& la, const Vec& gdn, double laN, double r) {
    double argT = la(0)-r*gdn(0);
    Mat d(1,3,NONINIT);
    if(abs(argT) < mu*fabs(laN)) {
      //d_dargT = Mat(2,2,EYE);
      d(0,0) = 1;
      d(0,1) = -r;
      d(0,2) = 0;
    } else {
      d(0,0) = 0;
      d(0,1) = 0;
      d(0,2) = sign(argT)*sign(laN)*mu;
    }
    return d;
  }

  Vec PlanarCoulombFriction::solve(const SqrMat& G, const Vec& gdn, double laN) {
    double laNmu = fabs(laN)*mu;
    double sdG = -gdn(0)/G(0,0);
    if(fabs(sdG)<=laNmu) 
      return Vec(1,INIT,sdG);
    else 
      return Vec(1,INIT,(laNmu<=sdG) ? laNmu : -laNmu);
  }

  bool PlanarCoulombFriction::isFulfilled(const Vec& la, const Vec& gdn, double laN, double laTol, double gdTol) {
    if(fabs(la(0) + gdn(0)/fabs(gdn(0))*mu*fabs(laN)) <= laTol)
      return true;
    else if(fabs(la(0)) <= mu*fabs(laN)+laTol && fabs(gdn(0)) <= gdTol)
      return true;
    else 
      return false;
  }

  Vec PlanarCoulombFriction::dlaTdlaN(const Vec& gd, double laN) {
    return Vec(1,INIT,-mu*sign(gd(0)));
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

  Vec SpatialCoulombFriction::project(const Vec& la, const Vec& gdn, double laN, double r) {
    return proxCT3D(la-r*gdn,mu*fabs(laN));
  }

  Mat SpatialCoulombFriction::diff(const Vec& la, const Vec& gdn, double laN, double r) {
    Vec argT = la-r*gdn;
    Mat E(2,2,EYE);
    Mat d(2,5,NONINIT);
    if(nrm2(argT) < mu*fabs(laN)) {
      //d_dargT = Mat(2,2,EYE);
      d(Index(0,1),Index(0,1)) = E;
      d(Index(0,1),Index(2,3)) = -r*E;
      d(Index(0,1),Index(4,4)).init(0);
    } else {
      Mat d_dargT = (E - (argT*trans(argT))/(trans(argT)*argT))*mu*la(0)/nrm2(argT);
      d(Index(0,1),Index(0,1)) = d_dargT;
      d(Index(0,1),Index(2,3)) = -r*d_dargT;
      d(Index(0,1),Index(4,4)) = argT/nrm2(argT)*mu;
    }
    return d;
  }

  Vec SpatialCoulombFriction::solve(const SqrMat& G, const Vec& gdn, double laN) {
    cout << "solve is not implemented for spatial Coulomb friction" << endl;
    throw 5;
  }

  bool SpatialCoulombFriction::isFulfilled(const Vec& la, const Vec& gdn, double laN, double laTol, double gdTol) {
    if(nrm2(la + gdn/nrm2(gdn)*mu*fabs(laN)) <= laTol)
      return true;
    else if(nrm2(la) <= mu*fabs(laN)+laTol && nrm2(gdn) <= gdTol)
      return true;
    else 
      return false;
  }

  Vec SpatialCoulombFriction::dlaTdlaN(const Vec& gd, double laN) {
    return -mu*gd/nrm2(gd);
  }

  void SpatialCoulombFriction::initializeUsingXML(TiXmlElement *element) {
    FrictionForceLaw::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMNS"frictionCoefficient");
    setFrictionCoefficient(atof(e->GetText()));
  }

  Vec PlanarCoulombImpact::project(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r) {
    return Vec(1,INIT,proxCT2D(la(0)-r*gdn(0),mu*fabs(laN)));
  }

  Mat PlanarCoulombImpact::diff(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r) {
    double argT = la(0)-r*gdn(0);
    Mat d(1,3,NONINIT);
    if(abs(argT) < mu*fabs(laN)) {
      //d_dargT = Mat(2,2,EYE);
      d(0,0) = 1;
      d(0,1) = -r;
      d(0,2) = 0;
    } else {
      d(0,0) = 0;
      d(0,1) = 0;
      d(0,2) = sign(argT)*sign(laN)*mu;
    }
    return d;
  }

  Vec PlanarCoulombImpact::solve(const SqrMat& G, const Vec& gdn, const Vec& gda, double laN) {
    double laNmu = fabs(laN)*mu;
    double sdG = -gdn(0)/G(0,0);
    if(fabs(sdG)<=laNmu) 
      return Vec(1,INIT,sdG);
    else 
      return Vec(1,INIT,(laNmu<=sdG) ? laNmu : -laNmu);
  }

  bool PlanarCoulombImpact::isFulfilled(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double laTol, double gdTol) {
    if(fabs(la(0) + gdn(0)/fabs(gdn(0))*mu*fabs(laN)) <= laTol)
      return true;
    else if(fabs(la(0)) <= mu*fabs(laN)+laTol && fabs(gdn(0)) <= gdTol)
      return true;
    else 
      return false;
  }

  Vec SpatialCoulombImpact::project(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r) {
    return proxCT3D(la-r*gdn,mu*fabs(laN));
  }

  Mat SpatialCoulombImpact::diff(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r) {
    Vec argT = la-r*gdn;
    Mat E(2,2,EYE);
    Mat d(2,5,NONINIT);
    if(nrm2(argT) < mu*fabs(laN)) {
      //d_dargT = Mat(2,2,EYE);
      d(Index(0,1),Index(0,1)) = E;
      d(Index(0,1),Index(2,3)) = -r*E;
      d(Index(0,1),Index(4,4)).init(0);
    } else {
      Mat d_dargT = (E - (argT*trans(argT))/(trans(argT)*argT))*mu*la(0)/nrm2(argT);
      d(Index(0,1),Index(0,1)) = d_dargT;
      d(Index(0,1),Index(2,3)) = -r*d_dargT;
      d(Index(0,1),Index(4,4)) = argT/nrm2(argT)*mu;
    }
    return d;
  }

  Vec SpatialCoulombImpact::solve(const SqrMat& G, const Vec& gdn, const Vec& gda, double laN) {
    cout << "solve is not implemented for spatial Coulomb friction" << endl;
    throw 5;
  }

  bool SpatialCoulombImpact::isFulfilled(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double laTol, double gdTol) {
    if(nrm2(la + gdn/nrm2(gdn)*mu*fabs(laN)) <= laTol)
      return true;
    else if(nrm2(la) <= mu*fabs(laN)+laTol && nrm2(gdn) <= gdTol)
      return true;
    else 
      return false;
  }

  void SpatialCoulombImpact::initializeUsingXML(TiXmlElement *element) {
    FrictionImpactLaw::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMNS"frictionCoefficient");
    setFrictionCoefficient(atof(e->GetText()));
  }

  void LinearRegularizedBilateralConstraint::initializeUsingXML(TiXmlElement *element) {
    RegularizedBilateralConstraint::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMNS"stiffnessCoefficient");
    c=atof(e->GetText());
    e=element->FirstChildElement(MBSIMNS"dampingCoefficient");
    d=atof(e->GetText());
  }

  void LinearRegularizedUnilateralConstraint::initializeUsingXML(TiXmlElement *element) {
    RegularizedUnilateralConstraint::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMNS"stiffnessCoefficient");
    c=atof(e->GetText());
    e=element->FirstChildElement(MBSIMNS"dampingCoefficient");
    d=atof(e->GetText());
  }

  void LinearRegularizedSpatialCoulombFriction::initializeUsingXML(TiXmlElement *element) {
    FrictionForceLaw::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMNS"frictionCoefficient");
    setFrictionCoefficient(atof(e->GetText()));
  }
}

