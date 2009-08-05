/* Copyright (C) 2004-2006  Martin Förg

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

#include <mbsim/utils/function.h>
#include <mbsim/objectfactory.h>

using namespace fmatvec;

namespace MBSim {

  void LinearSpringDamperForce::initializeUsingXML(TiXmlElement *element) {
    Function2<double,double,double>::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMNS"stiffnessCoefficient");
    c=atof(e->GetText());
    e=element->FirstChildElement(MBSIMNS"dampingCoefficient");
    d=atof(e->GetText());
    e=element->FirstChildElement(MBSIMNS"unloadedLength");
    l0=atof(e->GetText());
  }

  void LinearRegularizedUnilateralConstraint::initializeUsingXML(TiXmlElement *element) {
    Function2<double,double,double>::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMNS"stiffnessCoefficient");
    c=atof(e->GetText());
    e=element->FirstChildElement(MBSIMNS"dampingCoefficient");
    d=atof(e->GetText());
  }

  void LinearRegularizedBilateralConstraint::initializeUsingXML(TiXmlElement *element) {
    Function2<double,double,double>::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMNS"stiffnessCoefficient");
    c=atof(e->GetText());
    e=element->FirstChildElement(MBSIMNS"dampingCoefficient");
    d=atof(e->GetText());
  }

  void LinearRegularizedCoulombFriction::initializeUsingXML(TiXmlElement *element) {
    Function2<Vec,Vec,double>::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMNS"marginalVelocity");
    if(e) gdLim=atof(e->GetText());
    e=element->FirstChildElement(MBSIMNS"frictionCoefficient");
    mu=atof(e->GetText());
  }

  Vec LinearRegularizedPlanarStribeckFriction::operator()(const Vec &gd, const double& laN) { 
    int nFric = gd.size();
    Vec la(nFric,NONINIT);
    double normgd = nrm2(gd(0,nFric-1));
    if(normgd < gdLim) {
      double mu0 = (*fmu)(0);
      la(0,nFric-1) = gd(0,nFric-1)*(-laN*mu0/gdLim);
    } else {
      double mu = (*fmu)(nrm2(gd(0,nFric-1))-gdLim); //oder (*fmu)(nrm2(gd(0,nFric-1)))
      la(0,nFric-1) = gd(0,nFric-1)*(-laN*mu/normgd);
    }
    return la;
  }

  void LinearRegularizedPlanarStribeckFriction::initializeUsingXML(TiXmlElement *element) {
    Function2<Vec,Vec,double>::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMNS"marginalVelocity");
    if(e) gdLim=atof(e->GetText());
    e=element->FirstChildElement(MBSIMNS"frictionFunction");
    Function1<double,double> *f=ObjectFactory::getInstance()->createFunction1_SS(e->FirstChildElement());
    setFrictionFunction(f);
    f->initializeUsingXML(e->FirstChildElement());
  }

}
