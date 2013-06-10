#include "system.h"
#include "mbsim/contact.h"
#include "mbsim/rigid_body.h"
#include "mbsim/utils/function.h"
#include "mbsim/environment.h"
#include "mbsim/contours/plane.h"
#include "mbsim/contours/frustum.h"
#include "mbsim/contours/cuboid.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/utils/rotarymatrices.h"

#include "openmbvcppinterface/ivbody.h"
#include "openmbvcppinterface/cube.h"

#include <sstream>

using namespace fmatvec;

System::System(const string &name) : DynamicSystemSolver(name) {

  Vec grav(3);
  grav(2)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  double mu = 0.3;
  Plane *ground = new Plane("Ground");
  addFrame(new FixedRelativeFrame("Q",Vec(3),BasicRotAIKy(-M_PI/2)));
  ground->setFrameOfReference(getFrame("Q"));
  addContour(ground);

  double m = 0.1;
  double l,b,h;
  l = 0.1;
  b = 0.1;
  h = 0.1;
  double A=m/12.*(b*b+h*h);
  double B=m/12.*(h*h+l*l);
  double C=m/12.*(l*l+b*b);

  ground->enableOpenMBV(0.05);
  
  const int nWuerfel = 2;
  RigidBody* body[nWuerfel];
  for(int i=0; i<nWuerfel; i++) {
    stringstream nameBody;
    nameBody <<"Koerper"<< i;
    cout << nameBody.str()<<endl;

    body[i] = new RigidBody(nameBody.str());
    body[i]->setTranslation(new TranslationInXYZDirection);
    body[i]->setRotation(new CardanAngles);
    addObject(body[i]);
    body[i]->setMass(m);
    SymMat Theta(3);
    Theta(0,0)=A;
    Theta(1,1)=B;
    Theta(2,2)=C;
    body[i]->setInertiaTensor(Theta);

    Vec q0(6);
    if(i==0) {
      q0(2) = l*1.1;
      body[i]->setInitialGeneralizedPosition(q0);
    }
    else if(i==1) {
      q0(0) = -0.02; 
      q0(1) = 0.02; 
      q0(2) = 2*l*1.5;
      q0(3) = 0.1;
      q0(4) = 0.2;
      q0(5) = -0.1;
      body[i]->setInitialGeneralizedPosition(q0);
    }
    Cuboid *cuboid = new Cuboid("Cuboid");
    cuboid->setFrameOfReference(body[i]->getFrame("C"));
    body[i]->addContour(cuboid);
    cuboid->setXLength(l);
    cuboid->setYLength(b);
    cuboid->setZLength(h);
    cuboid->enableOpenMBV();

    stringstream nameContact;
    nameContact << "ContactGround" << i;
    Contact *cnf = new Contact(nameContact.str());
    cnf->setContactForceLaw(new UnilateralConstraint);
    cnf->setContactImpactLaw(new UnilateralNewtonImpact(0.0));
    cnf->setFrictionForceLaw(new SpatialCoulombFriction(mu));
    cnf->setFrictionImpactLaw(new SpatialCoulombImpact(mu));
    cnf->connect(getContour("Ground"), body[i]->getContour("Cuboid"));
    addLink(cnf);

    for(int j=0; j<i; j++) {
      stringstream nameContact;
      nameContact << "ContactCuboid" << i << j;
      Contact *cnf = new Contact(nameContact.str());
      cnf->setContactForceLaw(new UnilateralConstraint);
      cnf->setContactImpactLaw(new UnilateralNewtonImpact(0.0));
      cnf->setFrictionForceLaw(new SpatialCoulombFriction(mu));
      cnf->setFrictionImpactLaw(new SpatialCoulombImpact(mu));
      cnf->connect(body[j]->getContour("Cuboid"), body[i]->getContour("Cuboid"));
      addLink(cnf);
    }
  }

}

