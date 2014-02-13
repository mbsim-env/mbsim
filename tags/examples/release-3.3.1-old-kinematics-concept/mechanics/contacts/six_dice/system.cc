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

class Angle : public  Function1<double,double> {
  public:
    double operator()(const double& t, const void*) {
      double al;
      if(t <= 0.35)
	al = 0.0;
      else if(t <= 1.7)
	al =  2*(t-0.35);
      else
	al = 2*(1.7-0.35);
      return al;
    }
};

class Omega : public  Function1<Vec3,double> {
  public:
    Vec3 operator()(const double& t, const void*) {
      Vec3 om(3);
      if(t <= 0.35)
	om(1) = 0.0;
      else if(t <= 1.7)
	om(1) = 2;
      else
	om(1) = 0;
      return om;
    }
};

System::System(const string &name) : DynamicSystemSolver(name) {

  Vec grav(3);
  grav(2)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

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
  double  B=m/12.*(h*h+l*l);
  double  C=m/12.*(l*l+b*b);
  double mu = 0.3;

  Vec rB(3);
  rB(2) = 0.6;

  RigidBody* cup = new RigidBody("Cup");
  cup->setRotation(new TimeDependentRotationAboutFixedAxis(new Angle,"[0;1;0]"));
  cup->setGuidingVelocityOfRotation(new TimeDependentGuidingVelocity(new Omega));
  SymMat Theta(3);
  Theta(1,1) = 0.5*0.1*0.2*0.2;
  addFrame(new FixedRelativeFrame("Is",rB,SqrMat(3,EYE)));
  cup->setFrameOfReference(getFrame("Is"));
  addObject(cup);
  OpenMBV::IvBody *obj=new OpenMBV::IvBody;
  obj->setIvFileName("iv/cup.iv");
  obj->setScaleFactor(0.1);
  obj->setInitialRotation(0,0,0);
  obj->setInitialTranslation(0,0,0);
  cup->setOpenMBVRigidBody(obj);

  Frustum *frustum = new Frustum("CupFrustum");
  frustum->setHeight(0.4);
  Vec radii(2,INIT,0.16);
  frustum->setRadii(radii);
  cup->addFrame(new FixedRelativeFrame("P","[0;0;-0.2]",BasicRotAIKx(M_PI/2)));
  frustum->setFrameOfReference(cup->getFrame("P"));
  cup->addContour(frustum);
  Plane *wall = new Plane("CupGround");
  cup->addFrame(new FixedRelativeFrame("Q","[0;0;-0.2]",BasicRotAIKy(-M_PI/2)));
  wall->setFrameOfReference(cup->getFrame("Q"));
  cup->addContour(wall);

  //frustum->enableOpenMBV();
  //ground->enableOpenMBV(0.05);

  const int nWuerfel = 6;
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

    Vec WrOS0(3);
    WrOS0(1) = 0.0;
    WrOS0(0) = 0.0;
    if(i==0) {
      WrOS0(0) = 0.06;
      WrOS0(2) = rB(2)-0.2-l/2+1.1*l;
    } else if(i==1) {
      WrOS0(0) = -0.06;
      WrOS0(2) = rB(2)-0.2-l/2+1.11*l;
    } else if(i==2) {
      WrOS0(1) = 0.06;
      WrOS0(2) = rB(2)-0.2-l/2+2*1.1*l;
    } else if(i==3) {
      WrOS0(1) = -0.06;
      WrOS0(2) = rB(2)-0.2-l/2+2*1.11*l;
    } else if(i==4) {
      WrOS0(0) = 0.06;
      WrOS0(2) = rB(2)-0.2-l/2+3*1.1*l;
    } else if(i==5) {
      WrOS0(0) = -0.06;
      WrOS0(2) = rB(2)-0.2-l/2+3*1.11*l;
    }
    q0(0,2) = WrOS0;
    SqrMat AWK0(3);
    double a = (i)*0.01;
    AWK0(0,0) = cos(a);
    AWK0(0,1) = sin(a);
    AWK0(1,0) = -sin(a);
    AWK0(1,1) = cos(a);
    AWK0(2,2) = 1;
    SqrMat AWK02(3);
    AWK02(2,2) = cos(a);
    AWK02(2,1) = sin(a);
    AWK02(1,2) = -sin(a);
    AWK02(1,1) = cos(a);
    AWK02(0,0) = 1;
    q0(3) = a;
    q0(5) = a;
    body[i]->setInitialGeneralizedPosition(q0);

    Cuboid *cuboid = new Cuboid("Cuboid");
    cuboid->setFrameOfReference(body[i]->getFrame("C"));
    body[i]->addContour(cuboid);
    cuboid->setXLength(l);
    cuboid->setYLength(b);
    cuboid->setZLength(h);
    //cuboid->enableOpenMBV();

    obj=new OpenMBV::IvBody;
    stringstream ivname;
    ivname << "iv/cube" << ceil((i+1)/2.) << ".iv";
    obj->setIvFileName(ivname.str());
    obj->setScaleFactor(0.1);
    obj->setInitialRotation(0,0,0);
    obj->setInitialTranslation(0,0,0);
    body[i]->setOpenMBVRigidBody(obj);

    stringstream nameContact;
    nameContact <<"ContactCuboidCupFrustum" << i;
    Contact *cnf = new Contact(nameContact.str());
    cnf->connect(cup->getContour("CupFrustum"), body[i]->getContour("Cuboid"));
    cnf->setContactForceLaw(new UnilateralConstraint);
    cnf->setContactImpactLaw(new UnilateralNewtonImpact(0.0));
    cnf->setFrictionForceLaw(new SpatialCoulombFriction(mu));
    cnf->setFrictionImpactLaw(new SpatialCoulombImpact(mu));
    addLink(cnf);

    nameContact.clear();
    nameContact <<"ContactCuboidCupGround" << i;
    cnf = new Contact(nameContact.str());
    cnf->connect(cup->getContour("CupGround"), body[i]->getContour("Cuboid"));
    cnf->setContactForceLaw(new UnilateralConstraint);
    cnf->setContactImpactLaw(new UnilateralNewtonImpact(0.0));
    cnf->setFrictionForceLaw(new SpatialCoulombFriction(mu));
    cnf->setFrictionImpactLaw(new SpatialCoulombImpact(mu));
    addLink(cnf);

    nameContact.clear();
    nameContact << "ContactCuboidGround" << i;
    cnf = new Contact(nameContact.str());
    cnf->setContactForceLaw(new UnilateralConstraint);
    cnf->setContactImpactLaw(new UnilateralNewtonImpact(0.0));
    cnf->setFrictionForceLaw(new SpatialCoulombFriction(mu));
    cnf->setFrictionImpactLaw(new SpatialCoulombImpact(mu));
    cnf->connect(getContour("Ground"), body[i]->getContour("Cuboid"));
    addLink(cnf);

    for(int j=0; j<i; j++) {
      stringstream nameContact;
      nameContact << "ContactCuboidCuboid" << i << j;
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

