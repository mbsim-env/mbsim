#include "system.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/links/joint.h"
#include "mbsim/links/contact.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/plate.h"
#include "mbsim/constitutive_laws/constitutive_laws.h"
#include "mbsim/environment.h"
#include "mbsim/functions/kinematics/kinematics.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/sphere.h"
#include "openmbvcppinterface/cube.h"
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

  Vec grav(3,INIT,0.);
  grav(0) = -10.;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  /* disk */
  Plate* plate = new Plate("Plate");
  addContour(plate);
  plate->setYLength(3);
  plate->setZLength(2);
  plate->enableOpenMBV();


  /* ball */ 
  vector<RigidBody*> balls;
  vector<Point*> points;
  vector<std::shared_ptr<OpenMBV::Sphere> > sphere;
  vector<Contact*> contact;

  double r = 1e-2; // radius of ball
  double mass = 0.5; // mass of ball
  SymMat Theta(3,INIT,0.); // inertia of ball
  Theta(0,0) = 2./5.*mass*r*r;
  Theta(1,1) = 2./5.*mass*r*r;
  Theta(2,2) = 2./5.*mass*r*r;
  int nB = 4; // number of balls

  // bodies
  for(int k=0; k<nB; k++) {
    stringstream name;
    name << "Ball_" << k;
    balls.push_back(new RigidBody(name.str()));
    this->addObject(balls[k]);

    stringstream frame; // ball location
    frame << "B_"  << k;
    Vec WrOS0B(3,INIT,0.);
    WrOS0B(0) = 0.5;
    WrOS0B(1) = 0.35*sin(k*2.*M_PI/nB);
    WrOS0B(2) = 0.25*cos(k*2.*M_PI/nB);
    this->addFrame(new FixedRelativeFrame(frame.str(),WrOS0B,SqrMat(3,EYE),this->getFrame("I")));

    balls[k]->setFrameOfReference(this->getFrame(frame.str()));
    balls[k]->setFrameForKinematics(balls[k]->getFrame("C"));
    balls[k]->setMass(mass);
    balls[k]->setInertiaTensor(Theta);
    balls[k]->setTranslation(new TranslationAlongAxesXYZ<VecV>); // only translational dof because of point masses
    Vec3 u0;
    if(k==0)
      u0(1) = 1;
    else if(k==1)
      u0(1) = -1;
    else if(k==2)
      u0(2) = 1;
    else
      u0(2) = -1;

    balls[k]->setInitialGeneralizedVelocity(u0);

#ifdef HAVE_OPENMBVCPPINTERFACE
    sphere.push_back(OpenMBV::ObjectFactory::create<OpenMBV::Sphere>());
    sphere[k]->setRadius(r);
    sphere[k]->setDiffuseColor((1-(double)k/nB)*2/3, 1, 1);
    balls[k]->setOpenMBVRigidBody(sphere[k]);
#endif
  }

  // contacts
  for(int k=0; k<nB; k++) {
    stringstream pointname; // point contour at lower position
    pointname << "Point_" << k;
    Vec BR(3,INIT,0.); BR(2)=-r;
    balls[k]->addFrame(new FixedRelativeFrame(pointname.str(),BR,SqrMat(3,EYE),balls[k]->getFrame("C")));
    points.push_back(new Point(pointname.str(),balls[k]->getFrame(pointname.str())));
    balls[k]->addContour(points[k]);

    stringstream contactname; // fricional contact
    contactname << "Contact_" << k;
    contact.push_back(new Contact(contactname.str()));
    contact[k]->setNormalForceLaw(new UnilateralConstraint);
    contact[k]->setNormalImpactLaw(new UnilateralNewtonImpact(.5));
//    contact[k]->setTangentialForceLaw(new SpatialCoulombFriction(0.4));
//    contact[k]->setTangentialImpactLaw(new SpatialCoulombImpact(0.4));
    contact[k]->connect(points[k], plate);
    contact[k]->enableOpenMBVContactPoints();
    this->addLink(contact[k]);
  }

  //  /* cube */
  //  double e = 1e-2; // edge length of cube
  //  double mass = 0.5; // mass of cube
  //  SymMat Theta(3,INIT,0.); // inertia of cube
  //  Theta(0,0) = 1./6.*mass*e*e;
  //  Theta(1,1) = 1./6.*mass*e*e;
  //  Theta(2,2) = 1./6.*mass*e*e;
  //
  //  // body
  //  RigidBody* body = new RigidBody("Cube");
  //  this->addObject(body);
  //
  //  Vec WrOS0B(3,INIT,0.);
  //  WrOS0B(0) = (rI+rO)*0.25;
  //  WrOS0B(2) = b(1)*0.5 + e*0.5 + 1e-2;
  //  this->addFrame("B",WrOS0B,SqrMat(3,EYE),this->getFrame("I"));
  //
  //  body->setFrameOfReference(this->getFrame("B"));
  //  body->setFrameForKinematics(body->getFrame("C"));
  //  body->setMass(mass);
  //  body->setInertiaTensor(Theta);
  //  body->setTranslation(new LinearTranslation(SqrMat(3,EYE))); 
  //  body->setRotation(new CardanAngles());
  //
  //  Vec q0_body(6,INIT,0.); q0_body(4)=-M_PI*0.25; q0_body(5)=-M_PI*0.25;
  //  body->setInitialGeneralizedPosition(q0_body);
  //  Vec u0_body(6,INIT,0.); u0_body(0)=-1.; u0_body(2)=-1.; u0_body(3)=50; u0_body(4)=30; u0_body(5)=20;
  //  body->setInitialGeneralizedVelocity(u0_body);
  //
  //#ifdef HAVE_OPENMBVCPPINTERFACE
  //  std::shared_ptr<OpenMBV::Cube> cube = OpenMBV::ObjectFactory::create<OpenMBV::Cube>();
  //  cube->setLength(e);
  //  cube->setDiffuseColor(1/3.0, 1, 1);
  //  body->setOpenMBVRigidBody(cube);
  //#endif
  //
  //  // contacts
  //  vector<Point*> points;
  //  vector<Contact*> contact;
  //  for(int k=0;k<8;k++) {
  //    stringstream pointname; 
  //    pointname << "Point_" << k;
  //    points.push_back(new Point(pointname.str()));
  //
  //    Vec BR(3,INIT,0.);
  //    if(k==0) { BR(0)=-e*0.5; BR(1)=-e*0.5; BR(2)=-e*0.5; }
  //    if(k==1) { BR(0)=-e*0.5; BR(1)=-e*0.5; BR(2)= e*0.5; }
  //    if(k==2) { BR(0)=-e*0.5; BR(1)= e*0.5; BR(2)=-e*0.5; }
  //    if(k==3) { BR(0)=-e*0.5; BR(1)= e*0.5; BR(2)= e*0.5; }
  //    if(k==4) { BR(0)= e*0.5; BR(1)=-e*0.5; BR(2)=-e*0.5; }
  //    if(k==5) { BR(0)= e*0.5; BR(1)=-e*0.5; BR(2)= e*0.5; }
  //    if(k==6) { BR(0)= e*0.5; BR(1)= e*0.5; BR(2)=-e*0.5; }
  //    if(k==7) { BR(0)= e*0.5; BR(1)= e*0.5; BR(2)= e*0.5; }
  //
  //    body->addContour(points[k],BR,SqrMat(3,EYE),body->getFrame("C"));
  //
  //    stringstream contactname; // fricional contact
  //    contactname << "Contact_" << k;
  //    contact.push_back(new Contact(contactname.str()));
  //    contact[k]->setNormalForceLaw(new UnilateralConstraint);
  //    contact[k]->setNormalImpactLaw(new UnilateralNewtonImpact(0.5));
  //    contact[k]->setTangentialForceLaw(new SpatialCoulombFriction(0.4));
  //    contact[k]->setTangentialImpactLaw(new SpatialCoulombImpact(0.4));
  //    contact[k]->connect(body->getContour(pointname.str()),disk->getContour("SurfaceContour"));
  //    this->addLink(contact[k]);
  //  }

} 

