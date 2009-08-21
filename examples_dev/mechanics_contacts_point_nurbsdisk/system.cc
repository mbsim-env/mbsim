#include "system.h"

#include "mbsim/flexible_body/flexible_body_2s_13_disk.h"
#include "mbsim/rigid_body.h"
#include "mbsim/joint.h"
#include "mbsim/contact.h"
#include "mbsim/contours/point.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/environment.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/nurbsdisk.h"
#include "openmbvcppinterface/sphere.h"
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

  Vec grav(3,INIT,0.);
  grav(2) = -10.;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  /* disk */
  // body
  FlexibleBody2s13Disk *disk = new FlexibleBody2s13Disk("Disk");
  double E = 5.e7; // Young's modulus  
  double rho = 9.2e2; // density
  double nu = 0.3; // poisson ratio
  double rI = 0.1; // inner radius
  double rO = 0.3; // outer radius
  Vec d(3); d(0) = 0.1; // thickness

  int nr = 3; // radial number of elements
  int nj = 9; // azimuthal number of elements

  disk->setEModul(E);
  disk->setDensity(rho);
  disk->setPoissonRatio(nu);
  disk->setRadius(rI,rO);
  disk->setThickness(d,d);
  disk->setFrameOfReference(this->getFrame("I")); // location of disk
  disk->setLockType(innerring); // inner ring has no elastic dof
  disk->setReferenceInertia(1.,1.); // inertia of the reference frame
  disk->setNumberElements(nr,nj);

  Vec q0 = Vec(2+nr*nj*3,INIT,0.); // initial position
  disk->setq0(q0);
  Vec u0 = Vec(2+nr*nj*3,INIT,0.); // initial velocity
  u0(1) = M_PI;
  disk->setu0(u0);
  this->addObject(disk);

  // bearing
  Joint *joint = new Joint("Clamping");
  joint->setForceDirection(Mat("[0;0;1]"));
  joint->connect(disk->getFrame("COG"),this->getFrame("I"));
  joint->setForceLaw(new BilateralConstraint());
  joint->setImpactForceLaw(new BilateralImpact());
  this->addLink(joint);

  /* ball */ 
  vector<RigidBody*> balls;
  vector<Point*> points;
  vector<OpenMBV::Sphere*> sphere;
  vector<Contact*> contact;

  double r = 1e-2; // radius of ball
  double mass = 20.; // mass of ball
  SymMat Theta(3,INIT,0.); // inertia of ball
  Theta(0,0) = 2./5.*mass*r*r;
  Theta(1,1) = 2./5.*mass*r*r;
  Theta(2,2) = 2./5.*mass*r*r;
  int nB = 12; // number of balls

  // bodies
  for(int k=0; k<nB; k++) {
    stringstream name;
    name << "Ball_" << k;
    balls.push_back(new RigidBody(name.str()));
    this->addObject(balls[k]);

    stringstream frame; // ball location
    frame << "B_"  << k;
    Vec WrOS0B(3,INIT,0.);
    WrOS0B(0) = (rI+rO)*0.3*cos(k*2.*M_PI/nB);
    WrOS0B(1) = (rI+rO)*0.6*sin(k*2.*M_PI/nB);
    WrOS0B(2) = d(0)+r + 1e-2*(1.+cos(k*2.*M_PI/nB));
    this->addFrame(frame.str(),WrOS0B,SqrMat(3,EYE),this->getFrame("I"));

    balls[k]->setFrameOfReference(this->getFrame(frame.str()));
    balls[k]->setFrameForKinematics(balls[k]->getFrame("C"));
    balls[k]->setMass(mass);
    balls[k]->setInertiaTensor(Theta);
    balls[k]->setTranslation(new LinearTranslation(SqrMat(3,EYE))); // only translational dof because of point masses

#ifdef HAVE_OPENMBVCPPINTERFACE
    sphere.push_back(new OpenMBV::Sphere);
    sphere[k]->setRadius(r);
    sphere[k]->setStaticColor((double)k/(double)nB);
    balls[k]->setOpenMBVRigidBody(sphere[k]);
#endif
  }

  // contacts
  for(int k=0; k<nB; k++) {
    stringstream pointname; // point contour at lower position
    pointname << "Point_" << k;
    points.push_back(new Point(pointname.str()));

    Vec BR(3,INIT,0.); BR(2)=-r;
    balls[k]->addContour(points[k],BR,SqrMat(3,EYE),balls[k]->getFrame("C"));

    stringstream contactname; // fricional contact
    contactname << "Contact_" << k;
    contact.push_back(new Contact(contactname.str()));
    contact[k]->setContactForceLaw(new UnilateralConstraint);
    contact[k]->setContactImpactLaw(new UnilateralNewtonImpact(0.));
    contact[k]->setFrictionForceLaw(new SpatialCoulombFriction(0.4));
    contact[k]->setFrictionImpactLaw(new SpatialCoulombImpact(0.4));
    contact[k]->connect(balls[k]->getContour(pointname.str()),disk->getContour("SurfaceContour"));
    this->addLink(contact[k]);
  }

} 

