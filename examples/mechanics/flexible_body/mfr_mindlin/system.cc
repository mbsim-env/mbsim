#include "system.h"

#include "mbsimFlexibleBody/flexible_body/flexible_body_2s_13_mfr_mindlin.h"
#include "mbsimFlexibleBody/contact_kinematics/point_nurbsdisk2s.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/links/joint.h"
#include "mbsim/links/contact.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/frames/fixed_contour_frame.h"
#include "mbsimFlexibleBody/frames/frame_2s.h"
#include "mbsimFlexibleBody/contours/nurbs_disk_2s.h"
#include "mbsim/contours/point.h"
#include "mbsim/constitutive_laws/constitutive_laws.h"
#include "mbsim/environment.h"
#include "mbsim/functions/kinetics/kinetics.h"
#include "mbsim/functions/kinematics/kinematics.h"
#include "mbsim/observers/contact_observer.h"

#include "openmbvcppinterface/sphere.h"
#include "openmbvcppinterface/cube.h"

using namespace MBSimFlexibleBody;
using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

  Vec grav(3,INIT,0.);
  grav(2) = -10.;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  /* disk */
  // body
  FlexibleBody2s13MFRMindlin *disk = new FlexibleBody2s13MFRMindlin("Disk");
  double E = 5.e7; // Young's modulus  
  double rho = 9.2e2; // density
  double nu = 0.3; // poisson ratio
  double rI = 0.01; // inner radius
  double rO = 0.4; // outer radius
  Vec d(3); d(0) = 0.025; // thickness at r==0 
  SqrMat A(2,NONINIT); A(0,0) = rI*rI; A(0,1) = rI; A(1,0) = rO*rO; A(1,1) = rO;
  Vec b(2,NONINIT); b(0) = 0.; b(1) = 0.15; // differences to thickness d(0) for r==rI and r==rO with b(1) = 0.05 for CUBE
  d(1,2) = slvLU(A,b);

  int nr = 3; // radial number of elements
  int nj = 6; // azimuthal number of elements

  disk->setEModul(E);
  disk->setDensity(rho);
  disk->setPoissonRatio(nu);
  disk->setRadius(rI,rO);
  disk->setThickness(d);
  disk->setFrameOfReference(this->getFrame("I")); // location of disk
  disk->setLockType(FlexibleBody2s13::innerring); // inner ring has no elastic dof
  disk->setReferenceInertia(1.,SymMat(3,EYE)); // inertia of the reference frame
  disk->setNumberElements(nr,nj);

  Vec q0 = Vec(6+nr*nj*3,INIT,0.); // initial position
  disk->setq0(q0);
  Vec u0 = Vec(6+nr*nj*3,INIT,0.); // initial velocity
  u0(5) = 4.*M_PI;
  disk->setu0(u0);
  this->addObject(disk);

  NurbsDisk2s *contour = new NurbsDisk2s("SurfaceContour");
  contour->enableOpenMBV();
  disk->addContour(contour);
  Vec nodes(2);
  nodes(0) = rI;
  nodes(1) = rO;
  contour->setEtaNodes(nodes);

  disk->addFrame(new Frame2s("COG",Vec2()));
//  disk->addFrame(new FixedContourFrame("P","[0.1;0]",contour));
//  disk->getFrame("P")->enableOpenMBV(0.01);

  // bearing
  Joint *joint = new Joint("Clamping");
  joint->setForceDirection(Mat("[1,0,0;0,1,0;0,0,1]"));
  joint->setMomentDirection(Mat("[1,0;0,1;0,0]"));
  joint->connect(this->getFrame("I"),disk->getFrame("COG"));
  joint->setForceLaw(new RegularizedBilateralConstraint(new LinearRegularizedBilateralConstraint(1.e6,0.)));
  joint->setMomentLaw(new RegularizedBilateralConstraint(new LinearRegularizedBilateralConstraint(1.e6,0.)));
  joint->setIntegrateGeneralizedRelativeVelocityOfRotation(true);
//  joint->setForceLaw(new BilateralConstraint);
//  joint->setMomentLaw(new BilateralConstraint);
  this->addLink(joint);

  /* ball */ 
  vector<RigidBody*> balls;
  vector<Point*> points;
  vector<std::shared_ptr<OpenMBV::Sphere> > sphere;
  vector<Contact*> contact;

  double r = 1e-2; // radius of ball
  double mass = 50.; // mass of ball
  SymMat Theta(3,INIT,0.); // inertia of ball
  Theta(0,0) = 2./5.*mass*r*r;
  Theta(1,1) = 2./5.*mass*r*r;
  Theta(2,2) = 2./5.*mass*r*r;
  int nB = 1; // number of balls

  // bodies
  for(int k=0; k<nB; k++) {
    stringstream name;
    name << "Ball_" << k;
    balls.push_back(new RigidBody(name.str()));
    this->addObject(balls[k]);

    stringstream frame; // ball location
    frame << "B_"  << k;
    Vec WrOS0B(3,INIT,0.);
    WrOS0B(0) = (rI+rO)*0.25*cos(k*2.*M_PI/nB);
    WrOS0B(1) = (rI+rO)*0.35*sin(k*2.*M_PI/nB);
    WrOS0B(2) = b(1)*0.5 + r + 1e-2*(1.+cos(k*2.*M_PI/nB));
    this->addFrame(new FixedRelativeFrame(frame.str(),WrOS0B,SqrMat(3,EYE),this->getFrame("I")));

    balls[k]->setFrameOfReference(this->getFrame(frame.str()));
    balls[k]->setFrameForKinematics(balls[k]->getFrame("C"));
    balls[k]->setMass(mass);
    balls[k]->setInertiaTensor(Theta);
    balls[k]->setTranslation(new TranslationAlongAxesXYZ<VecV>); // only translational dof because of point masses

    sphere.push_back(OpenMBV::ObjectFactory::create<OpenMBV::Sphere>());
    sphere[k]->setRadius(r);
    sphere[k]->setDiffuseColor((double)k/(double)nB,(double)k/(double)nB,(double)k/(double)nB);
    balls[k]->setOpenMBVRigidBody(sphere[k]);
  }

  // contacts
  for(int k=0; k<nB; k++) {
    stringstream pointname; // point contour at lower position
    pointname << "Point_" << k;
    points.push_back(new Point(pointname.str()));

    Vec BR(3,INIT,0.); BR(2)=-r;
    balls[k]->addFrame(new FixedRelativeFrame("Point",BR,SqrMat(3,EYE),balls[k]->getFrame("C")));
    points[k]->setFrameOfReference(balls[k]->getFrame("Point"));
    balls[k]->addContour(points[k]);

    stringstream contactname; // fricional contact
    contactname << "Contact_" << k;
    contact.push_back(new Contact(contactname.str()));
    contact[k]->setNormalForceLaw(new UnilateralConstraint);
    contact[k]->setNormalImpactLaw(new UnilateralNewtonImpact(0.));
    contact[k]->setTangentialForceLaw(new SpatialCoulombFriction(0.4));
    contact[k]->setTangentialImpactLaw(new SpatialCoulombImpact(0.4));
    contact[k]->connect(balls[k]->getContour(pointname.str()),disk->getContour("SurfaceContour"), new ContactKinematicsPointNurbsDisk2s());
    this->addLink(contact[k]);

    ContactObserver *observer = new ContactObserver(contactname.str()+"_Observer");
    addObserver(observer);
    observer->setContact(contact[k]);
    observer->enableOpenMBVContactPoints(0.01);
  }

  setPlotFeatureRecursive(generalizedPosition, true);
  setPlotFeatureRecursive(generalizedVelocity, true);
  setPlotFeatureRecursive(generalizedRelativePosition, true);
  setPlotFeatureRecursive(generalizedRelativeVelocity, true);
  setPlotFeatureRecursive(generalizedForce, true);
} 

