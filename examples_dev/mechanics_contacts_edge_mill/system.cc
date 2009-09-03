#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/tree.h"
#include "mbsim/mbsim_event.h"
#include "mbsim/contact.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/contour.h"
#include "mbsim/contours/circle.h"
#include "mbsim/contours/frustum.h"
#include "mbsim/contact_kinematics/circle_frustum.h"
#include "mbsim/environment.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/ivbody.h>
#include <openmbvcppinterface/frustum.h>
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

  /* preliminaries */
  Vec WrOK(3);
  Vec KrKS(3);
  Vec KrKP(3);
  SymMat Theta(3);
  SqrMat AWK(3);

  Vec grav(3);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  /* mill as tree */
  Tree *mill = new Tree("Mill");
  this->addGroup(mill);

  /* axis */
  RigidBody *axis = new RigidBody("Axis");
  Node *node_axis = mill->addObject(0,axis);

  double l_axis = 0.1;
  double r_axis = 0.025;
  double m_axis = 1.;
  double Jyy_axis = 0.5 * m_axis * r_axis * r_axis;
  Vec JR_axis(3);

  Theta(1,1) = Jyy_axis;
  JR_axis(1) = 1.;
  KrKS(1) = 0.5*l_axis;

  this->addFrame("R",WrOK,SqrMat(3,EYE));
  axis->addFrame("R",-KrKS,SqrMat(3,EYE));
  axis->setFrameOfReference(getFrame("R"));
  axis->setFrameForKinematics(axis->getFrame("R"));
  axis->setRotation(new RotationAboutFixedAxis(JR_axis));
  axis->setMass(m_axis);
  axis->setInertiaTensor(Theta);
  axis->setInitialGeneralizedVelocity(Vec(1,INIT,2.));

  /* pole */
  RigidBody *pole = new RigidBody("Pole");
  Node *node_pole = mill->addObject(node_axis,pole);

  double l_pole = 0.5;
  double r_pole = 0.025;
  double m_pole = 1.;

  // double delta = 110. / 180. * M_PI; // in [-M_PI,M_PI]
  // double delta = -70. / 180. * M_PI;
   double delta = 20. / 180. * M_PI;
  // double delta = -160. / 180. * M_PI;

  double Jzz_pole = 1./12. * m_pole *(3*r_pole*r_pole + l_pole*l_pole);
  Vec JR_pole(3);

  Theta(2,2) = Jzz_pole;
  JR_pole(2) = 1.; 
  AWK(0,0) = cos(delta), AWK(0,1) = -sin(delta);
  AWK(1,0) = sin(delta), AWK(1,1) = cos(delta);
  AWK(2,2) = 1.;
  WrOK(1) = l_axis;
  KrKS(0) = 0.5*l_pole, KrKS(1) = 0.;

  axis->addFrame("P",WrOK,AWK,axis->getFrame("R"));
  pole->addFrame("R",-KrKS,SqrMat(3,EYE));
  pole->setFrameOfReference(axis->getFrame("P"));
  pole->setFrameForKinematics(pole->getFrame("R"));
  pole->setRotation(new RotationAboutFixedAxis(JR_pole));
  pole->setMass(m_pole);
  pole->setInertiaTensor(Theta);

  /* reference of muller */
  RigidBody* muller = new RigidBody("Muller");
  mill->addObject(node_pole,muller);

  double l_muller = 0.025;
  double r_muller = 0.1;
  double m_muller = 100.;
  double Jxx_muller = 0.5 * m_muller * r_muller * r_muller;
  Vec JR_muller(3);
  JR_muller(0) = 1.;

  Theta(0,0) = Jxx_muller;
  WrOK(0) = l_pole, WrOK(1) = 0.;
  KrKS(0) = 0.5*l_muller;

  pole->addFrame("P",WrOK,SqrMat(3,EYE),pole->getFrame("R"));
  muller->addFrame("R",-KrKS,SqrMat(3,EYE));
  muller->setFrameOfReference(pole->getFrame("P"));
  muller->setFrameForKinematics(muller->getFrame("R"));
  muller->setRotation(new RotationAboutFixedAxis(JR_muller));	
  muller->setMass(m_muller);
  muller->setInertiaTensor(Theta);

  /* contour of muller */
  Vec radii(2);

  Circle* disk = new Circle("Disk");
  AWK(0,0) = cos(M_PI*0.5); AWK(0,1) = 0.; AWK(0,2) = sin(M_PI*0.5);
  AWK(1,0) = 0., AWK(1,1) = 1.;
  AWK(2,0) = -sin(M_PI*0.5), AWK(2,2) = cos(M_PI*0.5);
  disk->setOutCont(true);
  disk->setRadius(r_muller);
  muller->addContour(disk,Vec(3,INIT,0.),AWK);

  /* stony ground */
  RigidBody* groundBase = new RigidBody("GroundBase");
  Frustum* ground = new Frustum("Ground");

  double h, dh; // height and offset of frustum
  if(delta>=0.) {
    if(delta<M_PI/2.) {
      h = tan(delta)*((l_pole+2.*l_muller)*cos(delta)+r_muller*sin(delta));
      radii(0) = 0.;
      radii(1) = h/tan(delta); // above radius of frustum
      dh = r_muller / cos(delta) - l_axis;
      ground->setOutCont(false);
    }
    else if(delta>M_PI/2.) {
      h = tan(delta)*((l_pole+2.*l_muller)*cos(delta)-r_muller*sin(delta));
      radii(0) = 0.;
      radii(1) = h/tan(-delta+M_PI);
      dh = -r_muller / cos(delta) - l_axis;
      ground->setOutCont(false);
    }
    else throw MBSimError("Bad Configuration!");
  }
  else {
    if(abs(delta)<M_PI/2.) {
      h = tan(abs(delta))*((l_pole+2.*l_muller)*cos(abs(delta))+r_muller*sin(abs(delta)));
      radii(0) = h/tan(abs(delta));
      radii(1) = 0.;
      dh = r_muller / cos(delta) - l_axis + h;
      ground->setOutCont(true);
    }
    else if(abs(delta)>M_PI/2.) {
      h = tan(abs(delta))*((l_pole+2.*l_muller)*cos(abs(delta))-r_muller*sin(abs(delta)));

      radii(0) = abs(h/tan(delta));
      radii(1) = 0.;
      dh = -r_muller / cos(delta) - l_axis + h;
      ground->setOutCont(true);
    }
    else throw MBSimError("Bad Configuration!");
  }

  WrOK(0) = 0., WrOK(1) = -dh;
  this->addFrame("K",WrOK,SqrMat(3,EYE),this->getFrame("I"));
  groundBase->setFrameOfReference(this->getFrame("K"));
  groundBase->setFrameForKinematics(groundBase->getFrame("C"));
  groundBase->setMass(1.);
  groundBase->setInertiaTensor(SymMat(3,EYE));
  this->addObject(groundBase);

  ground->setRadii(radii);
  ground->setHeight(h);
  groundBase->addContour(ground,Vec(3,INIT,0.),SqrMat(3,EYE));

  /* contact */
  Contact *contact = new Contact("Contact");
  contact->connect(groundBase->getContour("Ground"),muller->getContour("Disk"));
  contact->setContactForceLaw(new UnilateralConstraint);
  contact->setContactImpactLaw(new UnilateralNewtonImpact);
  contact->setFrictionForceLaw(new SpatialCoulombFriction(0.4));
  contact->setFrictionImpactLaw(new SpatialCoulombImpact(0.4));
  contact->enableOpenMBVContactPoints();
  this->addLink(contact);

  /* OpenMBV */
#ifdef HAVE_OPENMBVCPPINTERFACE
  /* axis */
  OpenMBV::Frustum *obj1 = new OpenMBV::Frustum;
  obj1->setBaseRadius(r_axis);
  obj1->setTopRadius(r_axis);
  obj1->setHeight(l_axis);
  obj1->setInitialRotation(-M_PI/2,0.,0.);
  obj1->setInitialTranslation(0.,l_axis*0.5,0.);
  obj1->setStaticColor(0.75);
  axis->setOpenMBVRigidBody(obj1);

  /* pole */
  OpenMBV::Frustum *obj2 = new OpenMBV::Frustum;
  obj2->setBaseRadius(r_pole);
  obj2->setTopRadius(r_pole);
  obj2->setHeight(l_pole);
  obj2->setInitialRotation(0.,M_PI/2,0.);
  obj2->setInitialTranslation(l_pole*0.5,0.,0.);
  pole->setOpenMBVRigidBody(obj2);

  /* muller */
  OpenMBV::Frustum *obj3 = new OpenMBV::Frustum;
  obj3->setBaseRadius(r_muller);
  obj3->setTopRadius(r_muller);
  obj3->setHeight(l_muller);
  obj3->setInitialRotation(0.,M_PI/2,0.);
  obj3->setInitialTranslation(l_muller*0.5,0.,0.);
  muller->setOpenMBVRigidBody(obj3);

  /* Ground */
  int k=0; // numbering vertices
  int grid = 30; // grid points (INCREASE FOR BETTER LOOK)
  vector<Vertex> intDisc(grid+1);
  vector<Vertex> outDisc(grid+1);	

  intDisc[0].num = k; intDisc[0].x = 0.; intDisc[0].y = 0.; intDisc[0].z = 0.;
  for(int i=1;i<grid+1;i++) {
    k++;
    intDisc[i].num = k; intDisc[i].x = radii(0)*cos(i*2.*M_PI/grid); intDisc[i].y = 0.; intDisc[i].z = -radii(0)*sin(i*2.*M_PI/grid);
    outDisc[i-1].num = k+grid; outDisc[i-1].x = radii(1)*cos(i*2.*M_PI/grid); outDisc[i-1].y = h; outDisc[i-1].z = -radii(1)*sin(i*2.*M_PI/grid);	
  }
  outDisc[grid].num = grid+(k+1); outDisc[grid].x = 0.; outDisc[grid].y = h; outDisc[grid].z = 0.;

  ofstream frustumVRML; // VRML file
  frustumVRML.open("frustum.iv");

  frustumVRML << "#Inventor V2.1 ascii" << endl << endl;

  frustumVRML << "Separator" << endl << "{" << endl;

  frustumVRML << "  Coordinate3" << endl << "  {" << endl; // vertices BEGIN
  frustumVRML << "    point [" << endl;

  for(int i=0;i<grid+1;i++) { // vertices inner disk
    frustumVRML << "      " << intDisc[i].x << " " << intDisc[i].y << " " << intDisc[i].z << "," << endl;
  }
  for(int i=0;i<grid+1;i++) { // vertices outer disk
    if(i==grid) {
      frustumVRML << "      " << outDisc[i].x << " " << outDisc[i].y << " " << outDisc[i].z << endl;
    }
    else {
      frustumVRML << "      " << outDisc[i].x << " " << outDisc[i].y << " " << outDisc[i].z << "," << endl;
    }
  }

  frustumVRML << "    ]" << endl;
  frustumVRML << "  }" << endl << endl; // vertices END

  frustumVRML << "  ShapeHints" << endl << "  {" << endl; // hints BEGIN
  frustumVRML << "    vertexOrdering COUNTERCLOCKWISE" << endl; //  CLOCKWISE means look inside
  frustumVRML << "    shapeType UNKNOWN_SHAPE_TYPE" << endl; 
  frustumVRML << "    creaseAngle 0.393" << endl;
  frustumVRML << "  }" << endl << endl; // hints END

  frustumVRML << "  IndexedFaceSet" << endl << "  {" << endl; // faces BEGIN
  frustumVRML << "    coordIndex [" << endl;

  for(int i=1;i<grid+1;i++) { // faces inner disk
    if(i==grid) {
      frustumVRML << "      " << intDisc[i].num << ", " << intDisc[0].num << ", " << intDisc[1].num << ", -1" << endl;
    }
    else {
      frustumVRML << "      " << intDisc[i].num << ", " << intDisc[0].num << ", " << intDisc[i+1].num << ", -1," << endl;
    }
  }
  frustumVRML << "    ]" << endl;
  frustumVRML << "  }" << endl << endl; // faces END

  frustumVRML << "  IndexedFaceSet" << endl << "  {" << endl; // faces BEGIN
  frustumVRML << "    coordIndex [" << endl;
  for(int i=0;i<grid;i++) { // faces inner-outer disk
    if(i==grid-1) {
      frustumVRML << "      " << intDisc[1].num << ", " << outDisc[0].num << ", " << outDisc[i].num << ", " << intDisc[i+1].num << ", -1" << endl;
    }
    else {
      frustumVRML << "      " << intDisc[i+2].num << ", " << outDisc[i+1].num << ", " << outDisc[i].num << ", " << intDisc[i+1].num << ", -1," << endl;
    }
  }

  frustumVRML << "    ]" << endl;
  frustumVRML << "  }" << endl << endl; // faces END

  frustumVRML << "}" << endl << endl;

  frustumVRML.close();

  OpenMBV::IvBody* frustumMBV = new OpenMBV::IvBody;
  frustumMBV->setIvFileName("frustum.iv");
  frustumMBV->setStaticColor(1.);
  frustumMBV->setInitialTranslation(0.,0.,0.);
  frustumMBV->setInitialRotation(0.,0.,0.);
  groundBase->setOpenMBVRigidBody(frustumMBV);
#endif
}

void System::init() {
  DynamicSystemSolver::init();

  for(unsigned i=0; i<link.size(); i++) {
    dynamic_cast<ContactKinematicsCircleFrustum*>(dynamic_cast<Contact*>(link[i])->getContactKinematics())->setDebug(false);
    dynamic_cast<ContactKinematicsCircleFrustum*>(dynamic_cast<Contact*>(link[i])->getContactKinematics())->setWarnLevel(0);
    dynamic_cast<ContactKinematicsCircleFrustum*>(dynamic_cast<Contact*>(link[i])->getContactKinematics())->setLocalSearch(true);
  }
}

