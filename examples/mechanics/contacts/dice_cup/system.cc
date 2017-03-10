#include "system.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/frustum.h"
#include "mbsim/links/contact.h"
#include "mbsim/constitutive_laws/constitutive_laws.h"
#include "mbsim/environment.h"
#include "mbsim/functions/kinematics/kinematics.h"
#include "mbsim/observers/contact_observer.h"

#include <openmbvcppinterface/ivbody.h>
#include "openmbvcppinterface/cube.h"

using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {
  /* Dice-Cup - SetUp */
  // Preliminaries
  double normalRestitutionCoefficient = 1;

  // Dice-Reference
  Vec WrOK0_dice(3);
  WrOK0_dice(0) = -0.005;
  WrOK0_dice(1) = 0.05;
  WrOK0_dice(2) = -0.01;
  this->addFrame(new FixedRelativeFrame("D",WrOK0_dice,SqrMat(3,EYE),this->getFrame("I")));

  RigidBody* dice = new RigidBody("Dice");
  dice->setFrameOfReference(this->getFrame("D"));
  dice->setFrameForKinematics(dice->getFrame("C"));
  double mass = 10.;
  dice->setMass(mass);
  double length = 0.01;
  SymMat Theta(3);
  Theta(0,0)=1./12.*mass*(length*length+length*length);
  Theta(1,1)=1./12.*mass*(length*length+length*length);
  Theta(2,2)=1./12.*mass*(length*length+length*length);
  dice->setInertiaTensor(Theta);
  dice->setTranslation(new TranslationAlongAxesXYZ<VecV>);
  dice->setRotation(new RotationAboutAxesXYZ<VecV>);

  this->addObject(dice);

  // Dice-Contour	
  vector<Point*> contour;

  for(int i=0;i<8;i++) {
    stringstream nameContour, nameFrame; // (BS -> STRINGSTREAM 21.5.3)
    nameContour <<"point_"<< i+1;
    nameFrame <<"P_"<< i+1;
    contour.push_back(new Point(nameContour.str()));
    Vec p(3,INIT,0.);

    if(i==0) {
      p(0) = -length/2.;
      p(1) = -length/2.;
      p(2) = -length/2.;
      dice->addFrame(new FixedRelativeFrame(nameFrame.str(),p,SqrMat(3,EYE)));
    }
    else if(i==1) {
      p(0) = -length/2.;
      p(1) = -length/2.;
      p(2) = length/2.;
      dice->addFrame(new FixedRelativeFrame(nameFrame.str(),p,SqrMat(3,EYE)));
    }
    else if(i==2) {
      p(0) = -length/2.;
      p(1) = length/2.;
      p(2) = -length/2.;
      dice->addFrame(new FixedRelativeFrame(nameFrame.str(),p,SqrMat(3,EYE)));
    }
    else if(i==3) {
      p(0) = -length/2.;
      p(1) = length/2.;
      p(2) = length/2.;
      dice->addFrame(new FixedRelativeFrame(nameFrame.str(),p,SqrMat(3,EYE)));
    }
    else if(i==4) {
      p(0) = length/2.;
      p(1) = -length/2.;
      p(2) = -length/2.;
      dice->addFrame(new FixedRelativeFrame(nameFrame.str(),p,SqrMat(3,EYE)));
    }
    else if(i==5) {
      p(0) = length/2.;
      p(1) = -length/2.;
      p(2) = length/2.;
      dice->addFrame(new FixedRelativeFrame(nameFrame.str(),p,SqrMat(3,EYE)));
    }
    else if(i==6) {
      p(0) = length/2.;
      p(1) = length/2.;
      p(2) = -length/2.;
      dice->addFrame(new FixedRelativeFrame(nameFrame.str(),p,SqrMat(3,EYE)));
    }
    else {
      p(0) = length/2.;
      p(1) = length/2.;
      p(2) = length/2.;
      dice->addFrame(new FixedRelativeFrame(nameFrame.str(),p,SqrMat(3,EYE)));
    }
    contour[i]->setFrameOfReference(dice->getFrame(nameFrame.str()));
    dice->addContour(contour[i]);
  }

  // Dice-Visualisation
  std::shared_ptr<OpenMBV::Cube> diceAMV = OpenMBV::ObjectFactory::create<OpenMBV::Cube>();
  diceAMV->setLength(length);
  diceAMV->setDiffuseColor(1/3.0, 1, 1);
  dice->setOpenMBVRigidBody(diceAMV);
  // ************************************************************************

  // Frustum-Reference
  SqrMat AWKO_frustum(3);
  double phi_frustum = 0.; // rotation around inertial x-axis
  AWKO_frustum(1,1) = cos(phi_frustum); AWKO_frustum(1,2) = -sin(phi_frustum);
  AWKO_frustum(2,1) = sin(phi_frustum); AWKO_frustum(2,2) = cos(phi_frustum); AWKO_frustum(0,0) = 1.;
  this->addFrame(new FixedRelativeFrame("F",Vec(3,INIT,0.),AWKO_frustum,this->getFrame("I")));
  RigidBody* frustumRef = new RigidBody("frustumRef");
  frustumRef->setFrameOfReference(this->getFrame("F"));
  frustumRef->setFrameForKinematics(frustumRef->getFrame("C"));
  frustumRef->setMass(1.);
  frustumRef->setInertiaTensor(SymMat(3,EYE));
  this->addObject(frustumRef);

  // Frustum-Contour
  Frustum* frustum = new Frustum("Frustum"); // frustum contour	
  double height = 0.1;
  frustum->setHeight(height);
  Vec radii(2); // radii of the frustum
  radii(0) = 0.005; radii(1) = 0.05;
  frustum->setRadii(radii);
  frustum->setOutCont(false);
  frustumRef->addContour(frustum);

  // Frustum-Visualisation
  int k=0; // numbering vertices
  int grid = 40; // grid points (INCREASE FOR BETTER LOOK)
  vector<Vertex> intDisc(grid+1);
  vector<Vertex> outDisc(grid+1);	

  intDisc[0].num = k; intDisc[0].x = 0.; intDisc[0].y = 0.; intDisc[0].z = 0.;
  for(int i=1;i<grid+1;i++) {
    k++;
    intDisc[i].num = k; intDisc[i].x = radii(0)*cos(i*2.*M_PI/grid); intDisc[i].y = 0.; intDisc[i].z = -radii(0)*sin(i*2.*M_PI/grid);
    outDisc[i-1].num = k+grid; outDisc[i-1].x = radii(1)*cos(i*2.*M_PI/grid); outDisc[i-1].y = height; outDisc[i-1].z = -radii(1)*sin(i*2.*M_PI/grid);	
  }
  outDisc[grid].num = grid+(k+1); outDisc[grid].x = 0.; outDisc[grid].y = height; outDisc[grid].z = 0.;

  ofstream frustumVRML;
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
  frustumVRML << "    vertexOrdering CLOCKWISE" << endl; //  CLOCKWISE means look inside
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

  std::shared_ptr<OpenMBV::IvBody> frustumMBV = OpenMBV::ObjectFactory::create<OpenMBV::IvBody>();
  frustumMBV->setIvFileName("frustum.iv");
  frustumMBV->setDiffuseColor(0.6666,1,1);
  frustumMBV->setInitialTranslation(0.,0.,0.);
  frustumMBV->setInitialRotation(0.,0.,0.);
  frustumRef->setOpenMBVRigidBody(frustumMBV); 
  // ************************************************************************

  // Contact
  vector<Contact*> contact;
  
  for(int i=0;i<8;i++) {
    stringstream nameContact; // (BS -> STRINGSTREAM 21.5.3)
    nameContact <<"Contact_"<< i+1;
    stringstream nameContour;
    nameContour <<"point_"<< i+1;
    contact.push_back(new Contact(nameContact.str()));
    contact[i]->setNormalForceLaw(new UnilateralConstraint());
    contact[i]->setNormalImpactLaw(new UnilateralNewtonImpact(normalRestitutionCoefficient));
    contact[i]->connect(frustumRef->getContour("Frustum"),dice->getContour(nameContour.str()));
    this->addLink(contact[i]);
    ContactObserver *observer = new ContactObserver(nameContact.str()+"_Observer");
    addObserver(observer);
    observer->setContact(contact[i]);
    observer->enableOpenMBVContactPoints(0.05);
  } 	

  MBSimEnvironment::getInstance()->setAccelerationOfGravity(Vec("[0;-10;0]"));

  setPlotFeatureRecursive("generalizedPosition",enabled);
  setPlotFeatureRecursive("generalizedVelocity",enabled);
  setPlotFeatureRecursive("generalizedRelativePosition",enabled);
  setPlotFeatureRecursive("generalizedRelativeVelocity",enabled);
  setPlotFeatureRecursive("generalizedForce",enabled);
}
