#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/mbsim_event.h"
#include "mbsim/contact.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/joint.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/ivbody.h>
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

  /* preliminaries */
  setProjectDirectory("plot");

  Vec WrOK(3);

  Vec grav(3);
  grav(1)=-9.81;
  this->setAccelerationOfGravity(grav);

  /* axis */
  double l_axis = 0.1;

  /* pole */
  double l_pole = 0.5;
  double delta = -20. / 180. * M_PI; // in [-M_PI,M_PI]

  /* reference of muller */
  double l_muller = 0.025;
  double r_muller = 0.1;
  
  /* contour of muller */
  Vec radii(2);

  /* stony ground */
  RigidBody* groundBase = new RigidBody("GroundBase");
  // Frustum* ground = new Frustum("Ground");

  double h, dh; // height and offset of frustum
  if(delta>=0.) {
    if(delta<M_PI/2.) {
      h = tan(delta)*((l_pole+2.*l_muller)*cos(delta)+r_muller*sin(delta));
      radii(0) = 0.;
      radii(1) = h/tan(delta); // above radius of frustum
      dh = r_muller / cos(delta) - l_axis;
      // ground->setOutCont(false);
    }
    else if(delta>M_PI/2.) {
      h = tan(delta)*((l_pole+2.*l_muller)*cos(delta)-r_muller*sin(delta));
      radii(0) = 0.;
      radii(1) = h/tan(-delta+M_PI);
      dh = -r_muller / cos(delta) - l_axis;
      // ground->setOutCont(false);
    }
    else throw MBSimError("Bad Configuration!");
  }
  else {
    if(abs(delta)<M_PI/2.) {
      h = tan(abs(delta))*((l_pole+2.*l_muller)*cos(abs(delta))+r_muller*sin(abs(delta)));
      radii(0) = h/tan(abs(delta));
      radii(1) = 0.;
      dh = r_muller / cos(delta) - l_axis + h;
      // ground->setOutCont(true);
    }
    else if(abs(delta)>M_PI/2.) {
      h = tan(abs(delta))*((l_pole+2.*l_muller)*cos(abs(delta))-r_muller*sin(abs(delta)));

      radii(0) = abs(h/tan(delta));
      radii(1) = 0.;
      dh = -r_muller / cos(delta) - l_axis + h;
      // ground->setOutCont(true);
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

  // ground->setRadii(radii);
  // ground->setHeight(h);
  // ground->setAxis("[0;1;0]");
  // groundBase->addContour(ground,"[0;0;0]");

  /* OpenMBV */
#ifdef HAVE_OPENMBVCPPINTERFACE
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
  frustumVRML << "    vertexOrdering CLOCKWISE" << endl; //  CLOCKWISE means look inside
  frustumVRML << "    creaseAngle 3.1415" << endl;
  frustumVRML << "  }" << endl << endl; // hints END

  frustumVRML << "  Material { diffuseColor [ 1 0 0 ] }" << endl << endl; // material r g b

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
  frustumMBV->setInitialTranslation(0.,0.,0.);
  frustumMBV->setInitialRotation(0.,0.,0.);
  // frustumMBV->setCalculationOfNormals(3); // automatic calculation of normals for colouring
  // frustumMBV->setVertexEPS(1e-5); // deleting dublicate points
  // frustumMBV->setNormalEPS(1e-5); // deleting dublicate points
  groundBase->setOpenMBVRigidBody(frustumMBV);
#endif
}

