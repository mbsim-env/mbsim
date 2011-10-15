#include "system.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_33_cosserat.h"
#include "mbsim/environment.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#include <openmbvcppinterface/polygonpoint.h>
#endif

using namespace MBSimFlexibleBody;
using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

  Vec grav(3,INIT,0.);
  //grav(1) = -9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  double l0 = 1.; // length
  double b0 = 0.05; // width
  double E = 1.e5; // E-Modul
  double mu = 0.3; // Poisson ratio
  double G = E/(2*(1+mu)); // shear modulus
  double A = b0*b0; // cross-section area
  double I1 = 1./12.*b0*b0*b0*b0; // moment inertia
  double I2 = 1./12.*b0*b0*b0*b0; 
  double I0 = I1 + I2;
  double rho = 9.2e2; // density
  int elements = 20; // number of finite elements

  FlexibleBody1s33Cosserat* rod = new FlexibleBody1s33Cosserat("Rod",false);
  rod->setLength(l0);
  rod->setEGModuls(E,G);
  rod->setCrossSectionalArea(A);
  rod->setMomentsInertia(I1,I2,I0);
  rod->setDensity(rho);
  //rod->setMassProportionalDamping(20.);
  rod->setMaterialDamping(0.1,0.1,0.1);
  rod->setFrameOfReference(this->getFrame("I"));
  rod->setNumberElements(elements);
  rod->setCuboid(b0,b0);

#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::SpineExtrusion *cuboid = new OpenMBV::SpineExtrusion;
  cuboid->setNumberOfSpinePoints(elements*4+1); 
  cuboid->setStaticColor(0.5);
  cuboid->setScaleFactor(1.); 
  vector<OpenMBV::PolygonPoint*> *rectangle = new vector<OpenMBV::PolygonPoint*>; 
  OpenMBV::PolygonPoint *corner1 = new OpenMBV::PolygonPoint(b0*0.5,b0*0.5,1);
  rectangle->push_back(corner1);
  OpenMBV::PolygonPoint *corner2 = new OpenMBV::PolygonPoint(b0*0.5,-b0*0.5,1);
  rectangle->push_back(corner2);
  OpenMBV::PolygonPoint *corner3 = new OpenMBV::PolygonPoint(-b0*0.5,-b0*0.5,1);
  rectangle->push_back(corner3);
  OpenMBV::PolygonPoint *corner4 = new OpenMBV::PolygonPoint(-b0*0.5,b0*0.5,1);
  rectangle->push_back(corner4);

  cuboid->setContour(rectangle);
  rod->setOpenMBVSpineExtrusion(cuboid);
#endif

  // circle shape
  Vec q0 = Vec(6*elements,INIT,0.);
  double R = l0*(1.2)/(2.*M_PI); // stretched circle
  double dphi = (2*M_PI)/elements;
  double phi0 = M_PI/2. + dphi;
  for(int i=0; i<elements; i++) { 
    double phi = phi0 - i*dphi;	
    q0(6*i) = R*cos(phi);
    q0(6*i+1) = R*sin(phi);
    q0(6*i+5) = phi - dphi/2.-M_PI/2.;
  }
  rod->setq0(q0);
  this->addObject(rod);
}
