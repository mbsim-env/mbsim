#include "system.h"

#include "mbsimHydraulics/hydline.h"
#include "mbsimHydraulics/hydnode.h"
#include "mbsimHydraulics/hydnode_mec.h"
#include "mbsim/userfunction.h"

#include "mbsim/rigid_body.h"
#include "mbsim/tree.h"
#include "mbsim/spring_damper.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/utils/function.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/nonlinear_algebra.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/polygonpoint.h"
#include "openmbvcppinterface/frustum.h"
#include "openmbvcppinterface/compoundrigidbody.h"
#include "openmbvcppinterface/extrusion.h"
#include "openmbvcppinterface/coilspring.h"
#endif

using namespace std;
using namespace MBSim;
using namespace fmatvec;

string getBodyName(int i) {
  string name;
  switch (i) {
    case 0:
      name="L";
      break;
    case 1:
      name="LM";
      break;
    case 2:
      name="M";
      break;
    case 3:
      name="RM";
      break;
    case 4:
      name="R";
      break;
  }
  return name;
}

vector<OpenMBV::PolygonPoint*> * createPiece(double rI, double rA, double phi0, double dphi) {
  int nI = int((dphi*rI)/(5e-2*rI));
  int nA = int((dphi*rA)/(5e-2*rI));

  vector<OpenMBV::PolygonPoint*> * vpp = new vector<OpenMBV::PolygonPoint*>();
  for (int i=0; i<=nI; i++) {
    double phi = phi0 + double(i)/double(nI) * dphi;
    double x=rI*cos(phi);
    double y=rI*sin(phi);
    int b=((i==0)||(i==nI))?1:0;
    vpp->push_back(new OpenMBV::PolygonPoint(x, y, b));
  }
  for (int i=nA; i>=0; i--) {
    double phi = phi0 + double(i)/double(nA) * dphi;
    double x=rA*cos(phi);
    double y=rA*sin(phi);
    int b=((i==0)||(i==nA))?1:0;
    vpp->push_back(new OpenMBV::PolygonPoint(x, y, b));
  }
  vpp->push_back((*vpp)[0]);

  return vpp;
}

class problem : public Function1<double, double> {
  public:
    problem(double MSoll_, double rM_, double cF_, double phi0_) : MSoll(MSoll_), rM(rM_), cF(cF_), phi0(phi0_) {
    }
    double operator()(const double& phiD) {
      double l0 = rM * sqrt(2.-2.*cos(phi0));
      double l1 = rM * sqrt(2.-2.*cos(phi0+phiD));
      Vec dir(3, INIT, 0);
      dir(0)=sin(phi0+phiD);
      dir(1)=-cos(phi0+phiD);
      Vec jac(3, INIT, 0);
      jac(0)=-sin((phi0+phiD)/2.);
      jac(1)=cos((phi0+phiD)/2.);
      double MFz = - rM * cF * (l1-l0) * trans(jac) * dir;
      return MFz-MSoll;
    }
  private:
    double MSoll, rM, cF, phi0;
};

System::System(const string &name, bool unilateral) : Group(name) {
  Tree * tree = new Tree("Baum");
  addDynamicSystem(tree);

  cout.precision(16);
  double dI=0.01;
  double dA=0.05;
  double h=0.02;
  double pRB=2e5;
  double cF=1e4;
  
  double phiSolid=degtorad(20.);
  cout << "phiSolid=" << radtodeg(phiSolid) << endl;
  double phiFree=(2.*M_PI-5.*phiSolid)/5.;
  cout << "phiFree=" << radtodeg(phiFree) << endl;
  double dphi=2.*M_PI/5.;
  
  double factor=(dA*dA-dI*dI)/8.*h;
  cout << "factor=" << factor << endl;
  
  double V0=phiFree*factor;
  cout << "V0=" << V0 << endl;

  double rM=dI/2.+(dA-dI)/4.;
  cout << "rM=" << rM << endl;
  double l0=rM*sqrt(2-2.*cos(phiFree));
  cout << "l0=" << l0 << endl;
  double area=(dA-dI)/2.*h;
  cout << "area=" << area << endl;
  double torque=pRB*area*rM;
  cout << "torque due to hydraulic pressure=" << torque << endl;

  problem * p = new problem(torque, rM, cF, phiFree);
  RegulaFalsi solver(p);
  double  phiRes = solver.solve(0, M_PI/2.);
  cout << "resultierende generalisierte Koordinate= " << phiRes << " [rad] = " << radtodeg(phiRes) << " [deg.]" << endl;
  double lRes=rM*sqrt(2.-2.*cos(phiFree+phiRes));
  cout << "resultierende Federlaenge= " << lRes << endl;
  double VRes=(phiFree+phiRes)*factor;
  cout << "VRes=" << VRes*1e9 << endl;
  cout.precision(5);


  RigidBody * traeger = new RigidBody("Traeger");
  Node * node = tree->addObject(0, traeger);
  double phi=0;
  for (int i=0; i<5; i++) {
    traeger->addFrame(getBodyName(i), Vec(3), BasicRotAIKz(phi));
    traeger->getFrame(getBodyName(i))->enableOpenMBV(h/2.);
    phi+=dphi;
  }
  traeger->setFrameOfReference(getFrame("I"));
  traeger->setFrameForKinematics(traeger->getFrame("C"));
  traeger->setMass(2.);
  traeger->setInertiaTensor(0.001*SymMat(3, EYE));
  traeger->setTranslation(new LinearTranslation(SqrMat(3, EYE)));
//  traeger->setRotation(new CardanAngles());
//  traeger->setu0(Vec("[0.; 0; 0; -1; 2; 3]"));
//  traeger->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
//  traeger->setu0(Vec("[4.; -1.; 2]"));
//  traeger->setu0(Vec("12"));
#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::CompoundRigidBody * traegerVisu = new OpenMBV::CompoundRigidBody();
  OpenMBV::Frustum * traegerVisuBoden = new OpenMBV::Frustum();
  traegerVisuBoden->setBaseRadius(dA/2.);
  traegerVisuBoden->setTopRadius(dA/2.);
  traegerVisuBoden->setHeight(h/4.);
  traegerVisuBoden->setInitialRotation(0, 0, 0);
  traegerVisuBoden->setInitialTranslation(0, 0, 0);
  traegerVisuBoden->setStaticColor(0);
  traegerVisu->addRigidBody(traegerVisuBoden);
  OpenMBV::Frustum * traegerVisuMitte = new OpenMBV::Frustum();
  traegerVisuMitte->setBaseRadius(dI/2.);
  traegerVisuMitte->setTopRadius(dI/2.);
  traegerVisuMitte->setHeight(h);
  traegerVisuMitte->setInitialRotation(0, 0, 0);
  traegerVisuMitte->setInitialTranslation(0, 0, h);
  traegerVisuMitte->setStaticColor(0);
  traegerVisu->addRigidBody(traegerVisuMitte);
  traeger->setOpenMBVRigidBody(traegerVisu);
#endif

  Vec r(3, INIT, 0);
  r(0)=rM;
  r(2)=h/2.;
  for (int i=0; i<5; i++) {
    RigidBody * scheibe = new RigidBody("Scheibe_"+getBodyName(i));
    tree->addObject(node, scheibe);
    scheibe->setMass(2.);
    scheibe->addFrame("L", BasicRotAIKz(phiSolid/2.)*r, BasicRotAIKz(phiSolid/2.));
    scheibe->getFrame("L")->enableOpenMBV(h/2.);
    scheibe->addFrame("R", BasicRotAIKz(-phiSolid/2.)*r, BasicRotAIKz(-phiSolid/2.));
    scheibe->getFrame("R")->enableOpenMBV(h/2.);
    scheibe->setFrameOfReference(traeger->getFrame(getBodyName(i)));
    scheibe->setFrameForKinematics(scheibe->getFrame("C"));
    scheibe->setInertiaTensor(0.001*SymMat(3, EYE));
    scheibe->setPlotFeature(state, enabled);
    scheibe->setPlotFeature(stateDerivative, enabled);
    scheibe->setPlotFeature(rightHandSide, enabled);
    if (i>0)
      scheibe->setRotation(new RotationAboutFixedAxis(Vec("[0; 0; 1]")));
#ifdef HAVE_OPENMBVCPPINTERFACE
    OpenMBV::Extrusion * scheibeVisu = new OpenMBV::Extrusion();
    scheibeVisu->setHeight(h);
    scheibeVisu->addContour(createPiece(dI/2., dA/2., 0, phiSolid));
    scheibeVisu->setInitialRotation(0, 0, -phiSolid/2.);
    scheibeVisu->setStaticColor((i)/4.);
    scheibe->setOpenMBVRigidBody(scheibeVisu);
#endif

    if (i>0) {
      SpringDamper * sp = new SpringDamper("Spring_"+getBodyName(i-1)+"_"+getBodyName(i));
      addLink(sp);
      sp->setForceFunction(new LinearSpringDamperForce(cF,0.05*cF,l0));
      sp->connect(
          dynamic_cast<RigidBody*>(getDynamicSystem("Baum")->getObject("Scheibe_"+getBodyName(i-1)))->getFrame("L"), 
          dynamic_cast<RigidBody*>(getDynamicSystem("Baum")->getObject("Scheibe_"+getBodyName(i)))->getFrame("R"));
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::CoilSpring * spVisu = new OpenMBV::CoilSpring();
      spVisu->setSpringRadius(.75*.1*h);
      spVisu->setCrossSectionRadius(.1*.25*h);
      spVisu->setNumberOfCoils(5);
      sp->setOpenMBVSpring(spVisu);
#endif
    }
  }
  SpringDamper * sp = new SpringDamper("Spring_"+getBodyName(4)+"_"+getBodyName(0));
  addLink(sp);
  sp->setForceFunction(new LinearSpringDamperForce(cF,0.05*cF,l0));
  sp->connect(
      dynamic_cast<RigidBody*>(getDynamicSystem("Baum")->getObject("Scheibe_"+getBodyName(4)))->getFrame("L"), 
      dynamic_cast<RigidBody*>(getDynamicSystem("Baum")->getObject("Scheibe_"+getBodyName(0)))->getFrame("R"));
#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::CoilSpring * spVisu = new OpenMBV::CoilSpring();
  spVisu->setSpringRadius(.75*.1*h);
  spVisu->setCrossSectionRadius(.1*.25*h);
  spVisu->setNumberOfCoils(5);
  sp->setOpenMBVSpring(spVisu);
#endif

  HydLine * l04 = new HydLine("l04");
  addObject(l04);
  l04->setDiameter(5e-3);
  l04->setLength(.7);
  l04->addPressureLoss(new PressureLossZeta("zeta1", 14));
  
  HydNodeConstrained * n0 = new HydNodeConstrained("n0");
  addLink(n0);
  // n0->setpFunction(new FuncConst(Vec(1, INIT, 3e5)));
  n0->setpFunction(new FuncHarmonic(Vec(1, INIT, 2e5), 2*M_PI*8, 0, Vec(1, INIT, 3e5)));
  n0->addOutFlow(l04);
  
  HydNodeMecConstrained * n1 = new HydNodeMecConstrained("n_"+getBodyName(0)+"_"+getBodyName(1));
  addLink(n1);
  n1->enableOpenMBV(.005);
  n1->setV0(V0);
  n1->setpFunction(new FuncConst(Vec(1, INIT, pRB)));
  n1->addRotMecArea(dynamic_cast<RigidBody*>(getDynamicSystem("Baum")->getObject("Scheibe_"+getBodyName(1)))->getFrame("R"), "[0;1;0]", area, traeger->getFrame("C"));
  n1->addRotMecArea(dynamic_cast<RigidBody*>(getDynamicSystem("Baum")->getObject("Scheibe_"+getBodyName(0)))->getFrame("L"), "[0;-1;0]", area, traeger->getFrame("C"));
  n1->enableOpenMBVArrows(.01);

  HydNodeMecElastic * n2 = new HydNodeMecElastic("n_"+getBodyName(1)+"_"+getBodyName(2));
  n2->setFracAir(0.08);
  n2->setp0(10e5);
  addLink(n2);
  n2->enableOpenMBVArrows(.01);
  n2->enableOpenMBV(.005);
  n2->setV0(V0);
  n2->addRotMecArea(dynamic_cast<RigidBody*>(getDynamicSystem("Baum")->getObject("Scheibe_"+getBodyName(2)))->getFrame("R"), Vec("[0;1;0]"), area, traeger->getFrame("C")); 
  n2->addRotMecArea(dynamic_cast<RigidBody*>(getDynamicSystem("Baum")->getObject("Scheibe_"+getBodyName(1)))->getFrame("L"), Vec("[0;-1;0]"), area, traeger->getFrame("C"));

  HydNodeMecElastic * n3 = new HydNodeMecElastic("n_"+getBodyName(2)+"_"+getBodyName(3));
  n3->setFracAir(0.08);
  n3->setp0(1e5);
  addLink(n3);
  n3->enableOpenMBVArrows(.01);
  n3->enableOpenMBV(.005);
  n3->setV0(V0);
  n3->addRotMecArea(dynamic_cast<RigidBody*>(getDynamicSystem("Baum")->getObject("Scheibe_"+getBodyName(3)))->getFrame("R"), Vec("[0;1;0]"), area, traeger->getFrame("C"));
  n3->addRotMecArea(dynamic_cast<RigidBody*>(getDynamicSystem("Baum")->getObject("Scheibe_"+getBodyName(2)))->getFrame("L"), Vec("[0;-1;0]"), area, traeger->getFrame("C"));

  HydNodeMecRigid * n4 = new HydNodeMecRigid("n_"+getBodyName(3)+"_"+getBodyName(4));
  addLink(n4);
  n4->setV0(V0);
  n4->enableOpenMBVArrows(.01);
  n4->enableOpenMBV(.005);
  n4->addRotMecArea(dynamic_cast<RigidBody*>(getDynamicSystem("Baum")->getObject("Scheibe_"+getBodyName(4)))->getFrame("R"), Vec("[0;1;0]"), area, traeger->getFrame("C")); 
  n4->addRotMecArea(dynamic_cast<RigidBody*>(getDynamicSystem("Baum")->getObject("Scheibe_"+getBodyName(3)))->getFrame("L"), Vec("[0;-1;0]"), area, traeger->getFrame("C"));
  n4->addInFlow(l04);
}
