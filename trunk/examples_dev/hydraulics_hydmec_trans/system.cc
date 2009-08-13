#include "system.h"

#include "mbsimHydraulics/hydline.h"
#include "mbsimHydraulics/hydnode.h"
#include "mbsimHydraulics/hydnode_mec.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsim/utils/function.h"

#include "mbsim/rigid_body.h"
#include "mbsim/tree.h"
#include "mbsim/spring_damper.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/frustum.h"
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

System::System(const string &name, bool unilateral) : Group(name) {
  Tree * tree = new Tree("Baum");
  addDynamicSystem(tree);

  double l=0.3;
  double lScheibe=l/20.;
  double dA=0.05;
  double dI=0.01;
  double unloadedLength=l/4.-lScheibe;

  double stiffness=1e4;

  RigidBody * traeger = new RigidBody("Traeger");
  Node * node = tree->addObject(0, traeger);
  Vec KrCF(3, INIT, 0);
  KrCF(0)=-l/2.;
  traeger->addFrame(getBodyName(0), KrCF, SqrMat(3, EYE));
  traeger->getFrame(getBodyName(0))->enableOpenMBV(3.*lScheibe);
  KrCF(0)=-l/4.;
  traeger->addFrame(getBodyName(1), KrCF, SqrMat(3, EYE));
  traeger->getFrame(getBodyName(1))->enableOpenMBV(3.*lScheibe);
  KrCF(0)=0;
  traeger->addFrame(getBodyName(2), KrCF, SqrMat(3, EYE));
  traeger->getFrame(getBodyName(2))->enableOpenMBV(3.*lScheibe);
  KrCF(0)=l/4.;
  traeger->addFrame(getBodyName(3), KrCF, SqrMat(3, EYE));
  traeger->getFrame(getBodyName(3))->enableOpenMBV(3.*lScheibe);
  KrCF(0)=l/2.;
  traeger->addFrame(getBodyName(4), KrCF, SqrMat(3, EYE));
  traeger->getFrame(getBodyName(4))->enableOpenMBV(3.*lScheibe);
  traeger->setFrameOfReference(getFrame("I"));
  traeger->setFrameForKinematics(traeger->getFrame("C"));
  traeger->setMass(2.);
  traeger->setInertiaTensor(0.001*SymMat(3, EYE));
  traeger->setTranslation(new LinearTranslation(SqrMat(3, EYE)));
  traeger->setRotation(new CardanAngles());
  traeger->setInitialGeneralizedVelocity(Vec("[0.; -0.; 0.; -30; 30; 30]"));
#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::Frustum * traegerVisu = new OpenMBV::Frustum();
  traegerVisu->setBaseRadius(dI/2.);
  traegerVisu->setTopRadius(dI/2.);
  traegerVisu->setHeight(l+lScheibe);
  traegerVisu->setInitialRotation(0, -M_PI/2., 0);
  traegerVisu->setInitialTranslation(-(l+lScheibe)/2., 0, 0);
  traegerVisu->setStaticColor(0);
  traeger->setOpenMBVRigidBody(traegerVisu);
#endif

  for (int i=0; i<5; i++) {
    Vec vML(3, INIT, 0);
    vML(0)=-lScheibe/2.;
    Vec vMR(3, INIT, 0);
    vMR(0)=lScheibe/2.;
    RigidBody * scheibe = new RigidBody("Scheibe_"+getBodyName(i));
    tree->addObject(node, scheibe);
    scheibe->setMass(2.);
    scheibe->addFrame("L", vML, SqrMat(3, EYE));
    scheibe->addFrame("R", vMR, SqrMat(3, EYE));
    scheibe->setFrameOfReference(traeger->getFrame(getBodyName(i)));
    scheibe->setFrameForKinematics(scheibe->getFrame("C"));
    scheibe->setInertiaTensor(0.001*SymMat(3, EYE));
    if (i>0)
      scheibe->setTranslation(new LinearTranslation(Vec("[1; 0; 0]")));
#ifdef HAVE_OPENMBVCPPINTERFACE
    OpenMBV::Frustum * scheibeVisu = new OpenMBV::Frustum();
    scheibeVisu->setBaseRadius(dA/2.);
    scheibeVisu->setTopRadius(dA/2.);
    scheibeVisu->setInnerBaseRadius(dI/2.);
    scheibeVisu->setInnerTopRadius(dI/2.);
    scheibeVisu->setHeight(lScheibe);
    scheibeVisu->setInitialRotation(0, M_PI/2., 0);
    scheibeVisu->setInitialTranslation(lScheibe/2., 0, 0);
    scheibeVisu->setStaticColor((i)/4.);
    scheibe->setOpenMBVRigidBody(scheibeVisu);
#endif

    if (i>0) {
      SpringDamper * sp = new SpringDamper("Spring_"+getBodyName(i-1)+"_"+getBodyName(i));
      addLink(sp);
      sp->setForceFunction(new LinearSpringDamperForce(stiffness,0.05*stiffness,unloadedLength));
      sp->connect(
          dynamic_cast<RigidBody*>(getDynamicSystem("Baum")->getObject("Scheibe_"+getBodyName(i-1)))->getFrame("R"), 
          dynamic_cast<RigidBody*>(getDynamicSystem("Baum")->getObject("Scheibe_"+getBodyName(i)))->getFrame("L"));
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::CoilSpring * spVisu = new OpenMBV::CoilSpring();
      spVisu->setSpringRadius(.75*.5*dA);
      spVisu->setCrossSectionRadius(.1*.25*dA);
      spVisu->setNumberOfCoils(5);
      sp->setOpenMBVSpring(spVisu);
#endif
    }
  }
  
  HydLine * l04 = new HydLine("l04");
  addObject(l04);
  l04->setDiameter(5e-3);
  l04->setLength(.7);
  l04->addPressureLoss(new PressureLossZeta("zeta1", 14));

  double area=M_PI*(dA*dA-dI*dI)/4.;
  double pressure=.75*unloadedLength*stiffness/area;
  double V0=area*unloadedLength;
  
  HydNodeConstrained * n0 = new HydNodeConstrained("n0");
  addLink(n0);
  n0->setpFunction(new ConstantFunction1<double, double>(1e5));
  n0->addOutFlow(l04);

  HydNodeMecEnvironment * n1Inf = new HydNodeMecEnvironment("n1Inf");
  addLink(n1Inf);
  n1Inf->addTransMecArea(dynamic_cast<RigidBody*>(getDynamicSystem("Baum")->getObject("Scheibe_"+getBodyName(0)))->getFrame("L"), Vec("[1; 0; 0]"), area);
  
  HydNodeMecConstrained * n1 = new HydNodeMecConstrained("n_"+getBodyName(0)+"_"+getBodyName(1));
  addLink(n1);
  n1->setInitialVolume(V0);
  n1->setpFunction(new ConstantFunction1<double, double>(pressure));
  n1->addTransMecArea(dynamic_cast<RigidBody*>(getDynamicSystem("Baum")->getObject("Scheibe_"+getBodyName(0)))->getFrame("R"), Vec("[-1; 0; 0]"), area);
  n1->addTransMecArea(dynamic_cast<RigidBody*>(getDynamicSystem("Baum")->getObject("Scheibe_"+getBodyName(1)))->getFrame("L"), Vec("[1; 0; 0]"), area);

  HydNodeMecElastic * n2 = new HydNodeMecElastic("n_"+getBodyName(1)+"_"+getBodyName(2));
  n2->setFracAir(0.08);
  n2->setp0(10e5);
  addLink(n2);
  n2->setInitialVolume(V0);
  n2->addTransMecArea(dynamic_cast<RigidBody*>(getDynamicSystem("Baum")->getObject("Scheibe_"+getBodyName(1)))->getFrame("R"), Vec("[-1; 0; 0]"), area); 
  n2->addTransMecArea(dynamic_cast<RigidBody*>(getDynamicSystem("Baum")->getObject("Scheibe_"+getBodyName(2)))->getFrame("L"), Vec("[1; 0; 0]"), area);

  HydNodeMecElastic * n3 = new HydNodeMecElastic("n_"+getBodyName(2)+"_"+getBodyName(3));
  n3->setFracAir(0.08);
  n3->setp0(1e5);
  addLink(n3);
  n3->setInitialVolume(V0);
  n3->addTransMecArea(dynamic_cast<RigidBody*>(getDynamicSystem("Baum")->getObject("Scheibe_"+getBodyName(2)))->getFrame("R"), Vec("[-1; 0; 0]"), area);
  n3->addTransMecArea(dynamic_cast<RigidBody*>(getDynamicSystem("Baum")->getObject("Scheibe_"+getBodyName(3)))->getFrame("L"), Vec("[1; 0; 0]"), area);

  HydNodeMecRigid * n4 = new HydNodeMecRigid("n_"+getBodyName(3)+"_"+getBodyName(4));
  addLink(n4);
  n4->setInitialVolume(V0);
  n4->addTransMecArea(dynamic_cast<RigidBody*>(getDynamicSystem("Baum")->getObject("Scheibe_"+getBodyName(3)))->getFrame("R"), Vec("[-1; 0; 0]"), area); 
  n4->addTransMecArea(dynamic_cast<RigidBody*>(getDynamicSystem("Baum")->getObject("Scheibe_"+getBodyName(4)))->getFrame("L"), Vec("[1; 0; 0]"), area);
  n4->addInFlow(l04);

  HydNodeMecEnvironment * n4Inf = new HydNodeMecEnvironment("n4Inf");
  addLink(n4Inf);
  n4Inf->addTransMecArea(dynamic_cast<RigidBody*>(getDynamicSystem("Baum")->getObject("Scheibe_"+getBodyName(4)))->getFrame("R"), Vec("[-1; 0; 0]"), area);
}
