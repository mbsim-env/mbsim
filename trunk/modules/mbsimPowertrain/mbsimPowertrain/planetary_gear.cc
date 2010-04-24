#include "planetary_gear.h"
#include "mbsim/rigid_body.h"
#include "mbsim/constraint.h"
#include "mbsim/utils/rotarymatrices.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/frustum.h"
#include "openmbvcppinterface/compoundrigidbody.h"
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimPowertrain {

  PlanetaryGear::PlanetaryGear(const std::string &name) : Group(name) {
    double mS = 1;
    double mH = 1.5;
    double mT = 0.3;
    double mP = 0.1;
    double lS = 0.1;
    double lH = lS;
    double lT = 0.3;
    double lP = lH;
    double rS = 0.2;
    double rH = 2*rS;
    double rP = (rH-rS)/2;
    double rT2 = rS+rP;
    double rT1 = 0.1;
    double JS = 0.5*mS*rS*rS; 
    double JH = 0.5*mH*rH*rH; 
    double JT = 0.5*mT*rT2*rT2; 
    double JP = 0.5*mP*rP*rP; 

    SymMat Theta(3);
    RigidBody* housing = new RigidBody("Housing");
    addObject(housing);

    housing->setFrameOfReference(getFrame("I"));
    housing->setFrameForKinematics(housing->getFrame("C"));

    housing->setMass(1);
    Theta(2,2) = 1;
    housing->setInertiaTensor(Theta);
    housing->getFrame("C")->enableOpenMBV(0.3);

    RigidBody* sun = new RigidBody("Sun");
    addObject(sun);

    sun->setFrameOfReference(housing->getFrame("C"));
    sun->setFrameForKinematics(sun->getFrame("C"));
    sun->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));

    sun->setMass(mS);
    Theta(2,2) = JS;
    sun->setInertiaTensor(Theta);
    Vec r(3);
    r(2) = lS/2;
    sun->getFrame("C")->enableOpenMBV(0.3);

    RigidBody* annulus = new RigidBody("Annulus");
    addObject(annulus);

    r(2) = lS/2-lH/2*1.01;
    r(1) = 0;
    housing->addFrame("Q",r,BasicRotAKIy(0));

    annulus->setFrameOfReference(housing->getFrame("Q"));
    annulus->setFrameForKinematics(annulus->getFrame("C"));
    annulus->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
    annulus->getFrame("C")->enableOpenMBV(0.3);

    annulus->setMass(mH);
    Theta(2,2) = JH;
    annulus->setInertiaTensor(Theta);

    RigidBody* carrier = new RigidBody("Carrier");
    addObject(carrier);

    r.init(0);
    r(2) = lS/2+lT/2;
    housing->addFrame("R",r,BasicRotAKIy(0));
    carrier->setFrameOfReference(housing->getFrame("R"));
    carrier->setFrameForKinematics(carrier->getFrame("C"));
    carrier->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
    carrier->getFrame("C")->enableOpenMBV(0.3);

    carrier->setMass(mT);
    Theta(2,2) = JT;
    carrier->setInertiaTensor(Theta);

    Constraint2* constraint = new Constraint2("C1",carrier);
    addObject(constraint);
    constraint->addDependency(sun,0.5*rS/rT2);
    constraint->addDependency(annulus,0.5*rH/rT2);

    r.init(0);
    r(2) = -lT/2-lP/2;
    r(1) = rT2;

    vector<Frame*> P;
    vector<RigidBody*> planet;
    int numP = 3;
    for(int i=0; i<numP; i++) {
      stringstream str;
      stringstream shaftName;
      str << "P" << i;
      shaftName << "Planet" << i;
      P.push_back(new Frame(str.str()));
      carrier->addFrame(P[i],BasicRotAKIz(M_PI*2*i/3.)*r,BasicRotAKIy(0));
      planet.push_back(new RigidBody(shaftName.str()));
      addObject(planet[i]);
      planet[i]->setFrameOfReference(carrier->getFrame(str.str()));
      planet[i]->setFrameForKinematics(planet[i]->getFrame("C"));
      planet[i]->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
      planet[i]->getFrame("C")->enableOpenMBV(0.3);

      planet[i]->setMass(mP);
      Theta(2,2) = JP;
      planet[i]->setInertiaTensor(Theta);

      Constraint2* constraint = new Constraint2(string("C_")+shaftName.str(),planet[i]);
      addObject(constraint);
      constraint->addDependency(sun,-0.5*(rS/rP+rS/rT2));
      constraint->addDependency(annulus,0.5*(rH/rP-rH/rT2));
    }

#ifdef HAVE_OPENMBVCPPINTERFACE

    OpenMBV::Frustum *cylinder;

    OpenMBV::CompoundRigidBody *sunOMBV = new OpenMBV::CompoundRigidBody;
    cylinder=new OpenMBV::Frustum;
    cylinder->setTopRadius(rS/2);
    cylinder->setBaseRadius(rS/2);
    cylinder->setHeight(lS*5);
    cylinder->setStaticColor(0.1);
    cylinder->setInitialTranslation(0,0,0);
    sunOMBV->addRigidBody(cylinder);
    cylinder=new OpenMBV::Frustum;
    cylinder->setTopRadius(rS);
    cylinder->setBaseRadius(rS);
    cylinder->setHeight(lS);
    cylinder->setStaticColor(0.1);
    cylinder->setInitialTranslation(0,0,0);
    sunOMBV->addRigidBody(cylinder);
    sunOMBV->setInitialTranslation(0,0,lS/2);
    sun->setOpenMBVRigidBody(sunOMBV);


    //  cylinder=new OpenMBV::Frustum;
    //  cylinder->setTopRadius(rS);
    //  cylinder->setBaseRadius(rS);
    //  cylinder->setHeight(lS);
    //  cylinder->setStaticColor(0.1);
    //  sun->setOpenMBVRigidBody(cylinder);
    //  cylinder->setInitialTranslation(0,0,lS/2);

    cylinder=new OpenMBV::Frustum;
    cylinder->setTopRadius(rH);
    cylinder->setBaseRadius(rH);
    cylinder->setHeight(lH);
    cylinder->setStaticColor(0.3);
    annulus->setOpenMBVRigidBody(cylinder);
    cylinder->setInitialTranslation(0,0,lH/2);

    cylinder=new OpenMBV::Frustum;
    cylinder->setTopRadius(rT1);
    cylinder->setBaseRadius(rT1);
    cylinder->setHeight(lT);
    cylinder->setStaticColor(0.5);
    carrier->setOpenMBVRigidBody(cylinder);
    cylinder->setInitialTranslation(0,0,lT/2);

    for(int i=0; i<numP; i++) {

      cylinder=new OpenMBV::Frustum;
      cylinder->setTopRadius(rP);
      cylinder->setBaseRadius(rP);
      cylinder->setHeight(lP);
      cylinder->setStaticColor(0.8);
      planet[i]->setOpenMBVRigidBody(cylinder);
      cylinder->setInitialTranslation(0,0,lP/2);
    }

#endif

  }

}


