#include <config.h>
#include "planetary_gear.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/constraint.h"
#include "mbsim/gear.h"
//#include "mbsim/gearing.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/functions/kinematic_functions.h"
#include "mbsim/frames/fixed_relative_frame.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/frustum.h"
#include "openmbvcppinterface/compoundrigidbody.h"
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace boost;

namespace MBSimPowertrain {

  PlanetaryGear::PlanetaryGear(const std::string &name, int model_) : Group(name), model(model_) {
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
#ifdef HAVE_OPENMBVCPPINTERFACE
    housing->getFrame("C")->enableOpenMBV(0.3);
#endif

    RigidBody* sun = new RigidBody("Sun");
    addObject(sun);

    sun->setFrameOfReference(housing->getFrame("C"));
    sun->setFrameForKinematics(sun->getFrame("C"));
    sun->setRotation(new RotationAboutFixedAxis<VecV>(Vec("[0;0;1]")));

    sun->setMass(mS);
    Theta(2,2) = JS;
    sun->setInertiaTensor(Theta);
    Vec r(3);
    r(2) = lS/2;
#ifdef HAVE_OPENMBVCPPINTERFACE
    sun->getFrame("C")->enableOpenMBV(0.3);
#endif

    RigidBody* annulus = new RigidBody("Annulus");
    addObject(annulus);

    r(2) = lS/2-lH/2*1.0;
    r(1) = 0;
    housing->addFrame(new FixedRelativeFrame("Q",r,BasicRotAKIy(0)));

    annulus->setFrameOfReference(housing->getFrame("Q"));
    annulus->setFrameForKinematics(annulus->getFrame("C"));
    annulus->setRotation(new RotationAboutFixedAxis<VecV>(Vec("[0;0;1]")));
#ifdef HAVE_OPENMBVCPPINTERFACE
    annulus->getFrame("C")->enableOpenMBV(0.3);
#endif

    annulus->setMass(mH);
    Theta(2,2) = JH;
    annulus->setInertiaTensor(Theta);

    RigidBody* carrier = new RigidBody("Carrier");
    addObject(carrier);

    r.init(0);
    r(2) = lS/2+lT/2;
    housing->addFrame(new FixedRelativeFrame("R",r,BasicRotAKIy(0)));
    carrier->setFrameOfReference(housing->getFrame("R"));
    carrier->setFrameForKinematics(carrier->getFrame("C"));
    carrier->setRotation(new RotationAboutFixedAxis<VecV>(Vec("[0;0;1]")));
#ifdef HAVE_OPENMBVCPPINTERFACE
    carrier->getFrame("C")->enableOpenMBV(0.3);
#endif

    carrier->setMass(mT);
    Theta(2,2) = JT;
    carrier->setInertiaTensor(Theta);


    r.init(0);
    r(2) = -lT/2-lP/2;
    r(1) = rT2;

    vector<FixedRelativeFrame*> P;
    vector<RigidBody*> planet;
    int numP = 3;
    for(int i=0; i<numP; i++) {
      stringstream str;
      stringstream shaftName;
      str << "P" << i;
      shaftName << "Planet" << i;
      P.push_back(new FixedRelativeFrame(str.str(),BasicRotAKIz(M_PI*2*i/3.)*r,BasicRotAKIy(0)));
      carrier->addFrame(P[i]);
      planet.push_back(new RigidBody(shaftName.str()));
      addObject(planet[i]);
      planet[i]->setFrameOfReference(carrier->getFrame(str.str()));
      planet[i]->setFrameForKinematics(planet[i]->getFrame("C"));
      planet[i]->setRotation(new RotationAboutFixedAxis<VecV>(Vec("[0;0;1]")));
#ifdef HAVE_OPENMBVCPPINTERFACE
      planet[i]->getFrame("C")->enableOpenMBV(0.3);
#endif

      planet[i]->setMass(mP);
      Theta(2,2) = JP;
      planet[i]->setInertiaTensor(Theta);

    }

    if(model==1) {
      GearConstraint* constraint = new GearConstraint("C1");
      addConstraint(constraint);
      constraint->setDependentBody(carrier);
      constraint->addTransmission(Transmission(sun,0.5*rS/rT2));
      constraint->addTransmission(Transmission(annulus,0.5*rH/rT2));
    } else if(model==0) {
      Gear *gear;
      gear = new Gear("Gear1");
      addLink(gear);
      gear->setDependentBody(carrier);
      gear->addTransmission(Transmission(sun,0.5*rS/rT2));
      gear->addTransmission(Transmission(annulus,0.5*rH/rT2));
      //gear->setForceFunction(new LinearSpringDamperForce(5,0.5,0));

     // gear->setDependentBody(sun);
     // gear->addDependency(carrier,rT2/rS-rP/rS);
     // gear->addDependency(planet[0],-rP/rS);
     // gear->setRatio2(rT2/rS,1);
     // gear->setRatio2(-rP/rS,2);
     // gear->setFrame(housing->getFrame("C"));
    } else {
    }
    for(int i=0; i<numP; i++) {
      stringstream str;
      stringstream shaftName;
      str << "P" << i;
      shaftName << "Planet" << i;
      if(model==1) {
        GearConstraint* constraint = new GearConstraint(string("C_")+shaftName.str());
        addConstraint(constraint);
        constraint->setDependentBody(planet[i]);
        constraint->addTransmission(Transmission(sun,-0.5*(rS/rP+rS/rT2)));
        constraint->addTransmission(Transmission(annulus,0.5*(rH/rP-rH/rT2)));
      } else if (model==0) {
        Gear *gear;
        gear = new Gear(string("Gear_")+shaftName.str());
        addLink(gear);
        gear->setDependentBody(planet[i]);
        gear->addTransmission(Transmission(sun,-0.5*(rS/rP+rS/rT2)));
        gear->addTransmission(Transmission(annulus,0.5*(rH/rP-rH/rT2)));
        //gear->setForceFunction(new LinearSpringDamperForce(5,0.5,0));

        //gear->setDependentBody(annulus);
        //gear->addDependency(carrier,rT2/rH+rP/rH);
        //gear->addDependency(planet[0],rP/rH);
        //gear->setRatio2(rT2/rH,1);
        //gear->setRatio2(rP/rH,2);
        //gear->setFrame(housing->getFrame("C"));

      } else {
        //Gearing *gear;
        //gear = new Gearing(string("S_")+shaftName.str());
        //addLink(gear);
        //gear->connect(rS,sun->getFrame("C"),rP,planet[i]->getFrame("C"));//,rS,sun->getFrame("C"));
        //gear = new Gearing(string("A_")+shaftName.str());
        //gear->setReverse(true);
        //addLink(gear);
        //gear->connect(rP,planet[i]->getFrame("C"),rH,annulus->getFrame("C"));
      }
    }

#ifdef HAVE_OPENMBVCPPINTERFACE

    shared_ptr<OpenMBV::Frustum> cylinder;

    shared_ptr<OpenMBV::CompoundRigidBody> sunOMBV = OpenMBV::ObjectFactory::create<OpenMBV::CompoundRigidBody>();
    cylinder=OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
    cylinder->setTopRadius(rS/2);
    cylinder->setBaseRadius(rS/2);
    cylinder->setHeight(lS*5);
    cylinder->setDiffuseColor(0.1,1,1);
    cylinder->setInitialTranslation(0,0,0);
    cylinder->setName("frustum1");
    sunOMBV->addRigidBody(cylinder);
    cylinder=OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
    cylinder->setTopRadius(rS);
    cylinder->setBaseRadius(rS);
    cylinder->setHeight(lS);
    cylinder->setDiffuseColor(0.1,1,1);
    cylinder->setInitialTranslation(0,0,0);
    cylinder->setName("frustum2");
    sunOMBV->addRigidBody(cylinder);
    sunOMBV->setInitialTranslation(0,0,lS/2);
    sun->setOpenMBVRigidBody(sunOMBV);


    //  cylinder=OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
    //  cylinder->setTopRadius(rS);
    //  cylinder->setBaseRadius(rS);
    //  cylinder->setHeight(lS);
    //  cylinder->setDiffuseColor(0.1,1,1);
    //  sun->setOpenMBVRigidBody(cylinder);
    //  cylinder->setInitialTranslation(0,0,lS/2);

    cylinder=OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
    cylinder->setTopRadius(rH);
    cylinder->setBaseRadius(rH);
    cylinder->setInnerTopRadius(rH);
    cylinder->setInnerBaseRadius(rH);
    cylinder->setHeight(lH);
    cylinder->setDiffuseColor(0.3,1,1);
    annulus->setOpenMBVRigidBody(cylinder);
    cylinder->setInitialTranslation(0,0,lH/2);

    cylinder=OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
    cylinder->setTopRadius(rT1);
    cylinder->setBaseRadius(rT1);
    cylinder->setHeight(lT);
    cylinder->setDiffuseColor(0.5,1,1);
    carrier->setOpenMBVRigidBody(cylinder);
    cylinder->setInitialTranslation(0,0,lT/2);

    for(int i=0; i<numP; i++) {

      cylinder=OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
      cylinder->setTopRadius(rP);
      cylinder->setBaseRadius(rP);
      cylinder->setHeight(lP);
      cylinder->setDiffuseColor(0.8,1,1);
      planet[i]->setOpenMBVRigidBody(cylinder);
      cylinder->setInitialTranslation(0,0,lP/2);
    }

#endif

  }

}


