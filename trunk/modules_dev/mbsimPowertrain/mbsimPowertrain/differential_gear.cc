#include "differential_gear.h"
#include "shaft.h"
#include "mbsim/utils/rotarymatrices.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/frustum.h"
#include "openmbvcppinterface/compoundrigidbody.h"
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimPowertrain {

  DifferentialGear::DifferentialGear(const std::string &name) : Group(name) {

    double m0 = 1;
    double m1 = 1;
    double m2 = 1.5;
    double mP = 0.3;
    double m4 = 1;
    double m5 = m4;
    double l = 1;
    double l2 = l/10;
    double l3 = 0.3;
    double l4 = 1;
    double l5 = l4;
    double R1 = 0.2;
    double R2 = 2*R1;
    double RP = 0.1;
    double R4 = RP;
    double R5 = R4;
    double J = 0.5*m1*R1*R1; 
    double J2 = 0.5*m2*R2*R2; 
    double JP = 0.5*mP*RP*RP; 
    double J4 = 0.5*m4*R4*R4; 
    double J5 = 0.5*m5*R5*R5; 

    SymMat Theta(3);
    RigidBody* housing = new RigidBody("Housing");
    addObject(housing);

    housing->setFrameOfReference(getFrame("I"));
    housing->setFrameForKinematics(housing->getFrame("C"));

    housing->setMass(m0);
    Theta(2,2) = J;
    housing->setInertiaTensor(Theta);
    housing->getFrame("C")->enableOpenMBV(0.3);

    Shaft* shaft1 = new Shaft("Shaft1");
    addObject(shaft1);

    shaft1->setFrameOfReference(housing->getFrame("C"));
    shaft1->setFrameForKinematics(shaft1->getFrame("C"));

    shaft1->setMass(m1);
    Theta(2,2) = J;
    shaft1->setInertiaTensor(Theta);
    //shaft1->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
    Vec r(3);
    r(2) = l/2;
    shaft1->addFrame("Q",r,SqrMat(3,EYE));
    shaft1->getFrame("Q")->enableOpenMBV(0.3);
    shaft1->getFrame("C")->enableOpenMBV(0.3);

    Shaft* shaft2 = new Shaft("Shaft2");
    addObject(shaft2);

    r(2) = l;
    r(1) = 1.5*R1;

    housing->addFrame("Q",r,BasicRotAKIy(M_PI/2));
    shaft2->setFrameOfReference(housing->getFrame("Q"));

    shaft2->setFrameForKinematics(shaft2->getFrame("C"));
    shaft2->getFrame("C")->enableOpenMBV(0.3);

    shaft2->setMass(m2);
    Theta(2,2) = J2;
    shaft2->setInertiaTensor(Theta);
    shaft2->addDependecy(shaft1,-R1/R2);

    Shaft* planet = new Shaft("Planet");
    addObject(planet);

    r.init(0);
    r(2) = l2/2+RP*1.2;
    shaft2->addFrame("Q",r,BasicRotAKIy(M_PI/2));
    shaft2->getFrame("Q")->enableOpenMBV(0.3);
    planet->setFrameOfReference(shaft2->getFrame("Q"));
    planet->setFrameForKinematics(planet->getFrame("C"));
    planet->getFrame("C")->enableOpenMBV(0.3);

    planet->setMass(mP);
    Theta(2,2) = JP;
    planet->setInertiaTensor(Theta);
    //planet->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));

    Shaft* shaft4 = new Shaft("Shaft4");
    addObject(shaft4);

    r.init(0);
    r(2) = -l4/2-0.2;
    housing->addFrame("L",r,BasicRotAKIy(0),housing->getFrame("Q"));
    shaft4->setFrameOfReference(housing->getFrame("L"));

    shaft4->setFrameForKinematics(shaft4->getFrame("C"));
    shaft4->getFrame("C")->enableOpenMBV(0.3);

    shaft4->setMass(m4);
    Theta(2,2) = J4;
    shaft4->setInertiaTensor(Theta);
    shaft4->addDependecy(shaft2,1);
    shaft4->addDependecy(planet,RP/R4);

    Shaft* shaft5 = new Shaft("Shaft5");
    addObject(shaft5);

    r.init(0);
    r(2) = +l5/2+0.2;
    housing->addFrame("R",r,BasicRotAKIy(0),housing->getFrame("Q"));
    shaft5->setFrameOfReference(housing->getFrame("R"));

    shaft5->setFrameForKinematics(shaft5->getFrame("C"));
    shaft5->getFrame("C")->enableOpenMBV(0.3);

    shaft5->setMass(m5);
    Theta(2,2) = J5;
    shaft5->setInertiaTensor(Theta);
    shaft5->addDependecy(shaft2,1);
    shaft5->addDependecy(planet,-RP/R4);

#ifdef HAVE_OPENMBVCPPINTERFACE
    OpenMBV::Frustum *cylinder=new OpenMBV::Frustum;
    cylinder->setTopRadius(R1);
    cylinder->setBaseRadius(R1);
    cylinder->setHeight(l);
    cylinder->setStaticColor(0.1);
    shaft1->setOpenMBVRigidBody(cylinder);
    cylinder->setInitialTranslation(0,0,l/2);

    cylinder=new OpenMBV::Frustum;
    cylinder->setTopRadius(R2);
    cylinder->setBaseRadius(R2);
    cylinder->setHeight(l2);
    cylinder->setStaticColor(0.4);
    shaft2->setOpenMBVRigidBody(cylinder);
    cylinder->setInitialTranslation(0,0,l2/2);

    cylinder=new OpenMBV::Frustum;
    cylinder->setTopRadius(RP);
    cylinder->setBaseRadius(RP);
    cylinder->setHeight(l3);
    cylinder->setStaticColor(0.5);
    planet->setOpenMBVRigidBody(cylinder);
    cylinder->setInitialTranslation(0,0,l3/2);

    cylinder=new OpenMBV::Frustum;
    cylinder->setTopRadius(R4);
    cylinder->setBaseRadius(R4);
    cylinder->setHeight(l4);
    cylinder->setStaticColor(0.5);
    shaft4->setOpenMBVRigidBody(cylinder);

    cylinder->setInitialTranslation(0,0,l5/2);
    cylinder=new OpenMBV::Frustum;
    cylinder->setTopRadius(R5);
    cylinder->setBaseRadius(R5);
    cylinder->setHeight(l5);
    cylinder->setStaticColor(0.5);
    shaft5->setOpenMBVRigidBody(cylinder);
    cylinder->setInitialTranslation(0,0,l5/2);
#endif

  }

}


