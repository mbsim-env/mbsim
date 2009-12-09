#include "differential_gear.h"
#include "shaft.h"
#include "mbsim/utils/rotarymatrices.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/frustum.h"
#include "openmbvcppinterface/cube.h"
#include "openmbvcppinterface/compoundrigidbody.h"
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimPowertrain {

  DifferentialGear::Data::Data() {
    massHousing = 2;
    massInputShaft = -1;
    massLeftOutputShaft = -1;
    massRightOutputShaft = -1;
    massPlanet = -1;
    radiusInputShaft = 0.15;
    radiusLeftOutputShaft = 0.05;
    radiusRightOutputShaft = 0.05;
    radiusPlanet = 0.05;
    lengthInputShaft = radiusInputShaft/4.;
    lengthLeftOutputShaft = radiusLeftOutputShaft*3;
    lengthRightOutputShaft = radiusRightOutputShaft*3;
    lengthPlanet = radiusPlanet*2;
  }

  DifferentialGear::DifferentialGear(const std::string &name, Data param) : Group(name), data(param) {
    double density = 785;

    if(data.massInputShaft == -1) 
      data.massInputShaft = density*data.lengthInputShaft*pow(data.radiusInputShaft,2)*M_PI;
    if(data.massLeftOutputShaft == -1) 
      data.massLeftOutputShaft = density*data.lengthLeftOutputShaft*pow(data.radiusLeftOutputShaft,2)*M_PI;
    if(data.massRightOutputShaft == -1) 
      data.massRightOutputShaft = density*data.lengthRightOutputShaft*pow(data.radiusRightOutputShaft,2)*M_PI;
    if(data.massPlanet == -1) 
      data.massPlanet = density*data.lengthPlanet*pow(data.radiusPlanet,2)*M_PI;
    if(data.inertiaTensorHousing.size() == 0) {
      data.inertiaTensorHousing = SymMat(3,EYE)*data.massHousing/10.;
    }
    if(data.inertiaTensorInputShaft.size() == 0) {
      data.inertiaTensorInputShaft.resize(3);
      data.inertiaTensorInputShaft(0,0) = data.massInputShaft*pow(data.lengthInputShaft,2)/12.0;
      data.inertiaTensorInputShaft(1,1) = data.massInputShaft*pow(data.lengthInputShaft,2)/12.0;
      data.inertiaTensorInputShaft(2,2) = data.massInputShaft*pow(data.radiusInputShaft,2)/2.0;
    }
    if(data.inertiaTensorLeftOutputShaft.size() == 0) {
      data.inertiaTensorLeftOutputShaft.resize(3);
      data.inertiaTensorLeftOutputShaft(0,0) = data.massLeftOutputShaft*pow(data.lengthLeftOutputShaft,2)/12.0;
      data.inertiaTensorLeftOutputShaft(1,1) = data.massLeftOutputShaft*pow(data.lengthLeftOutputShaft,2)/12.0;
      data.inertiaTensorLeftOutputShaft(2,2) = data.massLeftOutputShaft*pow(data.radiusLeftOutputShaft,2)/2.0;
    }
    if(data.inertiaTensorRightOutputShaft.size() == 0) {
      data.inertiaTensorRightOutputShaft.resize(3);
      data.inertiaTensorRightOutputShaft(0,0) = data.massRightOutputShaft*pow(data.lengthRightOutputShaft,2)/12.0;
      data.inertiaTensorRightOutputShaft(1,1) = data.massRightOutputShaft*pow(data.lengthRightOutputShaft,2)/12.0;
      data.inertiaTensorRightOutputShaft(2,2) = data.massRightOutputShaft*pow(data.radiusRightOutputShaft,2)/2.0;
    }
    if(data.inertiaTensorPlanet.size() == 0) {
      data.inertiaTensorPlanet.resize(3);
      data.inertiaTensorPlanet(0,0) = data.massPlanet*pow(data.lengthPlanet,2)/12.0;
      data.inertiaTensorPlanet(1,1) = data.massPlanet*pow(data.lengthPlanet,2)/12.0;
      data.inertiaTensorPlanet(2,2) = data.massPlanet*pow(data.radiusPlanet,2)/2.0;
    }

    RigidBody* housing = new RigidBody("Housing");
    addObject(housing);

    housing->setFrameOfReference(getFrame("I"));
    housing->setFrameForKinematics(housing->getFrame("C"));

    housing->setMass(data.massHousing);
    housing->setInertiaTensor(data.inertiaTensorHousing);

    shaft2 = new Shaft("InputShaft");
    addObject(shaft2);

    double a = data.lengthInputShaft/2+data.radiusPlanet*1.2;
    Vec r(3);
    r(2) = -a;
    housing->addFrame("Q",r,SqrMat(3,EYE));
    shaft2->setFrameOfReference(housing->getFrame("Q"));
    shaft2->setFrameForKinematics(shaft2->getFrame("C"));

    shaft2->setMass(data.massInputShaft);
    shaft2->setInertiaTensor(data.inertiaTensorInputShaft);

    Shaft* planet = new Shaft("Planet");
    addObject(planet);

    r.init(0);
    r(2) = a;
    shaft2->addFrame("Q",r,BasicRotAKIy(M_PI/2));
    planet->setFrameOfReference(shaft2->getFrame("Q"));
    planet->setFrameForKinematics(planet->getFrame("C"));
    planet->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));

    planet->setMass(data.massPlanet);
    planet->setInertiaTensor(data.inertiaTensorPlanet);

    shaft4 = new Shaft("LeftOutputShaft");
    addObject(shaft4);

    r.init(0);
    r(2) = -data.lengthLeftOutputShaft/2+a;
    housing->addFrame("L",r,SqrMat(3,EYE),housing->getFrame("Q"));
    shaft4->setFrameOfReference(housing->getFrame("L"));
    shaft4->setFrameForKinematics(shaft4->getFrame("C"));
    r(2) = -data.lengthLeftOutputShaft/2;
    shaft4->addFrame("Q",r,BasicRotAKIy(M_PI));
    shaft4->getFrame("Q")->enableOpenMBV(0.3);

    shaft4->setMass(data.massLeftOutputShaft);
    shaft4->setInertiaTensor(data.inertiaTensorLeftOutputShaft);
    shaft4->addDependecy(shaft2,1);
    shaft4->addDependecy(planet,data.radiusPlanet/data.radiusLeftOutputShaft);

    shaft5 = new Shaft("RightOutputShaft");
    addObject(shaft5);

    r.init(0);
    r(2) = data.lengthRightOutputShaft/2+a;
    housing->addFrame("R",r,SqrMat(3,EYE),housing->getFrame("Q"));
    shaft5->setFrameOfReference(housing->getFrame("R"));
    shaft5->setFrameForKinematics(shaft5->getFrame("C"));
    r(2) = data.lengthRightOutputShaft/2;
    shaft5->addFrame("Q",r,SqrMat(3,EYE));
    shaft5->getFrame("Q")->enableOpenMBV(0.3);

    shaft5->setMass(data.massRightOutputShaft);
    shaft5->setInertiaTensor(data.inertiaTensorRightOutputShaft);
    shaft5->addDependecy(shaft2,1);
    shaft5->addDependecy(planet,-data.radiusPlanet/data.radiusRightOutputShaft);

#ifdef HAVE_OPENMBVCPPINTERFACE
    OpenMBV::Cube *cube=new OpenMBV::Cube;
    cube->setLength(0.01);
    cube->setStaticColor(0.1);
    housing->setOpenMBVRigidBody(cube);

    OpenMBV::Frustum *cylinder;

    cylinder=new OpenMBV::Frustum;
    cylinder->setTopRadius(data.radiusInputShaft);
    cylinder->setBaseRadius(data.radiusInputShaft);
    cylinder->setHeight(data.lengthInputShaft);
    cylinder->setStaticColor(0.2);
    shaft2->setOpenMBVRigidBody(cylinder);
    cylinder->setInitialTranslation(0,0,data.lengthInputShaft/2);

    cylinder=new OpenMBV::Frustum;
    cylinder->setTopRadius(data.radiusPlanet);
    cylinder->setBaseRadius(data.radiusPlanet);
    cylinder->setHeight(data.lengthPlanet);
    cylinder->setStaticColor(0.88);
    planet->setOpenMBVRigidBody(cylinder);
    cylinder->setInitialTranslation(0,0,data.lengthPlanet/2);

    cylinder=new OpenMBV::Frustum;
    cylinder->setTopRadius(data.radiusLeftOutputShaft);
    cylinder->setBaseRadius(data.radiusLeftOutputShaft);
    cylinder->setHeight(data.lengthLeftOutputShaft);
    cylinder->setStaticColor(0.5);
    shaft4->setOpenMBVRigidBody(cylinder);
    cylinder->setInitialTranslation(0,0,data.lengthLeftOutputShaft/2);

    cylinder=new OpenMBV::Frustum;
    cylinder->setTopRadius(data.radiusRightOutputShaft);
    cylinder->setBaseRadius(data.radiusRightOutputShaft);
    cylinder->setHeight(data.lengthRightOutputShaft);
    cylinder->setStaticColor(0.7);
    shaft5->setOpenMBVRigidBody(cylinder);
    cylinder->setInitialTranslation(0,0,data.lengthRightOutputShaft/2);
#endif

  }

}


