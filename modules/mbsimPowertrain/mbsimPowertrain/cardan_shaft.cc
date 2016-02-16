#include <config.h>
#include "cardan_shaft.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/functions/translation_along_z_axis.h"
#include "mbsim/functions/rotation_about_axes_xy.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/objects/rigid_body.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/frustum.h"
#include "openmbvcppinterface/compoundrigidbody.h"
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace boost;

namespace MBSimPowertrain {

  CardanShaft::Data::Data() {
    massInputShaft = -1;
    massIntermediateShaft = -1;
    massOutputShaft = -1;
    lengthInputShaft = 0.2;
    lengthIntermediateShaft = 0.2;
    lengthOutputShaft = 0.2;
    radiusInputShaft = 0.05;
    radiusIntermediateShaft = 0.05;
    radiusOutputShaft = 0.05;
    lengthInputShaft = radiusInputShaft*3;
    lengthIntermediateShaft = radiusIntermediateShaft*8;
    lengthOutputShaft = radiusOutputShaft*3;
  }

  CardanShaft::CardanShaft(const std::string &name, Data param) : Group(name), data(param) {

    double density = 785;

    if(data.massInputShaft == -1) 
      data.massInputShaft = density*data.lengthInputShaft*pow(data.radiusInputShaft,2)*M_PI;
    if(data.massIntermediateShaft == -1) 
      data.massIntermediateShaft = density*data.lengthIntermediateShaft*pow(data.radiusIntermediateShaft,2)*M_PI;
    if(data.massOutputShaft == -1) 
      data.massOutputShaft = density*data.lengthOutputShaft*pow(data.radiusOutputShaft,2)*M_PI;

    if(data.inertiaTensorInputShaft.size() == 0) {
      data.inertiaTensorInputShaft.resize(3);
      data.inertiaTensorInputShaft(0,0) = data.massInputShaft*pow(data.lengthInputShaft,2)/12.0;
      data.inertiaTensorInputShaft(1,1) = data.massInputShaft*pow(data.lengthInputShaft,2)/12.0;
      data.inertiaTensorInputShaft(2,2) = data.massInputShaft*pow(data.radiusInputShaft,2)/2.0;
    }
    if(data.inertiaTensorIntermediateShaft.size() == 0) {
      data.inertiaTensorIntermediateShaft.resize(3);
      data.inertiaTensorIntermediateShaft(0,0) = data.massIntermediateShaft*pow(data.lengthIntermediateShaft,2)/12.0;
      data.inertiaTensorIntermediateShaft(1,1) = data.massIntermediateShaft*pow(data.lengthIntermediateShaft,2)/12.0;
      data.inertiaTensorIntermediateShaft(2,2) = data.massIntermediateShaft*pow(data.radiusIntermediateShaft,2)/2.0;
    }
    if(data.inertiaTensorOutputShaft.size() == 0) {
      data.inertiaTensorOutputShaft.resize(3);
      data.inertiaTensorOutputShaft(0,0) = data.massOutputShaft*pow(data.lengthOutputShaft,2)/12.0;
      data.inertiaTensorOutputShaft(1,1) = data.massOutputShaft*pow(data.lengthOutputShaft,2)/12.0;
      data.inertiaTensorOutputShaft(2,2) = data.massOutputShaft*pow(data.radiusOutputShaft,2)/2.0;
    }

    welle1 = new RigidBody("InputShaft");
    addObject(welle1);

    welle1->setMass(data.massInputShaft);
    welle1->setInertiaTensor(data.inertiaTensorInputShaft);

    Vec SrSP(3);
    SrSP.init(0);
    SrSP(2) = -data.lengthInputShaft/2;

    welle1->addFrame(new FixedRelativeFrame("K",SrSP,SqrMat(3,EYE)));
    welle1->setFrameForKinematics(welle1->getFrame("K"));

    SrSP(2) = data.lengthInputShaft/2;
    welle1->addFrame(new FixedRelativeFrame("Q",SrSP,SqrMat(3,EYE)));

    welle2 = new RigidBody("IntermediateShaft");
    addObject(welle2);

    welle2->setMass(data.massIntermediateShaft);
    welle2->setInertiaTensor(data.inertiaTensorIntermediateShaft);

    welle2->setTranslation(new TranslationAlongZAxis<VecV>);
    welle2->setRotation(new RotationAboutAxesXY<VecV>);
    welle2->setFrameOfReference(welle1->getFrame("Q"));
    SrSP.init(0);
    SrSP(2) = -data.lengthIntermediateShaft/2;
    welle2->addFrame(new FixedRelativeFrame("K",SrSP,SqrMat(3,EYE)));
    welle2->setFrameForKinematics(welle2->getFrame("K"));

    SrSP(2) = data.lengthIntermediateShaft/2;
    welle2->addFrame(new FixedRelativeFrame("Q",SrSP,BasicRotAKIz(M_PI/2)));

    welle3 = new RigidBody("OutputShaft");
    addObject(welle3);

    welle3->setMass(data.massOutputShaft);
    welle3->setInertiaTensor(data.inertiaTensorOutputShaft);

    SrSP.init(0);
    SrSP(2) = -data.lengthOutputShaft/2;

    welle3->setRotation(new RotationAboutAxesXY<VecV>);
    welle3->setFrameOfReference(welle2->getFrame("Q"));
    welle3->addFrame(new FixedRelativeFrame("K",SrSP,SqrMat(3,EYE)));
    welle3->setFrameForKinematics(welle3->getFrame("K"));

    SrSP(2) = data.lengthOutputShaft/2;
    welle3->addFrame(new FixedRelativeFrame("Q",SrSP,SqrMat(3,EYE)));

#ifdef HAVE_OPENMBVCPPINTERFACE
    shared_ptr<OpenMBV::Frustum> cylinder = OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
    cylinder->setBaseRadius(data.radiusInputShaft);
    cylinder->setTopRadius(data.radiusInputShaft);
    cylinder->setHeight(data.lengthInputShaft);
    cylinder->setDiffuseColor(0.3,1,1);
    welle1->setOpenMBVRigidBody(cylinder);
    cylinder -> setInitialTranslation(0,0,data.lengthInputShaft/2);
    cylinder -> setInitialRotation(0,0,0);

    cylinder = OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
    cylinder->setBaseRadius(data.radiusIntermediateShaft);
    cylinder->setTopRadius(data.radiusIntermediateShaft);
    cylinder->setHeight(data.lengthIntermediateShaft);
    cylinder->setDiffuseColor(0.5,1,1);
    welle2->setOpenMBVRigidBody(cylinder);
    cylinder -> setInitialTranslation(0,0,data.lengthIntermediateShaft/2);
    cylinder -> setInitialRotation(0,0,0);

    cylinder = OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
    cylinder->setBaseRadius(data.radiusOutputShaft);
    cylinder->setTopRadius(data.radiusOutputShaft);
    cylinder->setHeight(data.lengthOutputShaft);
    cylinder->setDiffuseColor(0.7,1,1);
    welle3->setOpenMBVRigidBody(cylinder);
    cylinder -> setInitialTranslation(0,0,data.lengthOutputShaft/2);
    cylinder -> setInitialRotation(0,0,0);


#endif

  }

}


