#ifndef DIFFERENTIAL_GEAR_H_
#define DIFFERENTIAL_GEAR_H_

#include "mbsim/group.h"
#include "mbsim/rigid_body.h"

namespace MBSimPowertrain {

  class DifferentialGear : public MBSim::Group { 
    public:
      struct Data {
	double massHousing;
	double massInputShaft;
	double massRightOutputShaft;
	double massLeftOutputShaft;
	double massPlanet;
	fmatvec::SymMat inertiaTensorHousing;
	fmatvec::SymMat inertiaTensorInputShaft;
	fmatvec::SymMat inertiaTensorRightOutputShaft;
	fmatvec::SymMat inertiaTensorLeftOutputShaft;
	fmatvec::SymMat inertiaTensorPlanet;
	double lengthInputShaft;
	double lengthRightOutputShaft;
	double lengthLeftOutputShaft;
	double lengthPlanet;
	double radiusInputShaft;
	double radiusLeftOutputShaft;
	double radiusRightOutputShaft;
	double radiusPlanet;
	Data();
      };
    protected:
      Data data;
      MBSim::RigidBody *shaft2, *shaft4, *shaft5;

    public:
      DifferentialGear(const std::string &name, Data data=Data(), bool planetIndependent=false);
      double getRadiusInputShaft() const {return data.radiusInputShaft;}
      MBSim::RigidBody* getInputShaft() {return shaft2;}
      MBSim::RigidBody* getRightOutputShaft() {return shaft5;}
      MBSim::RigidBody* getLeftOutputShaft() {return shaft4;}
  };

}

#endif
