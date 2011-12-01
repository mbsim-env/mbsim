#ifndef CARDAN_SHAFT_H_
#define CARDAN_SHAFT_H_

#include "mbsim/group.h"

namespace MBSim {
  class RigidBody;
}

namespace MBSimPowertrain {

  class CardanShaft : public MBSim::Group { 
    public:
      struct Data {
	double massInputShaft;
	double massIntermediateShaft;
	double massOutputShaft;
	fmatvec::SymMat inertiaTensorInputShaft;
	fmatvec::SymMat inertiaTensorIntermediateShaft;
	fmatvec::SymMat inertiaTensorOutputShaft;
	double lengthInputShaft;
	double lengthIntermediateShaft;
	double lengthOutputShaft;
	double radiusInputShaft;
	double radiusIntermediateShaft;
	double radiusOutputShaft;
	Data();
      };
    protected:
      MBSim::RigidBody *welle1, *welle2, *welle3;
      Data data;

    public:
      //CardanShaft(const std::string &name);
      CardanShaft(const std::string &name, Data param=Data());
      MBSim::RigidBody* getInputShaft() {return welle1;}
      MBSim::RigidBody* getIntermediateShaft() {return welle2;}
      MBSim::RigidBody* getOutputShaft() {return welle3;}
      const Data& getData() const {return data;}
  };

}

#endif
