#ifndef _CONTACT_RIGID_H_
#define _CONTACT_RIGID_H_

#include "rigid_contact.h"

namespace MBSim {

  class ContactRigid : public RigidContact {
    protected:
      double mu;
      int nFric;
    public:
      ContactRigid(const string &name) : RigidContact(name), mu(0), nFric(0) { }
      void setFrictionCoefficient(double mu_) { mu = mu_; }
      void setFrictionDirections(int nFric_) { nFric = nFric_; }

      void calclaSize() {
	if(nFric==1) {
	  setFrictionLaw(new PlanarCoulombFriction(mu));
	  setTangentialImpactLaw(new PlanarCoulombImpact(mu));
	} else if(nFric==2) {
	  setFrictionLaw(new SpatialCoulombFriction(mu));
	  setTangentialImpactLaw(new SpatialCoulombImpact(mu));
	}
	setContactLaw(new UnilateralConstraint);
	setNormalImpactLaw(new UnilateralNewtonImpact);

	RigidContact::calclaSize();
      }
      void init() {
	RigidContact::init();
      }
  };

}

#endif
