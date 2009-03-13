#ifndef _CONTACT_RIGID_H_
#define _CONTACT_RIGID_H_

#include "contact.h"
#include "constitutive_laws.h"

namespace MBSim {

  class ContactRigid : public Contact {
    protected:
      double mu;
      int nFric;
    public:
      ContactRigid(const string &name) : Contact(name), mu(0), nFric(0) { }
      void setFrictionCoefficient(double mu_) { mu = mu_; }
      void setFrictionDirections(int nFric_) { nFric = nFric_; }

      void calclaSize() {
	if(nFric==1) {
	  setFrictionForceLaw(new PlanarCoulombFriction(mu));
	  setFrictionImpactImpactLaw(new PlanarCoulombImpact(mu));
	} else if(nFric==2) {
	  setFrictionForceLaw(new SpatialCoulombFriction(mu));
	  setFrictionImpactLaw(new SpatialCoulombImpact(mu));
	}
	setContactForceLaw(new UnilateralConstraint);
	setContactImpactLaw(new UnilateralNewtonImpact);

	Contact::calclaSize();
      }
      void init() {
	Contact::init();
      }
  };

}

#endif
