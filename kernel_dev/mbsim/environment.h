#ifndef _MBSIM_ENVIRONMENT_H_
#define _MBSIM_ENVIRONMENT_H_

#include "mbsimtinyxml/tinyxml-src/tinyxml.h"
#include "fmatvec.h"

namespace MBSim {

class Environment {
  protected:
    Environment() {};
    virtual ~Environment() {};
  public:
    virtual void initializeUsingXML(TiXmlElement *element)=0;
};

class MBSimEnvironment : public Environment {
  protected:
    static MBSimEnvironment *instance;
    MBSimEnvironment() : Environment(), grav(3) {}
    fmatvec::Vec grav;
  public:
    static MBSimEnvironment *getInstance() { return instance?instance:(instance=new MBSimEnvironment); }
    virtual void initializeUsingXML(TiXmlElement *element);
    void setAccelerationOfGravity(const fmatvec::Vec &grav_) { grav=grav_; }
    const fmatvec::Vec& getAccelerationOfGravity() const { return grav; }
};

}

#endif
