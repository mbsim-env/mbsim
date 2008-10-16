#include "system.h"
#include "rigid_body.h"
#include "tree.h"
#include "cuboid.h"
#include "contour.h"
#include "load.h"
#include "cube.h"

using namespace AMVis;

class Moment : public UserFunction {
  public:
    Vec operator()(double t) {
      Vec a(3);
      a(0) = 0.001*cos(t);
      a(1) = 0.0005*sin(t);
      //a(2) = 0.15*sin(t+M_PI/8);
      return a;
    }
};

class MyRot : public Rotation {
  public:
    virtual SqrMat operator()(const Vec &q, double t) {
      SqrMat A(3);
      for(int i=0; i<3; i++)
	A(i,i) = 1;
      return A;
    }; 
};

class MyPos : public Translation {
  public:
    int getqSize() const {return 0;}
    virtual Vec operator()(const Vec &q, double t) {
      Vec PrPK(3);
      double om = 1;
      PrPK(0) = cos(om*t);
      PrPK(1) = sin(om*t);
      return PrPK;
    }; 
};

class jT : public TimeDep {
  public:
    Vec operator()(double t) {
      Vec j(3);
      double om = 1;
      j(0) = -sin(om*t)*om;
      j(1) =  cos(om*t)*om;
      return j;
    }
};

class djT : public TimeDep {
  public:
    Vec operator()(double t) {
      Vec dj(3);
      double om = 1;
      dj(0) = -cos(om*t)*om*om;
      dj(1) =  -sin(om*t)*om*om;
      return dj;
    }
};

System::System(const string &projectName) : MultiBodySystem(projectName) {
  // Gravitation
  Vec grav(3);
  grav(1)=-9.81;
  setAccelerationOfGravity(grav);
  // Parameters
  double l = 0.3;              		
  double h =  0.02;
  double d = 0.1;
  double m = 0.7;
  SymMat Theta(3);
  Theta(1,1) = m*l*l/12.;
  Theta(2,2) = Theta(1,1);
  double alpha = 3.0 * M_PI/180.; 
  double deltax = 0.2;           
  double mu  = 0.3;

  RigidBody* body = new RigidBody("Rod");
  addObject(body);

  body->setFrameOfReference(getCoordinateSystem("I"));
  body->setCoordinateSystemForKinematics(body->getCoordinateSystem("C"));
  body->setMass(m);
  body->setInertiaTensor(Theta);
  body->setTranslation(new MyPos);
  body->setGuidingVelocityOfTranslation(new jT);
  body->setDerivativeOfGuidingVelocityOfTranslation(new djT);

  Cuboid *cubeoid = new Cuboid(body->getFullName(),1,false);
  cubeoid->setSize(l,h,d);
  cubeoid->setColor(0);
  body->setAMVisBody(cubeoid);

  // Just to have somtething to integrate ;-)
  body = new RigidBody("Rod2");
  addObject(body);
  body->setFrameOfReference(getCoordinateSystem("I"));
  body->setCoordinateSystemForKinematics(body->getCoordinateSystem("C"));
  body->setMass(m);
  body->setInertiaTensor(Theta);
  body->setTranslation(new LinearTranslation("[0; 1; 0]"));
}

