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

class MyPos : public Translation {
    public:
      int getqSize() const {return 1;} 
      Vec operator()(const Vec &q, double t) {
	Vec PrPK(3);
	PrPK(0) = cos(q(0));
	PrPK(1) = sin(q(0));
	return PrPK;
      }; 
  };

class JacobianT : public Jacobian {
  public:
    int getuSize() const {return 1;} 
    Mat operator()(const Vec& q, double t) {
      Mat J(3,1);
      J(0,0) = -sin(q(0));
      J(1,0) =  cos(q(0));
      return J;
    }
};
class JacobianR : public Jacobian {
  public:
    Mat operator()(const Vec& q, double t) {
      Mat J(3,1);
      return J;
    }
};

class MyDerT : public DerJac {
  public:
    Mat operator()(const Vec &qd, const Vec& q, double t) {
      Mat J(3,1);
      J(0,0) = -cos(q(0))*qd(0);
      J(1,0) = -sin(q(0))*qd(0);
      return J;
    }
};

class MyDerR : public DerJac {
  public:
    Mat operator()(const Vec &qd, const Vec& q, double t) {
      Mat J(3,1);
      return J;
    }
};

System::System(const string &projectName) : MultiBodySystem(projectName) {
 // Gravitation
  Vec grav(3);
  grav(1)=-9.81;
  setAccelerationOfGravity(grav);
 // Parameters
  double l = 0.8;              		
  double h =  0.02;
  double d = 0.1;
  double m = 0.7;
  SymMat Theta(3);
  Theta(1,1) = m*l*l/12.;
  Theta(2,2) = Theta(1,1);
  double alpha = 3.0 * M_PI/180.; 
  double deltax = 0.2;           
  double mu  = 0.3;

  Tree *tree = new Tree("Baum"); 
  addObject(tree);
  BodyRigid* body = new BodyRigid("Rod");

  tree->addObject(body);
  body->setFrameOfReference(getCoordinateSystem("I"));
  body->setCoordinateSystemForKinematics(body->getCoordinateSystem("S"));
  body->setMass(m);
  body->setInertiaTensor(Theta);
  body->setTranslation(new MyPos);
  body->setJacobianOfTranslation(new JacobianT);
  body->setDerivativeOfJacobianOfTranslation(new MyDerT);

   Cuboid *cubeoid = new Cuboid(body->getFullName(),1,false);
   cubeoid->setSize(l,h,d);
   cubeoid->setColor(0);
   body->setAMVisBody(cubeoid);


}

