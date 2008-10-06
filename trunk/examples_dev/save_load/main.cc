#include "system.h"
#include "rigid_body.h"
#include <integrators.h>
#include "objobject.h"

using namespace std;
using namespace MBSim;
using namespace AMVis;

int main (int argc, char* argv[])
{
  // build up model
  MultiBodySystem *sys = new System("MBS");
  sys->init();

  // save the model in directory "model"
  MultiBodySystem::save("./model",sys);

  // discard model
  delete sys;

  // load model form directory "model"
  sys = MultiBodySystem::load("./model");

  // set-up visualisation (not saved at the moment)
  ObjObject *obj = new ObjObject(sys->getObject("Rod")->getFullName(),1,false);
  obj->setObjFilename("objects/rod.obj");
  static_cast<RigidBody*>(sys->getObject("Rod"))->setAMVisBody(obj);
  obj->setInitialRotation(M_PI/2,0,0);
  obj->setScaleFactor(0.1);
  obj->setCalculationOfNormals(3);
  obj->setVertexEPS(1e-5);
  obj-> setNormalEPS(1e-5);
  obj-> setAngleEPS(M_PI*2/9);

  // continue as usual...
  sys->init();

  TimeSteppingIntegrator integrator;
  integrator.setdt(1e-4);
  integrator.setdtPlot(1e-2);
  integrator.settEnd(2.5);

  integrator.integrate(*sys);
  cout << "finished"<<endl;

  delete sys;

  return 0;

}

