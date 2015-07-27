#include "press_bound.h"

using namespace MBSim;
using namespace std;

PressBound::PressBound(string name) : Link(name) {
}

void PressBound::updateh(double t) {
  for(unsigned int i=0; i<conIn.size(); i++)
    conIn[i]->geth()+=trans(conIn[i]->getJ(t))*p;
  for(unsigned int i=0; i<conOut.size(); i++)
    conOut[i]->geth()-=trans(conOut[i]->getJ(t))*p;
}
