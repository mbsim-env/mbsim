#include "press_bound.h"

using namespace MBSim;
using namespace std;

PressBound::PressBound(string name) : Link(name) {
}

void PressBound::updateh(int k) {
  for(unsigned int i=0; i<conIn.size(); i++)
    conIn[i]->geth(0,false)+=trans(conIn[i]->evalJ())*p;
  for(unsigned int i=0; i<conOut.size(); i++)
    conOut[i]->geth(0,false)-=trans(conOut[i]->evalJ())*p;
}
