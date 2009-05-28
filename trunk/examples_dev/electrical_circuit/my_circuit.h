#ifndef _MY_CIRCUIT_H
#define _MY_CIRCUIT_H

#include "mbsimElectronics/simulation_classes.h"

class MyCircuit : public MBSim::SpecialGroup {
 private:
  public:
    MyCircuit(const std::string &projectName); 
};

#endif

