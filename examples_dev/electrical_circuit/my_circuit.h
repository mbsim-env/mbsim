#ifndef _MY_CIRCUIT_H
#define _MY_CIRCUIT_H

#include "simulation_classes.h"

class MyCircuit : public ElectricalCircuit {
 private:
  public:
    MyCircuit(const std::string &projectName); 
};

#endif

