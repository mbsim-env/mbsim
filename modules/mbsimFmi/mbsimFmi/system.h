#ifndef _SLIDERCRANKMECHANISM_H
#define _SLIDERCRANKMECHANISM_H

#include "mbsim/dynamic_system_solver.h"
#include "mbsimFmi/model_instance.h"
#include <string>

class System : public MBSim::DynamicSystemSolver {
  public:
    System(const std::string &projectName);

    static void* operator new(std::size_t n) throw(std::bad_alloc)
    {
      return fmi::ModelInstance::functions.allocateMemory(1,n);
    }
    static void operator delete(void * p) throw()
    {
      fmi::ModelInstance::functions.freeMemory(p);
    }

    static void* operator new[](std::size_t s) throw(std::bad_alloc)
    {
      return ::operator new(s);
    }
    static void operator delete[](void *p) throw()
    {
      ::operator delete(p);
    }
};

#endif

