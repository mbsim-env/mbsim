#ifndef SYSTEM_H_
#define SYSTEM_H_

#include <mbsim/dynamic_system_solver.h>

#include <fmatvec.h>

class System: public MBSim::DynamicSystemSolver {
  public:
  /**
   * \brief new dynamic system fpr MaxwellContact-example
   *
   * \param ContactType 0 = MaxwellContact, 1 = regularized Contact, 2 = unilateral constraint
   * \param circleNums  Number of used circle-bodies
   */
    System(const std::string &projectName, int ContactType, int circleNums);

};

#endif /* SYSTEM_H_ */
