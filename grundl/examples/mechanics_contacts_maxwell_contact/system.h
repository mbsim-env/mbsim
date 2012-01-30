#ifndef SYSTEM_H_
#define SYSTEM_H_

#include <mbsim/dynamic_system_solver.h>

#include <fmatvec.h>

class System: public MBSim::DynamicSystemSolver {
  public:
  /**
   * \brief new dynamic system for MaxwellContact-example
   *
   * \param ContactType           0 = MaxwellContact, 1 = regularized Contact, 2 = unilateral constraint
   * \param firstBall             Number of the first ball that is in contact with the beam
   * \param lastBall              Number of the last ball that is in contact with the beam
   * \param  ReferenceFrameShift  Shifts the system translational (for visualization of different contact types)
   */
    System(const std::string &projectName, int ContactType, int firstBall, int lastBall, const fmatvec::Vec& ReferenceFrameShift);

};

#endif /* SYSTEM_H_ */
