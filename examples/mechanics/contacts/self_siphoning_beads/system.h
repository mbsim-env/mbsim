#ifndef SYSTEM_H_
#define SYSTEM_H_

#include <mbsim/dynamic_system_solver.h>

#include <mbsim/rigid_body.h>

#include <fmatvec.h>

class System : public MBSim::DynamicSystemSolver {

  public:
    /**
     * \brief Dynamic system of the self-siphoning-beads
     *
     * \param elements Number of elements in the chain
     */
    System(const std::string &projectName, int elements);

    /*!
     * \brief add Trajectory for the pre-Integration
     */
    void addTrajectory();

  protected:
    std::vector<MBSim::RigidBody *> balls;

    const static double radius = 1e-3;
    const static double mass = 1e-3;
    const static double distance = 3e-3;

    const static bool ODE = true;

};

#endif /* SYSTEM_H_ */
