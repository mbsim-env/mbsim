#ifndef SYSTEM_H_
#define SYSTEM_H_

#include <mbsim/dynamic_system_solver.h>


#include <mbsim/rigid_body.h>

#include <fmatvec/fmatvec.h>

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
    void addTrajectory(double tEnd);

  protected:
    std::vector<MBSim::RigidBody *> balls;

    int elements;

    const static double radius = 1e-3;
    const static double mass = 2e-3;
    const static double distance = 3e-3;

    const static double stiffness = 1e4;
    const static double damping = 1;

    const static bool ODE = true;

};

#endif /* SYSTEM_H_ */
