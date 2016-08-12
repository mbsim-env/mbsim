#ifndef SYSTEM_H_
#define SYSTEM_H_

#include <mbsim/dynamic_system_solver.h>


#include <mbsim/objects/rigid_body.h>

#include <fmatvec/fmatvec.h>

class SelfSiphoningBeats : public MBSim::DynamicSystemSolver {

  public:
    /**
     * \brief Dynamic system of the self-siphoning-beads
     *
     * \param elements Number of elements in the chain
     */
    SelfSiphoningBeats(const std::string &projectName, int elements, double damping = 1e-7);

    /*!
     * \brief add Trajectory for the pre-Integration
     */
    void addTrajectory(double tEnd);

    /*!
     * \brief stupid function, should be removed again
     */
    void addEmptyLeader();

  protected:
    std::vector<MBSim::RigidBody *> balls;

    int elements;

    constexpr static double radius = 1e-3;
    constexpr static double mass = 2e-3;
    constexpr static double distance = 3e-3;
    constexpr static double angle = M_PI / 10;

    constexpr static double stiffness = 1e4;
    constexpr static double damping = 1;

    constexpr static bool ODE = false;

};

#endif /* SYSTEM_H_ */
