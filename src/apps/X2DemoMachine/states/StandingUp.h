/**
 * /file StandingUp.h
 * /author Justin Fong
 * /version 0.1
 * /date 2020-05-07
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef STANDINGUP_H_INCLUDED
#define STANDINGUP_H_INCLUDED

#include "State.h"
#include "X2Robot.h"
/**
 * @brief State for the ExoTestMachine (implementing ExoTestState) - representing when the exo is standing up (moving)
 * 
 * Starts the Standing Up trajectory on entry, executes in during, and exits when trajectory is complete
 */
class StandingUp : public State {
   private:
    /** 
    * Parameters associated with progression through a trajectory
    */
    double currTrajProgress = 0;
    timespec prevTime;
    X2Robot *robot;

   public:
    bool trajFinished = false;

    /**
    * \brief Prepare Robot and Trajectory Generator objects to tigger a stand motion
    * loads STAND paramaters into the Trajectory Generator object and runs robot startNewTrajectory function.
    * 
    */
    void entry(void);
    /**
     * \brief run the robot objects moveThroughtrajecoty function using the loaded trajectory
     * dictated by the state machines Trajectory Generator object.
     * 
     */
    void during(void);
    void exit(void);
    StandingUp(StateMachine *m, X2Robot *exo, const char *name = NULL) : State(m, name), robot(exo){};
};

#endif