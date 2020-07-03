#include "M3DemoMachine.h"

#define OWNER ((M3DemoMachine *)owner)

M3DemoMachine::M3DemoMachine() {
    robot = new RobotM3();

    // Create PRE-DESIGNED State Machine events and state objects.
    testState = new M3DemoState(this, robot);
    calibState = new M3CalibState(this, robot);
    standbyState = new M3MassCompensation(this, robot);
    endEffDemoState = new M3EndEffDemo(this, robot);
    impedanceState = new M3DemoImpedanceState(this, robot);
    timingState = new M3SamplingEstimationState(this, robot);
    endCalib = new EndCalib(this);

    /**
     * \brief add a tranisition object to the arch list of the first state in the NewTransition MACRO.
     * Effectively creating a statemachine transition from State A to B in the event of event c.
     * NewTranstion(State A,Event c, State B)
     *
     */
    //NewTransition(calibState, endCalib, timingState);
    //NewTransition(calibState, endCalib, standbyState);
    NewTransition(calibState, endCalib, endEffDemoState);

    //Initialize the state machine with first state of the designed state machine, using baseclass function.
    StateMachine::initialize(calibState);
}
M3DemoMachine::~M3DemoMachine() {
    delete testState;
    delete robot;
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */

void M3DemoMachine::init() {
    DEBUG_OUT("M3DemoMachine::init()")
    if(robot->initialise()) {
        initialised = true;
    }
    else {
        initialised = false;
        std::cerr << "Failed robot initialisation. Exiting..." << std::endl;
        std::raise(SIGTERM); //Clean exit
    }
    running = true;
}

void M3DemoMachine::end() {
    if(initialised) {
        currentState->exit();
        robot->stop();
    }
}

////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------
///////////////////////////////////////////////////////////////

/**
 * \brief Statemachine to hardware interface method. Run any hardware update methods
 * that need to run every program loop update cycle.
 *
 */
void M3DemoMachine::hwStateUpdate(void) {
    robot->updateRobot();
}






bool M3DemoMachine::EndCalib::check() {
    return OWNER->calibState->isCalibDone();
}
