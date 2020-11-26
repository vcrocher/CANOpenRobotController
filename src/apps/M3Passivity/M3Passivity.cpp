#include "M3Passivity.h"

#define OWNER ((M3Passivity *)owner)

M3Passivity::M3Passivity() {
    robot = new RobotM3();

    // Create PRE-DESIGNED State Machine events and state objects.
    calibState = new M3CalibState(this, robot);
    impedanceState = new M3DemoImpedanceState(this, robot);
    //minJerkState = new M3DemoMinJerkPosition(this, robot);

    endCalib = new EndCalib(this);
    goToNextState = new GoToNextState(this);


    /**
     * \brief add a tranisition object to the arch list of the first state in the NewTransition MACRO.
     * Effectively creating a statemachine transition from State A to B in the event of event c.
     * NewTranstion(State A,Event c, State B)
     *
     */
     NewTransition(calibState, endCalib, impedanceState);


    //Initialize the state machine with first state of the designed state machine, using baseclass function.
    StateMachine::initialize(calibState);
}
M3Passivity::~M3Passivity() {
    delete UIserver;
    delete robot;
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */
void M3Passivity::init() {
    spdlog::debug("M3Passivity::init()");
    if(robot->initialise()) {
        initialised = true;
        logHelper.initLogger("M3PassivityLog", "logs/M3Passivity.csv", LogFormat::CSV, true);
        logHelper.add(time_running, "Time (s)");
        logHelper.add(robot->getEndEffPositionRef(), "Position");
        logHelper.add(robot->getEndEffVelocityRef(), "Velocity");
        logHelper.add(robot->getEndEffForceRef(), "Force");
        logHelper.add(Power, "Power");
        logHelper.add(Energy, "Energy");
        logHelper.startLogger();
        UIserver = new FLNLHelper(robot, "192.168.7.2");
        /*UIserver->registerState(Power);
        UIserver->registerState(Energy);*/
    }
    else {
        initialised = false;
        spdlog::critical("Failed robot initialisation. Exiting...");
        std::raise(SIGTERM); //Clean exit
    }
    running = true;
    time_init = std::chrono::steady_clock::now();
    time_running = 0;
}

void M3Passivity::end() {
    if(initialised) {
        if(logHelper.isStarted())
            logHelper.endLog();
        UIserver->closeConnection();
        currentState->exit();
        robot->disable();
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
void M3Passivity::hwStateUpdate(void) {
    time_running = (std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - time_init).count()) / 1e6;
    robot->updateRobot();
    UIserver->sendState();
}



bool M3Passivity::EndCalib::check() {
    return OWNER->calibState->isCalibDone();
}


bool M3Passivity::GoToNextState::check() {
    //keyboard or joystick press
    if ( (OWNER->robot->joystick->isButtonPressed(1) || OWNER->robot->keyboard->getNb()==1) )
        return true;

    //Check incoming command requesting state change
    if ( OWNER->UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        OWNER->UIserver->getCmd(cmd, v);
        if (cmd == "GTNS") { //Go To Next State command received
            //Acknowledge
            OWNER->UIserver->sendCmd(string("OK"));

            return true;
        }
    }

    //Otherwise false
    return false;
}
