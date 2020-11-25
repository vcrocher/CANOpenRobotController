/**
 * /file M3DemoState.h
 * \author Vincent Crocher
 * \version 0.1
 * \date 2020-11-25
 *
 * \copyright Copyright (c) 2020
 *
 */

#ifndef M3PASSIVITYSTATE_H_DEF
#define M3PASSIVITYSTATE_H_DEF

#include <time.h>
#include <iostream>

#include "RobotM3.h"
#include "State.h"

using namespace std;

/**
 * \brief Conversion from a timespec structure to seconds (double)
 *
 */
double timeval_to_sec(struct timespec *ts);

/**
 * \brief Generic state type for used with M3DemoMachine, providing running time and iterations number.
 *
 */
class M3TimedState : public State {
   protected:
    /**
    *  \todo Might be good to make these Const
    *
    */
    RobotM3 *robot;                               /*<!Pointer to state machines robot object*/

    M3TimedState(StateMachine *m, RobotM3 *M3, const char *name = NULL): State(m, name), robot(M3) {};
   private:
    void entry(void) final {
        std::cout
        << "==================================" << std::endl
        << " STARTING  " << getName() << std::endl
        << "----------------------------------" << std::endl
        << std::endl;

        //Timing
        clock_gettime(CLOCK_MONOTONIC, &initTime);
        lastTime = timeval_to_sec(&initTime);

        elapsedTime=0;
        iterations=0;

        //Actual state entry
        entryCode();
    };
    void during(void) final {
        //Compute some basic time values
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);

        double now = timeval_to_sec(&ts);
        elapsedTime = (now-timeval_to_sec(&initTime));
        dt = now - lastTime;
        lastTime = now;

        iterations++;

        //Actual state during
        duringCode();
    };
    void exit(void) final {
        exitCode();
        std::cout
        << "----------------------------------" << std::endl
        << "EXIT "<< getName() << std::endl
        << "==================================" << std::endl
        << std::endl;
    };

   public:
    virtual void entryCode(){};
    virtual void duringCode(){};
    virtual void exitCode(){};


   protected:
    struct timespec initTime;   /*!< Time of state init */
    double lastTime;            /*!< Time of last during() call (in seconds since state init())*/
    double elapsedTime;         /*!< Time since state init() in seconds*/
    double dt;                  /*!< Time between last two during() calls (in seconds)*/
    unsigned long int iterations;
};



/**
 * \brief Position calibration of M3. Go to the bottom left stops of robot at constant torque for absolute position calibration.
 *
 */
class M3CalibState : public M3TimedState {

   public:
    M3CalibState(StateMachine *m, RobotM3 *M3, const char *name = "M3 Calib State"):M3TimedState(m, M3, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    bool isCalibDone() {return calibDone;}

   private:
     VM3 stop_reached_time;
     bool at_stop[3];
     bool calibDone=false;

};

/**
 * \brief Impedance control with PO/PC implementations
 *
 */
class M3DemoImpedanceState : public M3TimedState {

   public:
    M3DemoImpedanceState(StateMachine *m, RobotM3 *M3, const char *name = "M3 Demo Impedance State"):M3TimedState(m, M3, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

   private:
    VM3 lastX;
    double xWall = -0.5;    //! Virtual wall position
    double k = 1000;        //! Impedance proportional gain (spring)
    double d = 1;           //! Impedance derivative gain (damper)
    bool init=false;

    unsigned int nb_samples=10000;
    double dts[10000];
    double dX[10000];
    int new_value;
};


/**
 * \brief Go to init point to get ready
 *
 */
class M3DemoMinJerkPosition: public M3TimedState {

   public:
    M3DemoMinJerkPosition(StateMachine *m, RobotM3 *M3, const char *name = "M3 Minimum Jerk Position"):M3TimedState(m, M3, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

   private:
    static const unsigned int TrajNbPts=1;
    unsigned int TrajPtIdx=0;
    double startTime;
    VM3 TrajPt[TrajNbPts]={VM3(-0.60, 0, 0)};
    double TrajTime[TrajNbPts]={4};
    VM3 Xi, Xf;
    double T;
    float k_i=1.; //Integral gain
};


#endif
