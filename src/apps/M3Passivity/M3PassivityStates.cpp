#include "M3PassivityStates.h"
#include "M3Passivity.h"

#define OWNER ((M3Passivity *)owner)

double timeval_to_sec(struct timespec *ts)
{
    return (double)(ts->tv_sec + ts->tv_nsec / 1000000000.0);
}

VM3 impedance(Eigen::Matrix3d K, Eigen::Matrix3d D, VM3 X0, VM3 X, VM3 dX, VM3 dXd=VM3::Zero()) {
    return K*(X0-X) + D*(dXd-dX);
}

//minJerk(X0, Xf, T, t, &X, &dX)
double JerkIt(VM3 X0, VM3 Xf, double T, double t, VM3 &Xd, VM3 &dXd) {
    t = std::max(std::min(t, T), .0); //Bound time
    double tn=std::max(std::min(t/T, 1.0), .0);//Normalised time bounded 0-1
    double tn3=pow(tn,3.);
    double tn4=tn*tn3;
    double tn5=tn*tn4;
    Xd = X0 + ( (X0-Xf) * (15.*tn4-6.*tn5-10.*tn3) );
    dXd = (X0-Xf) * (4.*15.*tn4-5.*6.*tn5-10.*3*tn3)/t;
    return tn;
}

void M3CalibState::entryCode(void) {
    calibDone=false;
    for(unsigned int i=0; i<3; i++) {
        stop_reached_time[i] = .0;
        at_stop[i] = false;
    }
    robot->decalibrate();
    robot->initTorqueControl();
    robot->printJointStatus();
    std::cout << "Calibrating (keep clear)..." << std::flush;
}
//Move slowly on each joint until max force detected
void M3CalibState::duringCode(void) {
    VM3 tau(0, 0, 0);

    //Apply constant torque (with damping) unless stop has been detected for more than 0.5s
    VM3 vel=robot->getVelocity();
    double b = 3.;
    for(unsigned int i=0; i<3; i++) {
        tau(i) = std::min(std::max(2 - b * vel(i), .0), 2.);
        if(stop_reached_time(i)>0.5) {
            at_stop[i]=true;
        }
        if(vel(i)<0.01) {
            stop_reached_time(i) += dt;
        }
    }

    //Switch to gravity control when done
    if(robot->isCalibrated()) {
        robot->setEndEffForceWithCompensation(VM3(0,0,0));
        calibDone=true; //Trigger event
    }
    else {
        //If all joints are calibrated
        if(at_stop[0] && at_stop[1] && at_stop[2]) {
            robot->applyCalibration();
            std::cout << "OK." << std::endl;
        }
        else {
            robot->setJointTorque(tau);
            if(iterations%100==1) {
                std::cout << "." << std::flush;
            }
        }
    }
}
void M3CalibState::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM3(0,0,0));
}


void M3DemoImpedanceState::entryCode(void) {
    robot->initTorqueControl();
    std::cout << "Press Q to select reference point, S/W to tune K gain and A/D for D gain" << std::endl;
    lastX = robot->getEndEffPosition();
    OWNER->Energy = VM3(0,0,0);
}
void M3DemoImpedanceState::duringCode(void) {

    //K tuning
    if(robot->keyboard->getS()) {
        k -= 50;
        std::cout << "K=" << k << " D=" << d<< std::endl;
    }
    if(robot->keyboard->getW()) {
        k += 50;
        std::cout << "K=" << k << " D=" << d<< std::endl;
    }
    Eigen::Matrix3d K = k*Eigen::Matrix3d::Identity();

    //D tuning
    if(robot->keyboard->getD()) {
        d -= 10;
        std::cout << "K=" << k << " D=" << d<< std::endl;
    }
    if(robot->keyboard->getA()) {
        d += 10;
        std::cout << "K=" << k << " D=" << d<< std::endl;
    }
    Eigen::Matrix3d D = d*Eigen::Matrix3d::Identity();


    //Control force
    VM3 Fc(0,0,0);

    //Virtual wall
    VM3 X=robot->getEndEffPosition();
    VM3 Fe(0,0,0);

    OWNER->Power = (X-lastX).cwiseProduct(robot->getEndEffForce());
    lastX = X;
    OWNER->Energy = OWNER->Energy + OWNER->Power;

    if(X[0]>xWall) {
        //Apply wall impedance (spring)
        Fe[0] = k*(xWall-X[0]);
        Fc = Fe;
        //std::cout << "K=" << k << " D=" << d << " => F=" << F << " N" <<std::endl;
    }

    //Apply force
    robot->setEndEffForceWithCompensation(Fc, false);
}
void M3DemoImpedanceState::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM3::Zero());
}



void M3DemoMinJerkPosition::entryCode(void) {
    //Setup velocity control for position over velocity loop
    robot->initVelocityControl();
    robot->setJointVelocity(VM3::Zero());
    //Initialise to first target point
    TrajPtIdx=0;
    startTime=elapsedTime;
    Xi=robot->getEndEffPosition();
    Xf=TrajPt[TrajPtIdx];
    T=TrajTime[TrajPtIdx];
    k_i=1.;
}
void M3DemoMinJerkPosition::duringCode(void) {

    VM3 Xd, dXd;
    //Compute current desired interpolated point
    double status=JerkIt(Xi, Xf, T, elapsedTime-startTime, Xd, dXd);
    //Apply position control
    robot->setEndEffVelocity(dXd+k_i*(Xd-robot->getEndEffPosition()));

    //Display progression
    if(iterations%100==1) {
        std::cout << "Progress (Point "<< TrajPtIdx << ") |";
        for(int i=0; i<round(status*50.); i++)
            std::cout << "=";
        for(int i=0; i<round((1-status)*50.); i++)
            std::cout << "-";

        std::cout << "| (" << status*100 << "%)  ";
        robot->printStatus();
    }
}
void M3DemoMinJerkPosition::exitCode(void) {
    robot->setJointVelocity(VM3::Zero());
}



