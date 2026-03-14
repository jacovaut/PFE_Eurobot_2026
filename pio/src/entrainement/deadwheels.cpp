#include "deadwheels.h"

deadwheels::deadwheels(int A_0, int B_0, int A_1, int B_1,  int A_2, int B_2) 
    : A_0(A_0), B_0(B_0), A_1(A_1), B_1(B_1), A_2(A_2), B_2(B_2) {}

void deadwheels::begin() {
    encoder0.attachHalfQuad(A_0, B_0);
    encoder1.attachHalfQuad(A_1, B_1);
    encoder2.attachHalfQuad(A_2, B_2);
    encoder0.setCount(0); // reset counter
    encoder1.setCount(0); // reset counter
    encoder2.setCount(0); // reset counter
}

void deadwheels::deadwheel_odometry(double ticks0, double ticks1, double ticks2, double time){
    //pour le premier appel
    if (!initialized) {
    prevTicks[0] = ticks0;
    prevTicks[1] = ticks1;
    prevTicks[2] = ticks2;
    prevTime = time;
    initialized = true;
    return;
    }

    //calculs des delta ticks
    double dRTicks = ticks0 - prevTicks[0];
    double dLTicks = ticks1 - prevTicks[1];
    double dSTicks = ticks2 - prevTicks[2];
    prevTicks[0] = ticks0;
    prevTicks[1] = ticks1;
    prevTicks[2] = ticks2;

    //calculs des déplacements selon les axes du robot
    double rightDist = dRTicks * DEADWHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION[0];
    double leftDist = dLTicks * DEADWHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION[1];
    double dxr = 0.5 * (rightDist + leftDist);
    double dangle = (rightDist - leftDist) / DEADWHEEL_DISTANCE;
    double dyr = (dSTicks * DEADWHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION[2]) - OFFSET * dangle;
    double avgangle = deadwheelodo.angle + dangle/2; 

    //calculs des déplacements selon le world
    double c = std::cos(avgangle);
    double s = std::sin(avgangle);
    double dx = dxr*c - dyr*s;
    double dy = dyr*c + dxr*s;
    
    //mise à jour de la position du robot
    deadwheelodo.x = deadwheelodo.x + dx;
    deadwheelodo.y = deadwheelodo.y + dy;
    deadwheelodo.angle = normalizeAngleSigned(deadwheelodo.angle + dangle);

    //calculs de vitesse
    double dt = time - prevTime;
    prevTime = time;
    if (dt <= 1e-6) return;
    deadwheelodo.vx = dx/dt;
    deadwheelodo.vy = dy/dt;
    deadwheelodo.vangle = dangle/dt;
    
}

//fonction pour normaliser les angles à l'intervalle [-pi, pi]
double deadwheels::normalizeAngleSigned(double angle) {
    return std::remainder(angle, 2.0 * M_PI);
}


void deadwheels::getCount(int64_t *ticks){ 
    ticks[0] = encoder0.getCount();
    ticks[1] = encoder1.getCount();
    ticks[2] = encoder2.getCount();
}