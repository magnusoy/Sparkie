#ifndef NAVIGATION_H_
#define NAVIGATION_H_

/* Velocity commands from navigation node */
struct navigation {
    float VEL_LINEAR_X;
    float VEL_LINEAR_Y;
    float VEL_LINEAR_Z;

    float VEL_ANGULAR_X;
    float VEL_ANGULAR_Y;
    float VEL_ANGULAR_Z;
};

struct navigation NAVIGATION;



#endif // NAVIGATION_H_