#ifndef AHRS_TRACKLET_H_
#define AHRS_TRACKLET_H_

#include <math.h>
#include <stdio.h>
#include "ahrs_rotor.h"
#include "quaternion.h"

typedef struct AHRSTracklet {
    // a pure quaternion for tracklet
    float q1;
    float q2;
    float q3;

    AHRSRotor *rotor;

    bool isSlerp;
    int slerpNum;
    float *xSlerp;
    float *ySlerp;
    float *zSlerp;

    void (*update)(struct AHRSTracklet *, float gx, float gy, float gz, float ax, float ay, float az, float length);
    float (*getX)(struct AHRSTracklet *);
    float (*getY)(struct AHRSTracklet *);
    float (*getZ)(struct AHRSTracklet *);
} AHRSTracklet;

AHRSTracklet * newAHRSTracklet();
AHRSTracklet * newAHRSSlerpTracklet(int intplNum);
void deleteAHRSTracklet(AHRSTracklet *);
void updateTrackletQuaternions(struct AHRSTracklet * self, float gx, float gy, float gz, float ax, float ay, float az, float length);
float getX(struct AHRSTracklet * self);
float getY(struct AHRSTracklet * self);
float getZ(struct AHRSTracklet * self);

#endif
