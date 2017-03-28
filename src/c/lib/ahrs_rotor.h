#ifndef AHRS_ROTOR_H_
#define AHRS_ROTOR_H_

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "quaternion.h"

typedef struct AHRSRotor {
    // quaternion accumulated from sensors
    float q0;
    float q1;
    float q2;
    float q3;
    float beta;     // algorithm gain
    float invSampleFreq;
    float roll;
    float pitch;
    float yaw;

    bool isSlerp;
    int slerpNum;
    float *wSlerp;
    float *xSlerp;
    float *ySlerp;
    float *zSlerp;

    void (*begin)(struct AHRSRotor *, float freq);
    void (*update)(struct AHRSRotor *, float gx, float gy, float gz, float ax, float ay, float az);
    float (*getPitch)(struct AHRSRotor *);
    float (*getRoll)(struct AHRSRotor *);
    float (*getYaw)(struct AHRSRotor *);
    float (*invSqrt)(float x);

} AHRSRotor;

AHRSRotor * newAHRSRotor();
AHRSRotor * newAHRSSlerpRotor(int num);
void deleteAHRSRotor(AHRSRotor *);
float invSqrt(float x);
void beginRotorSetup(AHRSRotor *self, float sampleFrequency);
void updateRotorQuaternions(struct AHRSRotor *self, float gx, float gy, float gz, float ax, float ay, float az);
float getPitch(struct AHRSRotor *self);
float getRoll(struct AHRSRotor *self);
float getYaw(struct AHRSRotor *self);

#endif
