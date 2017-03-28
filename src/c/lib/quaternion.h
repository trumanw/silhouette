#ifndef QUATERNION_H_
#define QUATERNION_H_

#include "math.h"
#include "stdio.h"

typedef float Quat;
typedef struct Quaternion {
    Quat w;
    Quat x;
    Quat y;
    Quat z;
} Quaternion;

Quat magnitudeQ(const Quaternion *);
void multiplyQ(Quaternion *, const Quaternion *);
void divideQ(Quaternion *, const Quaternion *);

#endif
