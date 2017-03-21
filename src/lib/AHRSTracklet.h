#ifndef AHRSTRACKLET_H_
#define AHRSTRACKLET_H_
#include <math.h>
#include <iostream>
#include <boost/math/quaternion.hpp>
#include "AHRSMadgwick.h"

class AHRSTracklet{
private:
    AHRSMadgwick rotor;
    float invSampleFreq;
    // a pure quaternion for tracklet
    float q1;
    float q2;
    float q3;

public:
    AHRSTracklet(AHRSMadgwick rotor);
    AHRSTracklet(AHRSMadgwick rotor, float sampleFrequency);
    void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; rotor.begin(sampleFrequency); }
    void update(float gx, float gy, float gz, float ax, float ay, float az, float length);
    float getX() { return q1; }
    float getY() { return q2; }
    float getZ() { return q3; }
};
#endif
