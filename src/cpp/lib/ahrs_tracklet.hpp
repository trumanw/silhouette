#ifndef AHRSTRACKLET_H_
#define AHRSTRACKLET_H_
#include <math.h>
#include <iostream>
#include <boost/math/quaternion.hpp>
#include "ahrs_rotor.hpp"
using ::boost::math::quaternion;

class AHRSTracklet{
private:
    AHRSRotor rotor;
    float invSampleFreq;
    // a pure quaternion for tracklet
    float q1;
    float q2;
    float q3;

public:
    AHRSTracklet(AHRSRotor rotor);
    AHRSTracklet(AHRSRotor rotor, float sampleFrequency);
    void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; rotor.begin(sampleFrequency); }
    void update(float gx, float gy, float gz, float ax, float ay, float az, float length);
    float getX() { return q1; }
    float getY() { return q2; }
    float getZ() { return q3; }
    float convertRawAcceleration(int aRaw);
    float convertRawGyro(int gRaw);
    void computeAngles(float q0, float q1, float q2, float q3);
};
#endif
