#include <math.h>
#include "AHRSTracklet.h"

AHRSTracklet::AHRSTracklet(AHRSMadgwick rotor) {
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    rotor = rotor;
}

AHRSTracklet::AHRSTracklet(AHRSMadgwick rotor, float sampleFrequency) {
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    rotor = rotor;
    begin(sampleFrequency);
}

void AHRSTracklet::update(float gx, float gy, float gz, float ax, float ay, float az, float length) {
    // update AHRS rotor
    rotor.updateIMU(gx, gy, gz, ax, ay, az);

    // the product of rotor's quaternions and the unit quaternions v = (0, 0, 1, 0)
    // the rotated fomular should be q*v*q-1
    // Quaternion <float> rotorQ(rotor.q0, rotor.q1, rotor.q2, rotor.q3);
    // float v[3]= { 0.0, length * 1.0, 0.0 };
    // rotorQ.QuatRotation(v);
    ::boost::math::quaternion<float> q(rotor.q0, rotor.q1, rotor.q2, rotor.q3);
    ::boost::math::quaternion<float> qInverse(rotor.q0, rotor.q1, rotor.q2, rotor.q3);
    ::boost::math::quaternion<float> v(0.0, 0.0, length * 1.0, 0.0);
    q *= v;
    q /= qInverse;

    // accumulate quaternions on the tracklet
    q1 += q.R_component_2();
    q2 += q.R_component_3();
    q3 += q.R_component_4();
}
