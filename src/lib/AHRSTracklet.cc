#include "AHRSTracklet.h"
#include <math.h>

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

    // the product of rotor's quaternions and the unit quaternions (0, 0, 1, 0)
    // float di = -rotor.q3 * length;
    // float dj = rotor.q0 * length;
    // float dk = rotor.q1 * length;
    float di = -rotor.q3;
    float dj = rotor.q0;
    float dk = rotor.q1;

    // new quaternions of tracklet
    q1 += di;
    q2 += dj;
    q3 += dk;
}
