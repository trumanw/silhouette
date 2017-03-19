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
    if (rotor != null) {
        begin(sampleFrequency);
    }
}

AHRSTracklet::update(float gx, float gy, float gz, float ax, float ay, float az, float length) {
    // update AHRS rotor
    rotor.update(gx, gy, gz, ax, ay, az);

    // the product of rotor's quaternions and the unit quaternions (0, 0, 1, 0)
    di = -q3 * length;
    dj = q0 * length;
    dk = q1 * length;

    // new quaternions of tracklet
    q1 += di;
    q2 += dj;
    q3 += dk;
}
