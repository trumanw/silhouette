#include <math.h>
#include <iostream>
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

void AHRSTracklet::update(float gix, float giy, float giz, float aix, float aiy, float aiz, float length) {
    // update AHRS rotor
    float ax = convertRawAcceleration(aix);
    float ay = convertRawAcceleration(aiy);
    float az = convertRawAcceleration(aiz);
    // float gx = convertRawGyro(gix);
    // float gy = convertRawGyro(giy);
    // float gz = convertRawGyro(giz);
    float gx = gix;
    float gy = giy;
    float gz = giz;

    rotor.updateIMU(gx, gy, gz, ax, ay, az);

    // the product of rotor's quaternions and the unit quaternions v = (0, 0, 1, 0)
    // the rotated fomular should be q*v*q-1
    ::boost::math::quaternion<float> q(rotor.q0, rotor.q1, rotor.q2, rotor.q3);
    ::boost::math::quaternion<float> v(0.0, -1.0, 0.0, 0.0);
    // roll 90 degrees to get the directional vector
    ::boost::math::quaternion<float> yawQ = toQuaternion(0.0, 0.0, 90.0);
    yawQ *= q;

    ::boost::math::quaternion<float> rotor(yawQ.R_component_1(), yawQ.R_component_2(), yawQ.R_component_3(), yawQ.R_component_4());
    ::boost::math::quaternion<float> rotorInverse(yawQ.R_component_1(), yawQ.R_component_2(), yawQ.R_component_3(), yawQ.R_component_4());

    // print the euler angles
    // float q0 = rotor.R_component_1();
    // float q1 = rotor.R_component_2();
    // float q2 = rotor.R_component_3();
    // float q3 = rotor.R_component_4();
    // float roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
	// float pitch = asinf(-2.0f * (q1*q3 - q0*q2));
	// float yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
    // std::cout << "PRY : " << pitch * 57.29578f << " - " << roll * 57.29578f << " - " << yaw * 57.29578f + 180.0f << "\n";

    rotor *= v;
    rotor /= rotorInverse;

    // accumulate quaternions on the tracklet
    q1 += rotor.R_component_2() * length;
    q2 += rotor.R_component_3() + 1;
    q3 += rotor.R_component_4();
}

::boost::math::quaternion<float> AHRSTracklet::toQuaternion(float pitch, float roll, float yaw) {
    ;
	float t0 = std::cos(yaw * 0.5);
	float t1 = std::sin(yaw * 0.5);
	float t2 = std::cos(roll * 0.5);
	float t3 = std::sin(roll * 0.5);
	float t4 = std::cos(pitch * 0.5);
	float t5 = std::sin(pitch * 0.5);

	float qw = t0 * t2 * t4 + t1 * t3 * t5;
	float qx = t0 * t3 * t4 - t1 * t2 * t5;
	float qy = t0 * t2 * t5 + t1 * t3 * t4;
	float qz = t1 * t2 * t4 - t0 * t3 * t5;
    ::boost::math::quaternion<float> q(qw, qx, qy, qz);
	return q;
}

float AHRSTracklet::convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float AHRSTracklet::convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;
  return g;
}
