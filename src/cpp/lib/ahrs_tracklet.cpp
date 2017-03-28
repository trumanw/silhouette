#include <math.h>
#include "ahrs_tracklet.hpp"

AHRSTracklet::AHRSTracklet(AHRSRotor rotor) {
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    rotor = rotor;
}

AHRSTracklet::AHRSTracklet(AHRSRotor rotor, float sampleFrequency) {
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
    // 1. rotate the unit quaternions on x-aixs using sensors' data:
    // quaternions rotation: q*uv*q-1
    // 2. yaw the rotated unit quaternions 90 degrees to be the unit quaternions on y-axis (in object frame)
    // quaternions rotation: yaw*(q*uv*q-1)*yaw-1

    quaternion<float> q(rotor.q0, rotor.q1, rotor.q2, rotor.q3);
    quaternion<float> qInvs(rotor.q0, rotor.q1, rotor.q2, rotor.q3);
    quaternion<float> uv(0.0, 0.707*length, 0.707*length, 0.0);
    q *= uv;
    q /= qInvs;

    quaternion<float> yaw(0.924, 0.0, 0.0, 0.383);
    quaternion<float> yawInvs(0.924, 0.0, 0.0, 0.383);

    // float rq0 = rotor.q0;
    // float rq1 = rotor.q1;
    // float rq2 = rotor.q2;
    // float rq3 = rotor.q3;
    // computeAngles(rq0, rq1, rq2, rq3);
    // std::cout << "Rotor PRY: " << rotor.getPitch() << "," << rotor.getRoll() << "," << rotor.getYaw() << "\n";
    // std::cout << "\n";

    yaw *= q;
    yaw /= yawInvs;

    // accumulate quaternions on the tracklet
    q1 += yaw.R_component_2();
    q2 += yaw.R_component_3();
    q3 += yaw.R_component_4();
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

void AHRSTracklet::computeAngles(float q0, float q1, float q2, float q3) {
	double ysqr = q2 * q2;

	// roll (x-axis rotation)
	double t0 = +2.0 * (q1 * q0 + q2 * q3);
	double t1 = +1.0 - 2.0 * (q1 * q1 + ysqr);
	float roll = atan2(t0, t1);

	// pitch (y-axis rotation)
	double t2 = +2.0 * (q0 * q2 - q3 * q1);
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	float pitch = asin(t2);

	// yaw (z-axis rotation)
	double t3 = +2.0 * (q0 * q3 + q1 * q2);
	double t4 = +1.0 - 2.0 * (ysqr + q3 * q3);
	float yaw = atan2(t3, t4);
    std::cout << "PRY: " << pitch*57.29578f << "," << roll*57.29578f << "," << yaw*57.29578f << "\n";
}
