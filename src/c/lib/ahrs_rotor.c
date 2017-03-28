#include "ahrs_rotor.h"

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	25.0f		// sample frequency in Hz
#define betaDef		0.1f		// 2 * proportional gain

//====================================================================================================
// Functions
AHRSRotor * newAHRSRotor() {
	AHRSRotor *rotor = (AHRSRotor *) malloc(sizeof(AHRSRotor));

	rotor -> q0 = 1.0f;
	rotor -> q1 = 0.0f;
	rotor -> q2 = 0.0f;
	rotor -> q3 = 0.0f;
	rotor -> isSlerp = false;
	rotor -> slerpNum = 0;
	rotor -> beta = betaDef;

	rotor -> begin = beginRotorSetup;
	rotor -> update = updateRotorQuaternions;
	rotor -> getPitch = getPitch;
	rotor -> getRoll = getRoll;
	rotor -> getYaw = getYaw;
	rotor -> invSqrt = invSqrt;

	return rotor;
}

AHRSRotor * newAHRSSlerpRotor(int num) {
	AHRSRotor * rotor = newAHRSRotor();

	rotor -> isSlerp = true;
	rotor -> slerpNum = num;
	rotor -> wSlerp = (float *)calloc(num, sizeof(float));
	rotor -> xSlerp = (float *)calloc(num, sizeof(float));
	rotor -> ySlerp = (float *)calloc(num, sizeof(float));
	rotor -> zSlerp = (float *)calloc(num, sizeof(float));

	return rotor;
}

void deleteAHRSRotor(AHRSRotor * rotor) {
	if (rotor -> isSlerp) {
		free(rotor -> wSlerp);
		free(rotor -> xSlerp);
		free(rotor -> ySlerp);
		free(rotor -> zSlerp);
	}
	free(rotor);
}

void beginRotorSetup(AHRSRotor *self, float sampleFrequency) {
	// only needs to keep inversed sampling frequency
	self -> invSampleFreq = 1.0f / sampleFrequency;
}


float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void updateRotorQuaternions(struct AHRSRotor *self, float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Init q0,1,2,3 from rotor struct
	float q0 = self -> q0;
	float q1 = self -> q1;
	float q2 = self -> q2;
	float q3 = self -> q3;

	// keep the previous quaternion for SLERP
	Quaternion qStart = {q0, q1, q2, q3};

	// Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = self -> invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = self -> invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= self -> beta * s0;
		qDot2 -= self -> beta * s1;
		qDot3 -= self -> beta * s2;
		qDot4 -= self -> beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = self -> invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	// Update the value of quaternions back to the rotor struct
	self -> q0 = q0;
	self -> q1 = q1;
	self -> q2 = q2;
	self -> q3 = q3;

	// keep the current quaternion for SLERP
	Quaternion qEnd = {q0, q1, q2, q3};

	// start to SLERP all the sub quaternions between qStart and qEnd
	if (self -> isSlerp) {
		// init start and end quaternions for SLERP
		int total = self -> slerpNum;
		self -> wSlerp[0] = qStart.w;
		self -> xSlerp[0] = qStart.x;
		self -> ySlerp[0] = qStart.y;
		self -> zSlerp[0] = qStart.z;

		// calculate the dot product of the start and the end quaternions
		float dot = qStart.w*qEnd.w + qStart.x*qEnd.x + qStart.y*qEnd.y + qStart.z*qEnd.z;
		Quaternion qEndInv = {qEnd.w, qEnd.x, qEnd.y, qEnd.z};
		// if (dot < 0), q1 and q2 are more than 90 degrees apart,
		// so we can invert one to reduce spinning
		if (dot < 0) {
			dot = -dot;
			qEndInv.w = -qEnd.w;
			qEndInv.x = -qEnd.x;
			qEndInv.y = -qEnd.y;
			qEndInv.z = -qEnd.z;
		}
		float angle = acosf(dot);
		float denominator = sinf(angle);

		for (int i = 1; i < total - 1; i++) {
			// SLERP the sub-quaternions
			float t = i / total;
			self -> wSlerp[i] = (qStart.w * sinf(angle * (1-t)) + qEnd.w * sinf(angle * t)) / denominator;
			self -> xSlerp[i] = (qStart.x * sinf(angle * (1-t)) + qEnd.x * sinf(angle * t)) / denominator;
			self -> ySlerp[i] = (qStart.y * sinf(angle * (1-t)) + qEnd.y * sinf(angle * t)) / denominator;
			self -> zSlerp[i] = (qStart.z * sinf(angle * (1-t)) + qEnd.z * sinf(angle * t)) / denominator;
		}

		// keep the last rotating quaternion
		self -> wSlerp[total - 1] = qEndInv.w;
		self -> xSlerp[total - 1] = qEndInv.x;
		self -> ySlerp[total - 1] = qEndInv.y;
		self -> zSlerp[total - 1] = qEndInv.z;
	}
}

float getPitch(struct AHRSRotor *self) {
	return 0;
}

float getRoll(struct AHRSRotor *self) {
	return 0;
}

float getYaw(struct AHRSRotor *self) {
	return 0;
}
