#include "ahrs_tracklet.h"

AHRSTracklet * newAHRSTracklet() {
    AHRSTracklet * tracklet = (AHRSTracklet *) malloc(sizeof(AHRSTracklet));

    AHRSRotor * rotor = newAHRSRotor();
    tracklet -> rotor = rotor;
    tracklet -> q1 = 0.0f;
    tracklet -> q2 = 0.0f;
    tracklet -> q3 = 0.0f;

    tracklet -> isSlerp = false;
    tracklet -> slerpNum = 0;

    tracklet -> update = updateTrackletQuaternions;
    tracklet -> getX = getX;
    tracklet -> getY = getY;
    tracklet -> getZ = getZ;

    return tracklet;
}

AHRSTracklet * newAHRSSlerpTracklet(int intplNum) {
    AHRSTracklet * tracklet = (AHRSTracklet *) malloc(sizeof(AHRSTracklet));

    AHRSRotor * rotor = newAHRSSlerpRotor(intplNum);
    tracklet -> rotor = rotor;
    tracklet -> q1 = 0.0f;
    tracklet -> q2 = 0.0f;
    tracklet -> q3 = 0.0f;

    tracklet -> isSlerp = true;
    tracklet -> slerpNum = intplNum;
    tracklet -> xSlerp = (float *)calloc(intplNum, sizeof(float));
	tracklet -> ySlerp = (float *)calloc(intplNum, sizeof(float));
	tracklet -> zSlerp = (float *)calloc(intplNum, sizeof(float));

    tracklet -> update = updateTrackletQuaternions;
    tracklet -> getX = getX;
    tracklet -> getY = getY;
    tracklet -> getZ = getZ;

    return tracklet;
}

void deleteAHRSTracklet(AHRSTracklet * tracklet) {
    if (!(tracklet -> rotor)) {
        deleteAHRSRotor(tracklet -> rotor);
    }
    if (tracklet -> isSlerp) {
        free(tracklet -> xSlerp);
        free(tracklet -> ySlerp);
        free(tracklet -> zSlerp);
    }
    free(tracklet);
}

void updateTrackletQuaternions(struct AHRSTracklet * self, float gx, float gy, float gz, float ax, float ay, float az, float length) {
    // update AHRS rotor
    self -> rotor -> update(self -> rotor, gx, gy, gz, ax, ay, az);
    if (self -> isSlerp) {
        int total = self -> slerpNum;
        self -> xSlerp[0] = self -> xSlerp[total - 1];
        self -> ySlerp[0] = self -> ySlerp[total - 1];
        self -> zSlerp[0] = self -> zSlerp[total - 1];
        for (int i = 1; i < self -> slerpNum; i++) {
            float wSub = self -> rotor -> wSlerp[i];
            float xSub = self -> rotor -> xSlerp[i];
            float ySub = self -> rotor -> ySlerp[i];
            float zSub = self -> rotor -> zSlerp[i];
            Quaternion q = {wSub, xSub, ySub, zSub};
            Quaternion qInvs = {wSub, xSub, ySub, zSub};
            Quaternion uv = {0.0, 0.707*length/(total-1), 0.707*length/(total-1), 0.0};

            // calculate the result of q*uv*q-1
            multiplyQ(&q, &uv);
            divideQ(&q, &qInvs);

            Quaternion yaw = {0.924, 0.0, 0.0, 0.383};
            Quaternion yawInvs = {0.924, 0.0, 0.0, 0.383};

            // calculate the result of yaw*(q*uv*q-1)*yaw-1
            multiplyQ(&yaw, &q);
            divideQ(&yaw, &yawInvs);

            self -> xSlerp[i] = self -> xSlerp[i-1] + yaw.x;
            self -> ySlerp[i] = self -> ySlerp[i-1] + yaw.y;
            self -> zSlerp[i] = self -> zSlerp[i-1] + yaw.z;
        }

        // accumulate quaternions gain on the tracklet
        self -> q1 = self -> xSlerp[total - 1];
        self -> q2 = self -> ySlerp[total - 1];
        self -> q3 = self -> zSlerp[total - 1];
    } else {
        float w = self -> rotor -> q0;
        float x = self -> rotor -> q1;
        float y = self -> rotor -> q2;
        float z = self -> rotor -> q3;
        Quaternion q = {w, x, y, z};
        Quaternion qInvs = {w, x, y, z};
        Quaternion uv = {0.0, 0.707*length, 0.707*length, 0.0};

        // calculate the result of q*uv*q-1
        multiplyQ(&q, &uv);
        divideQ(&q, &qInvs);

        Quaternion yaw = {0.924, 0.0, 0.0, 0.383};
        Quaternion yawInvs = {0.924, 0.0, 0.0, 0.383};

        // calculate the result of yaw*(q*uv*q-1)*yaw-1
        multiplyQ(&yaw, &q);
        divideQ(&yaw, &yawInvs);

        // accumulate quaternions gain on the tracklet
        self -> q1 += yaw.x;
        self -> q2 += yaw.y;
        self -> q3 += yaw.z;
    }
}

float getX(struct AHRSTracklet * self) {
    return self -> q1;
}

float getY(struct AHRSTracklet * self) {
    return self -> q2;
}

float getZ(struct AHRSTracklet * self) {
    return self -> q3;
}
