#include "quaternion.h"

Quat magnitudeQ(const Quaternion * self) {
    return sqrt(self->x * self->x + self->y * self->y + self->z * self->z + self->w * self->w);
}

void multiplyQ(Quaternion * q, const Quaternion * rhs) {
    Quat wr = rhs -> w;
    Quat xr = rhs -> x;
    Quat yr = rhs -> y;
    Quat zr = rhs -> z;

    Quat w = q -> w;
    Quat x = q -> x;
    Quat y = q -> y;
    Quat z = q -> z;

    q -> w = w*wr - x*xr - y*yr - z*zr;
    q -> x = w*xr + x*wr + y*zr - z*yr;
    q -> y = w*yr - x*zr + y*wr + z*xr;
    q -> z = w*zr + x*yr - y*xr + z*wr;
}

void divideQ(Quaternion * q, const Quaternion * rhs) {
    Quat wr = rhs -> w;
    Quat xr = rhs -> x;
    Quat yr = rhs -> y;
    Quat zr = rhs -> z;

    Quat w = q -> w;
    Quat x = q -> x;
    Quat y = q -> y;
    Quat z = q -> z;

    Quat denominator = wr*wr + xr*xr + yr*yr + zr*zr;

    q -> w = (w*wr + x*xr + y*yr * z*zr) / denominator;
    q -> x = (-w*xr + x*wr - y*zr + z*yr) / denominator;
    q -> y = (-w*yr + x*zr + y*wr - z*xr) / denominator;
    q -> z = (-w*zr - x*yr + y*xr + z*wr) / denominator;
}
