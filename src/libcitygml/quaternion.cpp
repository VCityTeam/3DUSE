#include "quaternion.hpp"

#include <cmath>

namespace citygml
{

quaternion::quaternion()
{
    this->x = 0.0;
    this->y = 0.0;
    this->z = 0.0;
    this->w = 0.0;
}

quaternion::quaternion(double x, double y, double z, double w)
{
    this->x = x;
    this->y = y;
    this->z = z;
    this->w = w;
}

quaternion::quaternion(const TVec3d &axis, const double angle)
{
    double sinTheta = std::sin(angle/2.0);
    double cosTheta = std::cos(angle/2.0);
    TVec3d normalAxis = axis.normal();

    this->x = normalAxis.x * sinTheta;
    this->y = normalAxis.y * sinTheta;
    this->z = normalAxis.z * sinTheta;
    this->w = cosTheta;
}

quaternion operator*(const quaternion& q, double s)
{
    quaternion qOut = q;

    qOut.x = qOut.x * s;
    qOut.y = qOut.y * s;
    qOut.z = qOut.z * s;
    qOut.w = qOut.w * s;
}

quaternion operator*(double s, const quaternion& q)
{
    return q*s;
}

TVec3d operator*(quaternion const& q,TVec3d const& vec)
{
    //Compute conjugated quaternion
    quaternion qConj = quaternion(-q.x, -q.y, -q.z, q.w);

    quaternion qTemp = qConj * quaternion(vec.x, vec.y, vec.z, 0.0) * q;

    return TVec3d(qTemp.x, qTemp.y, qTemp.z);
}

quaternion operator*(const quaternion& q1, const quaternion& q2)
{
    return quaternion(q1.x*q2.w + q1.w*q2.x + q1.y*q2.z - q1.z*q2.y,
                      q1.y*q2.w + q1.w*q2.y + q1.z*q2.x - q1.x*q2.z,
                      q1.z*q2.w + q1.w*q2.z + q1.x*q2.y - q1.y*q2.x,
                      q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z);
}

std::ostream& operator<<(std::ostream& stream, const quaternion& q)
{
    stream<<q.x<<" ; "<<q.y<<" ; "<<q.z<<" ; "<<q.w;
    return stream;
}

}

