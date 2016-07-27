#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include <array>
#include "vecs.hpp"
#include "citygml_export.h"

#ifdef _MSC_VER
    #pragma warning(disable: 4251) // VC++ DLL jejune complains on STL members
#endif

namespace citygml
{

class CITYGML_EXPORT quaternion
{

public:
    quaternion();
    quaternion(double x, double y, double z, double w);
    quaternion(const TVec3d& axis, const double angle);

public:
    double x, y, z, w;
};

CITYGML_EXPORT quaternion operator*(const quaternion& q, double s);
CITYGML_EXPORT quaternion operator*(double s, const quaternion& q);

CITYGML_EXPORT TVec3d operator*(quaternion const& q,TVec3d const& vec);

CITYGML_EXPORT quaternion operator*(const quaternion& q1, const quaternion& q2);

CITYGML_EXPORT std::ostream& operator<<(std::ostream& stream, const quaternion& q);

}

#endif // QUATERNION_HPP
