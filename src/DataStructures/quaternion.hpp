// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include <array>
#include "libcitygml/vecs.hpp"
#include "vcitycore_export.h"

#ifdef _MSC_VER
    #pragma warning(disable: 4251) // VC++ DLL jejune complains on STL members
#endif

class VCITYCORE_EXPORT quaternion
{

public:
    ///
    /// \brief quaternion Default Constructor (x = 0.0, y=0.0, z =0.0, w =0.0)
    ///
    quaternion();

    ///
    /// \brief quaternion Constructor given x,y,z,w values
    /// \param x x value
    /// \param y y value
    /// \param z z value
    /// \param w w value
    ///
    quaternion(double x, double y, double z, double w);

    ///
    /// \brief quaternion Construct a quaternion from an axis and an angle
    /// \param axis Axis of rotation
    /// \param angle Angle of rotation
    ///
    quaternion(const TVec3d& axis, const double angle);

public:
    double x, y, z, w;
};

///
/// \brief operator * Multiplication of a quaternion by a scalar
/// \param q quaternion
/// \param s scalar
/// \return result as a quaternion
///
VCITYCORE_EXPORT quaternion operator*(const quaternion& q, double s);

///
/// \brief operator * Multiplication of a quaternion by a scalar
/// \param s scalar
/// \param q quaternion
/// \return result as a quaternion
///
VCITYCORE_EXPORT quaternion operator*(double s, const quaternion& q);

///
/// \brief operator * Multiplication of a quaternion by a vector (applies rotation quaternion to vector)
/// \param q quaternion
/// \param vec vector
/// \return new vector rotated
///
VCITYCORE_EXPORT TVec3d operator*(quaternion const& q,TVec3d const& vec);

///
/// \brief operator * Multiplication of a quaternion by a quaternion
/// \param q1 left side quaternion
/// \param q2 right side quaternion
/// \return result as a quaternion
///
VCITYCORE_EXPORT quaternion operator*(const quaternion& q1, const quaternion& q2);

///
/// \brief operator << print quaternion values
/// \param stream out stream parameter
/// \param q quaternion
/// \return Output stream of quaternion q : "q.x ; q.y ; q.z ; q.w"
///
VCITYCORE_EXPORT std::ostream& operator<<(std::ostream& stream, const quaternion& q);


#endif // QUATERNION_HPP
