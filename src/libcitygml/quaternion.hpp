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
    // ********************************************* //
    //  CONSTRUCTORS
    // ********************************************* //

    /** Empty constructor */
    quaternion();
    /** Direct constructor from quaternion value */
    quaternion(double x,double y,double z,double w);

    // ********************************************* //
    //  ACCESSOR
    // ********************************************* //

    /** Get x coordinate */
    double x() const;
    /** Get/set x coordinate */
    double& x();
    /** Get y coordinate */
    double y() const;
    /** Get/set y coordinate */
    double& y();
    /** Get z coordinate */
    double z() const;
    /** Get/set z coordinate */
    double& z();
    /** Get w coordinate */
    double w() const;
    /** Get/set w coordinate */
    double& w();

    // ********************************************* //
    //  SPECIAL INITIALIZER
    // ********************************************* //

    void set_axis_angle(const TVec3d &axis, const double angle);

    // ********************************************* //
    //  convert to other type
    // ********************************************* //

    /** Convert the quaternion to rotation matrix stored as an array of 9 doubles */
    std::array<double,9> to_rotation_matrix() const;

public:
    TVec4d data;

};

// ********************************************* //
//  Math operation
// ********************************************* //

/** Scalar product between quaternion */
CITYGML_EXPORT double dot(quaternion const& lhs,quaternion const& rhs);
/** Quaternion interpolation */
CITYGML_EXPORT quaternion slerp(quaternion const& q0,quaternion const& q1,double alpha);
/** Quaternion norm */
CITYGML_EXPORT double norm(quaternion const& q);
/** Normalization of the quaternion */
CITYGML_EXPORT quaternion normalized(quaternion const& q);
/** Conjugate of the quaternion (-v,w) */
CITYGML_EXPORT quaternion conjugated(quaternion const& q);

// ********************************************* //
//  Math operator
// ********************************************* //

/** Quaternion addition */
CITYGML_EXPORT quaternion& operator+=(quaternion& lhs,quaternion const& rhs);
/** Quaternion substraction */
CITYGML_EXPORT quaternion& operator-=(quaternion& lhs,quaternion const& rhs);
/** Quaternion multiplication (v1 w2 + v2 w1 - v1xv2 , w1 w2 - v1.v2) */
CITYGML_EXPORT quaternion& operator*=(quaternion& lhs,quaternion const& rhs);
/** Quaternion multiplication by a scalar */
CITYGML_EXPORT quaternion& operator*=(quaternion& q,double s);
/** Quaternion division by a scalar */
CITYGML_EXPORT quaternion& operator/=(quaternion& q,double s);

/** Quaternion addition */
CITYGML_EXPORT quaternion operator+(quaternion const& lhs,quaternion const& rhs);
/** Quaternion substraction */
CITYGML_EXPORT quaternion operator-(quaternion const& lhs,quaternion const& rhs);
/** Quaternion multiplication by a scalar */
CITYGML_EXPORT quaternion operator*(quaternion const& q,double s);
/** Quaternion multiplication by a scalar */
CITYGML_EXPORT quaternion operator*(double s,quaternion const& q);
/** Quaternion division by a scalar */
CITYGML_EXPORT quaternion operator/(quaternion const& q,double s);

/** Unary negation */
CITYGML_EXPORT quaternion operator-(quaternion const& q);

/** Quaternion multiplication (v1 w2 + v2 w1 - v1xv2 , w1 w2 - v1.v2) */
CITYGML_EXPORT quaternion operator*(quaternion const& lhs,quaternion const& rhs);

/** Applying quaternion to vec3 (rotation): q^bar v q */
CITYGML_EXPORT TVec3d operator*(quaternion const& lhs,TVec3d const& rhs);


/** Output the quaternion in ostream as (x,y,z,w) */
CITYGML_EXPORT std::ostream& operator<<(std::ostream& stream,quaternion const& q);

}

#endif // QUATERNION_HPP
