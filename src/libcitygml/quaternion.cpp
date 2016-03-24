#include "quaternion.hpp"

#include <cmath>

namespace citygml
{
    quaternion::quaternion()
        :data()
    {}

    quaternion::quaternion(double const x,double const y,double const z,double const w)
        :data(x,y,z,w)
    {}

    double quaternion::x() const
    {
        return data.x;
    }
    double& quaternion::x()
    {
        return data.x;
    }

    double quaternion::y() const
    {
        return data.y;
    }
    double& quaternion::y()
    {
        return data.y;
    }

    double quaternion::z() const
    {
        return data.z;
    }
    double& quaternion::z()
    {
        return data.z;
    }

    double quaternion::w() const
    {
        return data.w;
    }
    double& quaternion::w()
    {
        return data.w;
    }

    void quaternion::set_axis_angle(TVec3d const& axis,double const angle)
    {
        double const sin_phi=std::sin(angle/2.0f);
        double const cos_phi=std::cos(angle/2.0f);
        TVec3d const n_axis = axis.normal();

        this->x() = n_axis.x*sin_phi;
        this->y() = n_axis.y*sin_phi;
        this->z() = n_axis.z*sin_phi;
        this->w() =          cos_phi;
    }


    float dot(quaternion const& q0,quaternion const& q1)
    {
        return q0.x()*q1.x()+q0.y()*q1.y()+q0.z()*q1.z()+q0.w()*q1.w();
    }

    quaternion slerp(quaternion const& q0,quaternion const& q1_param,float const alpha)
    {
        if(alpha <= 0)
            return q0;
        if(alpha >= 1)
            return q1_param;

        float cos_omega=dot(q0,q1_param);
        quaternion q1=q1_param;

        if(cos_omega < 0)
        {
            q1=-q1;
            cos_omega=-cos_omega;
        }

        if(cos_omega>0.9999f)
            return (1.0f-alpha)*q0+alpha*q1;

        float const sin_omega=std::sqrt(1.0f-cos_omega*cos_omega);
        float const omega=std::atan2(sin_omega,cos_omega);
        float const one_over_sin_omega=1.0f/sin_omega;

        float const k0 = sin((1.0f-alpha)*omega)*one_over_sin_omega;
        float const k1 = sin(alpha*omega)*one_over_sin_omega;

        return k0*q0+k1*q1;
    }

    quaternion& operator+=(quaternion& lhs,quaternion const& rhs)
    {
        lhs.x() += rhs.x();
        lhs.y() += rhs.y();
        lhs.z() += rhs.z();
        lhs.w() += rhs.w();
        return lhs;
    }

    quaternion& operator-=(quaternion& lhs,quaternion const& rhs)
    {
        lhs.x() -= rhs.x();
        lhs.y() -= rhs.y();
        lhs.z() -= rhs.z();
        lhs.w() -= rhs.w();
        return lhs;
    }

    quaternion& operator*=(quaternion& q,float s)
    {
        q.x() *= s;
        q.y() *= s;
        q.z() *= s;
        q.w() *= s;
        return q;
    }

    quaternion& operator/=(quaternion& q,float s)
    {
        q.x() /= s;
        q.y() /= s;
        q.z() /= s;
        q.w() /= s;
        return q;
    }

    quaternion operator+(quaternion const& lhs,quaternion const& rhs)
    {
        quaternion temp=lhs;
        temp += rhs;
        return temp;
    }

    quaternion operator-(quaternion const& lhs,quaternion const& rhs)
    {
        quaternion temp=lhs;
        temp -= rhs;
        return temp;
    }

    quaternion operator*(quaternion const& q,float s)
    {
        quaternion temp=q;
        temp *= s;
        return temp;
    }

    quaternion operator*(float const s,quaternion const& q)
    {
        return q*s;
    }

    quaternion operator/(quaternion const& q,float s)
    {
        quaternion temp=q;
        temp /= s;
        return temp;
    }

    float norm(quaternion const& q)
    {
        return std::sqrt(q.x()*q.x()+q.y()*q.y()+q.z()*q.z()+q.w()*q.w());
    }

    quaternion normalized(quaternion const& q)
    {
        float const n=norm(q);
        if(std::abs(n)<1e-6f)
            return quaternion(1,0,0,0);
        else
            return q/n;
    }

    quaternion conjugated(quaternion const& q)
    {
        return quaternion(-q.x(),-q.y(),-q.z(),q.w());
    }

    quaternion& operator*=(quaternion& lhs,quaternion const& rhs)
    {
        quaternion temp=lhs*rhs;
        lhs=temp;
        return lhs;
    }

    quaternion operator*(quaternion const& lhs,quaternion const& rhs)
    {
        return quaternion(lhs.x()*rhs.w() + lhs.w()*rhs.x() + lhs.y()*rhs.z() - lhs.z()*rhs.y(),
                          lhs.y()*rhs.w() + lhs.w()*rhs.y() + lhs.z()*rhs.x() - lhs.x()*rhs.z(),
                          lhs.z()*rhs.w() + lhs.w()*rhs.z() + lhs.x()*rhs.y() - lhs.y()*rhs.x(),
                          lhs.w()*rhs.w() - lhs.x()*rhs.x() - lhs.y()*rhs.y() - lhs.z()*rhs.z());
    }

    quaternion operator-(quaternion const& q)
    {
        return quaternion(-q.x(),-q.y(),-q.z(),-q.w());
    }

    TVec3d operator*(quaternion const& lhs,TVec3d const& rhs)
    {
        quaternion q=conjugated(lhs)*quaternion(rhs.x,rhs.y,rhs.z,0.0f)*lhs;
        return TVec3d(q.x(),q.y(),q.z());
    }

    std::array<double,9> quaternion::to_rotation_matrix() const
    {
        float const x2=x()*x();
        float const y2=y()*y();
        float const z2=z()*z();
        float const xy=x()*y();
        float const xz=x()*z();
        float const yz=y()*z();
        float const wx=w()*x();
        float const wy=w()*y();
        float const wz=w()*z();

        std::array<double,9> rotation_matrix = {1.0-2.0*(y2+z2), 2.0*(xy-wz),     2.0*(xz+wy),
                                                2.0f*(xy+wz),    1.0-2.0*(x2+z2), 2.0*(yz-wx),
                                                2.0f*(xz-wy),    2.0*(yz+wx),     1.0-2.0*(x2+y2)};

        return rotation_matrix;
    }


    std::ostream& operator<<(std::ostream& stream,quaternion const& q)
    {
        stream<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w();
        return stream;
    }
}

