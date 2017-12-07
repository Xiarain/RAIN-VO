//
// Created by rain on 17-12-1.
//

#ifndef RAIN_VIO_SE3_H
#define RAIN_VIO_SE3_H

#include <iostream>

#include <eigen3/Eigen/Dense>

const double SMALL_EPS = 1e-10;

typedef Eigen::Matrix<double, 6, 1, Eigen::ColMajor> Vector6d;
typedef Eigen::Matrix<double, 7, 1, Eigen::ColMajor> Vector7d;

inline Eigen::Vector3d toAngleAxis(const Eigen::Quaterniond& quaterd, double* angle);
inline Eigen::Matrix3d skew(const Eigen::Vector3d &v);
inline Eigen::Quaterniond toQuaterniond(const Eigen::Vector3d& v3d, double* angle);
inline Eigen::Vector3d vee(const Eigen::Matrix3d & Omega);

class SE3
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

protected:
    Eigen::Quaterniond q;
    Eigen::Vector3d t;

public:

    SE3()
    {
      q.setIdentity();
      t.setZero();
    }

    SE3(const Eigen::Matrix3d& R_, const Eigen::Vector3d& t_):q(Eigen::Quaterniond(R_)), t(t_)
    {
        normalize();
    }

    SE3(const Eigen::Quaterniond& q_, const Eigen::Vector3d& t_):q(q_), t(t_)
    {
        normalize();
    }

    inline const Eigen::Vector3d& GetTranslation() const {return t;}

    inline Eigen::Vector3d& GetTranslation() {return t;}

    inline void SetTranslation(const Eigen::Vector3d& t_) {t = t_;}

    inline const Eigen::Quaterniond& GetRotation() const {return q;}

    inline Eigen::Quaterniond& GetRotation() {return q;}

    inline void SetRotation(const Eigen::Quaterniond& q_) {q = q_;}

    inline SE3 operator* (const SE3& tr2) const;

    inline SE3& operator*= (const SE3& tr2);

    inline Eigen::Vector3d operator* (Eigen::Vector3d& v) const;

    inline SE3 inverse() const;

    inline double operator[] (int i) const;

    inline Eigen::Matrix<double, 7, 1, Eigen::ColMajor> toVector() const;

    inline void fromVector(const Eigen::Matrix<double, 7, 1, Eigen::ColMajor>& v);

    inline Eigen::Matrix<double, 6, 1, Eigen::ColMajor> log() const;

    inline Eigen::Vector3d map(const Eigen::Vector3d & xyz) const;

    inline static SE3 exp(const Eigen::Matrix<double, 6, 1, Eigen::ColMajor> & update);

    inline Eigen::Matrix<double, 6, 6, Eigen::ColMajor> adj() const;

    inline Eigen::Matrix4d Hat(const Vector6d & v);

    inline Vector6d Vee(const Eigen::Matrix4d & Omega);

    inline void normalize();
};

inline SE3 SE3::operator*(const SE3 &tr2) const
{
    SE3 result(*this);

    result.t = q*tr2.t + t;
    result.q = q*tr2.q;

    result.normalize();

    return result;
}

inline SE3& SE3::operator*=(const SE3 &tr2)
{
    t = q*tr2.t + t;
    q = q*tr2.q;
    normalize();

    return *this;
}

inline Eigen::Vector3d SE3::operator* (Eigen::Vector3d& v) const
{
    return (q*v + t);
}

inline SE3 SE3::inverse() const
{
    SE3 ret;
    ret.q = q.conjugate();
    ret.t = -1*q.conjugate()*t;
    return ret;
}

inline double SE3::operator[] (int i) const
{
    assert(i<7);

    if (i < 4)
        return q.coeffs()[i];
    return t[i-4];
}

inline Eigen::Matrix<double, 7, 1, Eigen::ColMajor> SE3::toVector() const
{
    Eigen::Matrix<double, 7, 1, Eigen::ColMajor> v;

    v.head<4>() = Eigen::Vector4d(q.coeffs());
    v.tail<3>() = t;

    return v;
}

inline void SE3::fromVector(const Eigen::Matrix<double, 7, 1, Eigen::ColMajor>& v)
{
    q = Eigen::Quaterniond(v[3], v[0], v[1], v[2]);
    t = Eigen::Vector3d(v[4], v[5], v[6]);
}

inline void SE3::normalize()
{
    if (q.w() < 0)
    {
        q.coeffs() *= -1;
    }
    q.normalize();
}

inline Eigen::Matrix3d skew(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d m;
    m.setZero();

    m <<     0, -v(2),  v(1)
       ,  v(2),     0, -v(0)
       , -v(1),  v(0),   0;

    return m;
}

inline Eigen::Vector3d vee(const Eigen::Matrix3d & Omega)
{
    assert(fabs(Omega(2,1)+Omega(1,2))<SMALL_EPS);
    assert(fabs(Omega(0,2)+Omega(2,0))<SMALL_EPS);
    assert(fabs(Omega(1,0)+Omega(0,1))<SMALL_EPS);
    return Eigen::Vector3d(Omega(2,1), Omega(0,2), Omega(1,0));
}

inline Eigen::Matrix<double, 6, 1, Eigen::ColMajor> SE3::log() const
{
    // Computing exponentials of skew symmetric matrices and
    // logarithms of orthogonal matrices
    // p8
    Eigen::Matrix<double, 6, 1, Eigen::ColMajor> res;

    double theta;
    res.head<3>() = toAngleAxis(q, &theta);

    Eigen::Matrix3d Omega = skew(res.head<3>());
    Eigen::Matrix3d V_inv;
    if (theta<SMALL_EPS)
    {
        V_inv = Eigen::Matrix3d::Identity()- 0.5*Omega + (1./12.)*(Omega*Omega);
    }
    else
    {
        V_inv = ( Eigen::Matrix3d::Identity() - 0.5*Omega
                  + ( 1-theta/(2*tan(theta/2)))/(theta*theta)*(Omega*Omega) );
    }

    res.tail<3>() = V_inv*t;

    return res;
}

inline Eigen::Vector3d SE3::map(const Eigen::Vector3d & xyz) const
{
    return q*xyz + t;
}


inline SE3 SE3::exp(const Eigen::Matrix<double, 6, 1, Eigen::ColMajor>& update)
{
    // refer to the state estimation for robotics p251

    Eigen::Vector3d omega(update.data());
    Eigen::Vector3d upsilon(update.data()+3);

    double theta;
    Eigen::Matrix3d Omega = skew(omega);

    Eigen::Quaterniond R = toQuaterniond(omega, &theta);
    Eigen::Matrix3d V;
    if (theta<SMALL_EPS)
    {
        V = R.matrix();
    }
    else
    {
        Eigen::Matrix3d Omega2 = Omega*Omega;

        V = (Eigen::Matrix3d::Identity()
             + (1-cos(theta))/(theta*theta)*Omega
             + (theta-sin(theta))/(pow(theta,3))*Omega2);
    }
    return SE3(R, V*upsilon);
}

inline Eigen::Matrix<double, 6, 6, Eigen::ColMajor> SE3::adj() const
{
    // refer to the state estimation for robotics p251

    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Matrix<double, 6, 6, Eigen::ColMajor> res;
    res.block(0,0,3,3) = R;
    res.block(3,3,3,3) = R;
    res.block(3,0,3,3) = skew(t)*R;
    res.block(0,3,3,3) = Eigen::Matrix3d::Zero(3,3);
    return res;
}

inline Eigen::Matrix4d SE3::Hat(const Vector6d & v)
{
    Eigen::Matrix4d Omega;
    Omega.setZero();
    Omega.topLeftCorner<3,3>() = skew(v.tail<3>());
    Omega.col(3).head<3>() = v.head<3>();
    return Omega;
}

inline Vector6d SE3::Vee(const Eigen::Matrix4d & Omega)
{
    Vector6d upsilon_omega;
    upsilon_omega.head<3>() = Omega.col(3).head<3>();
    upsilon_omega.tail<3>() = vee(Omega.topLeftCorner<3,3>());
    return upsilon_omega;
}

/**
 * @brief convert the quaternion to the angle axis
 * @param quaterd
 * @param angle the theta
 * @return u the unit axis
 */
inline Eigen::Vector3d toAngleAxis(const Eigen::Quaterniond& quaterd, double* angle=NULL)
{
    Eigen::Quaterniond unit_quaternion = quaterd.normalized();
    double n = unit_quaternion.vec().norm();
    double w = unit_quaternion.w();
    double squared_w = w*w;

    double two_atan_nbyw_by_n;
    // Atan-based log thanks to
    //
    // C. Hertzberg et al.:
    // "Integrating Generic Sensor Fusion Algorithms with Sound State
    // Representation through Encapsulation of Manifolds"
    // Information Fusion, 2011
    // page 30, formula 31
    // \phi = u \theta

    if (n < SMALL_EPS) // v = 0;
    {
        // If quaternion is normalized and n=1, then w should be 1;
        // w=0 should never happen here!
        assert(fabs(w)>SMALL_EPS);

        two_atan_nbyw_by_n = 2./w - 2.*(n*n)/(w*squared_w);
    }
    else
    {
        if (fabs(w)<SMALL_EPS) // w = 0;
        {
            if (w>0) // w > 0
            {
                two_atan_nbyw_by_n = M_PI/n;
            }
            else // w < 0
            {
                two_atan_nbyw_by_n = -M_PI/n;
            }
        }
        else
        { // w != 0, v != 0
            two_atan_nbyw_by_n = 2*atan(n/w)/n;
        }

    }

    if(angle!=NULL) *angle = two_atan_nbyw_by_n*n;

    return two_atan_nbyw_by_n * unit_quaternion.vec();
}

/**
 * @brief convert the rotation vector to the quanternion
 * @param v3d
 * @param angle
 * @return
 */
inline Eigen::Quaterniond toQuaterniond(const Eigen::Vector3d& v3d, double* angle = NULL)
{
    // v = \phi u
    // q = [cos(theta/2), u*sin(theta/2)]
    // q = [cos(theta/2), v / \phi * sin(theta/2)]
    double theta = v3d.norm();
    if(angle != NULL)
        *angle = theta;
    double half_theta = 0.5*theta;

    double imag_factor;
    double real_factor = cos(half_theta);

    if(theta<SMALL_EPS)
    {
        double theta_sq = theta*theta;
        double theta_po4 = theta_sq*theta_sq;
        imag_factor = 0.5-0.0208333*theta_sq+0.000260417*theta_po4;// 1.0/48.0, 1.0/384.0
    }
    else
    {
        double sin_half_theta = sin(half_theta);
        imag_factor = sin_half_theta/theta;
    }

    return Eigen::Quaterniond(real_factor,
                              imag_factor*v3d.x(),
                              imag_factor*v3d.y(),
                              imag_factor*v3d.z());
}

inline std::ostream &operator<< (std::ostream &os, const SE3 &SE3_)
{
    os << SE3_.GetTranslation().transpose() << " "
       << SE3_.GetRotation().w() << " "
       << SE3_.GetRotation().x() << " "
       << SE3_.GetRotation().y() << " "
       << SE3_.GetRotation().z();
}

#endif //RAIN_VIO_SE3_H
