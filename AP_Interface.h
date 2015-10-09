#ifndef AP_INTF_H
#define AP_INTF_H

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include <type_traits>
#include <AP_Math/AP_Math.h>

/*
 * ArduPilot compatibility typedefs using the Eigen library
 * author: Daniel Frenzel <dgdanielf@gmail.com>
 */
namespace IntfDef {
    template <class T>
    using Vector2 = Eigen::Matrix<T, 2, 1>;
    template <class T>
    using Vector3 = Eigen::Matrix<T, 3, 1>;
    template <class T>
    using Matrix3 = Eigen::Matrix<T, 3, 3>;
}

namespace IntfImp {
    template <class T>
    class VectorE2 : public IntfDef::Vector2<T> {
    public:
        const T &x() const { return (*this)[0]; }
        const T &y() const { return (*this)[1]; }

        template <class OtherDerived>
        VectorE2(const Eigen::MatrixBase<OtherDerived>& other)  : IntfDef::Vector2<T>(other) { }
        template <class OtherDerived>
        VectorE2(const Vector2<OtherDerived> &v)                : IntfDef::Vector2<T>(v.x, v.y) { }
        VectorE2(const T &x, const T &y)                        : IntfDef::Vector2<T>(x, y) { }


        template <class OtherDerived>
        operator Vector2<OtherDerived>() const {
            Vector2<OtherDerived> v(x(), y() );
            if(std::is_integral<OtherDerived>::value && !std::is_integral<T>::value) {
                OtherDerived vx = roundf(x() );
                OtherDerived vy = roundf(y() );
                v = Vector2<OtherDerived>(vx, vy);
            }
            return v;
        }
        VectorE2<T> &operator=(const VectorE2<T> &other) {
            if(this != &other) {
                (*this)[0] = other.x();
                (*this)[1] = other.y();
            }
            return *this;
        }
        VectorE2<T> &operator=(const Vector2<T> &other) {
            if(this != &other) {
                (*this)[0] = other.x;
                (*this)[1] = other.y;
            }
            return *this;
        }
        
        float length() const {
            float sum = powf(x, 2) + powf(y, 2);
            return sqrt(sum);
        }
        bool is_nan(void) {
            return isnan(x() ) || isnan(y() );
        }
        bool is_inf(void) {
            return isinf(x() ) || isinf(y() );
        }
        float angle(const VectorE2<T> &v) const {
            VectorE2<T> v1 = this->normalized();
            VectorE2<T> v2 = v.normalized();
            return acosf(v1.dot(v2) );
        }
    };

    template <class T>
    class VectorE3 : public IntfDef::Vector3<T> {
    public:
        const T &x() const { return (*this)[0]; }
        const T &y() const { return (*this)[1]; }
        const T &z() const { return (*this)[2]; }

        template <class OtherDerived>
        VectorE3(const Eigen::MatrixBase<OtherDerived>& other)   : IntfDef::Vector3<T>(other) { }
        template <class OtherDerived>
        VectorE3(const Vector3<OtherDerived> &v)                 : IntfDef::Vector3<T>(v.x, v.y, v.z) { }
        VectorE3(const T &x, const T &y, const T &z)             : IntfDef::Vector3<T>(x, y, z) { }


        template <class OtherDerived>
        operator Vector3<OtherDerived>() const {
            Vector3<OtherDerived> v(x(), y(), z() );
            if(std::is_integral<OtherDerived>::value && !std::is_integral<T>::value) {
                OtherDerived vx = roundf(x() );
                OtherDerived vy = roundf(y() );
                OtherDerived vz = roundf(z() );
                v = Vector3<OtherDerived>(vx, vy, vz);
            }
            return v;
        }
        VectorE3<T> &operator=(const VectorE3<T> &other) {
            if(this != &other) {
                (*this)[0] = other.x();
                (*this)[1] = other.y();
                (*this)[2] = other.z();
            }
            return *this;
        }
        VectorE3<T> &operator=(const Vector3<T> &other) {
            if(this != &other) {
                (*this)[0] = other.x;
                (*this)[1] = other.y;
                (*this)[2] = other.z;
            }
            return *this;
        }

        float length() const {
            float sum = powf(x(), 2) + powf(y(), 2) + powf(z(), 2);
            return sqrt(sum);
        }
        bool is_nan(void) {
            return isnanf(x() ) || isnanf(y() ) || isnanf(z() );
        }
        bool is_inf(void) {
            return isinf(x() ) || isinf(y() ) || isinf(z() );
        }
        float angle(const VectorE3<T> &v) const {
            VectorE3<T> v1 = this->normalized();
            VectorE3<T> v2 = v.normalized();
            return acosf(v1.dot(v2) );
        }
    };

    template <class T>
    class MatrixE3 : public IntfDef::Matrix3<T> {
    public:
        template <class OtherDerived>
        MatrixE3(const Eigen::MatrixBase<OtherDerived>& other) : IntfDef::Matrix3<T>(other) { }
        template <class OtherDerived>
        MatrixE3(const Matrix3<OtherDerived>& m)               : IntfDef::Matrix3<T>() {
            (*this) <<  m.a.x, m.a.y, m.a.z,
                        m.b.x, m.b.y, m.b.z,
                        m.c.x, m.c.y, m.c.z;
        }
        MatrixE3(T ax, T ay, T az, T bx, T by, T bz, T cx, T cy, T cz) : IntfDef::Matrix3<T>() {          
            (*this) <<  ax, ay, az,
                        bx, by, bz,
                        cx, cy, cz;
        }
    };
};

/**
 * @brief Convinient types
 */
namespace AP_Eigen {
    typedef IntfImp::VectorE2<float>    Vector2f;
    typedef IntfImp::VectorE2<double>   Vector2d;
    typedef IntfImp::VectorE2<int>      Vector2i;
    typedef IntfImp::VectorE2<long>     Vector2l;

    typedef IntfImp::VectorE3<float>    Vector3f;
    typedef IntfImp::VectorE3<double>   Vector3d;
    typedef IntfImp::VectorE3<int>      Vector3i;
    typedef IntfImp::VectorE3<long>     Vector3l;

    typedef IntfImp::MatrixE3<float>    Matrix3f;
    typedef IntfImp::MatrixE3<double>   Matrix3d;
    typedef IntfImp::MatrixE3<int>      Matrix3i;
    typedef IntfImp::MatrixE3<long>     Matrix3l;
}

#endif //AP_INTF_H