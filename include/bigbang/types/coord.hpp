#ifndef COORD_H
#define COORD_H

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include "opencv2/core/matx.hpp"
#include <QtGlobal>
#include <complex.h>

template <typename T>
struct Coord_ : public cv::Vec<T, 2>
{
    static_assert(std::is_arithmetic<T>(), "Must use arithmetics!");
    Coord_(): cv::Vec<T, 2>() {}
    Coord_(const cv::Vec<T, 2> &src) : cv::Vec<T, 2>(src) {}
    Coord_(cv::Vec<T, 2> &&src) : cv::Vec<T, 2>(std::move(src)) {}
    Coord_(T x, T y) : cv::Vec<T, 2>(x, y) {}
    Coord_ &operator=(const Coord_&other) = default;
    T getX() const noexcept{return (*this)[0];}
    T getY() const noexcept{return (*this)[1];}
    void setX(T val) noexcept{return (*this)[0] = val;}
    void setY(T val) noexcept{return (*this)[1] = val;}
    float angle_from_x_axis_to(const Coord_ &other) const {
        return std::atan2(static_cast<float>(other.getY() - getY()),
                           static_cast<float>(other.getX() - getX()));
    }
    float angle_from_x_axis() const {
        return std::atan2(static_cast<float>(getY()),
                          static_cast<float>(getX()));
    }
    Coord_ rotated_by(float theta) const noexcept {
        auto rotor = std::complex<float>(std::cos(theta), std::sin(theta));
        auto complexResult = rotor * std::complex<float>(static_cast<float>(getX()),
                                                         static_cast<float>(getY()));
        return {complexResult.real(), complexResult.imag()};
    }
    float dist_to(const Coord_ &other) const noexcept{
        return cv::norm(other - *this);
    }
    float norm() const noexcept{
        return cv::norm(*this);
    }
    Coord_ normalized() const noexcept{
        return {getX() / norm(), getY() / norm()};
    }
    Coord_<int> rounded() const noexcept{
        return {static_cast<int>(std::round(getX())),
                static_cast<int>(std::round(getY()))};
    }
    friend quint32 qHash(const Coord_& coord) noexcept {
        using type = typename QIntegerForSize<sizeof(T)*2>::Signed;
        return qHash(*reinterpret_cast<const type*>(&coord));
    }
};
typedef Coord_<int> Coord;
typedef Coord_<float> CoordF;
namespace std {
  template <typename T> struct hash<Coord_<T>>
  {
    size_t operator()(const Coord_<T> & x) const
    {
        using type = typename QIntegerForSize<sizeof(T)*2>::Signed;
        return hash<type>()(*reinterpret_cast<const type*>(&x));
    }
  };
}
Q_DECLARE_TYPEINFO(Coord, Q_MOVABLE_TYPE);
Q_DECLARE_TYPEINFO(CoordF, Q_MOVABLE_TYPE);

inline float normalized_theta(float theta) noexcept {
    if (abs(theta) > M_PI * 2)
        theta = std::remainder(theta, M_PI * 2);
    if (theta > M_PI) {
        theta -= M_PI * 2;
    } else if (theta < -M_PI) {
        theta += M_PI * 2;
    }
    return theta;
}

inline constexpr float toRadians(float degrees) noexcept {
    return degrees * (M_PI / 180);
}

inline constexpr float toDegrees(float radians) noexcept {
    return radians / (M_PI * 2) * 360;
}

#endif
