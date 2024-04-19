#ifndef POSITION_HPP
#define POSITION_HPP

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include "bigbang/types/coord.hpp"
#include "opencv2/core/matx.hpp"
#include <QtGlobal>
#include <complex.h>

template <typename T>
struct Position_ : public cv::Vec<T, 3> {
    static_assert(std::is_arithmetic<T>(), "Must use arithmetics!");
    Position_(T x = {}, T y = {}, T theta = {}) :
        cv::Vec<T, 3>{x, y, theta}
    {}
    Position_(const cv::Vec<T, 3>& other) : cv::Vec<T, 3>{other} {}
    explicit Position_(const cv::Vec<T, 2>& other) : cv::Vec<T, 3>{other[0], other[1], 0} {}
    Position_(const Position_& other) = default;
    Position_(Position_&& other) = default;
    Position_& operator=(const Position_& other) = default;
    Position_& operator=(Position_&& other) = default;
    friend quint32 qHash(const Position_& pos) noexcept {
        using type = typename QIntegerForSize<sizeof(T)*2>::Signed;
        return qHash(*reinterpret_cast<type*>(&pos)) + qHash(pos.getTheta());
    }
    T getX() const noexcept {return (*this)[0];}
    T getY() const noexcept {return (*this)[1];}
    T getTheta() const noexcept {return (*this)[2];}
    void setX(T val) noexcept {(*this)[0] = val;}
    void setY(T val) noexcept {(*this)[1] = val;}
    void setTheta(T val) {(*this)[2] = val;}
    float look_angle_to(const Coord_<T> &other) const noexcept {
        return normalized_theta(angle_from_x_axis_to(other) - getTheta());
    }
    float angle_from_x_axis_to(const Coord_<T> &other) const noexcept {
        return std::atan2(static_cast<float>(other.getY() - getY()),
                           static_cast<float>(other.getX() - getX()));
    }
    float angle_from_x_axis() const noexcept {
        return std::atan2(static_cast<float>(getY()),
                          static_cast<float>(getX()));
    }
    Position_ rotated_by(float theta) const noexcept {
        auto rotor = std::complex<float>(std::cos(theta), std::sin(theta));
        auto complexResult = rotor * std::complex<float>(static_cast<float>(getX()),
                                                         static_cast<float>(getY()));
        return {complexResult.real(), complexResult.imag(), getTheta() + theta};
    }
    Position_ &apply_delta(const Position_ &delta) noexcept {
        (*this)[2] += delta[2];
        auto deltaCoord = delta.toCoord().rotated_by(getTheta());
        (*this)[0] += deltaCoord[0];
        (*this)[1] += deltaCoord[1];
        return *this;
    }
    Coord_<T> toCoord() const noexcept {
        return {getX(), getY()};
    }
    float dist_to(const Coord_<T> &other) const noexcept {
        return cv::norm(other - toCoord());
    }
    float dist_to(const Position_ &other) const noexcept {
        return cv::norm(other.toCoord() - toCoord());
    }
    float norm() const noexcept {
        return cv::norm(toCoord());
    }
    float fullNorm() const noexcept {
        return cv::norm(*this);
    }
    Position_ normalized() const noexcept {
        auto normed = norm();
        return {getX() / normed, getY() / normed, getTheta()};
    }
};

using Position = Position_<int>;
using PositionF = Position_<float>;
Q_DECLARE_TYPEINFO(Position, Q_MOVABLE_TYPE);
Q_DECLARE_TYPEINFO(PositionF, Q_MOVABLE_TYPE);

#endif
