#pragma once

#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <common/coord.hpp>
#include <common/position.hpp>

using VecT = cv::Vec<float, 3>;
using CovT = cv::Matx<float, 3, 3>;

struct MeasurePair {
    VecT vec;
    CovT cov;
};

inline bool mergeMeasures(MeasurePair &target, const MeasurePair &source, cv::DecompTypes decomp)
{
    bool invOk = false;
    auto inv = (target.cov + source.cov).inv(decomp, &invOk);
    if (!invOk) {
        ROS_WARN("Invertion fail!");
        return false;
    }
    target.vec = inv * (target.cov * source.vec + source.cov * target.vec);
    bool invFirst, invSecond, invFinal = false;
    target.cov = (target.cov.inv(decomp, &invFirst) + source.cov.inv(decomp, &invSecond)).inv(decomp, &invFinal);
    if (!(invFirst && invSecond && invFinal)) {
        ROS_WARN_STREAM("Inv fail: First: " << invFirst << "; Second: " << invSecond << "; Final: " << invFinal);
        return false;
    }
    return true;
}

template<typename T, int n>
void checkCovarianceDiagMin(cv::Matx<T, n, n> &mat, T minUncerrtainty) {
    for(int i = 0; i < n; ++i) {
        mat(i,i) = mat(i,i) < minUncerrtainty ? minUncerrtainty : mat(i, i);
    }
}

template<typename T, int n>
void checkCovarianceMax(cv::Matx<T, n, n> &mat, T maxUncerrtainty) {
    for(int i = 0; i < n*n; ++i) {
        auto& ref = mat(i/n,i%n);
        ref = ref > maxUncerrtainty ? maxUncerrtainty : ref;
    }
}

template<typename T, int n>
void fillDiagOf(cv::Matx<T, n, n> &mat, T value) {
    for(int i = 0; i < n*n; ++i) {
        mat(i/n,i%n) = value;
    }
}

