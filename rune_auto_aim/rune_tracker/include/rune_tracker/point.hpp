#pragma once

#include <limits>

#include "mathematics.hpp"
#include "statistic.hpp"

namespace rune {

template<typename TLhs, typename TRhs, typename TResult = CommonType<TLhs, TRhs>>
cv::Point_<TResult> operator+(const cv::Point_<TLhs>& lhs, const cv::Point_<TRhs>& rhs) {
    return { lhs.x + rhs.x, lhs.y + rhs.y };
}

template<typename TLhs, typename TRhs, typename TResult = CommonType<TLhs, TRhs>>
cv::Point_<TResult> operator-(const cv::Point_<TLhs>& lhs, const cv::Point_<TRhs>& rhs) {
    return { lhs.x - rhs.x, lhs.y - rhs.y };
}

template<typename TLhs, typename TRhs, typename TResult = CommonType<TLhs, TRhs>>
cv::Point_<TResult> operator*(const cv::Point_<TLhs>& lhs, const TRhs& rhs) {
    return { lhs.x * rhs, lhs.y * rhs };
}

template<typename TLhs, typename TRhs, typename TResult = CommonType<TLhs, TRhs>>
cv::Point_<TResult> operator*(const TLhs& lhs, const cv::Point_<TRhs>& rhs) {
    return { lhs * rhs.x, lhs * rhs.y };
}

template<typename TLhs, typename TRhs, typename TResult = CommonType<TLhs, TRhs>>
cv::Point_<TResult> operator/(const cv::Point_<TLhs>& lhs, const TRhs& rhs) {
    return { lhs.x / rhs, lhs.y / rhs };
}

template<typename TLength, typename TAngle, typename TResult = CommonType<TLength, TAngle>>
cv::Point_<TResult> Polar(TLength length, TAngle angle) {
    return { length * std::cos(angle), length * std::sin(angle) };
}

template<typename TPoint, typename TResult = CommonType<TPoint>>
TResult Angle(const cv::Point_<TPoint>& point) {
    return std::atan2(point.y, point.x);
}

template<typename TPoint, typename TResult = CommonType<TPoint>>
TResult Length(const cv::Point_<TPoint>& point) {
    return std::sqrt(point.x * point.x + point.y * point.y);
}

template<typename TPoint, typename TResult = CommonType<TPoint>>
TResult LengthSquared(const cv::Point_<TPoint>& point) {
    return point.x * point.x + point.y * point.y;
}

template<typename TPoint, typename TResult = CommonType<TPoint>>
cv::Point_<TResult> Normalize(const cv::Point_<TPoint>& point) {
    return point / Length(point);
}

template<typename TPoint, typename TAngle, typename TResult = CommonType<TPoint, TAngle>>
cv::Point_<TResult> Rotate(const cv::Point_<TPoint>& point, TAngle angle) {
    return { point.x * std::cos(angle) - point.y * std::sin(angle),
             point.x * std::sin(angle) + point.y * std::cos(angle) };
}

template<typename TPoint, typename TCenter, typename TAngle, typename TResult = CommonType<TPoint, TCenter, TAngle>>
cv::Point_<TResult> Rotate(const cv::Point_<TPoint>& point, const cv::Point_<TCenter>& center, TAngle angle) {
    return center + Rotate(point - center, angle);
}

template<typename TLhs, typename TRhs, typename TResult = CommonType<TLhs, TRhs>>
TResult DistanceChebyshev(const cv::Point_<TLhs>& lhs, const cv::Point_<TRhs>& rhs) {
    return std::max(std::abs(lhs.x - rhs.x), std::abs(lhs.y - rhs.y));
}

template<typename TLhs, typename TRhs, typename TResult = CommonType<TLhs, TRhs>>
TResult DistanceManhattan(const cv::Point_<TLhs>& lhs, const cv::Point_<TRhs>& rhs) {
    return std::abs(lhs.x - rhs.x) + std::abs(lhs.y - rhs.y);
}

template<typename TLhs, typename TRhs, typename TResult = CommonType<TLhs, TRhs>>
TResult DistanceEuclidean(const cv::Point_<TLhs>& lhs, const cv::Point_<TRhs>& rhs) {
    return Length(lhs - rhs);
}

template<typename TLhs, typename TRhs, typename TResult = CommonType<TLhs, TRhs>>
TResult Distance(const cv::Point_<TLhs>& lhs, const cv::Point_<TRhs>& rhs) {
    return DistanceEuclidean(lhs, rhs);
}

template<typename TLhs, typename TRhs, typename TResult = CommonType<TLhs, TRhs>>
cv::Point_<TResult> CrossPoint(const std::array<cv::Point_<TLhs>, 2>& lhs, const std::array<cv::Point_<TRhs>, 2>& rhs) {
    auto lhs_a = lhs[0].y - lhs[1].y;
    auto lhs_b = lhs[1].x - lhs[0].x;
    auto lhs_c = lhs[0].x * lhs[1].y - lhs[1].x * lhs[0].y;
    auto rhs_a = rhs[0].y - rhs[1].y;
    auto rhs_b = rhs[1].x - rhs[0].x;
    auto rhs_c = rhs[0].x * rhs[1].y - rhs[1].x * rhs[0].y;
    auto denominator = lhs_a * rhs_b - lhs_b * rhs_a;
    if (Equals(denominator, 0)) {
        return { std::numeric_limits<TResult>::max(), std::numeric_limits<TResult>::max() };
    } else {
        return { (lhs_b * rhs_c - lhs_c * rhs_b) / denominator, (lhs_c * rhs_a - lhs_a * rhs_c) / denominator };
    }
}

} // namespace rune
