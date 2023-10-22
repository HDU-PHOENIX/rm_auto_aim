#pragma once

#include <cmath>
#include <functional>
#include <limits>

#include "type_traits.hpp"

namespace rune {

    /**
     * @brief 计算一个值的平方
     *
     * @tparam TValue 值类型
     * @param value 底数
     */
    template<typename TValue>
    inline constexpr TValue Square(TValue value) {
        return value * value;
    }

    /**
     * @brief 计算一个值的立方
     *
     * @tparam TValue 值类型
     * @param value 底数
     */
    template<typename TValue>
    inline constexpr TValue Cube(TValue value) {
        return value * value * value;
    }

    /**
     * @brief 使用快速幂算法计算一个值的乘幂
     *
     * @tparam TValue 值的类型
     * @param value 底数
     * @param power 指数
     */
    template<typename TValue>
    inline constexpr TValue PowFast(TValue value, std::size_t power) {
        TValue result = 1;
        do {
            if (power & 1) {
                result *= value;
            }
            value *= value;
        } while (power >>= 1);
        return result;
    }

    /**
     * @brief 将值修订到指定范围
     *
     * @param value 待修订的值
     * @param lower 下界
     * @param upper 上界
     * @return 修订后的值
     */
    template<typename TValue, typename TLower, typename TUpper>
    inline constexpr TValue Revise(const TValue& value, const TLower& lower, const TUpper& upper) {
        auto&& interval = upper - lower;
        return value - std::floor((value - lower) / interval) * interval;
    }

    /**
     * @brief 判断一个值是否在指定范围内
     * 
     * @tparam TValue 值的类型
     * @tparam TLower 下界的类型
     * @tparam TUpper 上界的类型
     * @param value 待判断的值
     * @param lower 下界的值
     * @param upper 上界的值
     * @return 判断结果
     */
    template<typename TValue, typename TLower, typename TUpper>
    inline constexpr bool InRange(const TValue& value, const TLower& lower, const TUpper& upper) {
        return lower <= value && value <= upper;
    }

    template<typename TLhs, typename TRhs, typename TEpsilon = CommonType<TLhs, TRhs>>
    inline constexpr bool Equals(const TLhs& lhs, const TRhs& rhs, const TEpsilon& epsilon = std::numeric_limits<TEpsilon>::epsilon()) {
        return lhs == rhs || std::abs(lhs - rhs) <= epsilon ||
               std::abs(lhs - rhs) <= epsilon * std::max<CommonType<TLhs, TRhs>>(std::abs(lhs), std::abs(rhs));
    }
};  // namespace phoenix
