#pragma once

#include "concepts.hpp"
#include "usings.hpp"

namespace rune {

    template<typename TValue>
    //requires IsIntegral<TValue> || IsFloatingPoint<TValue>
    class Statistic {
    public:
        Statistic()
            : value(), min(), max(), count(), sum(), square_sum(), mean(), square_mean(), variance(){};

        void Clear() {
            value = min = max = 0;
            count = 0;
            sum = square_sum = mean = square_mean = variance = 0;
        }

        TValue Value() const {
            return value;
        }

        TValue Min() const {
            return min;
        }

        TValue Max() const {
            return max;
        }

        std::size_t Count() const {
            return count;
        }

        ldouble Sum() const {
            return sum;
        }

        ldouble SquareSum() const {
            return square_sum;
        }

        ldouble Mean() const {
            return mean;
        }

        ldouble SquareMean() const {
            return square_mean;
        }

        ldouble Variance() const {
            return variance;
        }

        void Push(TValue value) {
            this->value = value;
            if (count == 0) {
                min = max = value;
            } else {
                min = std::min(min, value);
                max = std::max(max, value);
            }
            mean = (sum += value) / (++count);
            square_mean = (square_sum += value * value) / count;
            variance = square_mean - mean * mean;
        }

        void Pop(TValue value) {
            assert(count > 0);
            if (!--count) {
                return Clear();
            }
            mean = (sum -= value) / count;
            square_mean = (square_sum -= value * value) / count;
            variance = square_mean - mean * mean;
        }

    private:
        TValue value, min, max;
        std::size_t count;
        ldouble sum, square_sum, mean, square_mean, variance;
    };

}  // namespace phoenix
