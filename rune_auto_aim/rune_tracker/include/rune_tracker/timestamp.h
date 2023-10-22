#pragma once

#include <chrono>
#include <compare>
#include <fmt/chrono.h>
#include <fmt/format.h>

#include "duration.h"
#include "formats.hpp"

namespace rune {

    /**
     * @brief 基于  std::chrono::system_clock 的时间戳
     */
    class Timestamp {
    public:
        using BaseClock = std::chrono::system_clock;
        using TimePoint = BaseClock::time_point;
        using Rep = BaseClock::rep;

        Timestamp() noexcept;

        Timestamp(const TimePoint& time_point) noexcept;
        Timestamp(const Rep& rep) noexcept;

        ~Timestamp() noexcept = default;

        Duration TimeSinceEpoch() const noexcept;

        Timestamp& operator+=(const Duration& duration) noexcept;
        Timestamp& operator-=(const Duration& duration) noexcept;

#if __cplusplus >= 202002L
        Timestamp& operator++() noexcept;
        Timestamp& operator++(int) noexcept;
        Timestamp& operator--() noexcept;
        Timestamp& operator--(int) noexcept;
#endif

        static Timestamp Min() noexcept;
        static Timestamp Max() noexcept;

        static Timestamp Now() noexcept;

        /**
         * @brief Set the Timestamp to the current time.
         */
        void SetNow() noexcept;

        /**
         * @brief Calculate the interval from the creation of the timestamp to the present.
         * 
         * @return a Duration representing the interval
         */
        Duration GetTimePassed() const noexcept;

        operator TimePoint() const noexcept;

        operator Rep() const noexcept;

        std::string ToString(std::string format = DATETIME_FORMAT) const;

    private:
        friend class fmt::formatter<rune::Timestamp>;
        friend Timestamp operator+(const Timestamp& lhs, const Duration& rhs) noexcept;
        friend Timestamp operator+(const Duration& lhs, const Timestamp& rhs) noexcept;
        friend Timestamp operator-(const Timestamp& lhs, const Duration& rhs) noexcept;
        friend Duration operator-(const Timestamp& lhs, const Timestamp& rhs) noexcept;
        friend bool operator==(const Timestamp& lhs, const Timestamp& rhs) noexcept;
        friend bool operator!=(const Timestamp& lhs, const Timestamp& rhs) noexcept;
        friend bool operator<(const Timestamp& lhs, const Timestamp& rhs) noexcept;
        friend bool operator<=(const Timestamp& lhs, const Timestamp& rhs) noexcept;
        friend bool operator>(const Timestamp& lhs, const Timestamp& rhs) noexcept;
        friend bool operator>=(const Timestamp& lhs, const Timestamp& rhs) noexcept;
        friend std::strong_ordering operator<=>(const Timestamp& lhs, const Timestamp& rhs) noexcept;
        /**
         * @brief the time_point of the Timestamp
         */
        TimePoint time_point;
    };

};  // namespace rune

namespace fmt {
    template<>
    struct formatter<rune::Timestamp> : public formatter<rune::Timestamp::TimePoint> {
        template<typename ParseContext>
        auto parse(ParseContext&& ctx) -> decltype(ctx.begin()) {
            return formatter<rune::Timestamp::TimePoint>::parse(ctx);
        }

        template<typename FormatContext>
        auto format(const rune::Timestamp& timestamp, FormatContext&& ctx) -> decltype(ctx.out()) {
            return formatter<rune::Timestamp::TimePoint>::format(timestamp.time_point, std::forward<FormatContext>(ctx));
        }
    };

}  // namespace fmt
