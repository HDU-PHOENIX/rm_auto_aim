#pragma once

#include <chrono>
#include <compare>

#include "usings.hpp"

namespace rune {

    class Timestamp;

    class Duration {
        using TReturnDefault = double;

    public:
        using BaseClock = std::chrono::system_clock;
        using TimePoint = BaseClock::time_point;
        using Rep = BaseClock::rep;

        Duration() noexcept;

        Duration(const BaseClock::duration& duration) noexcept;

        Duration(BaseClock::duration&& duration) noexcept;

        Duration& operator=(const Duration& rhs) noexcept;

        Rep Count() const noexcept;

        static Duration Zero() noexcept;

        static Duration Min() noexcept;

        static Duration Max() noexcept;

        Duration operator+() const noexcept;

        Duration operator-() const noexcept;

        Duration& operator++() noexcept;

        Duration& operator++(int) noexcept;

        Duration& operator--() noexcept;

        Duration& operator--(int) noexcept;

        Duration& operator+=(const Duration& rhs) noexcept;

        Duration& operator-=(const Duration& rhs) noexcept;

        Duration& operator*=(const Rep& rhs) noexcept;

        Duration& operator/=(const Rep& rhs) noexcept;

        Duration& operator%=(const Rep& rhs) noexcept;

        Duration& operator%=(const Duration& rhs) noexcept;

        /**
         * @brief Auto cast the Duration to seconds.
         */
        operator TReturnDefault() const noexcept;

        /** 
         * @brief Cast the Duration to specified units.
         * 
         * @tparam TPeriod the specified period
         */
        template<typename TReturn = TReturnDefault, typename TPeriod = std::ratio<1>>
        TReturn GetTime() const noexcept {
            return duration_cast<std::chrono::duration<TReturn, TPeriod>>(duration).count();
        }

#define GET_TIME_SPECIAL_CASE_IMPLEMENT(NAME, PERIOD)                                                \
    template<typename TReturn = TReturnDefault>                                                      \
    TReturn Get##NAME() const noexcept {                                                             \
        return std::chrono::duration_cast<std::chrono::duration<TReturn, PERIOD>>(duration).count(); \
    }
        GET_TIME_SPECIAL_CASE_IMPLEMENT(Attoseconds, std::atto)
        GET_TIME_SPECIAL_CASE_IMPLEMENT(Femtoseconds, std::femto)
        GET_TIME_SPECIAL_CASE_IMPLEMENT(Picoseconds, std::pico)
        GET_TIME_SPECIAL_CASE_IMPLEMENT(Nanoseconds, std::nano)
        GET_TIME_SPECIAL_CASE_IMPLEMENT(Microseconds, std::micro)
        GET_TIME_SPECIAL_CASE_IMPLEMENT(Milliseconds, std::milli)
        GET_TIME_SPECIAL_CASE_IMPLEMENT(Centiseconds, std::centi)
        GET_TIME_SPECIAL_CASE_IMPLEMENT(Deciseconds, std::deci)
        GET_TIME_SPECIAL_CASE_IMPLEMENT(Decaseconds, std::deca)
        GET_TIME_SPECIAL_CASE_IMPLEMENT(Hectoseconds, std::hecto)
        GET_TIME_SPECIAL_CASE_IMPLEMENT(Kiloseconds, std::kilo)
        GET_TIME_SPECIAL_CASE_IMPLEMENT(Megaseconds, std::mega)
        GET_TIME_SPECIAL_CASE_IMPLEMENT(Gigaseconds, std::giga)
        GET_TIME_SPECIAL_CASE_IMPLEMENT(Teraseconds, std::tera)
        GET_TIME_SPECIAL_CASE_IMPLEMENT(Petaseconds, std::peta)
        GET_TIME_SPECIAL_CASE_IMPLEMENT(Exaseconds, std::exa)
        GET_TIME_SPECIAL_CASE_IMPLEMENT(Seconds, std::ratio<1>)
        GET_TIME_SPECIAL_CASE_IMPLEMENT(Minutes, std::ratio<60>)
        GET_TIME_SPECIAL_CASE_IMPLEMENT(Quarters, std::ratio<900>)
        GET_TIME_SPECIAL_CASE_IMPLEMENT(Hours, std::ratio<3600>)
        GET_TIME_SPECIAL_CASE_IMPLEMENT(Days, std::ratio<86400>)
#undef GET_TIME_SPECIAL_CASE_IMPLEMENT

        operator BaseClock::duration() const noexcept;

    private:
        friend class Timestamp;
        friend Duration operator+(const Duration& lhs, const Duration& rhs) noexcept;
        friend Duration operator-(const Duration& lhs, const Duration& rhs) noexcept;
        friend Duration operator*(const Duration& lhs, const Rep& rhs) noexcept;
        friend Duration operator*(const Rep& lhs, const Duration& rhs) noexcept;
        friend Duration operator/(const Duration& lhs, const Rep& rhs) noexcept;
        friend Duration::Rep operator/(const Duration& lhs, const Duration& rhs) noexcept;
        friend Duration operator%(const Duration& lhs, const Rep& rhs) noexcept;
        friend Duration operator%(const Duration& lhs, const Duration& rhs) noexcept;

        friend bool operator==(const Duration& lhs, const Duration& rhs) noexcept;
        friend bool operator!=(const Duration& lhs, const Duration& rhs) noexcept;
        friend bool operator<(const Duration& lhs, const Duration& rhs) noexcept;
        friend bool operator<=(const Duration& lhs, const Duration& rhs) noexcept;
        friend bool operator>(const Duration& lhs, const Duration& rhs) noexcept;
        friend bool operator>=(const Duration& lhs, const Duration& rhs) noexcept;
        friend std::strong_ordering operator<=>(const Duration& lhs, const Duration& rhs) noexcept;

        friend Timestamp operator+(const Timestamp& lhs, const Duration& rhs) noexcept;
        friend Timestamp operator+(const Duration& lhs, const Timestamp& rhs) noexcept;
        friend Timestamp operator-(const Timestamp& lhs, const Duration& rhs) noexcept;

        BaseClock::duration duration;
    };

}  // namespace phoenix
