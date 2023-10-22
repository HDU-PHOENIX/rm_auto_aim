#include "rune_tracker/timestamp.h"

#include <compare>

namespace rune {

    Timestamp::Timestamp() noexcept
        : time_point() {}

    Timestamp::Timestamp(const TimePoint& time_point) noexcept
        : time_point(time_point) {}

    Timestamp::Timestamp(const Rep& rep) noexcept
        : time_point(BaseClock::duration(rep)) {}

    Duration Timestamp::TimeSinceEpoch() const noexcept {
        return time_point.time_since_epoch();
    }

    Timestamp& Timestamp::operator+=(const Duration& rhs) noexcept {
        time_point += rhs.duration;
        return *this;
    }

    Timestamp& Timestamp::operator-=(const Duration& rhs) noexcept {
        time_point -= rhs.duration;
        return *this;
    }

#if __cplusplus >= 202002L
    Timestamp& Timestamp::operator++() noexcept {
        return ++time_point, *this;
    }

    Timestamp& Timestamp::operator++(int) noexcept {
        return time_point++, *this;
    }

    Timestamp& Timestamp::operator--() noexcept {
        return --time_point, *this;
    }

    Timestamp& Timestamp::operator--(int) noexcept {
        return time_point--, *this;
    }
#endif

    Timestamp Timestamp::Min() noexcept {
        return TimePoint::min();
    }

    Timestamp Timestamp::Max() noexcept {
        return TimePoint::max();
    }

    Timestamp Timestamp::Now() noexcept {
        return Timestamp(BaseClock::now());
    }

    void Timestamp::SetNow() noexcept {
        time_point = BaseClock::now();
    }

    Duration Timestamp::GetTimePassed() const noexcept {
        return BaseClock::now() - time_point;
    }

    Timestamp::operator TimePoint() const noexcept {
        return time_point;
    }

    Timestamp::operator Rep() const noexcept {
        return time_point.time_since_epoch().count();
    }

    std::string Timestamp::ToString(std::string format) const {
        return fmt::format(fmt::runtime(format), time_point);
    }

    Timestamp operator+(const Timestamp& lhs, const Duration& rhs) noexcept {
        return lhs.time_point + rhs.duration;
    }

    Timestamp operator+(const Duration& lhs, const Timestamp& rhs) noexcept {
        return rhs.time_point + lhs.duration;
    }

    Timestamp operator-(const Timestamp& lhs, const Duration& rhs) noexcept {
        return lhs.time_point - rhs.duration;
    }

    Duration operator-(const Timestamp& lhs, const Timestamp& rhs) noexcept {
        return lhs.time_point - rhs.time_point;
    }

    bool operator==(const Timestamp& lhs, const Timestamp& rhs) noexcept {
        return lhs.time_point == rhs.time_point;
    }

    bool operator!=(const Timestamp& lhs, const Timestamp& rhs) noexcept {
        return lhs.time_point != rhs.time_point;
    }

    bool operator<(const Timestamp& lhs, const Timestamp& rhs) noexcept {
        return lhs.time_point < rhs.time_point;
    }

    bool operator<=(const Timestamp& lhs, const Timestamp& rhs) noexcept {
        return lhs.time_point <= rhs.time_point;
    }

    bool operator>(const Timestamp& lhs, const Timestamp& rhs) noexcept {
        return lhs.time_point > rhs.time_point;
    }

    bool operator>=(const Timestamp& lhs, const Timestamp& rhs) noexcept {
        return lhs.time_point >= rhs.time_point;
    }

    std::strong_ordering operator<=>(const Timestamp& lhs, const Timestamp& rhs) noexcept {
        return lhs.time_point <=> rhs.time_point;
    }

}  // namespace phoenix
