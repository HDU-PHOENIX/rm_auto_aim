#include "rune_tracker/duration.h"

namespace rune {

    Duration::Duration() noexcept
        : duration() {}

    Duration::Duration(const BaseClock::duration& duration) noexcept
        : duration(duration) {}

    Duration::Duration(BaseClock::duration&& duration) noexcept
        : duration(duration) {}

    Duration& Duration::operator=(const Duration& rhs) noexcept {
        return this->duration = rhs.duration, *this;
    }

    Duration::Rep Duration::Count() const noexcept {
        return duration.count();
    }

    Duration Duration::Zero() noexcept {
        return BaseClock::duration::zero();
    }

    Duration Duration::Min() noexcept {
        return BaseClock::duration::min();
    }

    Duration Duration::Max() noexcept {
        return BaseClock::duration::max();
    }

    Duration Duration::operator+() const noexcept {
        return duration;
    }

    Duration Duration::operator-() const noexcept {
        return -duration;
    }

    Duration& Duration::operator++() noexcept {
        return ++duration, *this;
    }

    Duration& Duration::operator++(int) noexcept {
        return duration++, *this;
    }

    Duration& Duration::operator--() noexcept {
        return --duration, *this;
    }

    Duration& Duration::operator--(int) noexcept {
        return duration--, *this;
    }

    Duration& Duration::operator+=(const Duration& rhs) noexcept {
        return duration += rhs.duration, *this;
    }

    Duration& Duration::operator-=(const Duration& rhs) noexcept {
        return duration -= rhs.duration, *this;
    }

    Duration& Duration::operator*=(const Rep& rhs) noexcept {
        return duration *= rhs, *this;
    }

    Duration& Duration::operator/=(const Rep& rhs) noexcept {
        return duration /= rhs, *this;
    }

    Duration& Duration::operator%=(const Rep& rhs) noexcept {
        return duration %= rhs, *this;
    }

    Duration& Duration::operator%=(const Duration& rhs) noexcept {
        return duration %= rhs.duration, *this;
    }

    Duration::operator TReturnDefault() const noexcept {
        return GetSeconds();
    }

    Duration::operator BaseClock::duration() const noexcept {
        return duration;
    }

    Duration operator+(const Duration& lhs, const Duration& rhs) noexcept {
        return lhs.duration + rhs.duration;
    }

    Duration operator-(const Duration& lhs, const Duration& rhs) noexcept {
        return lhs.duration - rhs.duration;
    }

    Duration operator*(const Duration& lhs, const Duration::Rep& rhs) noexcept {
        return lhs.duration * rhs;
    }

    Duration operator*(const Duration::Rep& lhs, const Duration& rhs) noexcept {
        return lhs * rhs.duration;
    }

    Duration operator/(const Duration& lhs, const Duration::Rep& rhs) noexcept {
        return lhs.duration / rhs;
    }

    Duration::Rep operator/(const Duration& lhs, const Duration& rhs) noexcept {
        return lhs.duration / rhs.duration;
    }

    Duration operator%(const Duration& lhs, const Duration::Rep& rhs) noexcept {
        return lhs.duration % rhs;
    }

    Duration operator%(const Duration& lhs, const Duration& rhs) noexcept {
        return lhs.duration % rhs.duration;
    }

    bool operator==(const Duration& lhs, const Duration& rhs) noexcept {
        return lhs.duration == rhs.duration;
    }

    bool operator!=(const Duration& lhs, const Duration& rhs) noexcept {
        return lhs.duration != rhs.duration;
    }

    bool operator<(const Duration& lhs, const Duration& rhs) noexcept {
        return lhs.duration < rhs.duration;
    }

    bool operator<=(const Duration& lhs, const Duration& rhs) noexcept {
        return lhs.duration <= rhs.duration;
    }

    bool operator>(const Duration& lhs, const Duration& rhs) noexcept {
        return lhs.duration > rhs.duration;
    }

    bool operator>=(const Duration& lhs, const Duration& rhs) noexcept {
        return lhs.duration >= rhs.duration;
    }

    std::strong_ordering operator<=>(const Duration& lhs, const Duration& rhs) noexcept {
        return lhs.duration <=> rhs.duration;
    }

}  // namespace phoenix
