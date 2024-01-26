#pragma once

#include "ring_buffer.hpp"
#include "statistic.hpp"

namespace rune {

template<typename TValue, size_t VSize = 1 << 4>
class RingBufferStatistic: public RingBuffer<TValue, VSize> {
public:
    explicit RingBufferStatistic() noexcept
        :
        RingBuffer<TValue, VSize>(),
        statistic() {}
    explicit RingBufferStatistic(const RingBufferStatistic& object) noexcept
        :
        RingBuffer<TValue, VSize>(object),
        statistic(object.statistic) {};
    explicit RingBufferStatistic(RingBufferStatistic&& object) noexcept
        :
        RingBuffer<TValue, VSize>(std::move(object)),
        statistic(std::move(object.statistic)) {};
    RingBufferStatistic& operator=(const RingBufferStatistic& object) noexcept {
        RingBuffer<TValue, VSize>::operator=(object);
        statistic = object.statistic;
        return *this;
    };
    RingBufferStatistic& operator=(RingBufferStatistic&& object) noexcept {
        RingBuffer<TValue, VSize>::operator=(std::move(object));
        statistic = std::move(object.statistic);
        return *this;
    };
    ~RingBufferStatistic() noexcept = default;

    virtual void Clear() noexcept override {
        RingBuffer<TValue, VSize>::Clear();
        statistic.Clear();
    }

    virtual bool Pop(TValue& item) noexcept override {
        bool result = RingBuffer<TValue, VSize>::Pop(item);
        if (result) {
            statistic.Pop(item);
        }
        return result;
    }

    virtual TValue&& Pop() override {
        auto&& item = RingBuffer<TValue, VSize>::Pop();
        statistic.Pop(item);
        return std::move(item);
    }

    virtual bool Push(const TValue& item) noexcept override {
        bool result = RingBuffer<TValue, VSize>::Push(item);
        if (result) {
            statistic.Push(item);
        }
        return result;
    }

    virtual bool Push(TValue&& item) noexcept override {
        bool result = RingBuffer<TValue, VSize>::Push(item);
        if (result) {
            statistic.Push(item);
        }
        return result;
    }

    void PushForcibly(const TValue& item) noexcept override {
        if (!Push(item)) {
            Pop();
            Push(item);
        }
    }

    void PushForcibly(TValue&& item) noexcept override {
        if (!Push(item)) {
            Pop();
            Push(item);
        }
    }

    ldouble Sum() const {
        return statistic.Sum();
    }

    ldouble SquareSum() const {
        return statistic.SquareSum();
    }

    ldouble Mean() const {
        return statistic.Mean();
    }

    ldouble SquareMean() const {
        return statistic.SquareMean();
    }

    ldouble Variance() const {
        return statistic.Variance();
    }

private:
    Statistic<TValue> statistic;
};

} // namespace rune
