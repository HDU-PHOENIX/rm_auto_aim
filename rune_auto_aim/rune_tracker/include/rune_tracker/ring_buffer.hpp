#pragma once

#include <fmt/format.h>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <vector>

namespace rune {

    template<typename TValue, size_t VSize = 1 << 4>
    class RingBuffer {
        int head;
        int count;
        std::vector<TValue> data;

        friend void swap(RingBuffer& lhs, RingBuffer& rhs) noexcept {
            using std::swap;
            swap(lhs.head, rhs.head);
            swap(lhs.count, rhs.count);
            swap(lhs.data, rhs.data);
        }

    public:
        explicit RingBuffer() noexcept
            : head(0),
              count(0),
              data(VSize) {
        }

        explicit RingBuffer(const RingBuffer& object) noexcept
            : head(object.head),
              count(object.count),
              data(object.data) {
        }
        explicit RingBuffer(RingBuffer&& object) noexcept
            : RingBuffer() {
            swap(*this, object);
        }
        RingBuffer& operator=(RingBuffer object) noexcept {
            swap(*this, object);
            return *this;
        }
        ~RingBuffer() noexcept = default;

        [[nodiscard]] static constexpr std::size_t Size() noexcept {
            return VSize;
        }

        [[nodiscard]] virtual std::size_t Head() const noexcept {
            return head;
        }

        [[nodiscard]] virtual std::size_t Tail() const noexcept {
            return (head + count) % VSize;
        }

        [[nodiscard]] virtual std::size_t Count() const noexcept {
            return count;
        }

        [[nodiscard]] virtual bool Empty() const noexcept {
            return count == 0;
        }

        [[nodiscard]] virtual bool Any() const noexcept {
            return count > 0;
        }

        [[nodiscard]] virtual bool Full() const noexcept {
            return count == VSize;
        }

        virtual void Clear() noexcept {
            head = count = 0;
        }

        virtual bool Top(TValue& item) noexcept {
            if (Empty()) {
                return false;
            }
            return item = data[head % VSize], true;
        }

        virtual TValue& Top() {
            if (Empty()) {
                throw std::out_of_range(fmt::format("at {}:{} in {}", __FILE__, __LINE__, __func__));
            }
            return data[head % VSize];
        }

        virtual bool Back(TValue& item) noexcept {
            if (Empty()) {
                return false;
            }
            return item = data[(head + count - 1) % VSize], true;
        }

        virtual TValue& Back() {
            if (Empty()) {
                throw std::out_of_range(fmt::format("at {}:{} in {}", __FILE__, __LINE__, __func__));
            }
            return data[(head + count - 1) % VSize];
        }

        virtual bool Pop(TValue& item) noexcept {
            if (Empty()) {
                return false;
            }
            return item = data[(count--, head++) % VSize], true;
        }

        virtual TValue&& Pop() {
            if (Empty()) {
                throw std::out_of_range(fmt::format("at {}:{} in {}", __FILE__, __LINE__, __func__));
            }
            return std::move(data[(count--, head++) % VSize]);
        }

        virtual bool Push(const TValue& item) noexcept {
            if (Full()) {
                return false;
            }
            return data[(head + count++) % VSize] = item, true;
        }

        virtual bool Push(TValue&& item) noexcept {
            if (Full()) {
                return false;
            }
            return data[(head + count++) % VSize] = std::move(item), true;
        }

        virtual void PushForcibly(const TValue& item) noexcept {
            if (!Push(item)) {
                data[(head++ + count) % VSize] = item;
            }
        }

        virtual void PushForcibly(TValue&& item) noexcept {
            if (!Push(item)) {
                data[(head++ + count) % VSize] = std::move(item);
            }
        }

        virtual bool Get(TValue& item, const int index) noexcept {
            if (index < -count || count <= index) {
                return false;
            }
            return item = data[(head + index + (index < 0 ? count : 0)) % VSize], true;
        }

        virtual TValue& Get(const int index) {
            if (index < -count || count <= index) {
                throw std::out_of_range(fmt::format("at {}:{} in {}", __FILE__, __LINE__, __func__));
            }
            return data[(head + index + (index < 0 ? count : 0)) % VSize];
        }

        virtual const TValue& Get(const int index) const {
            if (index < -count || count <= index) {
                throw std::out_of_range(fmt::format("at {}:{} in {}", __FILE__, __LINE__, __func__));
            }
            return data[(head + index + (index < 0 ? count : 0)) % VSize];
        }

        virtual TValue& operator[](const int index) {
            if (index < -count || count <= index) {
                throw std::out_of_range(fmt::format("at {}:{} in {}", __FILE__, __LINE__, __func__));
            }
            return data[(head + index + (index < 0 ? count : 0)) % VSize];
        }

        virtual const TValue& operator[](const int index) const {
            if (index < -count || count <= index) {
                throw std::out_of_range(fmt::format("at {}:{} in {}", __FILE__, __LINE__, __func__));
            }
            return data[(head + index + (index < 0 ? count : 0)) % VSize];
        }
    };
};  // namespace phoenix
