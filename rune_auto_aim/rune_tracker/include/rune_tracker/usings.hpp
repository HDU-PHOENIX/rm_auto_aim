#pragma once

#include <opencv2/opencv.hpp>

namespace rune {

using llong = long long;
using ldouble = long double;

using uchar = unsigned char;
using ushort = unsigned short;
using uint = unsigned int;
using ulong = unsigned long;
using ullong = unsigned long long;

#define SIGNED_DEFAULT_USING(NAME, TYPE) \
    using NAME = signed TYPE; \
    using u##NAME = unsigned TYPE;

SIGNED_DEFAULT_USING(int8, char)
SIGNED_DEFAULT_USING(int16, short)
SIGNED_DEFAULT_USING(int32, int)
SIGNED_DEFAULT_USING(int64, long long)

#undef SIGNED_DEFAULT_USING

#define UNSIGNED_DEFAULT_USING(NAME, TYPE) \
    using NAME = unsigned TYPE; \
    using s##NAME = signed TYPE;

UNSIGNED_DEFAULT_USING(byte, char)
UNSIGNED_DEFAULT_USING(word, short)
UNSIGNED_DEFAULT_USING(dword, int)
UNSIGNED_DEFAULT_USING(qword, long long)

#undef UNSIGNED_DEFAULT_USING

using float32 = float;
using float64 = double;
using float128 = long double;

using complex32 = std::complex<float32>;
using complex64 = std::complex<float64>;
using complex128 = std::complex<float128>;

template<typename TType>
using sptr = std::shared_ptr<TType>;
template<typename TType>
using wptr = std::weak_ptr<TType>;
template<typename TType>
using uptr = std::unique_ptr<TType>;

// template<typename T>
// union Pack {
//     T value;
//     char bytes[sizeof(T)];

//     explicit Pack(T value = {})
//         : value(value) {}
// } __attribute__((packed));

} // namespace rune
