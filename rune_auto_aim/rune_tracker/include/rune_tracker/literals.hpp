#pragma once

#include "usings.hpp"

namespace rune {

#define CONVERT_LITERAL(name, ratio)                              \
    constexpr ldouble operator""_##name(ldouble value) noexcept { \
        return value * (ratio);                                   \
    }                                                             \
    constexpr ldouble operator""_##name(ullong value) noexcept {  \
        return value * (ratio);                                   \
    }

#define INTERNATIONAL_UNIT_LITERAL(name, ratio) \
    CONVERT_LITERAL(Y##name, ratio * 1e24)      \
    CONVERT_LITERAL(Z##name, ratio * 1e21)      \
    CONVERT_LITERAL(E##name, ratio * 1e18)      \
    CONVERT_LITERAL(P##name, ratio * 1e15)      \
    CONVERT_LITERAL(T##name, ratio * 1e12)      \
    CONVERT_LITERAL(G##name, ratio * 1e9)       \
    CONVERT_LITERAL(M##name, ratio * 1e6)       \
    CONVERT_LITERAL(ma##name, ratio * 1e4)      \
    CONVERT_LITERAL(k##name, ratio * 1e3)       \
    CONVERT_LITERAL(h##name, ratio * 1e2)       \
    CONVERT_LITERAL(da##name, ratio * 1e1)      \
    CONVERT_LITERAL(name, ratio)                \
    CONVERT_LITERAL(d##name, ratio * 1e-1)      \
    CONVERT_LITERAL(c##name, ratio * 1e-2)      \
    CONVERT_LITERAL(m##name, ratio * 1e-3)      \
    CONVERT_LITERAL(mo##name, ratio * 1e-4)     \
    CONVERT_LITERAL(u##name, ratio * 1e-6)      \
    CONVERT_LITERAL(n##name, ratio * 1e-9)      \
    CONVERT_LITERAL(p##name, ratio * 1e-12)     \
    CONVERT_LITERAL(f##name, ratio * 1e-15)     \
    CONVERT_LITERAL(a##name, ratio * 1e-18)     \
    CONVERT_LITERAL(z##name, ratio * 1e-21)     \
    CONVERT_LITERAL(y##name, ratio * 1e-24)

    INTERNATIONAL_UNIT_LITERAL(s, 1.)
    INTERNATIONAL_UNIT_LITERAL(m, 1.)
    INTERNATIONAL_UNIT_LITERAL(g, 1e-3)
    INTERNATIONAL_UNIT_LITERAL(A, 1.)
    INTERNATIONAL_UNIT_LITERAL(K, 1.)
    INTERNATIONAL_UNIT_LITERAL(mol, 1.)
    INTERNATIONAL_UNIT_LITERAL(cd, 1.)

#undef INTERNATIONAL_UNIT_LITERAL

    CONVERT_LITERAL(d, 24. * 60. * 60.)
    CONVERT_LITERAL(h, 60. * 60.)
    CONVERT_LITERAL(min, 60.)

    CONVERT_LITERAL(rad, 1.)
    CONVERT_LITERAL(deg, M_PI / 180.)

#undef CONVERT_LITERAL

}  // namespace phoenix
