// MIT License

// Copyright (c) 2024

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once

#include <cmath>

namespace orbit {
enum UnitType {
    Distance,  // Default distance: kilometer
    Angle,     // Default distance: degree
    Mass,      // Default distance: kilogram
    Volume,    // Default distance: m^3
    Time       // Default distance: second
};

enum Distance { LightYear, AstronomicalUnit, Kilometer, Meter, Centimeter, Millimeter };

typedef double astronomical_unit;
typedef double light_year;
typedef double kilometer;
typedef double solar_mass;
typedef double degree;
typedef double radian;
typedef double meter_cube;
typedef double kilogram;
typedef double earth_masses;
typedef double joule;
typedef double second;

// Constants
constexpr double PI = 3.14159265358979323846;
constexpr double TWOPI = PI * 2;
constexpr double HALFPI = PI / 2;

constexpr double KmInAu = 149597870.700;

// Gravitional constant in m^3 * kg^-1 * s^-2
constexpr double G = 6.6743015e-11;
// Gravitional constant in km^3 * kg^-1 * s^-2
constexpr double G_km = 6.6743015e-20;

// GM of sun/sun gravitational constant in km^3 * s^-2
constexpr double SunMu = 1.32712400188e11;

#if __cplusplus == 202302L
// Then use fmod
#define floatmod std::fmod
#else
inline double constexpr floatmod(double x, double y) { return x - y * (int)(x / y); }
#endif

/// <summary>
/// Normalizes a radian to [0, PI*2)
/// </summary>
/// \param[in] Radian to normalizes
inline constexpr radian normalize_radian(const radian& radian) {
    double x = floatmod(radian, TWOPI);
    if (x < 0) {
        x += TWOPI;
    }
    return x;
}

inline constexpr double normalize_radian_coord(const radian& radian) {
    double r = floatmod(radian + PI, TWOPI);
    if (r < 0) r += TWOPI;
    return r - PI;
}

inline constexpr degree normalize_degree(const degree& radian) {
    double x = floatmod(radian, 360);
    if (x < 0) {
        x += 360;
    }
    return x;
}

// Conversions
inline constexpr astronomical_unit toAU(kilometer km) { return km / KmInAu; }
inline constexpr light_year toLightYear(astronomical_unit au) { return au / 63241; }
inline constexpr astronomical_unit LtyrtoAU(light_year ltyr) { return ltyr * 63241; }
inline constexpr kilometer toKm(astronomical_unit au) { return au * KmInAu; }
inline constexpr radian toRadian(degree theta) { return theta * (PI / 180.f); }
inline constexpr degree toDegree(radian theta) { return theta * (180.f / PI); }

inline constexpr double operator""_deg(const long double deg) { return normalize_radian(toRadian(deg)); }
inline constexpr double operator""_au(const long double au) { return toKm(au); }
#ifdef floatmod
#undef floatmod
#endif  // floatmod
}