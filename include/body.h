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

#include <limits>
#include <map>
#include <string>
#include <tuple>
#include <vector>

#include <entt/entt.hpp>

#include "units.h"

namespace orbit {
struct Body {
    /*
     * Radius of the body
     */
    kilometer radius;

    /// <summary>
    /// Radius of sphere of influence
    /// rsoi = a(m/M)^2/5
    /// </summary>
    kilometer SOI = std::numeric_limits<double>::infinity();
    kilogram mass;

    // gravitational constant in km^3 * s^-2
    double GM;

    // Rotation period in seconds
    double rotation;

    // Axial rotation
    double axial = 0.0;

    double rotation_offset = 0.0;
};

/// <summary>
/// Calculates SOI
/// </summary>
/// Make sure the units match up
/// <param name="mass">Mass of planet/body to calculate</param>
/// <param name="reference_mass">Mass of sun/reference body</param>
/// <param name="sma">Semi major axis of the planet</param>
/// <returns>SOI of planet</returns>
inline double CalculateSOI(const double& mass, const double& reference_mass, const double& sma) {
    return sma * std::pow(mass / reference_mass, 0.4);
}

/// <summary>
/// Calculates mass from gravitational constant
/// </summary>
/// Masses of bodies are described in gravitational constant because it's more accurate to describe like that
/// <param name="GM"></param>
/// <returns></returns>
inline double CalculateMass(const double& GM) { return GM / G_km; }

/// <summary>
/// Calculates the current planet rotation angle
/// </summary>
/// <param name="time">current time in seconds</param>
/// <param name="day_length">length of day in seconds</param>
/// <param name="offset">offset of day</param>
/// <returns>Angle the planet should be in radians</returns>
inline double GetPlanetRotationAngle(const double& time, const double& day_length, const double& offset) {
    return (time / day_length - offset) * TWOPI;
}

/// <summary>
/// An object for the children of an orbital object.
/// </summary>
struct OrbitalSystem {
    // Set the tree
    std::vector<entt::entity> children;
    void push_back(const entt::entity& entity) { children.push_back(entity); }
};
}
