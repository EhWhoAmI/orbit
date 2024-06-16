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

#include <vector>

#include <entt/entt.hpp>
namespace orbit {
/// @brief A
/// This class processes all the orbits. It doesn't have to be completely followed,
class SysOrbit<Registry> {
 public:
    explicit SysOrbit(Registry& registry) : registry(registry) {}
    void DoSystem(double time) override;

    void ParseOrbitTree(entt::entity parent, entt::entity body);
 private:
    Registry& registry;

    /// <summary>
    /// Change the current body's SOI into a child SOI
    /// </summary>
    /// <param name="universe"></param>
    /// <param name="parent"></param>
    /// <param name="body">Body that we want to check if it's entering a SOI</param>
    bool EnterSOI(const entt::entity& parent, const entt::entity& body);
    
    /// <summary>
    /// Check if the entity has crashed into its parent object
    /// </summary>
    /// <param name="registry"></param>
    /// <param name="orb"></param>
    /// <param name="body"></param>
    /// <param name="parent"></param>
    void CrashObject(Orbit& orb, entt::entity body, entt::entity parent);

    void LeaveSOI(const entt::entity& body, entt::entity& parent, Orbit& orb,
              Kinematics& pos, Kinematics& p_pos)
    entt::entity center;
};
}  // namespace systems
