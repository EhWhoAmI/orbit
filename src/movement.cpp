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
#include "movement.h"

#include <cmath>

#include "coordinates.h"
#include "orbit.h"
#include "units.h"

namespace orbit {
void SysOrbit::DoSystem(double time) {
    ParseOrbitTree(entt::null, center);
}

void SysOrbit::LeaveSOI(const entt::entity& body, entt::entity& parent, Orbit& orb,
              Kinematics& pos, Kinematics& p_pos) {
    // Then change parent, then set the orbit
    auto& p_orb = registry.get<Orbit>(parent);
    if (p_orb.reference_body == entt::null) {
        return;
    }
    // Then add to orbital system
    registry.get<OrbitalSystem>(p_orb.reference_body).push_back(body);

    auto& parent_parent_orb = registry.get<Body>(p_orb.reference_body);

    auto& pp_pos = registry.get<Kinematics>(p_orb.reference_body);
    // Remove from parent
    auto& pt = registry.get<OrbitalSystem>(parent);
    std::erase(pt.children, body);
    // Get velocity and change posiiton
    // Convert orbit
    orb = Vec3ToOrbit(pos.position + p_pos.position, pos.velocity + p_pos.velocity, parent_parent_orb.GM,
                             registry.date.ToSecond());
    orb.reference_body = p_orb.reference_body;

    // Set new position
    pos.position = pos.position + p_pos.position;
    pos.velocity = pos.velocity + p_pos.velocity;
    // Update dirty orbit
    registry.emplace_or_replace<DirtyOrbit>(body);
}

void CrashObject(Registry& universe, Orbit& orb, entt::entity body, entt::entity parent) {
    if (universe.any_of<Body>(body)) {
        return;
    }
    auto& p_bod = universe.get<Body>(parent);
    auto& pos = universe.get<Kinematics>(body);
    if (universe.any_of<cqsps::Crash>(body)) {
        pos.position = glm::vec3(0);
    }

    // Next time we need to account for the atmosphere
    if (glm::length(pos.position) <= p_bod.radius) {
        // Crash
        // Then remove from the tree or something like that
        universe.get_or_emplace<cqsps::Crash>(body);
        pos.position = glm::vec3(0);
        orb.semi_major_axis = 0;
    }
}

void CalculateImpulse(Universe& universe, Orbit& orb, entt::entity body, entt::entity parent) {
    if (universe.any_of<Impulse>(body)) {
        // Then add to the orbit the speed.
        // Then also convert the velocity
        auto& impulse = universe.get<Impulse>(body);
        auto reference = orb.reference_body;
        auto& pos = universe.get_or_emplace<Kinematics>(body);

        orb = Vec3ToOrbit(pos.position, pos.velocity + impulse.impulse, orb.GM, universe.date.ToSecond());
        orb.reference_body = reference;
        pos.position = toVec3(orb);
        pos.velocity = OrbitVelocityToVec3(orb, orb.v);
        universe.emplace_or_replace<DirtyOrbit>(body);
        // Remove impulse
        universe.remove<Impulse>(body);
    }
}

void UpdateCommandQueue(Orbit& orb, entt::entity body, entt::entity parent) {
    // Process thrust before updating orbit
    if (!universe.any_of<cqspc::CommandQueue>(body)) {
        return;
    }
    // Check if the current date is beyond the universe date
    auto& queue = universe.get<cqspc::CommandQueue>(body);
    if (queue.commands.empty()) {
        return;
    }
    auto& command = queue.commands.front();
    if (command.time > universe.date.ToSecond()) {
        return;
    }
    // Then execute the command
    orb = ApplyImpulse(orb, command.delta_v, command.time);
    universe.emplace_or_replace<DirtyOrbit>(body);
    queue.commands.pop_front();
}

void SysOrbit::ParseOrbitTree(entt::entity parent, entt::entity body) {
    if (!universe.valid(body)) {
        return;
    }

    auto& orb = registry.get<Orbit>(body);

    UpdateCommandQueue(registry, orb, body, parent);

    UpdateOrbit(orb, registry.date.ToSecond());
    auto& pos = registry.get_or_emplace<Kinematics>(body);
    if (registry.any_of<SetTrueAnomaly>(body)) {
        orb.v = registry.get<SetTrueAnomaly>(body).true_anomaly;
        // Set new mean anomaly at epoch
        registry.remove<SetTrueAnomaly>(body);
    }
    pos.position = toVec3(orb);
    pos.velocity = OrbitVelocityToVec3(orb, orb.v);

    if (parent != entt::null) {
        auto& p_pos = registry.get_or_emplace<Kinematics>(parent);
        // If distance is above SOI, then be annoyed
        auto& p_bod = registry.get<Body>(parent);
        if (glm::length(pos.position) > p_bod.SOI) {
            LeaveSOI(registry, body, parent, orb, pos, p_pos);
        }

        CrashObject(registry, orb, body, parent);

        CalculateImpulse(registry, orb, body, parent);
        pos.center = p_pos.center + p_pos.position;
        EnterSOI(registry, parent, body);
    }

    auto& future_pos = registry.get_or_emplace<FuturePosition>(body);
    future_pos.position = OrbitTimeToVec3(orb, registry.date.ToSecond() + components::StarDate::TIME_INCREMENT);
    future_pos.center = pos.center;

    if (!registry.any_of<OrbitalSystem>(body)) {
        return;
    }
    for (entt::entity entity : registry.get<OrbitalSystem>(body).children) {
        // Calculate position
        ParseOrbitTree(body, entity);
    }
}

bool EnterSOI(onst entt::entity& parent, const entt::entity& body) {
    // We should ignore bodies
    if (registry.any_of<Body>(body)) {
        return false;
    }

    auto& pos = registry.get<Kinematics>(body);
    auto& orb = registry.get<Orbit>(body);
    // Check parents for SOI if we're inters ecting with anything
    auto& o_system = registry.get<OrbitalSystem>(parent);

    for (entt::entity entity : o_system.children) {
        // Get the stuff
        if (entity == body) {
            continue;
        }
        // Check the distance
        if (!registry.all_of<Body, Kinematics>(entity)) {
            continue;
        }
        const auto& body_comp = registry.get<Body>(entity);
        const auto& target_position = registry.get<Kinematics>(entity);
        if (glm::distance(target_position.position, pos.position) <= body_comp.SOI) {
            // Calculate position
            orb = Vec3ToOrbit(pos.position - target_position.position, pos.velocity - target_position.velocity,
                                     body_comp.GM, registry.date.ToSecond());
            orb.reference_body = entity;
            // Calculate position, and change the thing
            pos.position = toVec3(orb);
            pos.velocity = OrbitVelocityToVec3(orb, orb.v);
            // Then change SOI
            registry.get_or_emplace<OrbitalSystem>(entity).push_back(body);
            auto& vec = registry.get<OrbitalSystem>(parent).children;
            vec.erase(std::remove(vec.begin(), vec.end(), body), vec.end());
            registry.emplace_or_replace<DirtyOrbit>(body);
            return true;
        }
        // Now check if it's intersecting with any things outside of stuff
        if (parent == registry.sun) {
            continue;
        }
    }
    return false;
}
}  // namespace cqsp::common::systems
