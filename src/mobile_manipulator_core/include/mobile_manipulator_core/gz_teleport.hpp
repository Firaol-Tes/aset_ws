#pragma once
#include <string>

// Shells out to the `gz service` CLI rather than linking gz-transport
// directly -- confirmed working via live testing against the running
// simulation (see project memory); avoids new CMake dependency risk for a
// mechanism that's only needed for experiment orchestration, not the core
// robot pipeline.
namespace gz_teleport {

// Teleports a named entity in `world` to (x, y, z), identity orientation.
// Returns true iff the service call reported success ("data: true").
//
// IMPORTANT: this only sets the instantaneous pose -- gravity resumes
// immediately afterward. The target must be physically supported (e.g.
// resting on a known surface at its correct height) or it will fall.
bool teleport(const std::string& world, const std::string& name,
              double x, double y, double z);

}  // namespace gz_teleport
