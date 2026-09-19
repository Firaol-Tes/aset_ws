#pragma once
#include <string>

// Ground-truth pose readback from the running simulation.
//
// Companion to gz_teleport.hpp and written in the same idiom: shells out to
// the `gz` CLI rather than linking gz-transport, since this is experiment
// orchestration, not part of the robot pipeline.
//
// WHY THIS EXISTS: trial success was previously self-reported -- the LLM's
// own task_complete(success=...) or the baseline's last step result -- and
// tool_server's place service returns success unconditionally, including on
// the fallback path where pre-place IK fails and the object is simply
// released from wherever the arm happens to be. That means a run could be
// scored a success with the cube on the floor outside the dispatch zone.
// This reads the cube's actual world pose so delivery can be checked
// against the zone geometry in warehouse.sdf instead of being taken on
// trust.
namespace gz_pose {

struct Pose { double x{0.0}, y{0.0}, z{0.0}; };

// Snapshots /world/<world>/pose/info and returns the pose published for
// `name` (exact match -- link-scoped names like "red_cube::link" are
// ignored). Returns false if the entity does not appear in `retries`
// consecutive snapshots; a single snapshot can legitimately miss an entity,
// so the retry is not optional.
bool get(const std::string& world, const std::string& name, Pose& out,
         int retries = 3);

// True iff `name` currently sits inside the axis-aligned box centred on
// (cx, cy) with half-extent `half_xy` in the world XY plane, and below
// `max_z`. The z bound distinguishes "placed in the zone" from "still in
// the gripper while the robot stands over the zone".
bool within_zone(const std::string& world, const std::string& name,
                 double cx, double cy, double half_xy, double max_z,
                 Pose& out);

}  // namespace gz_pose
