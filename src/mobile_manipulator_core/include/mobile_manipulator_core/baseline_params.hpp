#pragma once

// Single source of truth for the hardcoded baseline task parameters.
//
// These used to be duplicated: baseline_controller.cpp defined its own set,
// and experiment_runner.cpp defined a second set for the copy of the
// baseline sequence it runs under `--system baseline`. The two drifted --
// the runner kept APPROACH_NUDGE_M at 0.15 after the controller was
// calibrated to 0.30 -- so "the baseline" meant two different controllers
// depending on how it was launched, and their results were not comparable.
// Both now include this header.
namespace baseline_params {

constexpr const char* PICK_LOCATION    = "table";
constexpr const char* DELIVER_LOCATION = "dispatch_zone";

// Cube poses match the spawns in warehouse.sdf (model centre); the pick
// target is the top face, i.e. +half the 4 cm cube height. These are the
// TRUE positions, not fudged -- see APPROACH_NUDGE_M for why a fixed
// approach step is still needed before they are reachable.

// Red cube -- spawn: <pose>2.0 0.12 0.40 0 0 0</pose>
constexpr const char* RED_LABEL = "red_cube";
constexpr double RED_X = 2.0, RED_Y = 0.12, RED_Z = 0.42;

// Green cube -- spawn: <pose>2.0 0.00 0.40 0 0 0</pose>
constexpr const char* GREEN_LABEL = "green_cube";
constexpr double GREEN_X = 2.0, GREEN_Y = 0.00, GREEN_Z = 0.42;

// Approach nudge: the Nav2 standoff in front of the table leaves the cube
// at the edge of arm reach. 0.15 m was insufficient and pre-grasp IK
// failed; 0.30 m is the calibrated value. Changing this changes what
// "baseline" means -- re-run both the standalone controller and the
// experiment runner if it moves.
constexpr double APPROACH_NUDGE_M = 0.30;

}  // namespace baseline_params
