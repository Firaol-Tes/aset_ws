#include "mobile_manipulator_core/gz_pose.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

namespace gz_pose {
namespace {

// Pulls one message off /world/<world>/pose/info. The topic carries a
// Pose_V, printed as a sequence of `pose { name: ... position { x: ... } }`
// blocks -- one per model and per link.
std::string snapshot(const std::string& world)
{
  std::ostringstream cmd;
  cmd << "gz topic -e -t /world/" << world << "/pose/info -n 1 2>&1";

  std::array<char, 512> buf{};
  std::string out;
  FILE* pipe = popen(cmd.str().c_str(), "r");
  if (!pipe) return out;
  while (fgets(buf.data(), buf.size(), pipe) != nullptr) out += buf.data();
  pclose(pipe);
  return out;
}

std::string trim(const std::string& s)
{
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) return "";
  const auto e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

// Parses `key: value` and returns true on a match for `key`.
bool scalar(const std::string& line, const char* key, double& out)
{
  const std::string k = std::string(key) + ":";
  const std::string t = trim(line);
  if (t.rfind(k, 0) != 0) return false;
  try {
    out = std::stod(trim(t.substr(k.size())));
  } catch (const std::exception&) {
    return false;
  }
  return true;
}

// Walks the printed Pose_V looking for the block whose name is exactly
// `name`, then reads the x/y/z of the position block that follows it.
// Anything else (other entities, orientation quaternions) is skipped.
bool parse(const std::string& text, const std::string& name, Pose& out)
{
  const std::string want = "name: \"" + name + "\"";

  std::istringstream in(text);
  std::string line;
  bool in_target = false;      // inside the block for `name`
  bool in_position = false;    // inside that block's `position {`
  bool got_x = false, got_y = false, got_z = false;
  Pose p;

  while (std::getline(in, line)) {
    const std::string t = trim(line);

    if (t.rfind("name:", 0) == 0) {
      // A new named block starts here: either ours, or one that ends ours.
      in_target = (t == want);
      in_position = false;
      got_x = got_y = got_z = false;
      continue;
    }
    if (!in_target) continue;

    if (t.rfind("position", 0) == 0) { in_position = true; continue; }
    if (t.rfind("orientation", 0) == 0) { in_position = false; continue; }
    if (!in_position) continue;

    if (scalar(t, "x", p.x)) { got_x = true; continue; }
    if (scalar(t, "y", p.y)) { got_y = true; continue; }
    if (scalar(t, "z", p.z)) { got_z = true; continue; }

    if (got_x && got_y && got_z) break;
  }

  if (!(got_x && got_y && got_z)) return false;
  out = p;
  return true;
}

}  // namespace

bool get(const std::string& world, const std::string& name, Pose& out,
         int retries)
{
  for (int i = 0; i < retries; ++i) {
    const std::string text = snapshot(world);
    if (!text.empty() && parse(text, name, out)) return true;
  }
  return false;
}

bool within_zone(const std::string& world, const std::string& name,
                 double cx, double cy, double half_xy, double max_z,
                 Pose& out)
{
  if (!get(world, name, out)) return false;
  return std::fabs(out.x - cx) <= half_xy &&
         std::fabs(out.y - cy) <= half_xy &&
         out.z <= max_z;
}

}  // namespace gz_pose
