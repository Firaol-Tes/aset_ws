#include "mobile_manipulator_core/gz_teleport.hpp"

#include <array>
#include <cstdio>
#include <sstream>

namespace gz_teleport {

bool teleport(const std::string& world, const std::string& name,
              double x, double y, double z)
{
  std::ostringstream cmd;
  cmd << "gz service -s /world/" << world << "/set_pose"
      << " --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000"
      << " --req 'name: \"" << name << "\", position: {x: " << x
      << ", y: " << y << ", z: " << z << "}' 2>&1";

  std::array<char, 256> buf{};
  std::string output;
  FILE* pipe = popen(cmd.str().c_str(), "r");
  if (!pipe) return false;
  while (fgets(buf.data(), buf.size(), pipe) != nullptr) {
    output += buf.data();
  }
  pclose(pipe);

  return output.find("data: true") != std::string::npos;
}

}  // namespace gz_teleport
