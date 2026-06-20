cmake_minimum_required(VERSION 3.8)
project(mm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "srv/GetSceneState.srv"
  "srv/NavigateTo.srv"
  "srv/DetectObjects.srv"
  "srv/MoveArmToNamed.srv"
  "srv/MoveArmToPose.srv"
  "srv/Pick.srv"
  "srv/Place.srv"
  "srv/OpenGripper.srv"
  "srv/CloseGripper.srv"
  "srv/DriveDistance.srv"
  "action/ExecuteTask.action"
  DEPENDENCIES std_msgs builtin_interfaces action_msgs
)

ament_package()
