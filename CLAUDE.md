STANDING RULES for this project:
- Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (gz sim) — NOT Gazebo Classic.
- All ROS 2 nodes in C++ (rclcpp, ament_cmake) EXCEPT exactly two Python
  nodes: YOLOv8 perception node and LLM API client node (rclpy).
- Python venv at ~/aset_ws/venv (ultralytics, anthropic, openai installed).
- Packages:
  robotic_arm_description  (urdf/xacro/meshes)
  arm_moveit_config        (MoveIt 2 config)
  mobile_manipulator_core  (C++: tool server, state manager, baseline,
                            experiment runner)
  mm_perception            (Python: YOLO node)
  mm_llm_planner           (Python: LLM client node)
  mm_bringup               (launch files + worlds)
- Use MoveIt 2 C++ API (MoveGroupInterface) and Nav2 C++ action clients.
- Work incrementally: build and verify after each step, explain changes.
- Never invent joint limits or masses — use what's in my URDF unless I
  explicitly approve a change.
- After each phase passes verification, create a git commit with a
  descriptive message like "Phase 3: mobile base + warehouse world +
  SLAM/Nav2 working". Never commit broken builds. Do not add any
  co-author or AI attribution lines to commits.
