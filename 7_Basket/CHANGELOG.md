### Changes Made
1. **Explicit C++14 and CUDA Standard**:
   - Added `set(CMAKE_CXX_STANDARD 14)` and `set(CMAKE_CXX_STANDARD_REQUIRED ON)` to ensure C++14 is used for all CXX targets.
   - Added `set(CMAKE_CUDA_STANDARD 14)` and `set(CMAKE_CUDA_STANDARD_REQUIRED ON)` to ensure CUDA code also uses C++14, maintaining consistency.
   - This resolves the structured bindings error, as the provided `gpu_slam_node.cpp` already uses C++14-compatible syntax.

2. **Added `gazebo_msgs` for `gpu_slam_node`**:
   - Ensured `gazebo_msgs` is included in `ament_target_dependencies` for `gpu_slam_node`, which should fix the undefined reference to `get_message_type_support_handle<gazebo_msgs::msg::ContactsState_>`.

3. **Added `gpu_grid` to Install Targets**:
   - Added `gpu_grid` to the `install(TARGETS ...)` section to ensure the CUDA library is installed correctly, which is a minor improvement to your original `CMakeLists.txt`.

### Additional Notes
- **Verify `gazebo_msgs` Installation**:
  Ensure `gazebo_msgs` is installed in your ROS 2 environment:
  ```bash
  sudo apt install ros-<distro>-gazebo-msgs
  ```
  Replace `<distro>` with your ROS 2 distribution (e.g., `foxy`, `galactic`, or `humble`).

- **Contact Sensor Plugin**:
  The URDF includes contact sensors on the gripper tips (`left_sphere_link` and `right_sphere_link`) using `libgazebo_ros_contact.so`. Ensure the Gazebo ROS plugins are installed:
  ```bash
  sudo apt install ros-<distro>-gazebo-ros-pkgs
  ```
  Also, verify that your Gazebo simulation is publishing contact messages on the topics `/left_gripper_contact` and `/right_gripper_contact`. You can check this with:
  ```bash
  ros2 topic list
  ros2 topic echo /left_gripper_contact
  ```

- **Wiggle Behavior**:
  The wiggle behavior in `gpu_slam_node.cpp` is designed to detect wall contact via the gripper’s contact sensors. When contact is detected, the robot moves backward (`linear.x = -0.15`) and turns based on which gripper is in contact (left contact turns right, right contact turns left) for 2 seconds, then replans the path. This should help the robot avoid getting stuck against walls in your maze world.

- **Testing**:
  After updating the `CMakeLists.txt` and using the provided `gpu_slam_node.cpp`, rebuild your workspace:
  ```bash
  colcon build --symlink-install
  source install/setup.bash
  ```
  Launch your simulation and test the robot’s behavior. If the robot still gets stuck, you may need to:
  - Adjust the wiggle duration (currently 2 seconds) or speeds (`linear.x`, `angular.z`) in the `moveRobot` function.
  - Check if the contact sensor messages are being published correctly.
  - Ensure the laser scanner’s field of view and range are sufficient to detect walls early.

- **URDF Confirmation**:
  The URDF you provided earlier (with contact sensors on the gripper tips) is unchanged and matches your requirement to keep the gripper’s shape. The contact sensors are correctly configured to detect collisions, which triggers the wiggle behavior.

### Troubleshooting
If you encounter further issues:
1. **Linker Errors Persist**:
   - Double-check that `gazebo_msgs` is installed and that your ROS 2 environment is sourced correctly.
   - Verify that your workspace is clean before rebuilding:
     ```bash
     rm -rf build install log
     colcon build --symlink-install
     ```

2. **Robot Still Gets Stuck**:
   - Increase the laser scan obstacle detection range (e.g., change `min_front < 0.3` to `min_front < 0.5`) in the `moveRobot` function to avoid walls earlier.
   - Adjust the wiggle maneuver parameters (e.g., increase `cmd.linear.x = -0.2` or extend wiggle duration to 3 seconds by changing `elapsed < 2.0` to `elapsed < 3.0`).
   - Check the Gazebo world file to ensure walls are correctly aligned and detected by the laser scanner and contact sensors.

3. **Contact Sensor Issues**:
   - If contact messages are not being published, verify that the Gazebo simulation is running with the correct plugins. You may need to check your launch file to ensure the robot model is spawned with the URDF that includes the contact sensors.