# Koch ROS2 Package

## Setup Instructions

### Step 0: Verify Motor Setup
Ensure the Dynamixel motor ID and baud rate are correctly configured. Refer to the [Koch Setup Guide](https://github.com/hrc-pme/lerobot/blob/main/examples/7_get_started_with_real_robot.md) for detailed hardware setup instructions.  
⚠️ Important: you will need [dynamixel wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/) for motor checking. 

---

### Step 1: Set Custom USB Port
Create a custom symlink for your USB device in the `/dev` directory using `udev` rules. This ensures the device always appears with a unique name, such as `/dev/tty_changeme`.

1. **Identify the Device:**
   ```bash
   udevadm info -a -n /dev/ttyACM0
   ```
   Note attributes like `idVendor`, `idProduct`, or `serial`.

2. **Create a udev Rule File:**
   ```bash
   sudo vim /etc/udev/rules.d/99-koch-arm.rules
   ```
3. **Add the idVendorRule:**
   ```text
   SUBSYSTEM=="tty", ATTRS{idVendor}=="1234", ATTRS{idProduct}=="5678", ATTRS{serial}=="12345678", SYMLINK+="tty_changeme"
   ```
   Replace `1234`, `5678`, and `12345678` with your device's attributes.  
   Check [`.99-koch-arm.rules`](https://github.com/hrc-pme/koch_ros2_wrapper/blob/master/.99-koch-arm.rules) as example
4. **Reload and Test:**
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ls /dev/
   ```
   Confirm `/dev/tty_changeme` appears.

---

### Step 2: Verify Arm Configuration
Check the configuration YAML file for the correct arm setup. Example for a leader-follower arm configuration:

```yaml
leader_arms:
  - name: "left_leader"
    port: "/dev/ttykoch_left_leader"
    motors:
      shoulder_pan: [1, "xl330-m077"]
      shoulder_lift: [2, "xl330-m077"]
      elbow_flex: [3, "xl330-m077"]
      wrist_flex: [4, "xl330-m077"]
      wrist_roll: [5, "xl330-m077"]
      gripper: [6, "xl330-m077"]
follower_arms:
  - name: "left_follower"
    port: "/dev/ttykoch_left_follower"
    motors:
      shoulder_pan: [1, "xl430-w250"]
      shoulder_lift: [2, "xl430-w250"]
      elbow_flex: [3, "xl330-m288"]
      wrist_flex: [4, "xl330-m288"]
      wrist_roll: [5, "xl330-m288"]
      gripper: [6, "xl330-m288"]
```

---

### Step 3: Calibrate the Arm
Run the following to calibrate the arm for the first use:
```bash
ros2 run koch_ros2_wrapper koch_calibration --ros-args -p config_file:="/path/to/your_custom_config.yaml"
```
The calibration file will be saved to `~/koch_robot_arm/calibration`.

---

## Reminders
1. After modifying files in the ROS package, rebuild with:
   ```bash
   cd ros2_ws  
   colcon build --packages-select koch_ros2_wrapper
   ```
2. Load the workspace parameters before running ROS commands: (150 is the ROS_DOMAIN ID)
   ```bash
   source environment.sh 150
   ```
3. ⚠️ Name of launch file need to **XXX_OOO_launch.py**
---

## How to Run
### Setup Options
Now we have three example of koch leader-follower arm set.
1. **Single Leader**
2. **Single Follower**
3. **Single Leader-Follower**

Ensure calibration is complete before running setup.

### Running with Unity
Start a ROSBridge server for Unity integration:
```bash
source environment.sh
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
Verify the server starts on port `9090`.(Terminal will have info: server on 9090) 

#### Single Leader
1. Terminal1 **Run Control Node:**
   ```bash
   ros2 run koch_ros2_wrapper koch_leader_control --ros-args -p config_file:="/path/to/single_leader.yaml"
   e.g. ros2 run koch_ros2_wrapper koch_leader_control --ros-args -p config_file:="/home/$USER/koch_robot_arm/ros2_ws/src/koch_ros2_wrapper/config/single_leader.yaml"
   ```
2. Terminal2 **Verify Joint States:**
   ```bash
   ros2 topic echo /left_leader/joint_states
   ```

#### Single Follower
1. Terminal1 **Run Control Node:**
   ```bash
   ros2 run koch_ros2_wrapper koch_follower_control --ros-args -p config_file:="/path/to/single_follower.yaml"
   ```
2. Terminal2 **Verify Topics:**
   ```bash
   ros2 topic echo /left_follower/joint_states
   ```
3. Terminal3 **Reset Position:**
   ```bash
   ros2 service call /left_follower_reset_position_to_zero std_srvs/srv/Trigger
   ```

#### Single Leader-Follower
1. Terminal1 **Run Leader-Follower Node:**
   ```bash
   ros2 run koch_ros2_wrapper koch_leader_follower --ros-args -p config_file:="/path/to/single_leader_follower.yaml"
   ```
2. Terminal2 **Relay Joint States:**
   ```bash
   ros2 run topic_tools relay /left_leader/joint_states /left_follower/joint_states_control
   ```
Move the leader arm to control the follower arm.

---
### RUN With Camera
Now we have some launch file to run with koch.  
check the launch folder for different kinds of launch file.  
⚠️ Reminder: Name of launch file need to **XXX_OOO_launch.py**
1. launch 1 camera (only 1 camera can use)
```
ros2 run realsense2_camera rs_launch.py 
```
2. launch specific camera with serial_no
```
ros2 run realsense2_camera rs_far_launch.py
```
---

## Notes
- Customize YAML files for specific configurations.
- Confirm proper calibration before running any control setup.
- ⚠️ Check you have done `source environment.sh` when you open a new terminal, or you have done `source set_env_param.sh` in 1st terminal to set the parameter into ~/.bashrc 