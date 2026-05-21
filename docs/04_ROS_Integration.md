# 04. ROS 통합 설계 (ROS Integration)

## 1. 개요

본 문서는 ROS (Robot Operating System)를 활용한 Leader-Follower 암 시스템의 통합 설계를 다룹니다.
ROS는 Leader arm의 조인트 상태를 Follower arm에 전달하는 미들웨어 역할을 하며, Isaac Sim 및 Jetson Orin Nano와의 통합도 담당합니다.

## 2. ROS 버전 및 환경

### 2.1 권장 ROS 버전

| 버전 | 장점 | 단점 | 권장 용도 |
|------|------|------|----------|
| **ROS2 Humble** | Isaac Sim 네이티브 지원, micro-ROS, Jetson 지원 | 학습 곡선 | **⭐ 메인 플랫폼** |
| ROS1 Noetic | 안정적인 rosserial, 많은 예제 | 레거시, 2025년 EOL | ROS2 전환 전 임시 |

**본 프로젝트는 ROS2 Humble을 기준으로 설계합니다.**

### 2.2 시스템 요구사항

| 구성 요소 | 요구사항 |
|-----------|---------|
| 운영 체제 | Ubuntu 22.04 LTS |
| ROS 버전 | ROS2 Humble Hawksbill |
| ROS2 설치 | `ros-humble-desktop` (전체 설치) |
| 추가 패키지 | `ros-humble-robot-state-publisher` |
| | `ros-humble-joint-state-publisher` |
| | `ros-humble-ros2-control` |
| | `ros-humble-ros2-controllers` |
| | `ros-humble-xacro` |
| | `ros-humble-urdf` |
| | `micro_ros_setup` (from GitHub) |

## 3. ROS 워크스페이스 구조

### 3.1 패키지 개요

```
ros_ws/
├── src/
│   ├── leader_follower_description/     # URDF 모델, 메시, RViz config
│   │   ├── urdf/
│   │   │   ├── leader_arm.urdf.xacro    # Leader Arm URDF (xacro)
│   │   │   ├── follower_arm.urdf.xacro  # Follower Arm URDF
│   │   │   ├── leader_follower.urdf.xacro # 통합 URDF
│   │   │   └── macros.urdf.xacro        # 공통 매크로
│   │   ├── meshes/
│   │   │   ├── base_link.stl
│   │   │   ├── link_1.stl
│   │   │   └── ... (각 링크 STL)
│   │   ├── config/
│   │   │   ├── leader.rviz              # Leader RViz 설정
│   │   │   └── follower.rviz            # Follower RViz 설정
│   │   ├── launch/
│   │   │   ├── display_leader.launch.py # URDF 시각화
│   │   │   └── display_follower.launch.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── leader_arm_controller/           # Leader arm 제어 ROS2 노드
│   │   ├── src/
│   │   │   ├── leader_controller_node.cpp
│   │   │   ├── serial_interface.cpp     # STM32 직렬 통신
│   │   │   └── joint_publisher.cpp      # Joint state 발행
│   │   ├── include/
│   │   │   └── leader_controller/
│   │   │       ├── serial_interface.h
│   │   │       └── joint_publisher.h
│   │   ├── config/
│   │   │   └── leader_params.yaml       # Leader 파라미터
│   │   ├── launch/
│   │   │   └── leader_controller.launch.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── follower_arm_controller/         # Follower arm 제어 ROS2 노드
│   │   ├── src/
│   │   │   ├── follower_controller_node.cpp
│   │   │   ├── serial_interface.cpp     # STM32 직렬 통신
│   │   │   └── joint_subscriber.cpp     # Joint state 구독
│   │   ├── include/
│   │   │   └── follower_controller/
│   │   │       ├── serial_interface.h
│   │   │       └── joint_subscriber.h
│   │   ├── config/
│   │   │   └── follower_params.yaml
│   │   ├── launch/
│   │   │   └── follower_controller.launch.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── leader_follower_bringup/         # 시스템 실행 (Launch 파일 집합)
│   │   ├── launch/
│   │   │   ├── leader_follower_system.launch.py  # 전체 시스템
│   │   │   ├── leader_only.launch.py             # Leader 단독
│   │   │   ├── follower_only.launch.py           # Follower 단독
│   │   │   ├── simulation_mode.launch.py         # 시뮬레이션 모드
│   │   │   └── inference_mode.launch.py          # AI 추론 모드
│   │   ├── config/
│   │   │   ├── leader_controllers.yaml           # ros2_control config
│   │   │   └── follower_controllers.yaml
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── leader_follower_msgs/            # 커스텀 메시지/서비스
│   │   ├── msg/
│   │   │   ├── JointCommand.msg
│   │   │   └── ArmStatus.msg
│   │   ├── srv/
│   │   │   ├── SetMode.srv
│   │   │   ├── Calibrate.srv
│   │   │   └── RecordTrajectory.srv
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── leader_follower_sim/             # 시뮬레이션 브릿지
│       ├── src/
│       │   └── sim_bridge_node.cpp
│       ├── config/
│       │   └── sim_bridge_params.yaml
│       ├── launch/
│       │   └── sim_bridge.launch.py
│       ├── CMakeLists.txt
│       └── package.xml
│
├── build/                               # 빌드 출력
├── install/                             # 설치 파일
└── log/                                 # 로그
```

## 4. URDF 모델 설계

### 4.1 Leader Arm URDF (xacro)

파일: `leader_follower_description/urdf/leader_arm.urdf.xacro`

```xml
<?xml version="1.0"?>
<robot name="leader_arm" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Properties -->
  <xacro:property name="PI" value="3.14159265359"/>
  <xacro:property name="DEG_TO_RAD" value="0.0174533"/>

  <!-- Link Dimensions (mm → m) -->
  <xacro:property name="L1" value="0.06"/>   <!-- Base → Shoulder -->
  <xacro:property name="L2" value="0.14"/>   <!-- Upper Arm -->
  <xacro:property name="L3" value="0.12"/>   <!-- Forearm -->
  <xacro:property name="L4" value="0.08"/>   <!-- Wrist -->
  <xacro:property name="L5" value="0.05"/>   <!-- End Effector -->

  <!-- Colors -->
  <material name="blue">
    <color rgba="0.2 0.4 0.8 1.0"/>
  </material>
  <material name="gray">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>

  <!-- Links -->
  <link name="base_link">
    <visual>
      <geometry><box size="0.12 0.12 0.02"/></geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry><box size="0.12 0.12 0.02"/></geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0"
               izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint 1: Waist (Revolute, Yaw) -->
  <joint name="joint_1_waist" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.01" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-150*DEG_TO_RAD}" upper="${150*DEG_TO_RAD}"
           effort="1.5" velocity="2.0"/>
    <dynamics damping="0.1" friction="0.05"/>
  </joint>

  <link name="link_1">
    <visual>
      <origin xyz="0 0 ${L1/2}" rpy="0 0 0"/>
      <geometry><cylinder radius="0.03" length="${L1}"/></geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${L1/2}" rpy="0 0 0"/>
      <geometry><cylinder radius="0.03" length="${L1}"/></geometry>
    </collision>
    <inertial>
      <mass value="0.15"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0"
               iyy="0.0001" iyz="0.0"
               izz="0.0001"/>
    </inertial>
  </link>

  <!-- Joint 2: Shoulder (Revolute, Pitch) -->
  <joint name="joint_2_shoulder" type="revolute">
    <parent link="link_1"/>
    <child link="link_2"/>
    <origin xyz="0 0 ${L1}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-105*DEG_TO_RAD}" upper="${105*DEG_TO_RAD}"
           effort="1.5" velocity="2.0"/>
    <dynamics damping="0.1" friction="0.05"/>
  </joint>

  <link name="link_2">
    <visual>
      <origin xyz="0 0 ${L2/2}" rpy="0 0 0"/>
      <geometry><box size="0.04 0.04 ${L2}"/></geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${L2/2}" rpy="0 0 0"/>
      <geometry><box size="0.04 0.04 ${L2}"/></geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.0002" ixy="0.0" ixz="0.0"
               iyy="0.0002" iyz="0.0"
               izz="0.0001"/>
    </inertial>
  </link>

  <!-- Joint 3: Elbow (Revolute, Pitch) -->
  <joint name="joint_3_elbow" type="revolute">
    <parent link="link_2"/>
    <child link="link_3"/>
    <origin xyz="0 0 ${L2}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-105*DEG_TO_RAD}" upper="${105*DEG_TO_RAD}"
           effort="1.5" velocity="2.0"/>
    <dynamics damping="0.1" friction="0.05"/>
  </joint>

  <link name="link_3">
    <visual>
      <origin xyz="0 0 ${L3/2}" rpy="0 0 0"/>
      <geometry><box size="0.03 0.03 ${L3}"/></geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${L3/2}" rpy="0 0 0"/>
      <geometry><box size="0.03 0.03 ${L3}"/></geometry>
    </collision>
    <inertial>
      <mass value="0.15"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0"
               iyy="0.0001" iyz="0.0"
               izz="0.0001"/>
    </inertial>
  </link>

  <!-- Joint 4: Wrist Roll -->
  <joint name="joint_4_wrist_roll" type="revolute">
    <parent link="link_3"/>
    <child link="link_4"/>
    <origin xyz="0 0 ${L3}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-150*DEG_TO_RAD}" upper="${150*DEG_TO_RAD}"
           effort="1.5" velocity="2.0"/>
    <dynamics damping="0.05" friction="0.02"/>
  </joint>

  <link name="link_4">
    <visual>
      <origin xyz="0 0 ${L4/2}" rpy="0 0 0"/>
      <geometry><cylinder radius="0.025" length="${L4}"/></geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${L4/2}" rpy="0 0 0"/>
      <geometry><cylinder radius="0.025" length="${L4}"/></geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.00005" ixy="0.0" ixz="0.0"
               iyy="0.00005" iyz="0.0"
               izz="0.00005"/>
    </inertial>
  </link>

  <!-- Joint 5: Wrist Pitch -->
  <joint name="joint_5_wrist_pitch" type="revolute">
    <parent link="link_4"/>
    <child link="link_5"/>
    <origin xyz="0 0 ${L4}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-90*DEG_TO_RAD}" upper="${90*DEG_TO_RAD}"
           effort="1.5" velocity="2.0"/>
    <dynamics damping="0.05" friction="0.02"/>
  </joint>

  <link name="link_5">
    <visual>
      <origin xyz="0 0 ${L5/2}" rpy="0 0 0"/>
      <geometry><box size="0.025 0.025 ${L5}"/></geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${L5/2}" rpy="0 0 0"/>
      <geometry><box size="0.025 0.025 ${L5}"/></geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.00002" ixy="0.0" ixz="0.0"
               iyy="0.00002" iyz="0.0"
               izz="0.00002"/>
    </inertial>
  </link>

  <!-- Joint 6: Wrist Yaw -->
  <joint name="joint_6_wrist_yaw" type="revolute">
    <parent link="link_5"/>
    <child link="end_effector"/>
    <origin xyz="0 0 ${L5}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-150*DEG_TO_RAD}" upper="${150*DEG_TO_RAD}"
           effort="1.5" velocity="2.0"/>
    <dynamics damping="0.05" friction="0.02"/>
  </joint>

  <link name="end_effector">
    <visual>
      <origin xyz="0 0 0.02" rpy="0 0 0"/>
      <geometry><box size="0.02 0.02 0.04"/></geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.02" rpy="0 0 0"/>
      <geometry><box size="0.02 0.02 0.04"/></geometry>
    </collision>
    <inertial>
      <mass value="0.02"/>
      <inertia ixx="0.00001" ixy="0.0" ixz="0.0"
               iyy="0.00001" iyz="0.0"
               izz="0.00001"/>
    </inertial>
  </link>

  <!-- ROS 2 Control Transmission -->
  <ros2_control name="LeaderArm" type="system">
    <hardware>
      <plugin>leader_follower_hardware/LeaderHardware</plugin>
      <param name="serial_port">/dev/ttyUSB0</param>
      <param name="baud_rate">1000000</param>
    </hardware>
    <joint name="joint_1_waist">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="joint_2_shoulder">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <!-- Joint 3~6 동일 패턴 -->
  </ros2_control>
</robot>
```

### 4.2 DH 파라미터

```
DH Convention (Modified Denavit-Hartenberg):

| Joint | α(i-1) | a(i-1) | d(i) | θ(i) |
|-------|---------|--------|------|------|
| J1    | 0       | 0      | L1   | θ1   |
| J2    | -90°    | 0      | 0    | θ2   |
| J3    | 0       | L2     | 0    | θ3   |
| J4    | 0       | L3     | 0    | θ4   |
| J5    | -90°    | 0      | 0    | θ5   |
| J6    | 90°     | 0      | L4+L5| θ6   |

정방향 기구학 (Forward Kinematics):
T_base_to_ee = T1 × T2 × T3 × T4 × T5 × T6
```

## 5. Leader Arm Controller 노드

### 5.1 노드 설계

파일: `leader_arm_controller/src/leader_controller_node.cpp`

```cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "leader_follower_msgs/msg/joint_command.hpp"
#include "leader_follower_msgs/srv/set_mode.hpp"
#include "serial_interface.h"

class LeaderControllerNode : public rclcpp::Node {
public:
    LeaderControllerNode() : Node("leader_arm_controller") {
        // 파라미터 선언
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 1000000);
        this->declare_parameter<double>("publish_rate", 100.0);  // Hz

        // 직렬 인터페이스 초기화
        std::string port = this->get_parameter("serial_port").as_string();
        int baud = this->get_parameter("baud_rate").as_int();
        serial_iface_.init(port, baud);

        // Publisher: 조인트 상태
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/leader_joint_states", 10);

        // Subscriber: 조인트 명령 (Leader도 외부 명령 수신 가능)
        joint_cmd_sub_ = this->create_subscription<leader_follower_msgs::msg::JointCommand>(
            "/leader_joint_command", 10,
            std::bind(&LeaderControllerNode::jointCmdCallback, this, std::placeholders::_1));

        // Service: 모드 변경
        set_mode_srv_ = this->create_service<leader_follower_msgs::srv::SetMode>(
            "/set_mode",
            std::bind(&LeaderControllerNode::setModeCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // 타이머: 정기적인 조인트 상태 발행
        double rate = this->get_parameter("publish_rate").as_double();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(1000.0 / rate)),
            std::bind(&LeaderControllerNode::timerCallback, this));

        // 조인트 이름 초기화
        joint_names_ = {
            "joint_1_waist", "joint_2_shoulder", "joint_3_elbow",
            "joint_4_wrist_roll", "joint_5_wrist_pitch", "joint_6_wrist_yaw"
        };

        RCLCPP_INFO(this->get_logger(), "Leader Arm Controller started");
    }

private:
    void timerCallback() {
        // STM32로부터 현재 조인트 각도 읽기
        std::vector<double> positions;
        if (serial_iface_.readJointPositions(positions) && positions.size() == 6) {
            auto msg = sensor_msgs::msg::JointState();
            msg.header.stamp = this->now();
            msg.name = joint_names_;
            msg.position = positions;
            msg.velocity = std::vector<double>(6, 0.0);  // 속도 데이터는 선택
            msg.effort = std::vector<double>(6, 0.0);    // 토크 데이터는 선택
            joint_state_pub_->publish(msg);
        }
    }

    void jointCmdCallback(const leader_follower_msgs::msg::JointCommand::SharedPtr msg) {
        // Leader arm에 외부 명령 전달
        if (msg->joint_angles.size() >= 6) {
            serial_iface_.writeJointCommands(msg->joint_angles);
        }
    }

    void setModeCallback(
        const std::shared_ptr<leader_follower_msgs::srv::SetMode::Request> request,
        std::shared_ptr<leader_follower_msgs::srv::SetMode::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Mode change requested: %s",
                    request->mode.c_str());
        // 모드 변경 처리
        serial_iface_.setMode(request->mode);
        response->success = true;
        response->message = "Mode changed to " + request->mode;
    }

    // 멤버 변수
    SerialInterface serial_iface_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<leader_follower_msgs::msg::JointCommand>::SharedPtr joint_cmd_sub_;
    rclcpp::Service<leader_follower_msgs::srv::SetMode>::SharedPtr set_mode_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> joint_names_;
};
```

## 6. Follower Arm Controller 노드

### 6.1 노드 설계

파일: `follower_arm_controller/src/follower_controller_node.cpp`

```cpp
class FollowerControllerNode : public rclcpp::Node {
public:
    FollowerControllerNode() : Node("follower_arm_controller") {
        // 파라미터
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB1");
        this->declare_parameter<int>("baud_rate", 1000000);
        this->declare_parameter<bool>("auto_follow", true);
        this->declare_parameter<double>("joint_scale", 1.0);

        // 직렬 인터페이스
        std::string port = this->get_parameter("serial_port").as_string();
        int baud = this->get_parameter("baud_rate").as_int();
        serial_iface_.init(port, baud);

        // Publisher: Follower 조인트 상태
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/follower_joint_states", 10);

        // Subscriber: Leader 조인트 상태 → Follower 명령
        leader_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/leader_joint_states", 10,
            std::bind(&FollowerControllerNode::leaderJointCallback,
                      this, std::placeholders::_1));

        // Follower 상태 발행 타이머
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&FollowerControllerNode::timerCallback, this));

        joint_names_ = {
            "joint_1_waist", "joint_2_shoulder", "joint_3_elbow",
            "joint_4_wrist_roll", "joint_5_wrist_pitch", "joint_6_wrist_yaw"
        };

        RCLCPP_INFO(this->get_logger(), "Follower Arm Controller started");
    }

private:
    void leaderJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Leader의 조인트 각도를 받아 Follower에 전달
        // 자동 추종 모드일 때만 실행
        if (auto_follow_) {
            // 조인트 각도 스케일링 적용
            double scale = this->get_parameter("joint_scale").as_double();
            std::vector<double> commands(6, 0.0);
            for (size_t i = 0; i < std::min(msg->position.size(), (size_t)6); i++) {
                commands[i] = msg->position[i] * scale;
            }
            serial_iface_.writeJointCommands(commands);
        }
    }

    void timerCallback() {
        // Follower의 현재 상태 발행
        std::vector<double> positions;
        if (serial_iface_.readJointPositions(positions) && positions.size() == 6) {
            auto msg = sensor_msgs::msg::JointState();
            msg.header.stamp = this->now();
            msg.name = joint_names_;
            msg.position = positions;
            joint_state_pub_->publish(msg);
        }
    }

    SerialInterface serial_iface_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr leader_joint_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> joint_names_;
    bool auto_follow_ = true;
};
```

## 7. 직렬 인터페이스 (ROS2 ↔ STM32)

### 7.1 직렬 프로토콜

```cpp
// serial_interface.h
#pragma once
#include <string>
#include <vector>
#include <cstdint>

class SerialInterface {
public:
    SerialInterface() : fd_(-1) {}
    ~SerialInterface() { close(); }

    bool init(const std::string& port, int baud_rate);
    void close();
    bool isOpen() const { return fd_ >= 0; }

    // Leader: 조인트 각도 읽기
    bool readJointPositions(std::vector<double>& positions);

    // Follower: 조인트 명령 쓰기
    bool writeJointCommands(const std::vector<double>& commands);

    // 모드 설정 명령 전송
    bool setMode(const std::string& mode);

    // 캘리브레이션
    bool calibrateJoints();
    bool homeAllJoints();

private:
    int fd_;  // serial port file descriptor

    // 직렬 프레임 프로토콜:
    // [0xAA][0x55][Length][Command][Data...][Checksum]
    enum class Command : uint8_t {
        READ_POSITIONS   = 0x01,
        WRITE_COMMANDS   = 0x02,
        SET_MODE         = 0x03,
        CALIBRATE        = 0x04,
        HOME             = 0x05,
        STATUS           = 0x06,
        TORQUE_OFF       = 0x07,
        TORQUE_ON        = 0x08,
    };

    std::vector<uint8_t> buildPacket(Command cmd, const std::vector<uint8_t>& data);
    bool sendPacket(const std::vector<uint8_t>& packet);
    bool receivePacket(std::vector<uint8_t>& response, int timeout_ms = 100);
    uint8_t calculateChecksum(const std::vector<uint8_t>& data);
    bool configurePort(int baud_rate);
};
```

### 7.2 직렬 프로토콜 프레임 포맷

```
Request Packet (ROS → STM32):
┌────────┬────────┬────────┬────────┬──────────┬────────┐
│ 0xAA   │ 0x55   │ Length │ Command│ Payload  │CRC     │
│ (Header)│ (Header)│ (1byte)│ (1byte)│ (N bytes)│(1byte) │
└────────┴────────┴────────┴────────┴──────────┴────────┘

Response Packet (STM32 → ROS):
┌────────┬────────┬────────┬────────┬──────────┬────────┐
│ 0xAA   │ 0x55   │ Length │ Status │ Payload  │CRC     │
│        │        │        │ (0=OK) │ (N bytes)│        │
└────────┴────────┴────────┴────────┴──────────┴────────┘
```

## 8. 시스템 실행 (Launch 파일)

### 8.1 전체 시스템 Launch

파일: `leader_follower_bringup/launch/leader_follower_system.launch.py`

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Arguments
    leader_port = LaunchConfiguration('leader_port', default='/dev/ttyUSB0')
    follower_port = LaunchConfiguration('follower_port', default='/dev/ttyUSB1')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('leader_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('follower_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # Leader Arm Controller
        Node(
            package='leader_arm_controller',
            executable='leader_controller_node',
            name='leader_arm_controller',
            parameters=[{
                'serial_port': leader_port,
                'baud_rate': 1000000,
                'publish_rate': 100.0,
            }],
            output='screen',
        ),

        # Follower Arm Controller
        Node(
            package='follower_arm_controller',
            executable='follower_controller_node',
            name='follower_arm_controller',
            parameters=[{
                'serial_port': follower_port,
                'baud_rate': 1000000,
                'auto_follow': True,
                'joint_scale': 1.0,
            }],
            output='screen',
        ),

        # Robot State Publisher (URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': open(
                    os.path.join(
                        get_package_share_directory('leader_follower_description'),
                        'urdf', 'leader_follower.urdf.xacro'
                    )
                ).read()
            }],
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('leader_follower_description'),
                'config', 'leader_follower.rviz'
            )],
        ),
    ])
```

### 8.2 실행 명령어

```bash
# 1. ROS2 환경 설정
source /opt/ros/humble/setup.bash

# 2. 워크스페이스 빌드
cd ~/ros_ws
colcon build --symlink-install
source install/setup.bash

# 3. Leader 단독 실행 (Follower 없이)
ros2 launch leader_follower_bringup leader_only.launch.py

# 4. 전체 시스템 실행
ros2 launch leader_follower_bringup leader_follower_system.launch.py

# 5. 토픽 확인
ros2 topic list
ros2 topic echo /leader_joint_states
ros2 topic echo /follower_joint_states

# 6. RViz 시각화
ros2 run rviz2 rviz2
```

## 9. ros2_control 통합

### 9.1 컨트롤러 설정

파일: `leader_follower_bringup/config/leader_controllers.yaml`

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_controller:
      type: joint_state_controller/JointStateController

    joint_trajectory_controller:
      type: position_controllers/JointTrajectoryController

joint_trajectory_controller:
  ros__parameters:
    joints:
      - joint_1_waist
      - joint_2_shoulder
      - joint_3_elbow
      - joint_4_wrist_roll
      - joint_5_wrist_pitch
      - joint_6_wrist_yaw
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
      - effort
    state_publish_rate: 100.0
    action_monitor_rate: 20.0
    allow_partial_joints_goal: false
    open_loop_control: false
    allow_integration_in_goal_trajectories: true
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.0
```

## 10. 테스트 및 검증

### 10.1 단위 테스트

```bash
# 직렬 통신 테스트 (STM32 → ROS)
ros2 run leader_arm_controller test_serial_interface

# Leader 조인트 읽기 테스트
ros2 topic echo /leader_joint_states --once

# Follower 명령 수동 전송
ros2 topic pub /follower_joint_command std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]}"

# 시스템 모드 변경
ros2 service call /set_mode leader_follower_msgs/srv/SetMode "{mode: 'FOLLOWING'}"
```

### 10.2 통합 테스트 시나리오

| 테스트 | 방법 | 기대 결과 |
|--------|------|----------|
| Leader 단독 작동 | Leader arm 수동 조작 | `/leader_joint_states` 토픽에 6개 조인트 각도 발행 |
| Leader→Follower 전달 | Leader 조작 → 토픽 감시 | `/follower_joint_command`에 동일 각도 수신 |
| Follower 동기화 | Leader 조작 → 육안 확인 | Follower arm이 Leader와 동일 자세 |
| 지연 시간 측정 | Leader 조작 → 타임스탬프 비교 | 종단간 지연 < 20ms |
| E-Stop 테스트 | E-Stop 스위치 누름 | 모든 AX-12A 토크 OFF, 에러 메시지 발행 |
| rosbag 기록/재생 | `ros2 bag record` | 기록된 궤적 재생 시 일치하는 동작 |
