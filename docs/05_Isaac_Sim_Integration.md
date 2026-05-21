# 05. Isaac Sim 통합 설계 (Isaac Sim Integration)

## 1. 개요

NVIDIA Isaac Sim은 로봇 시뮬레이션을 위한 고성능 가상 환경입니다.
본 프로젝트에서는 Isaac Sim을 활용하여 Leader-Follower 암 시스템을 가상 환경에서 시뮬레이션하고,
실제 하드웨어 없이도 소프트웨어 개발 및 검증을 가능하게 합니다.

## 2. Isaac Sim 환경 설정

### 2.1 시스템 요구사항

| 구성 요소 | 최소 사양 | 권장 사양 |
|-----------|----------|----------|
| GPU | NVIDIA RTX 3060 8GB | NVIDIA RTX 4090 24GB |
| RAM | 16GB | 32GB |
| 저장소 | 20GB 여유 공간 | SSD 50GB |
| OS | Ubuntu 22.04 LTS | Ubuntu 22.04 LTS |
| NVIDIA 드라이버 | 545+ | 550+ |
| CUDA | 12.0+ | 12.4+ |
| Isaac Sim | 2023.1.1+ | 2024.1+ |

### 2.2 설치

```bash
# 1. NVIDIA 드라이버 확인
nvidia-smi

# 2. Isaac Sim 다운로드 (NVIDIA Package Manager)
# https://developer.nvidia.com/isaac-sim

# 3. 설치 스크립트 실행
chmod +x isaac_sim_2024.1.1_linux_installer.sh
./isaac_sim_2024.1.1_linux_installer.sh

# 4. ROS2 bridge 설치 (Isaac Sim 내장)
cd ~/.local/share/ov/pkg/isaac_sim-2024.1.1
./python.sh -m pip install ros2-humble

# 5. 환경 변수 설정 (bashrc에 추가)
echo "source ~/.local/share/ov/pkg/isaac_sim-2024.1.1/setup.sh" >> ~/.bashrc
echo "export ISAAC_SIM_PATH=~/.local/share/ov/pkg/isaac_sim-2024.1.1" >> ~/.bashrc
```

## 3. URDF → Isaac Sim 변환

### 3.1 URDF 임포트

Isaac Sim은 URDF 파일을 직접 임포트하여 USD(Universal Scene Description)로 변환할 수 있습니다.

**Python API를 사용한 URDF 임포트:**

```python
# simulation/isaac_sim/python_scripts/import_urdf.py
import carb
import omni.usd
from pxr import Usd, UsdGeom, Gf
from omni.isaac.core.utils.extensions import enable_extension

# URDF 임포트 확장 활성화
enable_extension("omni.importer.urdf")

from omni.importer.urdf import URDFImporter

def import_leader_arm_urdf():
    """Leader Arm URDF를 Isaac Sim USD로 변환"""
    urdf_path = "/workspace/ros_ws/src/leader_follower_description/urdf/leader_arm.urdf.xacro"
    output_path = "/workspace/simulation/isaac_sim/urdf/leader_arm.usd"

    # URDF 임포터 설정
    importer = URDFImporter(
        urdf_path=urdf_path,
        import_config={
            "merge_fixed_joints": False,
            "fix_base_link": True,
            "import_inertia": True,
            "distance_scale": 1.0,
            "self_collision": False,
            "replace_cylinders_with_capsules": False,
            "create_physics": True,
        }
    )

    # USD로 임포트
    stage = omni.usd.get_context().get_stage()
    importer.import_to_stage(stage, output_path)
    print(f"URDF imported to: {output_path}")

if __name__ == "__main__":
    import_leader_arm_urdf()
```

### 3.2 USD 파일 구조

Isaac Sim에서 임포트된 Leader Arm USD 구조:

```
/World
  /leader_arm              # Arm 모델 (루트)
    /base_link             # 베이스 링크
      /joint_1_waist       # Joint 1
        /link_1            # 링크 1
          /joint_2_shoulder
            /link_2
              /joint_3_elbow
                /link_3
                  /joint_4_wrist_roll
                    /link_4
                      /joint_5_wrist_pitch
                        /link_5
                          /joint_6_wrist_yaw
                            /end_effector
```

## 4. Isaac Sim Python 제어 스크립트

### 4.1 Leader Arm 시뮬레이션 제어

```python
# simulation/isaac_sim/python_scripts/leader_arm_control.py

import numpy as np
from omni.isaac.core import World
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.isaac.core.utils.prims as prim_utils

class LeaderArmSimulation:
    """Isaac Sim에서 Leader Arm을 제어하는 클래스"""

    def __init__(self, usd_path: str):
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()

        # URDF/USD 로드
        add_reference_to_stage(usd_path, "/World/leader_arm")
        self.world.reset()

        # Articulation View 생성 (조인트 제어용)
        self.leader_arm = ArticulationView(
            prims_paths_expr="/World/leader_arm",
            name="leader_arm_view"
        )
        self.world.scene.add(self.leader_arm)

        # 조인트 이름 매핑
        self.joint_names = [
            "joint_1_waist",
            "joint_2_shoulder",
            "joint_3_elbow",
            "joint_4_wrist_roll",
            "joint_5_wrist_pitch",
            "joint_6_wrist_yaw"
        ]
        self.num_joints = 6

    def set_joint_positions(self, joint_angles: np.ndarray):
        """조인트 각도 설정 (radians)"""
        assert len(joint_angles) == self.num_joints
        self.leader_arm.set_joint_positions(joint_angles)

    def get_joint_positions(self) -> np.ndarray:
        """현재 조인트 각도 읽기"""
        return self.leader_arm.get_joint_positions()

    def step(self):
        """시뮬레이션 한 스텝 진행"""
        self.world.step(render=True)

    def run(self, steps: int = 1000):
        """시뮬레이션 실행"""
        for _ in range(steps):
            self.step()

    def close(self):
        self.world.stop()
```

### 4.2 ROS2 Bridge 연동

Isaac Sim과 ROS2 간의 양방향 통신을 설정합니다.

```python
# simulation/isaac_sim/python_scripts/ros2_bridge.py

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from omni.isaac.core import World
from omni.isaac.core.articulations import ArticulationView

class IsaacSimROS2Bridge(Node):
    """Isaac Sim ↔ ROS2 브릿지 노드"""

    def __init__(self, arm_view: ArticulationView, joint_names: list):
        super().__init__('isaac_sim_ros2_bridge')

        self.arm_view = arm_view
        self.joint_names = joint_names
        self.num_joints = len(joint_names)

        # Publisher: 시뮬레이션 조인트 상태 → ROS2
        self.joint_pub = self.create_publisher(
            JointState, '/leader_joint_states', 10)

        # Subscriber: ROS2 명령 → 시뮬레이션
        self.cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/leader_joint_command',
            self.joint_command_callback,
            10)

        # 타이머: 100Hz 발행
        self.timer = self.create_timer(0.01, self.publish_joint_states)

        self.get_logger().info("Isaac Sim ROS2 Bridge initialized")
        self.last_command = None

    def publish_joint_states(self):
        """시뮬레이션 조인트 상태 발행"""
        if self.arm_view is None:
            return

        positions = self.arm_view.get_joint_positions()
        velocities = self.arm_view.get_joint_velocities()
        efforts = self.arm_view.get_joint_efforts()

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = positions.tolist() if positions is not None else [0.0]*self.num_joints
        msg.velocity = velocities.tolist() if velocities is not None else [0.0]*self.num_joints
        msg.effort = efforts.tolist() if efforts is not None else [0.0]*self.num_joints
        self.joint_pub.publish(msg)

    def joint_command_callback(self, msg: Float64MultiArray):
        """ROS2 명령을 받아 시뮬레이션 조인트 제어"""
        if len(msg.data) >= self.num_joints:
            cmd = np.array(msg.data[:self.num_joints], dtype=np.float32)
            self.last_command = cmd
            self.arm_view.set_joint_positions(cmd)

    def apply_last_command(self):
        """마지막 명령 재적용 (매 스텝 호출 필요)"""
        if self.last_command is not None:
            self.arm_view.set_joint_positions(self.last_command)
```

### 4.3 Leader-Follower 시뮬레이션

```python
# simulation/isaac_sim/python_scripts/leader_follower_sim.py

"""
Leader-Follower Arm 통합 시뮬레이션 스크립트

실행 방법:
  cd simulation/isaac_sim
  python python_scripts/leader_follower_sim.py
"""

import numpy as np
import time
import rclpy
from omni.isaac.core import World
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.stage import add_reference_to_stage
from ros2_bridge import IsaacSimROS2Bridge

class LeaderFollowerSimulation:
    """Leader-Follower 암 통합 시뮬레이션"""

    def __init__(self):
        # ROS2 초기화
        rclpy.init()

        # World 초기화
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()

        # Leader Arm 로드
        add_reference_to_stage(
            "/workspace/simulation/isaac_sim/urdf/leader_arm.usd",
            "/World/leader_arm"
        )

        # Follower Arm 로드 (Leader와 동일한 구조, 다른 위치)
        add_reference_to_stage(
            "/workspace/simulation/isaac_sim/urdf/follower_arm.usd",
            "/World/follower_arm"
        )

        # Follower Arm 위치 이동 (Leader에서 0.5m 떨어진 위치)
        from pxr import UsdGeom, Gf
        follower_prim = self.world.stage.GetPrimAtPath("/World/follower_arm")
        if follower_prim.IsValid():
            xform = UsdGeom.XformCommonAPI(follower_prim)
            xform.SetTranslate((0.5, 0.0, 0.0))

        self.world.reset()

        # Articulation Views
        self.leader_arm = ArticulationView(
            prims_paths_expr="/World/leader_arm",
            name="leader_arm_view"
        )
        self.follower_arm = ArticulationView(
            prims_paths_expr="/World/follower_arm",
            name="follower_arm_view"
        )
        self.world.scene.add(self.leader_arm)
        self.world.scene.add(self.follower_arm)

        # Joint names
        self.joint_names = [
            "joint_1_waist", "joint_2_shoulder", "joint_3_elbow",
            "joint_4_wrist_roll", "joint_5_wrist_pitch", "joint_6_wrist_yaw"
        ]

        # ROS2 Bridge
        self.bridge = IsaacSimROS2Bridge(self.leader_arm, self.joint_names)

        # 데이터 로깅
        self.trajectory_data = []

    def run(self, duration_sec: float = 60.0):
        """시뮬레이션 실행 (Leader 조작 → Follower 자동 추종)"""
        self.get_logger().info("Starting Leader-Follower simulation...")

        # 홈 포지션
        home_position = np.zeros(6, dtype=np.float32)

        start_time = time.time()
        sim_time = 0.0
        dt = 1.0 / 60.0  # 60Hz

        while sim_time < duration_sec:
            # ROS2 이벤트 처리
            rclpy.spin_once(self.bridge, timeout_sec=0)

            # Leader 현재 위치 읽기
            leader_pos = self.leader_arm.get_joint_positions()

            # Follower가 Leader를 추종하도록 설정
            self.follower_arm.set_joint_positions(leader_pos)

            # 시뮬레이션 스텝
            self.world.step(render=True)

            # 데이터 로깅 (10Hz)
            if int(sim_time * 10) > int((sim_time - dt) * 10):
                self.trajectory_data.append({
                    'timestamp': sim_time,
                    'leader_positions': leader_pos.copy(),
                    'follower_positions': self.follower_arm.get_joint_positions().copy()
                })

            sim_time += dt

        # 데이터 저장
        self.save_trajectory()
        self.get_logger().info("Simulation completed")

    def save_trajectory(self, filename: str = "trajectory_data.npz"):
        """궤적 데이터 저장 (Jetson 학습 데이터로 활용)"""
        timestamps = [d['timestamp'] for d in self.trajectory_data]
        leader_data = np.array([d['leader_positions'] for d in self.trajectory_data])
        follower_data = np.array([d['follower_positions'] for d in self.trajectory_data])

        np.savez(filename,
                 timestamps=timestamps,
                 leader_positions=leader_data,
                 follower_positions=follower_data)
        print(f"Trajectory saved: {filename} ({len(self.trajectory_data)} samples)")

    def get_logger(self):
        return rclpy.logging.get_logger("LeaderFollowerSim")

if __name__ == "__main__":
    sim = LeaderFollowerSimulation()
    try:
        sim.run(30.0)  # 30초 시뮬레이션
    finally:
        sim.world.stop()
        rclpy.shutdown()
```

## 5. Isaac Sim을 활용한 개발 워크플로우

### 5.1 단계별 개발 프로세스

```
Phase 1: URDF 준비
  ├── SolidWorks/Fusion 360에서 3D 모델링
  ├── URDF 내보내기 (sw_urdf_exporter)
  ├── Isaac Sim에서 URDF 임포트 및 USD 변환
  └── 시각적 검증 (조인트 움직임 확인)

Phase 2: ROS2 통신 설정
  ├── ROS2 Bridge 노드 실행
  ├── /leader_joint_states 토픽 발행 확인
  ├── /leader_joint_command 구독 확인
  └── RViz에서 시뮬레이션 상태 실시간 확인

Phase 3: 컨트롤러 개발/테스트
  ├── Leader Arm 컨트롤러 코드 시뮬레이션에서 테스트
  ├── Follower Arm 추종 로직 검증
  ├── 경로 계획 및 충돌 회피 테스트
  └── 성능 측정 (응답 시간, 정확도)

Phase 4: 데이터 수집
  ├── 다양한 궤적 시나리오 실행
  ├── rosbag으로 조인트 데이터 수집
  ├── 데이터 전처리 및 증강
  └── AI 학습 데이터셋 준비

Phase 5: 하드웨어 전환
  ├── 시뮬레이션 검증 완료된 코드 그대로 사용
  ├── Hardware → Simulation 간 전환 가능 (동일 ROS 인터페이스)
  ├── 실제 하드웨어에서 동일 동작 확인
  └── 시뮬레이션 대비 실제 성능 차이 분석
```

### 5.2 시뮬레이션 ↔ 실제 하드웨어 전환

```python
# simulation/isaac_sim/python_scripts/mode_switcher.py

"""
시뮬레이션 모드와 실제 하드웨어 모드 간 전환
동일한 ROS2 인터페이스를 사용하므로 코드 변경 최소화
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class ModeSwitcher(Node):
    """시뮬레이션/실제 하드웨어 모드 전환"""

    def __init__(self):
        super().__init__('mode_switcher')
        self.mode = 'SIMULATION'  # 또는 'HARDWARE'

        # 모드 전환 서비스
        self.srv = self.create_service(
            Trigger, '/switch_mode',
            self.switch_mode_callback)

        self.get_logger().info(f"Current mode: {self.mode}")

    def switch_mode_callback(self, request, response):
        if self.mode == 'SIMULATION':
            self.mode = 'HARDWARE'
        else:
            self.mode = 'SIMULATION'

        response.success = True
        response.message = f"Switched to {self.mode} mode"
        self.get_logger().info(response.message)

    def get_mode(self):
        return self.mode

def is_simulation_mode():
    """다른 노드에서 모드 확인용 함수"""
    # ROS 파라미터로 모드 확인 가능
    return rclpy.parameter.Parameter(
        '/use_sim_time',
        rclpy.parameter.Parameter.Type.BOOL,
        True
    ).value
```

## 6. 시뮬레이션 데이터 활용

### 6.1 데이터 수집 파이프라인

```
Isaac Sim 시뮬레이션
    │
    ├── rosbag record -a  (모든 토픽 기록)
    │
    ├── Python API (joint positions, velocities, efforts)
    │
    └── Camera (RGB-D 데이터)

데이터 전처리:
    ├── 타임스탬프 정렬
    ├── 노이즈 필터링 (Low-pass filter)
    ├── 이상치 제거
    └── 정규화 (Min-max scaling)

출력 포맷:
    ├── .npz (NumPy) — 학습용
    ├── .csv — 분석용
    └── .bag (rosbag) — 재생용
```

### 6.2 CSV 데이터 포맷

```csv
timestamp,j1_pos,j2_pos,j3_pos,j4_pos,j5_pos,j6_pos,j1_vel,j2_vel,j3_vel,j4_vel,j5_vel,j6_vel,mode
0.000,0.000,0.523,-0.785,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,leader
0.010,0.002,0.525,-0.783,0.001,0.000,0.001,0.200,0.200,0.200,0.100,0.000,0.100,leader
0.020,0.005,0.528,-0.780,0.003,0.001,0.003,0.300,0.300,0.300,0.200,0.100,0.200,leader
...
```

## 7. Isaac Sim 성능 최적화

### 7.1 실시간 성능 팁

```python
# 성능 최적화 설정
self.world = World(
    stage_units_in_meters=1.0,
    physics_dt=1.0 / 60.0,     # 물리 시뮬레이션 스텝: 60Hz
    rendering_dt=1.0 / 30.0,    # 렌더링: 30Hz (RTX 3060 기준)
)

# 불필요한 시각 효과 비활성화
from omni.kit.viewport import acquire_viewport_interface
vp = acquire_viewport_interface()
viewport = vp.get_viewport_window()
viewport.set_active(False)  # 헤드리스 모드 (필요시)
```

### 7.2 Physics 설정

```python
# 물리 엔진 설정 (PhysX)
from omni.physx import acquire_physx_interface
physx_iface = acquire_physx_interface()
physx_iface.set_physx_setting("timeStep", 1.0/60.0)
physx_iface.set_physx_setting("enableCCD", True)       # 연속 충돌 감지
physx_iface.set_physx_setting("enableEnhancedDeterminism", False)
physx_iface.set_physx_setting("gpuMaxNumPartitions", 8)
```

## 8. 자주 묻는 질문

### 8.1 URDF 임포트 오류

**문제:** URDF 임포트 시 "Failed to parse URDF" 오류  
**해결:** xacro 파일을 미리 처리하여 순수 URDF로 변환
```bash
cd ros_ws/src/leader_follower_description/urdf
ros2 run xacro xacro leader_arm.urdf.xacro > leader_arm_processed.urdf
```

### 8.2 ROS2 Bridge 연결 실패

**문제:** Isaac Sim과 ROS2 Bridge가 연결되지 않음  
**해결:** 환경 변수 확인
```bash
# Isaac Sim setup.sh가 ROS2 환경과 충돌하는 경우
source /opt/ros/humble/setup.bash
source ~/.local/share/ov/pkg/isaac_sim-2024.1.1/setup.sh  # 순서 중요

# 또는 Python 경로 확인
export PYTHONPATH=$ISAAC_SIM_PATH/python:$PYTHONPATH
```

### 8.3 실시간 성능 저하

**문제:** 시뮬레이션이 느리게 실행됨  
**해결:** 렌더링 해상도 낮추기
```python
# Viewport 해상도 조정
viewport.set_texture_resolution(1024, 768)  # 기본 1920x1080
```
