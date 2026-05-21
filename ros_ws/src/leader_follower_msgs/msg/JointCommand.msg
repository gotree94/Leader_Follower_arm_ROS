std_msgs/Header header
float64[6] joint_angles       # 목표 각도 (radians)
float64[6] joint_velocities   # 목표 속도 (rad/s, 선택)
uint8 mode_flag               # 0: 절대 위치, 1: 상대 위치, 2: 속도 제어
