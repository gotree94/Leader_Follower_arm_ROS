std_msgs/Header header
uint8 arm_id                   # 0: Leader, 1: Follower
uint8 system_mode              # 0: IDLE, 1: FOLLOWING, 2: SIMULATION, 3: INFERENCE, 4: CALIBRATION, 5: ERROR
uint8 error_code               # 0: NONE, 1: COMM_TIMEOUT, 2: OVERLOAD, 3: OVERHEAT, 4: VOLTAGE, 5: E_STOP
float32 present_voltage        # 현재 전압 (V)
float32 present_temperature    # 현재 온도 (°C)
uint16[6] raw_positions        # AX-12A Raw 값 (디버깅용)
