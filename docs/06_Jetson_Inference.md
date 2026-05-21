# 06. Jetson Orin Nano 추론 시스템 (Jetson Inference)

## 1. 개요

Jetson Orin Nano는 Leader arm의 조작 데이터를 학습하여 AI 모델로 추론하고,
추론 결과를 ROS 토픽으로 발행하여 Follower arm을 제어하거나 시뮬레이션을 구동합니다.

**목표:**
- Leader arm 조작 궤적 데이터 수집 및 전처리
- 딥러닝 모델 학습 (시계열 예측: Transformer / LSTM / TCN)
- TensorRT 변환을 통한 Jetson 최적화
- 실시간 추론 및 ROS2 토픽 발행

## 2. 시스템 구성

```
┌────────────────────────────────────────────────────────────────┐
│                    데이터 수집 파이프라인                        │
│                                                                │
│  실제 Leader Arm  ──→  rosbag  ──→  CSV/Parquet  ──→  Dataset  │
│  Isaac Sim        ──→  .npz    ──→  전처리         ──→  학습용  │
│  수동 조작 데이터  ──→  .csv   ──→  (정규화/증강)   ──→  준비   │
└────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌────────────────────────────────────────────────────────────────┐
│                      모델 학습 (PC/Cloud)                       │
│                                                                │
│  Dataset ──→ PyTorch Model ──→ ONNX Export ──→ TensorRT Engine │
│              (LSTM/Transformer)    (.onnx)        (.engine)    │
└────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌────────────────────────────────────────────────────────────────┐
│                   Jetson Orin Nano 배포                         │
│                                                                │
│  TensorRT Engine ──→ Inference Node ──→ ROS2 Topic             │
│  (.engine)          (C++/Python)      (/inference_joint_states)│
│                                    ──→ Follower Arm / Isaac Sim│
└────────────────────────────────────────────────────────────────┘
```

## 3. Jetson Orin Nano 환경 설정

### 3.1 사양 및 준비

| 항목 | 내용 |
|------|------|
| 모델 | Jetson Orin Nano (8GB) |
| CUDA | 11.4+ (JetPack 6.0) |
| TensorRT | 8.6+ |
| PyTorch | 2.1+ (Jetson용 사전 빌드) |
| ROS2 | Humble |
| 전원 | 5V/4A USB-C 또는 DC 잭 |
| 스토리지 | NVMe SSD 256GB+ (권장) |
| 네트워크 | Gigabit Ethernet (USB-to-ROS PC 연결) |

### 3.2 초기 설정

```bash
# 1. JetPack 설치 (NVIDIA SDK Manager 사용)
# https://developer.nvidia.com/sdk-manager

# 2. 시스템 업데이트
sudo apt update && sudo apt upgrade -y

# 3. ROS2 Humble 설치 (Jetson)
sudo apt install ros-humble-desktop python3-colcon-common-extensions

# 4. PyTorch 설치 (Jetson용 사전 빌드)
wget https://developer.download.nvidia.com/compute/redist/jp/v60/pytorch/torch-2.1.0a0+…  # JetPack 6.0 버전 확인

# 5. TensorRT 설치 확인
dpkg -l | grep tensorrt

# 6. 기타 의존성
pip install numpy pandas scikit-learn matplotlib onnx onnxruntime
sudo apt install python3-serial
```

## 4. 데이터 수집 및 전처리

### 4.1 rosbag을 통한 데이터 수집

```bash
# Leader arm 조작 데이터 녹음 (ROS PC)
ros2 bag record /leader_joint_states /follower_joint_states /arm_status

# 특정 시간 후 중지 (Ctrl+C)
# 저장 위치: rosbag2_YYYY_MM_DD-HH_MM_SS/

# rosbag → CSV 변환
ros2 bag play <bag_file> --rate 1.0
# 별도 CSV 기록 노드로 저장
```

### 4.2 데이터 전처리 스크립트

```python
# inference/jetson/data_collection/preprocess_data.py

import numpy as np
import pandas as pd
from sklearn.preprocessing import MinMaxScaler
from sklearn.model_selection import train_test_split

class JointDataPreprocessor:
    """조인트 데이터 전처리 클래스"""

    def __init__(self, sequence_length: int = 20):
        """
        Args:
            sequence_length: 입력 시퀀스 길이 (과거 20스텝 = 200ms @ 100Hz)
        """
        self.seq_len = sequence_length
        self.scaler_X = MinMaxScaler(feature_range=(-1, 1))
        self.scaler_y = MinMaxScaler(feature_range=(-1, 1))

    def load_csv(self, filepath: str) -> pd.DataFrame:
        """CSV 파일 로드"""
        df = pd.read_csv(filepath)
        required_cols = ['timestamp', 'j1_pos', 'j2_pos', 'j3_pos',
                        'j4_pos', 'j5_pos', 'j6_pos']
        assert all(col in df.columns for col in required_cols), \
            f"CSV must contain: {required_cols}"
        return df

    def create_sequences(self, data: np.ndarray) -> tuple:
        """시계열 시퀀스 생성 (슬라이딩 윈도우)"""
        X, y = [], []
        for i in range(len(data) - self.seq_len):
            X.append(data[i:i + self.seq_len])      # 입력: 과거 seq_len 스텝
            y.append(data[i + self.seq_len])         # 출력: 다음 1스텝
        return np.array(X), np.array(y)

    def preprocess(self, df: pd.DataFrame) -> tuple:
        """전체 전처리 파이프라인"""
        # 조인트 각도 컬럼 선택
        joint_cols = ['j1_pos', 'j2_pos', 'j3_pos', 'j4_pos', 'j5_pos', 'j6_pos']
        data = df[joint_cols].values  # shape: (N, 6)

        # 정규화
        data_scaled = self.scaler_X.fit_transform(data)

        # 시퀀스 생성
        X, y = self.create_sequences(data_scaled)

        # Train/Validation 분할
        X_train, X_val, y_train, y_val = train_test_split(
            X, y, test_size=0.2, random_state=42, shuffle=False
        )

        return (X_train, y_train), (X_val, y_val)

    def inverse_transform(self, scaled_data: np.ndarray) -> np.ndarray:
        """정규화 복원"""
        return self.scaler_X.inverse_transform(scaled_data)


# 사용 예시
if __name__ == "__main__":
    preprocessor = JointDataPreprocessor(sequence_length=20)
    df = preprocessor.load_csv("leader_trajectory_001.csv")
    (X_train, y_train), (X_val, y_val) = preprocessor.preprocess(df)
    print(f"Train: X={X_train.shape}, y={y_train.shape}")
    print(f"Validation: X={X_val.shape}, y={y_val.shape}")
    # Train: X=(9800, 20, 6), y=(9800, 6)  (10,000 샘플 기준)
```

## 5. 모델 설계

### 5.1 LSTM 기반 시계열 예측 모델 (권장)

```python
# inference/jetson/model/joint_prediction_model.py

import torch
import torch.nn as nn

class JointPredictionLSTM(nn.Module):
    """LSTM 기반 조인트 각도 예측 모델

    입력: (batch, seq_len, 6) — 과거 20스텝의 6개 조인트 각도
    출력: (batch, 6) — 다음 스텝의 6개 조인트 각도
    """

    def __init__(self,
                 input_size: int = 6,
                 hidden_size: int = 128,
                 num_layers: int = 3,
                 output_size: int = 6,
                 dropout: float = 0.2,
                 seq_len: int = 20):
        super().__init__()

        self.lstm = nn.LSTM(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=True,
            dropout=dropout,
            bidirectional=False
        )

        # Attention mechanism (선택)
        self.attention = nn.Sequential(
            nn.Linear(hidden_size, 64),
            nn.Tanh(),
            nn.Linear(64, 1)
        )

        # 출력 레이어
        self.fc = nn.Sequential(
            nn.Linear(hidden_size, 64),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(64, output_size)
        )

    def forward(self, x):
        # x: (batch, seq_len, 6)
        lstm_out, (hidden, cell) = self.lstm(x)
        # lstm_out: (batch, seq_len, hidden_size)
        # hidden: (num_layers, batch, hidden_size) — 마지막 타임스텝

        # 마지막 레이어의 hidden state 사용
        last_hidden = hidden[-1]  # (batch, hidden_size)

        # 출력
        output = self.fc(last_hidden)  # (batch, 6)
        return output


class JointPredictionTransformer(nn.Module):
    """Transformer 기반 조인트 각도 예측 모델 (고성능)"""

    def __init__(self,
                 input_size: int = 6,
                 d_model: int = 128,
                 nhead: int = 4,
                 num_layers: int = 4,
                 dim_feedforward: int = 512,
                 dropout: float = 0.1,
                 output_size: int = 6):
        super().__init__()

        self.input_projection = nn.Linear(input_size, d_model)
        self.positional_encoding = PositionalEncoding(d_model, dropout)

        encoder_layer = nn.TransformerEncoderLayer(
            d_model=d_model,
            nhead=nhead,
            dim_feedforward=dim_feedforward,
            dropout=dropout,
            batch_first=True
        )
        self.transformer_encoder = nn.TransformerEncoder(
            encoder_layer, num_layers=num_layers)

        self.output_projection = nn.Sequential(
            nn.Linear(d_model, 64),
            nn.ReLU(),
            nn.Linear(64, output_size)
        )

    def forward(self, x):
        # x: (batch, seq_len, 6)
        x = self.input_projection(x)       # (batch, seq_len, d_model)
        x = self.positional_encoding(x)     # (batch, seq_len, d_model)
        x = self.transformer_encoder(x)     # (batch, seq_len, d_model)
        # 마지막 타임스텝 사용
        x = x[:, -1, :]                     # (batch, d_model)
        output = self.output_projection(x)  # (batch, 6)
        return output


class PositionalEncoding(nn.Module):
    """Transformer Positional Encoding"""

    def __init__(self, d_model: int, dropout: float = 0.1, max_len: int = 100):
        super().__init__()
        self.dropout = nn.Dropout(p=dropout)
        pe = torch.zeros(max_len, d_model)
        position = torch.arange(0, max_len, dtype=torch.float).unsqueeze(1)
        div_term = torch.exp(torch.arange(0, d_model, 2).float() *
                             (-torch.log(torch.tensor(10000.0)) / d_model))
        pe[:, 0::2] = torch.sin(position * div_term)
        pe[:, 1::2] = torch.cos(position * div_term)
        pe = pe.unsqueeze(0)  # (1, max_len, d_model)
        self.register_buffer('pe', pe)

    def forward(self, x):
        x = x + self.pe[:, :x.size(1), :]
        return self.dropout(x)
```

### 5.2 학습 스크립트

```python
# inference/jetson/model/train.py

import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset
import numpy as np
from joint_prediction_model import JointPredictionLSTM

def train_model(model, train_loader, val_loader, epochs=100,
                learning_rate=1e-3, device='cuda'):
    """모델 학습"""

    model = model.to(device)
    criterion = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode='min', factor=0.5, patience=10)

    best_val_loss = float('inf')

    for epoch in range(epochs):
        # Training
        model.train()
        train_loss = 0.0
        for X_batch, y_batch in train_loader:
            X_batch, y_batch = X_batch.to(device), y_batch.to(device)

            optimizer.zero_grad()
            output = model(X_batch)
            loss = criterion(output, y_batch)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)  # Gradient clipping
            optimizer.step()

            train_loss += loss.item() * X_batch.size(0)

        train_loss /= len(train_loader.dataset)

        # Validation
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for X_batch, y_batch in val_loader:
                X_batch, y_batch = X_batch.to(device), y_batch.to(device)
                output = model(X_batch)
                loss = criterion(output, y_batch)
                val_loss += loss.item() * X_batch.size(0)

        val_loss /= len(val_loader.dataset)
        scheduler.step(val_loss)

        # Best model 저장
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            torch.save(model.state_dict(), 'best_model.pth')

        if (epoch + 1) % 10 == 0:
            print(f"Epoch [{epoch+1}/{epochs}] "
                  f"Train Loss: {train_loss:.6f}, "
                  f"Val Loss: {val_loss:.6f}")

    print(f"Training completed. Best Val Loss: {best_val_loss:.6f}")
    return model


if __name__ == "__main__":
    # 데이터 로드 (예시)
    X_train = np.random.randn(8000, 20, 6).astype(np.float32)
    y_train = np.random.randn(8000, 6).astype(np.float32)
    X_val = np.random.randn(2000, 20, 6).astype(np.float32)
    y_val = np.random.randn(2000, 6).astype(np.float32)

    train_dataset = TensorDataset(torch.from_numpy(X_train), torch.from_numpy(y_train))
    val_dataset = TensorDataset(torch.from_numpy(X_val), torch.from_numpy(y_val))

    train_loader = DataLoader(train_dataset, batch_size=64, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=64, shuffle=False)

    model = JointPredictionLSTM(
        input_size=6, hidden_size=128,
        num_layers=3, output_size=6
    )

    train_model(model, train_loader, val_loader, epochs=100, device='cuda')
```

## 6. TensorRT 변환

### 6.1 PyTorch → ONNX → TensorRT

```python
# inference/jetson/model/convert_to_trt.py

import torch
import torch.onnx
import onnx
import numpy as np
import tensorrt as trt

class ModelConverter:
    """PyTorch 모델 → TensorRT Engine 변환"""

    def __init__(self, model: torch.nn.Module, input_size=(1, 20, 6)):
        self.model = model
        self.input_size = input_size
        self.model.eval()

    def to_onnx(self, onnx_path: str = "model.onnx"):
        """PyTorch → ONNX 변환"""
        dummy_input = torch.randn(self.input_size)

        torch.onnx.export(
            self.model,
            dummy_input,
            onnx_path,
            input_names=['joint_sequence'],
            output_names=['predicted_joints'],
            dynamic_axes={
                'joint_sequence': {0: 'batch_size'},
                'predicted_joints': {0: 'batch_size'}
            },
            opset_version=17,
            do_constant_folding=True,
        )
        print(f"ONNX model saved: {onnx_path}")

        # ONNX 모델 검증
        onnx_model = onnx.load(onnx_path)
        onnx.checker.check_model(onnx_model)
        print("ONNX model verification passed")

    def to_tensorrt(self,
                    onnx_path: str = "model.onnx",
                    engine_path: str = "model.engine",
                    precision: str = "fp16"):
        """ONNX → TensorRT Engine 변환 (FP16 양자화)"""

        TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
        builder = trt.Builder(TRT_LOGGER)
        network = builder.create_network(
            1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
        )
        parser = trt.OnnxParser(network, TRT_LOGGER)

        # ONNX 파일 로드
        with open(onnx_path, 'rb') as f:
            if not parser.parse(f.read()):
                for error in range(parser.num_errors):
                    print(parser.get_error(error))
                raise RuntimeError("Failed to parse ONNX file")

        # Builder 설정
        config = builder.create_builder_config()
        config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 30)  # 1GB

        if precision == "fp16" and builder.platform_has_fast_fp16:
            config.set_flag(trt.BuilderFlag.FP16)
            print("FP16 precision enabled")

        # INT8 양자화 (선택, 캘리브레이션 데이터 필요)
        if precision == "int8" and builder.platform_has_fast_int8:
            config.set_flag(trt.BuilderFlag.INT8)
            print("INT8 precision enabled")
            # 캘리브레이션 데이터 로드 필요
            # config.int8_calibrator = ...

        # Engine 빌드
        serialized_engine = builder.build_serialized_network(network, config)
        if serialized_engine is None:
            raise RuntimeError("Failed to build TensorRT engine")

        # Engine 저장
        with open(engine_path, 'wb') as f:
            f.write(serialized_engine)
        print(f"TensorRT engine saved: {engine_path} (FP16={precision=='fp16'})")

    def benchmark(self, engine_path: str = "model.engine", iterations: int = 1000):
        """TensorRT 엔진 성능 벤치마크"""
        import time

        TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
        runtime = trt.Runtime(TRT_LOGGER)

        with open(engine_path, 'rb') as f:
            engine = runtime.deserialize_cengine(f.read())

        context = engine.create_execution_context()
        inputs = [torch.randn(self.input_size).cuda()]

        # Warm-up
        for _ in range(100):
            context.execute_v2([inp.data_ptr() for inp in inputs])

        # Benchmark
        start = time.time()
        for _ in range(iterations):
            context.execute_v2([inp.data_ptr() for inp in inputs])
        elapsed = time.time() - start

        fps = iterations / elapsed
        latency = elapsed / iterations * 1000  # ms
        print(f"TensorRT Inference: {fps:.1f} FPS, {latency:.3f} ms per inference")
        return fps, latency


if __name__ == "__main__":
    from joint_prediction_model import JointPredictionLSTM

    model = JointPredictionLSTM(input_size=6, hidden_size=128, num_layers=3, output_size=6)
    model.load_state_dict(torch.load('best_model.pth'))

    converter = ModelConverter(model)
    converter.to_onnx("joint_model.onnx")
    converter.to_tensorrt("joint_model.onnx", "joint_model_fp16.engine", precision="fp16")
    converter.benchmark("joint_model_fp16.engine")
```

## 7. ROS2 Inference Node (Jetson)

### 7.1 C++ Inference Node (실시간 성능 최적화)

```cpp
// inference/jetson/inference_node/src/inference_node.cpp

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <fstream>
#include <vector>
#include <sstream>
#include <cuda_runtime.h>
#include <NvInfer.h>

class InferenceNode : public rclcpp::Node {
public:
    InferenceNode() : Node("jetson_inference_node") {
        // 파라미터
        this->declare_parameter<std::string>("engine_path", "model.engine");
        this->declare_parameter<int>("sequence_length", 20);
        this->declare_parameter<double>("inference_rate", 100.0);

        // TensorRT 엔진 로드
        std::string engine_path = this->get_parameter("engine_path").as_string();
        loadEngine(engine_path);

        // Circular buffer for input sequence
        seq_length_ = this->get_parameter("sequence_length").as_int();
        input_buffer_.resize(seq_length_ * 6, 0.0f);

        // Publisher: 추론 결과 조인트 각도
        inference_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/inference_joint_states", 10);

        // Subscriber: Leader 조인트 상태 → 추론 입력
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/leader_joint_states", 10,
            std::bind(&InferenceNode::jointCallback, this, std::placeholders::_1));

        // Service: 추론 시작/중지
        start_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/inference_start",
            std::bind(&InferenceNode::startCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // 타이머: 추론 루프
        double rate = this->get_parameter("inference_rate").as_double();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(1000.0 / rate)),
            std::bind(&InferenceNode::inferenceLoop, this));

        // 조인트 이름
        joint_names_ = {
            "joint_1_waist", "joint_2_shoulder", "joint_3_elbow",
            "joint_4_wrist_roll", "joint_5_wrist_pitch", "joint_6_wrist_yaw"
        };

        inferencing_ = false;
        RCLCPP_INFO(this->get_logger(), "Jetson Inference Node started");
    }

    ~InferenceNode() {
        if (context_) context_->destroy();
        if (engine_) engine_->destroy();
        if (runtime_) runtime_->destroy();
    }

private:
    void loadEngine(const std::string& engine_path) {
        std::ifstream file(engine_path, std::ios::binary);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open engine: %s",
                        engine_path.c_str());
            return;
        }

        file.seekg(0, std::ios::end);
        size_t size = file.tellg();
        file.seekg(0, std::ios::beg);

        std::vector<char> engine_data(size);
        file.read(engine_data.data(), size);
        file.close();

        // TensorRT 런타임 및 엔진 생성
        runtime_ = nvinfer1::createInferRuntime(logger_);
        if (!runtime_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create TensorRT runtime");
            return;
        }

        engine_ = runtime_->deserializeCudaEngine(engine_data.data(), size);
        if (!engine_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to deserialize engine");
            return;
        }

        context_ = engine_->createExecutionContext();
        if (!context_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create execution context");
            return;
        }

        // CUDA 메모리 할당
        cudaMalloc(&input_device_, seq_length_ * 6 * sizeof(float));
        cudaMalloc(&output_device_, 6 * sizeof(float));

        RCLCPP_INFO(this->get_logger(), "TensorRT engine loaded successfully");
    }

    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Leader 조인트 각도를 입력 버퍼에 추가
        if (msg->position.size() >= 6) {
            // Shift buffer left by 6
            std::copy(input_buffer_.begin() + 6, input_buffer_.end(),
                      input_buffer_.begin());
            // Add new data at the end
            for (int i = 0; i < 6; i++) {
                input_buffer_[seq_length_ * 6 - 6 + i] = msg->position[i];
            }
        }
    }

    void inferenceLoop() {
        if (!inferencing_ || !context_ || !engine_) return;

        // 입력 데이터를 GPU로 복사
        cudaMemcpy(input_device_, input_buffer_.data(),
                   seq_length_ * 6 * sizeof(float),
                   cudaMemcpyHostToDevice);

        // TensorRT 추론 실행
        void* bindings[] = {input_device_, output_device_};
        context_->executeV2(bindings);

        // 출력 데이터를 CPU로 복사
        float output[6];
        cudaMemcpy(output, output_device_, 6 * sizeof(float),
                   cudaMemcpyDeviceToHost);

        // 추론 결과 발행
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.name = joint_names_;
        msg.position = std::vector<double>(output, output + 6);
        inference_pub_->publish(msg);
    }

    void startCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        inferencing_ = !inferencing_;
        response->success = true;
        response->message = inferencing_ ? "Inference started" : "Inference stopped";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    // Logger
    class Logger : public nvinfer1::ILogger {
        void log(Severity severity, const char* msg) noexcept override {
            if (severity <= Severity::kWARNING) {
                std::cerr << "[TensorRT] " << msg << std::endl;
            }
        }
    } logger_;

    // TensorRT
    nvinfer1::IRuntime* runtime_ = nullptr;
    nvinfer1::ICudaEngine* engine_ = nullptr;
    nvinfer1::IExecutionContext* context_ = nullptr;
    float *input_device_ = nullptr, *output_device_ = nullptr;

    // ROS2
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr inference_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Data
    std::vector<float> input_buffer_;
    int seq_length_;
    bool inferencing_;
    std::vector<std::string> joint_names_;
};
```

### 7.2 Launch 파일

```python
# inference/jetson/inference_node/launch/inference_node.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jetson_inference',
            executable='inference_node',
            name='jetson_inference_node',
            parameters=[{
                'engine_path': '/workspace/inference/jetson/model/joint_model_fp16.engine',
                'sequence_length': 20,
                'inference_rate': 100.0,
            }],
            output='screen',
        )
    ])
```

## 8. 추론 모드 통합 워크플로우

### 8.1 시스템 실행 시나리오

```bash
# Jetson Orin Nano에서 실행
source /opt/ros/humble/setup.bash
cd ~/ros_ws
colcon build --packages-select jetson_inference
source install/setup.bash

# 추론 노드 실행
ros2 launch jetson_inference inference_node.launch.py

# 모드 변경 (Leader-Follower 시스템에서)
ros2 service call /set_mode leader_follower_msgs/srv/SetMode "{mode: 'INFERENCE'}"

# 추론 시작
ros2 service call /inference_start std_srvs/srv/Trigger

# 추론 결과 확인
ros2 topic echo /inference_joint_states
```

### 8.2 성능 목표

| 지표 | 목표 | 측정 방법 |
|------|------|----------|
| 추론 지연 시간 | < 2ms | CUDA event timing |
| 추론 처리량 | > 500 FPS | 초당 추론 횟수 |
| 종단간 지연 | < 10ms (Leader 입력 → 추론 → Follower) | ROS 타임스탬프 |
| 모델 크기 | < 50MB | ONNX/Engine 파일 크기 |
| GPU 메모리 사용 | < 2GB | nvidia-smi |


## 9. 프로젝트 확장 방향

### 9.1 추가 기능

| 확장 기능 | 설명 | 우선순위 |
|-----------|------|---------|
| 선제적 모션 예측 | 10스텝 앞선 조인트 각도 예측 | 높음 |
| 이상 동작 감지 | Leader 조작 중 비정상 패턴 감지 | 중간 |
| Force/Torque 예측 | 조인트 부하 예측을 통한 안전 제어 | 중간 |
| 데모 기반 학습 | 여러 데모 궤적으로부터 일반화 모델 학습 | 낮음 (고급) |
| 강화 학습 (RL) | 시뮬레이션에서 RL로 최적 궤적 학습 | 낮음 (고급) |
