# Frame Burst Node 사용법

## 개요
Frame Burst Node는 Arena Camera에서 Frame Burst 모드를 사용하여 트리거 신호에 따라 지정된 수의 프레임을 연속으로 캡처하는 ROS2 노드입니다.

## 특징
- 하드웨어 트리거 (Line0, Rising Edge) 지원
- 소프트웨어 트리거 지원  
- UserSet 프로필 로드 지원
- 설정 가능한 버스트 프레임 수
- RViz 호환 frame_id 설정

## 매개변수

### 기본 카메라 설정
- `serial`: 카메라 시리얼 번호 (정수)
- `width`: 이미지 너비 (기본값: 1280)
- `height`: 이미지 높이 (기본값: 720) 
- `pixelformat`: 픽셀 포맷 (기본값: "rgb8")
- `gain`: 게인 값 (선택사항)
- `exposure_time`: 노출 시간 (마이크로초, 선택사항)

### Frame Burst 전용 설정
- `hardware_trigger`: 하드웨어 트리거 모드 활성화 (기본값: true)
- `software_trigger`: 소프트웨어 트리거 모드 활성화 (기본값: false)
- `burst_frame_count`: 버스트당 프레임 수 (기본값: 10)
- `user_set`: 로드할 UserSet (기본값: "UserSet1")

### ROS2 설정
- `topic`: 이미지 발행 토픽 (기본값: "/frame_burst_node/burst_images")
- `qos_history`: QoS 히스토리 정책
- `qos_history_depth`: QoS 히스토리 깊이
- `qos_reliability`: QoS 신뢰성 정책

## 사용 예시

### 1. 하드웨어 트리거 모드 (Line0, 10프레임 버스트)
```bash
ros2 run arena_camera_node frame_burst --ros-args \
  -p serial:=904240001 \
  -p hardware_trigger:=true \
  -p burst_frame_count:=10 \
  -p user_set:=UserSet1 \
  -p topic:=/camera/burst_images
```

### 2. 소프트웨어 트리거 모드
```bash
ros2 run arena_camera_node frame_burst --ros-args \
  -p serial:=904240001 \
  -p software_trigger:=true \
  -p hardware_trigger:=false \
  -p burst_frame_count:=5 \
  -p user_set:=UserSet1
```

### 3. 소프트웨어 트리거 호출
```bash
ros2 service call /frame_burst_node/trigger_burst std_srvs/srv/Trigger
```

### 4. 커스텀 해상도 및 설정
```bash
ros2 run arena_camera_node frame_burst --ros-args \
  -p serial:=904240001 \
  -p width:=1920 \
  -p height:=1080 \
  -p pixelformat:=rgb8 \
  -p hardware_trigger:=true \
  -p burst_frame_count:=15 \
  -p gain:=10.0 \
  -p exposure_time:=1000 \
  -p user_set:=UserSet1
```

### 5. Image 저장
```
  ros2 run arena_camera_node frame_burst --ros-args -p software_trigger:=false -p hardware_trigger:=true -p burst_frame_count:=10 -p frame_rate:=20.0 -p save_img_folder:="/home/ailab/burst_images"

``` 

## 트리거 설정 (Arena SDK에서 미리 설정 필요)

### UserSet1에 설정해야 할 항목:
1. **Trigger Selector**: FrameBurstStart
2. **Trigger Source**: Line0 (하드웨어) 또는 Software (소프트웨어)
3. **Trigger Activation**: RisingEdge
4. **Acquisition Burst Frame Count**: 원하는 프레임 수
5. **Line0 설정**: LineMode = Input

## RViz에서 시각화

```bash
# 터미널 1: Frame Burst Node 실행
ros2 run arena_camera_node frame_burst --ros-args -p serial:=904240001

# 터미널 2: RViz 실행
rviz2

# RViz에서:
# 1. Add -> By Topic -> /frame_burst_node/burst_images -> Image
# 2. Fixed Frame을 "camera_frame"으로 설정
```

## 서비스

### `/frame_burst_node/trigger_burst`
- 타입: `std_srvs/srv/Trigger`
- 설명: 소프트웨어 트리거 모드에서 프레임 버스트를 수동으로 트리거

## 로그 메시지

- 하드웨어 트리거 모드에서는 트리거 신호를 기다리는 동안 대기 메시지 출력
- 각 버스트 시퀀스 완료 시 캡처된 프레임 수 보고
- 설정된 UserSet 프로필 로드 확인 메시지

## 문제 해결

1. **카메라가 연결되지 않는 경우**: 시리얼 번호 확인
2. **트리거가 작동하지 않는 경우**: UserSet 설정 확인 
3. **RViz에서 이미지가 보이지 않는 경우**: 토픽 이름과 frame_id 확인
4. **타임아웃 에러**: 하드웨어 트리거 신호 연결 상태 확인
